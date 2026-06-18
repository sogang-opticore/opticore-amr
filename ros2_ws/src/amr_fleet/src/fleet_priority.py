#!/usr/bin/env python3
"""
F-3 proactive priority — 순수 코어 (ROS 비의존, GATE-F3-LOGIC 대상).

path + track 으로 충돌을 *시간기반* 예측하고, F-2 우선순위 인터페이스(/fleet/priorities)에
주입할 정수 배열을 결정론적으로 산출한다. 이 모듈은 stdlib 만 쓰며 rclpy 를 import 하지 않는다
→ `colcon test` 단위검증이 sim 없이 결정론으로 돈다.

핵심 계약(docs/fleet_f3/PLAN.md §3):
  - robot-robot: 같은 시각 예측위치 거리 < robot_conflict_radius_m 인 첫 t → 충돌.
      승자 = (a) priority_dominant ∧ 양쪽 priority ∧ 상이 → 고우선
           → (b) |Δarrival| ≥ epsilon → 충돌점 먼저 도착(right_of_way)
           → (c) else 낮은 id(id_tiebreak). 패자 양보.
      arrival 은 **geometric crossing** 까지의 호장/속도(대칭교차서 right_of_way 가 붕괴하지 않도록).
      handoff: 현재 중심거리 < handoff_distance_m 쌍은 **건드리지 않음**(F-2 영역).
  - robot-obstacle: 장애물 등속투영과 거리 < obstacle_conflict_radius_m → 로봇 **무조건 양보**.
      F-2 는 비-AMR 에 enforce 불가 → 결정/마커로만 노출, /fleet/priorities 배열은 **불변**.
  - priorities 배열: baseline = priority or (N - index)  (F-2 default 규약: 값 클수록 우선).
      decision(winner→loser) 을 **위상정렬 layering** 으로 적용 → 모든 edge 에 winner > loser 보장.

모든 비교는 dist² < radius²(보고용만 sqrt). 순회는 id 정렬(set 금지). 동률 tie-break = 최소 호장.
→ 동일 입력 → 동일 출력(flapping 0, T6).
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

XY = Tuple[float, float]

DEFAULT_PARAMS: Dict[str, object] = {
    'horizon_sec': 6.0,
    'horizon_dt': 0.5,
    'robot_conflict_radius_m': 1.5,
    'obstacle_conflict_radius_m': 2.0,
    'cruise_speed_mps': 0.8,
    'arrival_tie_epsilon_sec': 1.0,
    'handoff_distance_m': 2.0,
    'priority_dominant': True,
    'stopped_speed_eps': 0.05,
}


@dataclass
class Conflict:
    kind: str                       # 'robot-robot' | 'robot-obstacle'
    a: int                          # robot id
    b: Optional[int]                # other robot id (None for obstacle)
    obstacle_index: Optional[int]   # index into obstacles list (None for robot-robot)
    t_conflict: float               # first sample time conflict predicted (sec)
    point: XY                       # conflict location (map frame)
    arrival_a: Optional[float]      # robot a arrival time at conflict point
    arrival_b: Optional[float]      # robot b arrival (None for obstacle)


@dataclass
class Decision:
    yielding_id: int                # robot that yields
    winner_id: Optional[int]        # robot with right-of-way (None = yielded to obstacle)
    reason: str                     # priority_dominant | right_of_way | id_tiebreak | obstacle


@dataclass
class Result:
    conflicts: List[Conflict] = field(default_factory=list)
    decisions: List[Decision] = field(default_factory=list)
    priorities: Dict[int, int] = field(default_factory=dict)


# ----------------------------------------------------------------------------
# 기하 유틸 (순수)
# ----------------------------------------------------------------------------
def _eff_speed(speed: Optional[float], cruise: float, stopped_eps: float) -> float:
    """현재 속도; 멈춤(<eps)이면 cruise 로 path 투영(PLAN §3-1)."""
    if speed is not None and speed > stopped_eps:
        return float(speed)
    return float(cruise)


def _closest_point_on_segment(p: XY, a: XY, b: XY) -> Tuple[XY, float, float]:
    """점 p 에서 선분 a-b 위 최근접점 → (point, t∈[0,1], dist²)."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0.0:
        ddx, ddy = px - ax, py - ay
        return (ax, ay), 0.0, ddx * ddx + ddy * ddy
    t = ((px - ax) * dx + (py - ay) * dy) / L2
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    cx, cy = ax + t * dx, ay + t * dy
    ddx, ddy = px - cx, py - cy
    return (cx, cy), t, ddx * ddx + ddy * ddy


def _snap_to_path(pose_xy: XY, path: List[XY]) -> Tuple[XY, List[XY]]:
    """
    Pose 를 path 폴리라인에 스냅 → (스냅점, 그 점부터의 forward 폴리라인).

    동률(같은 dist²)이면 최소 호장 우선 → 결정론. path 비거나 1점이면 정지(forward=[]).
    """
    if not path:
        return pose_xy, []
    if len(path) == 1:
        return path[0], []
    best = None  # (dist2, arclen, seg_i, point)
    cum = 0.0
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        cpt, tparam, d2 = _closest_point_on_segment(pose_xy, a, b)
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        arc = cum + tparam * seg
        if (best is None or d2 < best[0]
                or (d2 == best[0] and arc < best[1])):
            best = (d2, arc, i, cpt)
        cum += seg
    _d2, _arc, seg_i, cpt = best
    forward = [cpt] + [tuple(q) for q in path[seg_i + 1:]]
    return cpt, forward


def _advance(polyline: List[XY], dist: float) -> XY:
    """polyline[0] 에서 호장 dist 만큼 전진한 점. 끝점 clamp."""
    if not polyline:
        raise ValueError('empty polyline')
    if len(polyline) == 1 or dist <= 0.0:
        return polyline[0]
    remaining = dist
    for i in range(len(polyline) - 1):
        a, b = polyline[i], polyline[i + 1]
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg == 0.0:
            continue
        if remaining <= seg:
            f = remaining / seg
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
        remaining -= seg
    return polyline[-1]


def _arclen_to_point(polyline: List[XY], point: XY) -> float:
    """polyline[0] 에서 point 의 최근접 투영까지 호장. 동률 → 최소 호장."""
    if len(polyline) < 2:
        return 0.0
    best_d2 = None
    best_arc = 0.0
    cum = 0.0
    for i in range(len(polyline) - 1):
        a, b = polyline[i], polyline[i + 1]
        _cpt, tparam, d2 = _closest_point_on_segment(point, a, b)
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        arc = cum + tparam * seg
        if best_d2 is None or d2 < best_d2 or (d2 == best_d2 and arc < best_arc):
            best_d2 = d2
            best_arc = arc
        cum += seg
    return best_arc


def _seg_intersect(p1: XY, p2: XY, p3: XY, p4: XY) -> Optional[XY]:
    """선분 p1p2 ∩ p3p4 의 교점(있으면). 평행/공선은 None."""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    d = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3)
    if d == 0.0:
        return None
    t = ((x3 - x1) * (y4 - y3) - (y3 - y1) * (x4 - x3)) / d
    u = ((x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1)) / d
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    return None


def _path_crossing(poly_a: List[XY], poly_b: List[XY]) -> Optional[XY]:
    """두 폴리라인의 첫 교차점(세그먼트 순서 결정론). 없으면 None."""
    if len(poly_a) < 2 or len(poly_b) < 2:
        return None
    for i in range(len(poly_a) - 1):
        for j in range(len(poly_b) - 1):
            pt = _seg_intersect(poly_a[i], poly_a[i + 1],
                                poly_b[j], poly_b[j + 1])
            if pt is not None:
                return pt
    return None


def sample_trajectory(pose_xy: XY, path: List[XY], eff_speed: float,
                      horizon: float, dt: float) -> List[Tuple[float, float, float]]:
    """
    Pose 를 path 에 스냅 후 eff_speed 로 전진 샘플 → [(t, x, y)] (t=0..horizon step dt).

    모든 로봇이 같은 t-grid → 같은 인덱스끼리 비교하면 '같은 시각' 비교가 된다.
    """
    start, forward = _snap_to_path(pose_xy, path)
    n = int(round(horizon / dt)) + 1
    if not forward:                      # 정지(경로 없음)
        return [(i * dt, start[0], start[1]) for i in range(n)]
    out = []
    for i in range(n):
        t = i * dt
        x, y = _advance(forward, eff_speed * t)
        out.append((t, x, y))
    return out


# ----------------------------------------------------------------------------
# 우선순위 배열 layering (위상정렬 — 단일패스 감소의 비건전성 회피)
# ----------------------------------------------------------------------------
def _kahn(nodes: List[int], edges: List[Tuple[int, int]]) -> Optional[List[int]]:
    """id-순 Kahn 위상정렬. 사이클이면 None."""
    indeg = {n: 0 for n in nodes}
    adj: Dict[int, List[int]] = {n: [] for n in nodes}
    for w, l in edges:
        adj[w].append(l)
        indeg[l] += 1
    queue = sorted(n for n in nodes if indeg[n] == 0)
    order: List[int] = []
    while queue:
        n = queue.pop(0)
        order.append(n)
        for m in sorted(adj[n]):
            indeg[m] -= 1
            if indeg[m] == 0:
                queue.append(m)
        queue.sort()
    return order if len(order) == len(nodes) else None


def topo_order(nodes: List[int], edges: List[Tuple[int, int]]
               ) -> Tuple[List[int], List[Tuple[int, int]], List[Tuple[int, int]]]:
    """
    결정론적 위상순서 + (살아남은 edge, drop 한 edge).

    edge dedup·self-loop 제거 후, 사이클이면 사전식 최대 edge 를 떨궈가며 DAG 화.
    drop 된 edge 는 enforce 되지 않는다(레이어링은 surviving 만 적용).
    """
    nodes = sorted(set(nodes))
    elist: List[Tuple[int, int]] = []
    seen = set()
    for w, l in edges:
        if w == l or (w, l) in seen:
            continue
        seen.add((w, l))
        elist.append((w, l))
    dropped: List[Tuple[int, int]] = []
    while True:
        order = _kahn(nodes, elist)
        if order is not None:
            return order, elist, dropped
        drop = sorted(elist)[-1]
        elist = [e for e in elist if e != drop]
        dropped.append(drop)


def layer_priorities(ids: List[int], baseline: Dict[int, int],
                     edges: List[Tuple[int, int]]) -> Dict[int, int]:
    """
    Baseline 에서 winner→loser edge 들을 위상순(승자 먼저)으로 적용한다.

    prio[loser] = min(prio[loser], prio[winner]-1). 한 노드가 승자이자 패자여도
    모든 (살아남은) edge 에 prio[winner] > prio[loser] 가 strict 하게 성립한다.
    """
    prio = dict(baseline)
    order, surviving, _dropped = topo_order(ids, edges)
    adj: Dict[int, List[int]] = {n: [] for n in order}
    for w, l in surviving:
        adj.setdefault(w, []).append(l)
    for node in order:
        for loser in sorted(adj.get(node, [])):
            if prio[loser] >= prio[node]:
                prio[loser] = prio[node] - 1
    return prio


# ----------------------------------------------------------------------------
# 메인
# ----------------------------------------------------------------------------
def _merge_params(params: Optional[dict]) -> dict:
    p = dict(DEFAULT_PARAMS)
    if params:
        p.update(params)
    return p


def compute_priorities(robots: List[dict], obstacles: Optional[List[dict]] = None,
                       params: Optional[dict] = None) -> Result:
    """
    충돌 예측 + 우선순위 결정. 입력/출력 계약은 모듈 docstring 참조.

    robots:    [{id:int, pose:(x,y[,yaw]), path:[(x,y)..], speed:float|None, priority:int|None}]
    obstacles: [{x,y,vx,vy,class_name}]
    """
    p = _merge_params(params)
    horizon = float(p['horizon_sec'])
    dt = float(p['horizon_dt'])
    rr2 = float(p['robot_conflict_radius_m']) ** 2
    orad2 = float(p['obstacle_conflict_radius_m']) ** 2
    cruise = float(p['cruise_speed_mps'])
    seps = float(p['stopped_speed_eps'])
    eps = float(p['arrival_tie_epsilon_sec'])
    handoff = float(p['handoff_distance_m'])
    pdom = bool(p['priority_dominant'])

    robs = sorted(robots, key=lambda r: r['id'])
    ids = [r['id'] for r in robs]
    n = len(robs)
    by_id = {r['id']: r for r in robs}

    traj: Dict[int, List[Tuple[float, float, float]]] = {}
    fwd: Dict[int, List[XY]] = {}
    espeed: Dict[int, float] = {}
    pose_xy: Dict[int, XY] = {}
    for r in robs:
        ex = _eff_speed(r.get('speed'), cruise, seps)
        px = (float(r['pose'][0]), float(r['pose'][1]))
        path = [(float(q[0]), float(q[1])) for q in (r.get('path') or [])]
        _start, forward = _snap_to_path(px, path)
        traj[r['id']] = sample_trajectory(px, path, ex, horizon, dt)
        fwd[r['id']] = forward
        espeed[r['id']] = ex
        pose_xy[r['id']] = px

    conflicts: List[Conflict] = []
    decisions: List[Decision] = []

    # ---- robot-robot ----
    for ii in range(n):
        for jj in range(ii + 1, n):
            a = ids[ii]          # a < b (정렬됨)
            b = ids[jj]
            # handoff 게이트: 이미 가까운 쌍은 F-2 반응존 → 손대지 않음
            if math.hypot(pose_xy[a][0] - pose_xy[b][0],
                          pose_xy[a][1] - pose_xy[b][1]) < handoff:
                continue
            sa, sb = traj[a], traj[b]
            m = min(len(sa), len(sb))
            cidx = None
            for k in range(m):
                ddx = sa[k][1] - sb[k][1]
                ddy = sa[k][2] - sb[k][2]
                if ddx * ddx + ddy * ddy < rr2:
                    cidx = k
                    break
            if cidx is None:
                continue
            t_conf = cidx * dt
            cross = _path_crossing(fwd[a], fwd[b])
            if cross is not None:
                arr_a = _arclen_to_point(fwd[a], cross) / espeed[a]
                arr_b = _arclen_to_point(fwd[b], cross) / espeed[b]
                point = cross
            else:                              # 비교차(평행/추종) → 동률 + midpoint
                arr_a = arr_b = t_conf
                point = ((sa[cidx][1] + sb[cidx][1]) / 2.0,
                         (sa[cidx][2] + sb[cidx][2]) / 2.0)
            conflicts.append(Conflict('robot-robot', a, b, None,
                                      t_conf, point, arr_a, arr_b))
            pa = by_id[a].get('priority')
            pb = by_id[b].get('priority')
            if pdom and pa is not None and pb is not None and pa != pb:
                winner = a if pa > pb else b
                reason = 'priority_dominant'
            elif abs(arr_a - arr_b) >= eps:
                winner = a if arr_a < arr_b else b
                reason = 'right_of_way'
            else:
                winner = a                     # a < b → 낮은 id
                reason = 'id_tiebreak'
            loser = b if winner == a else a
            decisions.append(Decision(loser, winner, reason))

    # ---- robot-obstacle (로봇 무조건 양보; 배열 비강제) ----
    obs = list(obstacles or [])
    for r in robs:
        sa = traj[r['id']]
        for oidx, ob in enumerate(obs):
            ox = float(ob['x'])
            oy = float(ob['y'])
            vx = float(ob.get('vx', 0.0))
            vy = float(ob.get('vy', 0.0))
            cidx = None
            for k in range(len(sa)):
                t = sa[k][0]
                ddx = sa[k][1] - (ox + vx * t)
                ddy = sa[k][2] - (oy + vy * t)
                if ddx * ddx + ddy * ddy < orad2:
                    cidx = k
                    break
            if cidx is None:
                continue
            t_conf = cidx * dt
            opx, opy = ox + vx * t_conf, oy + vy * t_conf
            point = ((sa[cidx][1] + opx) / 2.0, (sa[cidx][2] + opy) / 2.0)
            conflicts.append(Conflict('robot-obstacle', r['id'], None, oidx,
                                      t_conf, point, t_conf, None))
            decisions.append(Decision(r['id'], None, 'obstacle'))

    # ---- priorities 배열 (baseline = F-2 default 규약; obstacle 은 불변) ----
    baseline: Dict[int, int] = {}
    for idx, rid in enumerate(ids):
        pr = by_id[rid].get('priority')
        baseline[rid] = int(pr) if pr is not None else (n - idx)
    edges = [(d.winner_id, d.yielding_id) for d in decisions
             if d.reason != 'obstacle' and d.winner_id is not None]
    priorities = layer_priorities(ids, baseline, edges)

    return Result(conflicts=conflicts, decisions=decisions, priorities=priorities)


def to_array(priorities: Dict[int, int], n: int, default: int = 0) -> List[int]:
    """{id->prio} → Int32MultiArray.data (index = id-1, id ∈ 1..n)."""
    return [int(priorities.get(i + 1, default)) for i in range(n)]
