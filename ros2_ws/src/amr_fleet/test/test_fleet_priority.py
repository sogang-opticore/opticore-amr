#!/usr/bin/env python3
"""
GATE-F3-LOGIC — fleet_priority 순수코어 단위테스트 (CLAUDE.md §5a).

sim 불필요·결정론. 수치는 docs/fleet_f3/PLAN.md §3 의 t=0,0.5,..,6.0 샘플 그리드로 검증됨.

실행: source /workspace/env.sh && cd /workspace/ros2_ws \
      && colcon test --packages-select amr_fleet && colcon test-result --verbose
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

import fleet_priority as fp  # noqa: E402


def _rr_conflicts(res):
    return [c for c in res.conflicts if c.kind == 'robot-robot']


# ---------------------------------------------------------------------------
# T1 — robot-robot 직교 교차 (대칭 → arrival 동률 → id_tiebreak)
# ---------------------------------------------------------------------------
def test_T1_robot_robot_crossing():
    robots = [
        {'id': 1, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)], 'speed': 1.0},
        {'id': 2, 'pose': (5.0, -5.0, 0.0), 'path': [(5, -5), (5, 5)], 'speed': 1.0},
    ]
    res = fp.compute_priorities(robots, [], {})
    rr = _rr_conflicts(res)
    assert len(rr) == 1
    c = rr[0]
    assert (c.a, c.b) == (1, 2)
    assert abs(c.t_conflict - 4.0) < 1e-9          # 첫 충돌 t=4.0 (d=1.414<1.5)
    assert abs(c.arrival_a - c.arrival_b) < 1e-9   # (5,0) 교차 arrival 5.0 동률
    assert len(res.decisions) == 1
    d = res.decisions[0]
    assert d.winner_id == 1 and d.yielding_id == 2
    assert d.reason == 'id_tiebreak'
    assert res.priorities[1] > res.priorities[2]


# ---------------------------------------------------------------------------
# T2 — 평행(비교차) → 충돌 0, 우선순위 변경 0
# ---------------------------------------------------------------------------
def test_T2_parallel_no_conflict():
    robots = [
        {'id': 1, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)], 'speed': 1.0},
        {'id': 2, 'pose': (0.0, 3.0, 0.0), 'path': [(0, 3), (10, 3)], 'speed': 1.0},
    ]
    res = fp.compute_priorities(robots, [], {})
    assert res.conflicts == []
    assert res.decisions == []
    assert fp.to_array(res.priorities, 2) == [2, 1]   # baseline 그대로


# ---------------------------------------------------------------------------
# T3 — 공간교차하나 시간 어긋남 → 충돌 아님 (FP 방지)
# ---------------------------------------------------------------------------
def test_T3_temporal_separation_no_false_positive():
    robots = [
        {'id': 1, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)], 'speed': 1.0},
        {'id': 2, 'pose': (2.0, -5.0, 0.0), 'path': [(2, -5), (2, 5)], 'speed': 1.0},
    ]
    res = fp.compute_priorities(robots, [], {})
    # 같은 인덱스 최소 분리 2.121m @ t=3.5 > 1.5 → 충돌 0
    assert res.conflicts == []
    assert res.decisions == []


# ---------------------------------------------------------------------------
# T4 — robot-obstacle: 접근=양보 / 멀어짐=양보 안 함
# ---------------------------------------------------------------------------
def test_T4_robot_obstacle_yield_and_recede():
    base = [{'id': 1, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)], 'speed': 1.0}]

    approaching = [{'x': 5.0, 'y': 5.0, 'vx': 0.0, 'vy': -1.0, 'class_name': 'person'}]
    res = fp.compute_priorities(base, approaching, {})
    ro = [c for c in res.conflicts if c.kind == 'robot-obstacle']
    assert len(ro) == 1
    assert abs(ro[0].t_conflict - 4.0) < 1e-9
    dec = [d for d in res.decisions if d.reason == 'obstacle']
    assert len(dec) == 1
    assert dec[0].yielding_id == 1 and dec[0].winner_id is None
    assert fp.to_array(res.priorities, 1) == [1]    # 배열 불변

    receding = [{'x': 5.0, 'y': 5.0, 'vx': 0.0, 'vy': 1.0, 'class_name': 'person'}]
    res2 = fp.compute_priorities(base, receding, {})
    assert res2.conflicts == []
    assert res2.decisions == []


# ---------------------------------------------------------------------------
# T5 — handoff: 멀면(>2m) 지금 결정 / 가까우면(<2m) F-2 에 양도(손 안 댐)
# ---------------------------------------------------------------------------
def test_T5_handoff_far_decides_close_skips():
    far = [
        {'id': 1, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)], 'speed': 1.0},
        {'id': 2, 'pose': (8.0, 0.0, 0.0), 'path': [(8, 0), (0, 0)], 'speed': 1.0},
    ]
    rf = fp.compute_priorities(far, [], {})
    assert any(c.kind == 'robot-robot' for c in rf.conflicts)   # proactive 탐지
    assert len(rf.decisions) >= 1
    assert rf.decisions[0].winner_id == 1                       # 대칭 head-on → 낮은 id

    close = [
        {'id': 1, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)], 'speed': 1.0},
        {'id': 2, 'pose': (1.5, 0.0, 0.0), 'path': [(1.5, 0), (10, 0)], 'speed': 1.0},
    ]
    rc = fp.compute_priorities(close, [], {})
    assert rc.decisions == []                                   # handoff → F-2 영역


# ---------------------------------------------------------------------------
# T6 — 결정론 (동일입력 → 동일출력, 동률 → 낮은 id)
# ---------------------------------------------------------------------------
def test_T6_determinism_and_tie_lower_id():
    robots = [
        {'id': 1, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)], 'speed': 1.0},
        {'id': 2, 'pose': (5.0, -5.0, 0.0), 'path': [(5, -5), (5, 5)], 'speed': 1.0},
    ]
    r1 = fp.compute_priorities(robots, [], {})
    r2 = fp.compute_priorities(robots, [], {})
    assert r1 == r2                                  # flapping 0
    assert r1.decisions[0].winner_id == 1
    assert r1.decisions[0].reason == 'id_tiebreak'


# ---------------------------------------------------------------------------
# T7 — priority-dominant override (그리고 pd=false 시 right_of_way)
#   amr1 이 교차점에 먼저 도착(arr 5.0), amr2 늦게(arr 6.5). amr2 고우선.
# ---------------------------------------------------------------------------
def test_T7_priority_dominant_overrides_right_of_way():
    robots = [
        {'id': 1, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)],
         'speed': 1.0, 'priority': 1},
        {'id': 2, 'pose': (5.0, -6.5, 0.0), 'path': [(5, -6.5), (5, 3.5)],
         'speed': 1.0, 'priority': 5},
    ]
    rd = fp.compute_priorities(robots, [], {'priority_dominant': True})
    assert len(rd.decisions) == 1
    assert rd.decisions[0].winner_id == 2 and rd.decisions[0].yielding_id == 1
    assert rd.decisions[0].reason == 'priority_dominant'        # amr1 먼저여도 amr2 우선
    assert rd.priorities[2] > rd.priorities[1]

    rr = fp.compute_priorities(robots, [], {'priority_dominant': False})
    assert rr.decisions[0].winner_id == 1 and rr.decisions[0].yielding_id == 2
    assert rr.decisions[0].reason == 'right_of_way'             # 먼저 도착이 이김
    assert rr.priorities[1] > rr.priorities[2]


# ---------------------------------------------------------------------------
# H1 — right_of_way 가 id_tiebreak 와 구별됨 (고-id 가 먼저 도착하면 고-id 승)
# ---------------------------------------------------------------------------
def test_H1_right_of_way_distinct_from_id():
    # arrival: id1 교차점 6.5s(늦음), id2 5.0s(먼저). 낮은 id 가 아니라 먼저도착이 이겨야.
    robots = [
        {'id': 1, 'pose': (5.0, -6.5, 0.0), 'path': [(5, -6.5), (5, 3.5)], 'speed': 1.0},
        {'id': 2, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)], 'speed': 1.0},
    ]
    r = fp.compute_priorities(robots, [], {'priority_dominant': False})
    assert len(r.decisions) == 1
    d = r.decisions[0]
    assert d.winner_id == 2 and d.yielding_id == 1               # 고-id 가 먼저도착 → 승
    assert d.reason == 'right_of_way'
    assert r.priorities[2] > r.priorities[1]


# ---------------------------------------------------------------------------
# H2 — F-2 default 규약 일치 (4대 무충돌 → [4,3,2,1])
# ---------------------------------------------------------------------------
def test_H2_baseline_equals_f2_default():
    robots = [{'id': i, 'pose': (0.0, i * 3.0, 0.0),
               'path': [(0, i * 3), (10, i * 3)], 'speed': 1.0} for i in range(1, 5)]
    r = fp.compute_priorities(robots, [], {})
    assert r.conflicts == []
    assert fp.to_array(r.priorities, 4) == [4, 3, 2, 1]


# ---------------------------------------------------------------------------
# H3 — 위상 layering: 한 로봇이 한쪽에 지고 다른쪽 이김 → 두 edge 모두 strict
# ---------------------------------------------------------------------------
def test_H3_layering_winner_and_loser():
    ids = [1, 2, 3]
    baseline = {1: 3, 2: 2, 3: 1}
    edges = [(3, 1), (1, 2)]            # 3>1, 1>2 (robot1 은 패자이자 승자)
    prio = fp.layer_priorities(ids, baseline, edges)
    assert prio[3] > prio[1]
    assert prio[1] > prio[2]


# ---------------------------------------------------------------------------
# H4 — obstacle 충돌은 배열을 바꾸지 않는다 (robot-robot 무충돌 유지)
# ---------------------------------------------------------------------------
def test_H4_obstacle_does_not_change_array():
    robots = [
        {'id': 1, 'pose': (0.0, 0.0, 0.0), 'path': [(0, 0), (10, 0)], 'speed': 1.0},
        {'id': 2, 'pose': (0.0, 5.0, 0.0), 'path': [(0, 5), (10, 5)], 'speed': 1.0},
    ]
    obs = [{'x': 5.0, 'y': 5.0, 'vx': 0.0, 'vy': -1.0, 'class_name': 'forklift'}]
    r = fp.compute_priorities(robots, obs, {})
    assert any(c.kind == 'robot-obstacle' for c in r.conflicts)
    assert not any(c.kind == 'robot-robot' for c in r.conflicts)
    assert fp.to_array(r.priorities, 2) == [2, 1]    # obstacle 양보가 배열을 안 건드림


# ---------------------------------------------------------------------------
# H5 — 사이클 입력도 결정론적으로 깨고 일관 (방어)
# ---------------------------------------------------------------------------
def test_H5_cycle_break_deterministic():
    ids = [1, 2, 3]
    baseline = {1: 3, 2: 2, 3: 1}
    edges = [(1, 2), (2, 3), (3, 1)]    # 사이클
    p1 = fp.layer_priorities(ids, baseline, edges)
    p2 = fp.layer_priorities(ids, baseline, edges)
    assert p1 == p2                     # 결정론 (사전식 최대 edge (3,1) 드롭)
    assert p1[1] > p1[2] and p1[2] > p1[3]
