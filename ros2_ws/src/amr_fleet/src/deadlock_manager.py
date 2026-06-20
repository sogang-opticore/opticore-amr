#!/usr/bin/env python3
"""
F-2 fleet deadlock manager (🔴-1 골격 + 우선순위 주입).

설계(docs/fleet_f2/PLAN.md): 좁은 복도 정면 교착을 우선순위 기반으로 자동 해소한다.
- 액추에이터 = /amrN/goal_pose 조작만 (cmd_vel 직접 X) → DWA/A* 코드 변경 0 = F-1 무회귀.
  HOLD = goal:=현 pose, RETREAT = goal:=지나온 경로 D 뒤 점, RESUME = goal:=원 mission goal.
- 위치 = TF lookup(map, amrN/base_footprint) = AMCL+EKF localization 추정(결정론·식별 known).
  odometry/filtered 토픽 pose 는 per-robot odom_filtered 프레임이라 로봇간 비교 불가 →
  공통 map 프레임은 TF 로만. twist(속도)는 odometry 에서 보조로 받음.
  fused_tracker(확률적) 는 절대 미사용.
- 블로커가 '다른 AMR' 일 때만 교착 발동(pedestrian/forklift 는 odometry/TF 가 없어 시야 밖).

본 커밋(🔴-1) 범위: 패키지/노드 골격 + 구독·발행 배선 + 우선순위(주입/기본) + /fleet/deadlock_status 관측.
  교착 탐지(🔴-2) · 해소 FSM(🔴-3) 은 명시 TODO 스텁. FSM 은 전부 NORMAL 유지(아직 goal 미조작).

우선순위: /fleet/priorities (std_msgs/Int32MultiArray, transient_local) index=robotN-1, 값 클수록 우선.
  미수신 시 기본 = 낮은 id 높은우선. 동률 = 낮은 id tiebreak.
"""
import json
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, HistoryPolicy,
                       ReliabilityPolicy)

from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import tf2_ros


class FSM(str, Enum):
    NORMAL = 'NORMAL'
    HOLD = 'HOLD'
    RETREAT = 'RETREAT'
    RESUMING = 'RESUMING'


def yaw_from_quat(z, w):
    """planar(base_footprint) 가정: yaw 를 (z, w) 로만 복원."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class RobotState:
    def __init__(self, name, rid):
        self.name = name
        self.rid = rid                # 1-based id
        self.fsm = FSM.NORMAL
        self.pose = None              # (x, y, yaw) in map, or None
        self.pose_hist = []           # [(t, x, y, yaw)] ring buffer (map frame)
        self.lin_speed = 0.0          # |twist.linear| from odometry (보조 신호)
        self.mission_goal = None      # (x, y, yaw) — NORMAL 일 때 캐싱
        self.reached = False
        self.dwa_status = ''          # latest /amrN/dwa/status
        # 탐지(🔴-2)
        self.stuck = False
        self.blocked_by = None
        # 해소 FSM(🔴-3)
        self.hold_goal = None         # HOLD 시 고정 정지점 (x,y,yaw)
        self.retreat_goal = None      # RETREAT 목표점 (x,y)
        self.hold_since = None
        self.retreat_since = None
        self.resume_since = None
        self.flip_count = 0           # HOLD↔RETREAT 진동 횟수 (livelock 가드)
        self.livelock = False
        self.last_pub_t = 0.0         # goal 재발행 throttle


class DeadlockManager(Node):
    def __init__(self):
        super().__init__('deadlock_manager')

        # ---- 파라미터 (PLAN §3 기본값) ----
        self.declare_parameter('robots',
                               ['amr1:3.0:13.0', 'amr2:3.0:17.0',
                                'amr3:3.0:21.0', 'amr4:3.0:25.0'])
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame_suffix', 'base_footprint')
        # 탐지 (🔴-2)
        self.declare_parameter('t_stuck', 4.0)
        self.declare_parameter('eps_move', 0.10)
        self.declare_parameter('r_proximity', 2.0)
        self.declare_parameter('l_gate', 0.7)
        # 해소 (🔴-3)
        self.declare_parameter('goal_tol', 0.4)
        self.declare_parameter('x_hold', 6.0)
        self.declare_parameter('d_retreat', 4.0)
        self.declare_parameter('retreat_timeout', 20.0)
        self.declare_parameter('resume_clear_dist', 0.8)
        self.declare_parameter('livelock_max_n', 3)
        # 루프/관측
        self.declare_parameter('control_rate', 5.0)
        self.declare_parameter('status_rate', 2.0)
        self.declare_parameter('hist_window', 12.0)   # pose 이력 보관(sec)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_suffix = self.get_parameter('base_frame_suffix').value
        self.t_stuck = self.get_parameter('t_stuck').value
        self.eps_move = self.get_parameter('eps_move').value
        self.r_prox = self.get_parameter('r_proximity').value
        self.l_gate = self.get_parameter('l_gate').value
        self.goal_tol = self.get_parameter('goal_tol').value
        self.x_hold = self.get_parameter('x_hold').value
        self.d_retreat = self.get_parameter('d_retreat').value
        self.retreat_timeout = self.get_parameter('retreat_timeout').value
        self.resume_clear = self.get_parameter('resume_clear_dist').value
        self.livelock_max = self.get_parameter('livelock_max_n').value
        self.hist_window = self.get_parameter('hist_window').value

        # ---- 로봇 목록 (id = 등장 순서, 1-based) ----
        self.robots = {}    # name -> RobotState
        self.order = []     # names in id order
        for i, spec in enumerate(self.get_parameter('robots').value):
            name = spec.split(':')[0]
            self.order.append(name)
            self.robots[name] = RobotState(name, i + 1)
        self.n = len(self.order)

        # ---- 우선순위 ----
        # 기본: 낮은 id = 높은우선 → 값 = (n - index). 동률 시 낮은 id tiebreak(higher()).
        self.default_prio = {name: (self.n - idx)
                             for idx, name in enumerate(self.order)}
        self.injected_prio = {}   # name -> int (/fleet/priorities 수신 시)
        self.pairs = []           # 현재 탐지된 교착쌍 [{'stuck':i,'by':j}] (관측/게이트용)

        # ---- QoS ----
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        # ---- TF (로봇간 비교용 공통 map 프레임) ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- 구독/발행 ----
        self.goal_pubs = {}
        for name in self.order:
            self.create_subscription(
                Odometry, f'/{name}/odometry/filtered',
                lambda m, n=name: self._on_odom(n, m), 10)
            self.create_subscription(
                PoseStamped, f'/{name}/goal_pose',
                lambda m, n=name: self._on_goal(n, m), 10)
            self.create_subscription(
                String, f'/{name}/dwa/status',
                lambda m, n=name: self._on_status(n, m), 10)
            # 액추에이터: HOLD/RETREAT/RESUMING 일 때만 발행(🔴-3)
            self.goal_pubs[name] = self.create_publisher(
                PoseStamped, f'/{name}/goal_pose', 10)

        self.create_subscription(Int32MultiArray, '/fleet/priorities',
                                 self._on_priorities, latched)
        self.status_pub = self.create_publisher(
            String, '/fleet/deadlock_status', 10)

        # ---- 타이머 ----
        cr = self.get_parameter('control_rate').value
        sr = self.get_parameter('status_rate').value
        self.create_timer(1.0 / cr, self._control_tick)
        self.create_timer(1.0 / sr, self._publish_status)

        self.get_logger().info(
            f'deadlock_manager up — robots={self.order} '
            f'default_prio={self.default_prio} '
            f'(T={self.t_stuck} R={self.r_prox} D={self.d_retreat}) '
            f'[🔴-1+2+3: 우선순위·탐지·해소 활성]')

    # ====== 콜백 ======
    def _on_odom(self, name, msg):
        v = msg.twist.twist.linear
        self.robots[name].lin_speed = math.hypot(v.x, v.y)

    def _on_goal(self, name, msg):
        rs = self.robots[name]
        g = (msg.pose.position.x, msg.pose.position.y,
             yaw_from_quat(msg.pose.orientation.z, msg.pose.orientation.w))
        # NORMAL 일 때 들어온 goal 만 mission goal 로 (매니저 자기 override 와 구분)
        if rs.fsm == FSM.NORMAL:
            if rs.mission_goal is None or self._dist(rs.mission_goal, g) > 0.10:
                rs.mission_goal = g
                rs.reached = False
                self.get_logger().info(
                    f'{name}: mission goal = ({g[0]:.2f},{g[1]:.2f})')

    def _on_status(self, name, msg):
        self.robots[name].dwa_status = msg.data

    def _on_priorities(self, msg):
        data = list(msg.data)
        upd = {}
        for idx, name in enumerate(self.order):
            if idx < len(data):
                upd[name] = int(data[idx])
        if upd != self.injected_prio:
            self.injected_prio = upd
            self.get_logger().info(f'/fleet/priorities 수신·반영: {upd}')

    # ====== 우선순위 ======
    def prio(self, name):
        return self.injected_prio.get(name, self.default_prio[name])

    def higher(self, a, b):
        """Return True if a outranks b (larger value; lower id on tie)."""
        pa, pb = self.prio(a), self.prio(b)
        if pa != pb:
            return pa > pb
        return self.robots[a].rid < self.robots[b].rid

    # ====== 유틸 ======
    @staticmethod
    def _dist(p, q):
        return math.hypot(p[0] - q[0], p[1] - q[1])

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _update_pose(self, name):
        """TF map->amrN/base_footprint 로 map 프레임 pose 갱신 + 이력 적재."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, f'{name}/{self.base_suffix}',
                rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        t = tf.transform.translation
        r = tf.transform.rotation
        rs = self.robots[name]
        rs.pose = (t.x, t.y, yaw_from_quat(r.z, r.w))
        now = self._now()
        rs.pose_hist.append((now, t.x, t.y, rs.pose[2]))
        cutoff = now - self.hist_window
        while len(rs.pose_hist) > 2 and rs.pose_hist[0][0] < cutoff:
            rs.pose_hist.pop(0)
        if rs.mission_goal is not None:
            rs.reached = self._dist(rs.pose, rs.mission_goal) <= self.goal_tol

    # ====== 탐지 (🔴-2) ======
    def _win_disp(self, rs, win):
        """pose_hist 의 최근 win 초 구간 bounding-box 변위(m). 데이터 부족 시 None."""
        if not rs.pose_hist:
            return None
        t_last = rs.pose_hist[-1][0]
        seg = [(x, y) for (t, x, y, _yaw) in rs.pose_hist if t_last - t <= win]
        span = t_last - rs.pose_hist[0][0]
        if len(seg) < 2 or span < win * 0.8:
            return None     # 윈도를 못 채움(이력 부족)
        xs = [p[0] for p in seg]
        ys = [p[1] for p in seg]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    def _is_stuck(self, rs):
        """Mission goal 보유·미도달인데 t_stuck 동안 변위<eps_move → 정지."""
        if rs.mission_goal is None or rs.reached or rs.pose is None:
            return False
        d = self._win_disp(rs, self.t_stuck)
        return d is not None and d < self.eps_move

    def _blocker(self, name):
        """
        Name 의 진행방향(yaw) 전방 R 내·측방 L_gate 내 가장 가까운 '다른 AMR'. 없으면 None.

        pedestrian/forklift 는 odometry/TF 가 없어 self.robots 에 없음 → 구조적으로 제외.
        """
        rs = self.robots[name]
        if rs.pose is None:
            return None
        x, y, yaw = rs.pose
        c, s = math.cos(yaw), math.sin(yaw)
        best, best_f = None, 1e9
        for other in self.order:
            if other == name:
                continue
            op = self.robots[other].pose
            if op is None:
                continue
            dx, dy = op[0] - x, op[1] - y
            fwd = dx * c + dy * s          # 진행방향 성분
            lat = -dx * s + dy * c         # 측방 성분
            if 0.0 < fwd < self.r_prox and abs(lat) < self.l_gate and fwd < best_f:
                best, best_f = other, fwd
        return best

    def _detect(self):
        """교착쌍 갱신. 🔴-2: 관측만(self.pairs/상태 플래그). FSM 전이·goal 조작 없음."""
        pairs = []
        for name in self.order:
            rs = self.robots[name]
            stuck = self._is_stuck(rs)
            blocker = self._blocker(name) if stuck else None
            if stuck and blocker and (not rs.stuck or rs.blocked_by != blocker):
                self.get_logger().info(
                    f'교착 탐지: {name} stuck · 전방 AMR {blocker} '
                    f'(prio {name}={self.prio(name)} vs {blocker}={self.prio(blocker)} '
                    f'→ 양보: {name if not self.higher(name, blocker) else blocker})')
            rs.stuck = stuck
            rs.blocked_by = blocker
            if stuck and blocker:
                pairs.append({'stuck': name, 'by': blocker})
        self.pairs = pairs

    # ====== 해소 FSM (🔴-3) ======
    def _pub_goal(self, name, x, y, yaw, force=False):
        """/amrN/goal_pose 발행(0.5s throttle). HOLD/RETREAT/RESUMING 에서만 호출."""
        rs = self.robots[name]
        now = self._now()
        if not force and (now - rs.last_pub_t) < 0.5:
            return
        rs.last_pub_t = now
        m = PoseStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pubs[name].publish(m)

    def _retreat_point(self, rs):
        """지나온 pose 이력서 arc-length D 뒤 점(이미 통과 → clear). 이력 부족 시 heading 반대 기하."""
        hist = rs.pose_hist
        acc = 0.0
        for k in range(len(hist) - 1, 0, -1):
            _, x, y, _yaw = hist[k]
            _, px, py, _pyaw = hist[k - 1]
            acc += math.hypot(x - px, y - py)
            if acc >= self.d_retreat:
                return (px, py)
        x, y, yaw = rs.pose
        return (x - self.d_retreat * math.cos(yaw),
                y - self.d_retreat * math.sin(yaw))

    def _is_loser(self, rs):
        """Stuck + 전방 AMR + 그 AMR 보다 저우선 → 양보 대상."""
        return bool(rs.stuck and rs.blocked_by
                    and not self.higher(rs.name, rs.blocked_by))

    def _enter_hold(self, rs):
        rs.fsm = FSM.HOLD
        rs.hold_goal = rs.pose
        rs.hold_since = self._now()
        rs.last_pub_t = 0.0
        self.get_logger().info(f'{rs.name}: NORMAL→HOLD (양보, {rs.blocked_by} 우선)')

    def _enter_retreat(self, rs):
        rs.flip_count += 1
        if rs.flip_count > self.livelock_max:
            rs.livelock = True
            rs.fsm = FSM.HOLD
            self.get_logger().warn(
                f'{rs.name}: LIVELOCK (flip>{self.livelock_max}) — HOLD 동결')
            return
        rs.fsm = FSM.RETREAT
        rs.retreat_goal = self._retreat_point(rs)
        rs.retreat_since = self._now()
        rs.last_pub_t = 0.0
        self.get_logger().info(
            f'{rs.name}: HOLD→RETREAT map({rs.retreat_goal[0]:.1f},'
            f'{rs.retreat_goal[1]:.1f}) flip={rs.flip_count}')

    def _enter_resuming(self, rs):
        rs.fsm = FSM.RESUMING
        rs.resume_since = self._now()
        rs.last_pub_t = 0.0
        self.get_logger().info(f'{rs.name}: →RESUMING (mission goal 복원)')

    def _to_normal(self, rs):
        rs.fsm = FSM.NORMAL
        rs.hold_since = rs.retreat_since = rs.resume_since = None

    def _resolve(self):
        """패자만 FSM 구동(HOLD→RETREAT→RESUMING). 승자는 무개입(자동 재진행)."""
        for name in self.order:
            rs = self.robots[name]
            if rs.livelock:
                if rs.hold_goal:
                    self._pub_goal(name, rs.hold_goal[0], rs.hold_goal[1], rs.hold_goal[2])
                continue

            if rs.fsm == FSM.NORMAL:
                if self._is_loser(rs):
                    self._enter_hold(rs)

            elif rs.fsm == FSM.HOLD:
                if rs.blocked_by is None:                       # 막힘 해소
                    self._enter_resuming(rs)
                elif self._now() - rs.hold_since > self.x_hold:  # 대기 초과 → 후진
                    self._enter_retreat(rs)
                else:
                    g = rs.hold_goal
                    self._pub_goal(name, g[0], g[1], g[2])

            elif rs.fsm == FSM.RETREAT:
                g = rs.retreat_goal
                reached = (rs.pose is not None and
                           math.hypot(rs.pose[0] - g[0], rs.pose[1] - g[1]) < self.goal_tol)
                if reached or rs.blocked_by is None or \
                        self._now() - rs.retreat_since > self.retreat_timeout:
                    self._enter_resuming(rs)
                else:
                    self._pub_goal(name, g[0], g[1], rs.pose[2] if rs.pose else 0.0)

            elif rs.fsm == FSM.RESUMING:
                mg = rs.mission_goal
                if mg is None or rs.reached or not rs.stuck:     # 복귀/이동 시작 → NORMAL
                    self._to_normal(rs)
                elif self._now() - rs.resume_since > self.x_hold and self._is_loser(rs):
                    self._enter_hold(rs)                         # 재교착 → 새 사이클
                else:
                    self._pub_goal(name, mg[0], mg[1], mg[2])

    # ====== 컨트롤 루프 ======
    def _control_tick(self):
        for name in self.order:
            self._update_pose(name)
        self._detect()
        self._resolve()

    # ====== 관측 (/fleet/deadlock_status) ======
    def _publish_status(self):
        robots = {}
        for name in self.order:
            rs = self.robots[name]
            robots[name] = {
                'prio': self.prio(name),
                'fsm': rs.fsm.value,
                'stuck': rs.stuck,
                'blocked_by': rs.blocked_by,
                'reached': rs.reached,
                'has_goal': rs.mission_goal is not None,
                'livelock': rs.livelock,
                'flip': rs.flip_count,
                'pose': [round(c, 3) for c in rs.pose] if rs.pose else None,
            }
        msg = String()
        msg.data = json.dumps({
            't': round(self._now(), 2),
            'prio_source': 'injected' if self.injected_prio else 'default',
            'robots': robots,
            'pairs': self.pairs,   # [{'stuck':i,'by':j}] — 🔴-2 탐지쌍
        })
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = DeadlockManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
