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
        # 🔴-2/🔴-3 작업용 필드 (현재 미사용 스텁)
        self.stuck = False
        self.blocked_by = None
        self.hold_since = None
        self.flip_count = 0


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
            f'[🔴-1 골격: 탐지/해소 미활성]')

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
        """a 가 b 보다 우선이면 True (값 큼; 동률이면 낮은 id)."""
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

    # ====== 컨트롤 루프 ======
    def _control_tick(self):
        for name in self.order:
            self._update_pose(name)
        # TODO 🔴-2: 교착 탐지 — stuck_i(윈도 변위<eps_move & goal 미도달)
        #            AND blocked_by(i)=j(전방 R 내 다른 AMR) → 양보 후보.
        # TODO 🔴-3: 우선순위 비교 → 저우선 i 의 FSM HOLD→RETREAT→RESUMING goal 발행.
        # 🔴-1 골격: 미활성 — 전부 NORMAL 유지, goal 미조작(F-1 무회귀).

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
                'pose': [round(c, 3) for c in rs.pose] if rs.pose else None,
            }
        msg = String()
        msg.data = json.dumps({
            't': round(self._now(), 2),
            'prio_source': 'injected' if self.injected_prio else 'default',
            'robots': robots,
            'pairs': [],   # 🔴-2 에서 탐지쌍으로 채움
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
