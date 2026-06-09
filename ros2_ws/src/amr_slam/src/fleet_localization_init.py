#!/usr/bin/env python3
"""
F-1 fleet localization 초기화 (🔴-2) — 원샷 launch 로 수동 pub 0.

두 가지를 한 노드가 책임진다:
  (1) per-robot map_server + amcl 를 active 로 보장. lifecycle_manager(autostart)가
      CPU 경합으로 change_state 응답 타임아웃 → amcl 'unconfigured' 로 남는 문제
      (특히 마지막 스폰 amr4)를 직접 lifecycle 전이(configure→activate, 재시도)로 self-heal.
  (2) amcl active 후 per-robot initialpose 발행:
      map 초기위치 = (wx - map_origin_x, wy - map_origin_y, yaw=0).  spawn 좌표 단일 소스.

robots 파라미터: ["amr1:3.0:13.0", ...]  (name:world_x:world_y)
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, HistoryPolicy,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State

T_CONFIGURE = Transition.TRANSITION_CONFIGURE   # 1
T_ACTIVATE = Transition.TRANSITION_ACTIVATE     # 3
S_UNCONF = State.PRIMARY_STATE_UNCONFIGURED     # 1
S_INACTIVE = State.PRIMARY_STATE_INACTIVE       # 2
S_ACTIVE = State.PRIMARY_STATE_ACTIVE           # 3


class FleetLocInit(Node):
    def __init__(self):
        super().__init__('fleet_localization_init')
        self.declare_parameter('robots',
                               ['amr1:3.0:13.0', 'amr2:3.0:17.0',
                                'amr3:3.0:21.0', 'amr4:3.0:25.0'])
        self.declare_parameter('map_origin_x', 3.0)
        self.declare_parameter('map_origin_y', 15.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.robots = []
        for spec in self.get_parameter('robots').value:
            name, wx, wy = spec.split(':')
            self.robots.append((name, float(wx), float(wy)))
        self.ox = self.get_parameter('map_origin_x').value
        self.oy = self.get_parameter('map_origin_y').value
        self.yaw = self.get_parameter('initial_yaw').value
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pose_pubs = {n: self.create_publisher(
            PoseWithCovarianceStamped, f'/{n}/initialpose', qos)
            for n, _, _ in self.robots}

    # ---- lifecycle 헬퍼 ----
    def _call(self, cli, req, timeout):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def get_state(self, node):
        cli = self.create_client(GetState, f'/{node}/get_state')
        r = self._call(cli, GetState.Request(), 5.0)
        self.destroy_client(cli)
        return r.current_state.id if r else None

    def change_state(self, node, tid):
        cli = self.create_client(ChangeState, f'/{node}/change_state')
        req = ChangeState.Request()
        req.transition.id = tid
        r = self._call(cli, req, 20.0)   # map_server 로드가 느릴 수 있어 넉넉히
        self.destroy_client(cli)
        return bool(r and r.success)

    def ensure_active(self, node, attempts=15):
        for _ in range(attempts):
            st = self.get_state(node)
            if st == S_ACTIVE:
                return True
            if st == S_UNCONF:
                self.change_state(node, T_CONFIGURE)
            elif st == S_INACTIVE:
                self.change_state(node, T_ACTIVATE)
            else:
                time.sleep(1.5)   # 전이중/미응답 — 잠시 대기
            time.sleep(1.0)
        return self.get_state(node) == S_ACTIVE

    def pub_initialpose(self, name, wx, wy, repeat=4):
        mx, my = wx - self.ox, wy - self.oy
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.position.x = mx
        m.pose.pose.position.y = my
        m.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.0685
        m.pose.covariance = cov
        for _ in range(repeat):
            m.header.stamp = self.get_clock().now().to_msg()
            self.pose_pubs[name].publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.4)
        return (mx, my)

    def run(self):
        ok_all = True
        for name, wx, wy in self.robots:
            ms_ok = self.ensure_active(f'{name}/map_server')
            amcl_ok = self.ensure_active(f'{name}/amcl')
            if ms_ok and amcl_ok:
                mx, my = self.pub_initialpose(name, wx, wy)
                self.get_logger().info(
                    f'{name}: localization READY (map_server+amcl active) '
                    f'initialpose=map({mx:+.2f},{my:+.2f})')
            else:
                ok_all = False
                self.get_logger().error(
                    f'{name}: FAILED to bring up '
                    f'(map_server active={ms_ok}, amcl active={amcl_ok})')
        self.get_logger().info(
            f'fleet_localization_init DONE — all_ready={ok_all}')


def main():
    rclpy.init()
    node = FleetLocInit()
    try:
        node.run()
        # 유휴 스핀: latched initialpose 유지 + 늦은 구독자 대응
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
