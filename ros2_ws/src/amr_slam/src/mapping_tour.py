#!/usr/bin/env python3
"""
mapping_tour.py — Opticore AMR 창고 맵핑 자율주행 노드
- 창고 크기: 40m x 30m
- 스폰 위치: (3.0, 15.0)
- 선반 4열(y=9, 14, 19, 24) 복도를 모두 스캔하는 경로
- 속도: 0.15 m/s (느리게 → 맵 품질 향상)
- 동적 장애물(person_1, forklift_1) 제거 후 실행 권장
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


# ─────────────────────────────────────────────
#  웨이포인트 정의 (x, y)
#  벽 여유: 1.5m / 선반 복도 중앙 통과
# ─────────────────────────────────────────────
WAYPOINTS = [
    # ① 외곽 남쪽 (Loading → 남동쪽)
    (3.0,  2.5),   # 남서 코너 근처
    (37.0, 2.5),   # 남동 코너 (Charging Station 옆)

    # ② 외곽 동쪽
    (37.0, 27.5),  # 북동 코너

    # ③ 외곽 북쪽
    (3.0,  27.5),  # 북서 코너

    # ④ Row4 남쪽 복도 (y=24 선반 남쪽: y≈21.5)
    (3.0,  21.5),
    (37.0, 21.5),

    # ⑤ Row3 남쪽 복도 (y=19 선반 남쪽: y≈16.5)
    (37.0, 16.5),
    (3.0,  16.5),

    # ⑥ Row2 남쪽 복도 (y=14 선반 남쪽: y≈11.5)
    (3.0,  11.5),
    (37.0, 11.5),

    # ⑦ Row1 남쪽 복도 (y=9 선반 남쪽: y≈6.5)
    (37.0, 6.5),
    (3.0,  6.5),

    # ⑧ 귀환
    (3.0,  15.0),  # 시작 위치
]

# ─────────────────────────────────────────────
#  파라미터
# ─────────────────────────────────────────────
LINEAR_SPEED   = 0.15   # m/s  — 느릴수록 맵 품질 ↑
ANGULAR_SPEED  = 0.35   # rad/s
GOAL_TOLERANCE = 0.30   # m — 웨이포인트 도달 판정 거리
ANGLE_TOLERANCE = 0.05  # rad — 회전 완료 판정


class MappingTour(Node):
    def __init__(self):
        super().__init__('mapping_tour')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.x   = 3.0
        self.y   = 15.0
        self.yaw = 0.0

        self.wp_index = 0
        self.state    = 'ROTATE'   # ROTATE → DRIVE → DONE

        # 제어 루프: 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('=== Mapping Tour 시작 ===')
        self.get_logger().info(f'총 웨이포인트: {len(WAYPOINTS)}개')

    # ── 오도메트리 콜백 ──────────────────────────
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    # ── 각도 정규화 [-π, π] ──────────────────────
    @staticmethod
    def normalize_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── 제어 루프 ────────────────────────────────
    def control_loop(self):
        if self.wp_index >= len(WAYPOINTS):
            self.stop()
            if self.state != 'DONE':
                self.state = 'DONE'
                self.get_logger().info('=== 맵핑 투어 완료! map_saver_cli로 저장하세요 ===')
                self.get_logger().info(
                    'ros2 run nav2_map_server map_saver_cli '
                    '-f /workspace/github/opticore-amr/ros2_ws/src/'
                    'amr_bringup/maps/warehouse_map'
                )
            return

        tx, ty = WAYPOINTS[self.wp_index]
        dx = tx - self.x
        dy = ty - self.y
        dist   = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        angle_err  = self.normalize_angle(target_yaw - self.yaw)

        # ── 웨이포인트 도달 ──
        if dist < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info(
                f'[{self.wp_index+1}/{len(WAYPOINTS)}] 도달: ({tx:.1f}, {ty:.1f})'
            )
            self.wp_index += 1
            self.state = 'ROTATE'
            return

        twist = Twist()

        if self.state == 'ROTATE':
            # 목표 방향으로 제자리 회전
            if abs(angle_err) > ANGLE_TOLERANCE:
                twist.angular.z = ANGULAR_SPEED * (1.0 if angle_err > 0 else -1.0)
            else:
                self.state = 'DRIVE'

        elif self.state == 'DRIVE':
            # 직진 + 경미한 각도 보정
            if abs(angle_err) > 0.15:
                # 방향이 많이 틀어지면 다시 ROTATE
                self.state = 'ROTATE'
            else:
                twist.linear.x  = LINEAR_SPEED
                twist.angular.z = 1.2 * angle_err   # P 제어

        self.cmd_pub.publish(twist)

    # ── 정지 ────────────────────────────────────
    def stop(self):
        self.cmd_pub.publish(Twist())


# ─────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MappingTour()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 — 로봇 정지')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
