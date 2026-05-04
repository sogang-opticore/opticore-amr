#!/usr/bin/env python3
"""
mapping_tour_ver6.py — Opticore AMR 창고 맵핑 자율주행 노드 v6
- 웨이포인트를 벽/선반/기둥에서 안전 거리 이상 떨어지게 재설계
- 회피 로직 단순화 (안전 경로이므로 비상용으로만 유지)
- /odom 기반 위치 추적
- 창고: 40x30m / 선반 4열(y=9,14,19,24) / 스폰: (3.0, 15.0)

안전 여유:
  벽:   1.5m  → x: 1.5~38.5 / y: 1.5~28.5
  선반: 1.2m  → 복도 중앙 통과
  기둥: 1.0m  → (5,7.5) (35,7.5) (5,25.5) (35,25.5)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


# ─────────────────────────────────────────────
#  안전 웨이포인트 설계
#
#  경로 개요 (top-view):
#
#  Y=28.5 ──────────────────────────────── 북벽 안전선
#         [Row4 북쪽 복도 y=27.0]  →←
#  Y=24   ████████████████████████  Row4 선반
#         [Row3~4 복도 y=21.5]      →←
#  Y=19   ████████████████████████  Row3 선반
#         [Row2~3 복도 y=16.5]      →←
#  Y=14   ████████████████████████  Row2 선반
#         [Row1~2 복도 y=11.5]      →←
#  Y=9    ████████████████████████  Row1 선반
#         [Row1 남쪽 복도 y=6.0]    →←
#  Y=1.5  ──────────────────────────────── 남벽 안전선
#         x=4.0(서)              x=36.0(동)
# ─────────────────────────────────────────────
WAYPOINTS = [
    # ① 남쪽 외곽 이동 (Row1 아래)
    (4.0,  6.0),    # 남서 진입점 (기둥(5,7.5)에서 충분히 이격)
    (36.0, 6.0),    # 남동 진입점 (기둥(35,7.5)에서 충분히 이격)

    # ② 동쪽 외곽 → 북쪽으로
    (36.0, 27.0),   # 북동 진입점 (기둥(35,25.5)에서 충분히 이격)

    # ③ 북쪽 외곽
    (4.0,  27.0),   # 북서 진입점 (기둥(5,25.5)에서 충분히 이격)

    # ④ Row4 북쪽 복도 (y=24 선반 북쪽: 24+1.0+1.2=26.2 → y=27.0 유지)
    #    서→동 스캔
    (4.0,  27.0),   # 이미 있음, 동쪽으로
    (36.0, 27.0),   # Row4 북쪽 끝

    # ⑤ Row3~4 사이 복도 중앙 y=21.5
    (36.0, 21.5),
    (4.0,  21.5),

    # ⑥ Row2~3 사이 복도 중앙 y=16.5
    (4.0,  16.5),
    (36.0, 16.5),

    # ⑦ Row1~2 사이 복도 중앙 y=11.5
    (36.0, 11.5),
    (4.0,  11.5),

    # ⑧ Row1 남쪽 복도 (이미 ①에서 스캔했지만 반대 방향으로 한 번 더)
    (4.0,  6.0),
    (36.0, 6.0),

    # ⑨ 귀환 (스폰 위치)
    (36.0, 15.0),   # 동쪽 중앙 경유 (언로딩독 앞 y=15)
    (4.0,  15.0),   # 서쪽 중앙
    (3.0,  15.0),   # 스폰 위치
]

# ─────────────────────────────────────────────
#  주행 파라미터
# ─────────────────────────────────────────────
LINEAR_SPEED    = 0.25   # m/s
ANGULAR_SPEED   = 0.50   # rad/s
GOAL_TOLERANCE  = 0.40   # m (안전 경로라 조금 여유있게)
ANGLE_TOLERANCE = 0.05   # rad

# LiDAR 감지
FRONT_ANGLE_DEG    = 30.0
OBSTACLE_STOP_DIST = 0.50  # m — 비상 정지
OBSTACLE_SLOW_DIST = 0.90  # m — 감속

# 비상 회피 (안전 경로에서는 거의 안 쓰임)
BACKUP_SPEED    = -0.12
BACKUP_DURATION = 1.5
MAX_AVOID_ATTEMPTS = 3    # 3회 초과 시 스킵


class MappingTour(Node):
    def __init__(self):
        super().__init__('mapping_tour')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar', self.scan_callback, 10)

        self.x   = 3.0
        self.y   = 15.0
        self.yaw = 0.0

        self.dist_front    = float('inf')
        self.obstacle_stop = False
        self.obstacle_slow = False

        self.state          = 'ROTATE'
        self.wp_index       = 0
        self.done           = False
        self.avoid_attempts = 0
        self.avoid_timer    = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('=== Mapping Tour v6 시작 ===')
        self.get_logger().info(f'총 웨이포인트: {len(WAYPOINTS)}개')
        self._log_next_wp()

    # ── Odometry 콜백 ────────────────────────
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    # ── LiDAR 콜백 ───────────────────────────
    def scan_callback(self, msg: LaserScan):
        front_rad = math.radians(FRONT_ANGLE_DEG)
        d_front   = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.01:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= front_rad:
                d_front = min(d_front, r)

        self.dist_front    = d_front
        self.obstacle_stop = d_front < OBSTACLE_STOP_DIST
        self.obstacle_slow = d_front < OBSTACLE_SLOW_DIST

    # ── 각도 정규화 ───────────────────────────
    @staticmethod
    def normalize_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _log_next_wp(self):
        if self.wp_index < len(WAYPOINTS):
            tx, ty = WAYPOINTS[self.wp_index]
            self.get_logger().info(
                f'→ 다음 목표 [{self.wp_index+1}/{len(WAYPOINTS)}]: '
                f'({tx:.1f}, {ty:.1f})'
            )

    # ── 제어 루프 (20Hz) ──────────────────────
    def control_loop(self):
        if self.done:
            return

        if self.wp_index >= len(WAYPOINTS):
            self.stop()
            self.done = True
            self.get_logger().info('=== 맵핑 투어 완료! ===')
            self.get_logger().info(
                '맵 저장:\nros2 run nav2_map_server map_saver_cli '
                '-f /workspace/github/opticore-amr/ros2_ws/src/'
                'amr_bringup/maps/warehouse_map'
            )
            return

        tx, ty     = WAYPOINTS[self.wp_index]
        dx         = tx - self.x
        dy         = ty - self.y
        dist       = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        angle_err  = self.normalize_angle(target_yaw - self.yaw)
        twist      = Twist()

        # ════════════════════════════════════════
        #  비상 후진 (안전 경로에서 만약을 위해)
        # ════════════════════════════════════════
        if self.state == 'AVOID_BACKUP':
            self.avoid_timer -= 0.05
            if self.avoid_timer > 0:
                twist.linear.x = BACKUP_SPEED
                self.cmd_pub.publish(twist)
            else:
                self.state = 'ROTATE'
                self.get_logger().info('후진 완료 → 재정렬')
            return

        # ── 웨이포인트 도달 판정 ──
        if dist < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info(
                f'[{self.wp_index+1}/{len(WAYPOINTS)}] ✓ ({tx:.1f}, {ty:.1f}) 도달'
            )
            self.wp_index      += 1
            self.avoid_attempts = 0
            self.state = 'ROTATE'
            self._log_next_wp()
            return

        # ════════════════════════════════════════
        #  ROTATE: 목표 방향 정렬
        # ════════════════════════════════════════
        if self.state == 'ROTATE':
            if abs(angle_err) > ANGLE_TOLERANCE:
                twist.angular.z = ANGULAR_SPEED * (1.0 if angle_err > 0 else -1.0)
            else:
                self.state = 'DRIVE'
            self.cmd_pub.publish(twist)
            return

        # ════════════════════════════════════════
        #  DRIVE: 직진
        # ════════════════════════════════════════
        if self.state == 'DRIVE':

            # 비상 장애물 감지
            if self.obstacle_stop:
                self.stop()
                self.avoid_attempts += 1

                if self.avoid_attempts > MAX_AVOID_ATTEMPTS:
                    self.get_logger().warn(
                        f'웨이포인트 ({tx:.1f},{ty:.1f}) 3회 초과 → 스킵'
                    )
                    self.wp_index      += 1
                    self.avoid_attempts = 0
                    self.state = 'ROTATE'
                    self._log_next_wp()
                    return

                self.avoid_timer = BACKUP_DURATION
                self.state = 'AVOID_BACKUP'
                self.get_logger().warn(
                    f'⚠ 비상: 전방 {self.dist_front:.2f}m '
                    f'— 후진 #{self.avoid_attempts}'
                )
                return

            # 방향 틀어짐 → 재정렬
            if abs(angle_err) > 0.20:
                self.state = 'ROTATE'
                self.cmd_pub.publish(twist)
                return

            speed = LINEAR_SPEED * 0.5 if self.obstacle_slow else LINEAR_SPEED
            twist.linear.x  = speed
            twist.angular.z = 1.2 * angle_err
            self.cmd_pub.publish(twist)

    # ── 정지 ─────────────────────────────────
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
