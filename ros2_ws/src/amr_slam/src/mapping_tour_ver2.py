#!/usr/bin/env python3
"""
mapping_tour.py — Opticore AMR 창고 맵핑 자율주행 노드 v2
- ground_truth odom으로 실제 위치 추적
- LiDAR 전방 장애물 감지 → 정지 후 회피
- 창고 크기: 40m x 30m / 스폰: (3.0, 15.0)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


# ─────────────────────────────────────────────
#  웨이포인트 (x, y)
# ─────────────────────────────────────────────
WAYPOINTS = [
    # 외곽 남쪽
    (3.0,  2.5),
    (37.0, 2.5),
    # 외곽 동쪽
    (37.0, 27.5),
    # 외곽 북쪽
    (3.0,  27.5),
    # Row4 복도 (y=21.5)
    (3.0,  21.5),
    (37.0, 21.5),
    # Row3 복도 (y=16.5)
    (37.0, 16.5),
    (3.0,  16.5),
    # Row2 복도 (y=11.5)
    (3.0,  11.5),
    (37.0, 11.5),
    # Row1 복도 (y=6.5)
    (37.0, 6.5),
    (3.0,  6.5),
    # 귀환
    (3.0,  15.0),
]

# ─────────────────────────────────────────────
#  파라미터
# ─────────────────────────────────────────────
LINEAR_SPEED    = 0.25   # m/s
ANGULAR_SPEED   = 0.50   # rad/s
GOAL_TOLERANCE  = 0.35   # m
ANGLE_TOLERANCE = 0.05   # rad

# LiDAR 장애물 감지
OBSTACLE_STOP_DIST = 0.55   # m — 이 거리 이내면 정지
OBSTACLE_SLOW_DIST = 1.0    # m — 이 거리 이내면 감속
FRONT_ANGLE_DEG    = 30.0   # 전방 ±30도 범위만 감지


class MappingTour(Node):
    def __init__(self):
        super().__init__('mapping_tour')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        # ground_truth로 실제 위치 추적
        self.gt_sub   = self.create_subscription(
            Odometry, '/ground_truth', self.gt_callback, 10)

        # LiDAR 장애물 감지
        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar', self.scan_callback, 10)

        self.x   = 3.0
        self.y   = 15.0
        self.yaw = 0.0

        self.obstacle_stop = False
        self.obstacle_slow = False

        self.wp_index = 0
        self.state    = 'ROTATE'
        self.done     = False

        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info('=== Mapping Tour v2 시작 ===')
        self.get_logger().info(f'총 웨이포인트: {len(WAYPOINTS)}개')

    # ── Ground Truth 콜백 ────────────────────
    def gt_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    # ── LiDAR 콜백 ──────────────────────────
    def scan_callback(self, msg: LaserScan):
        """전방 ±FRONT_ANGLE_DEG 범위의 최소 거리 확인"""
        front_rad = math.radians(FRONT_ANGLE_DEG)
        min_dist  = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= front_rad:
                min_dist = min(min_dist, r)

        self.obstacle_stop = min_dist < OBSTACLE_STOP_DIST
        self.obstacle_slow = min_dist < OBSTACLE_SLOW_DIST

        if self.obstacle_stop:
            self.get_logger().warn(
                f'⚠ 전방 장애물: {min_dist:.2f}m — 정지',
                throttle_duration_sec=1.0)

    # ── 각도 정규화 ──────────────────────────
    @staticmethod
    def normalize_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── 제어 루프 ────────────────────────────
    def control_loop(self):
        if self.done:
            return

        if self.wp_index >= len(WAYPOINTS):
            self.stop()
            self.done = True
            self.get_logger().info('=== 맵핑 투어 완료! ===')
            self.get_logger().info(
                '맵 저장 명령어:\n'
                'ros2 run nav2_map_server map_saver_cli '
                '-f /workspace/github/opticore-amr/ros2_ws/src/'
                'amr_bringup/maps/warehouse_map'
            )
            return

        tx, ty    = WAYPOINTS[self.wp_index]
        dx        = tx - self.x
        dy        = ty - self.y
        dist      = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        angle_err  = self.normalize_angle(target_yaw - self.yaw)

        # 웨이포인트 도달
        if dist < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info(
                f'[{self.wp_index+1}/{len(WAYPOINTS)}] ✓ ({tx:.1f}, {ty:.1f}) 도달'
            )
            self.wp_index += 1
            self.state = 'ROTATE'
            return

        twist = Twist()

        # ROTATE: 목표 방향으로 제자리 회전
        if self.state == 'ROTATE':
            if abs(angle_err) > ANGLE_TOLERANCE:
                twist.angular.z = ANGULAR_SPEED * (1.0 if angle_err > 0 else -1.0)
            else:
                self.state = 'DRIVE'
            self.cmd_pub.publish(twist)
            return

        # DRIVE: 직진 + P 제어 각도 보정
        if self.state == 'DRIVE':

            # 방향 틀어짐 → 재정렬
            if abs(angle_err) > 0.20:
                self.state = 'ROTATE'
                self.cmd_pub.publish(twist)
                return

            # 전방 장애물 정지
            if self.obstacle_stop:
                self.stop()
                return

            # 감속 or 정속
            speed = LINEAR_SPEED * 0.5 if self.obstacle_slow else LINEAR_SPEED

            twist.linear.x  = speed
            twist.angular.z = 1.2 * angle_err
            self.cmd_pub.publish(twist)

    # ── 정지 ────────────────────────────────
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
