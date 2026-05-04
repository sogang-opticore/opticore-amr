#!/usr/bin/env python3
"""
mapping_tour.py — Opticore AMR 창고 맵핑 자율주행 노드 v3
- /odom 기반 위치 추적
- LiDAR 전방 장애물 감지 → 후진 + 회전 회피
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
LINEAR_SPEED     = 0.25   # m/s
ANGULAR_SPEED    = 0.50   # rad/s
GOAL_TOLERANCE   = 0.35   # m
ANGLE_TOLERANCE  = 0.05   # rad

# LiDAR 장애물 감지
OBSTACLE_STOP_DIST  = 0.55   # m — 정지
OBSTACLE_SLOW_DIST  = 1.0    # m — 감속
FRONT_ANGLE_DEG     = 30.0   # 전방 ±30도

# 회피 파라미터
BACKUP_SPEED     = -0.12  # m/s (후진)
BACKUP_DURATION  = 1.5    # 초
ROTATE_DURATION  = 1.8    # 초 (약 90도)


class MappingTour(Node):
    def __init__(self):
        super().__init__('mapping_tour')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar', self.scan_callback, 10)

        # 위치/자세
        self.x   = 3.0
        self.y   = 15.0
        self.yaw = 0.0

        # 장애물 플래그
        self.obstacle_stop = False
        self.obstacle_slow = False
        self.min_front_dist = float('inf')

        # 상태머신
        # ROTATE → DRIVE → AVOID_BACKUP → AVOID_ROTATE → ROTATE(재시도)
        self.state      = 'ROTATE'
        self.wp_index   = 0
        self.done       = False

        # 회피 타이머
        self.avoid_timer = 0.0
        self.avoid_yaw_target = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info('=== Mapping Tour v3 시작 ===')
        self.get_logger().info(f'총 웨이포인트: {len(WAYPOINTS)}개')

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
        min_dist  = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= front_rad:
                min_dist = min(min_dist, r)

        self.min_front_dist = min_dist
        self.obstacle_stop  = min_dist < OBSTACLE_STOP_DIST
        self.obstacle_slow  = min_dist < OBSTACLE_SLOW_DIST

    # ── 각도 정규화 ───────────────────────────
    @staticmethod
    def normalize_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── 제어 루프 (20Hz) ──────────────────────
    def control_loop(self):
        if self.done:
            return

        # ── 완료 ──
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
        #  AVOID_BACKUP: 후진
        # ════════════════════════════════════════
        if self.state == 'AVOID_BACKUP':
            self.avoid_timer -= 0.05
            if self.avoid_timer > 0:
                twist.linear.x = BACKUP_SPEED
                self.cmd_pub.publish(twist)
            else:
                # 후진 완료 → 회전 시작 (왼쪽 90도)
                self.avoid_yaw_target = self.normalize_angle(self.yaw + math.pi / 2)
                self.avoid_timer = ROTATE_DURATION
                self.state = 'AVOID_ROTATE'
                self.get_logger().info('회피: 후진 완료 → 회전 시작')
            return

        # ════════════════════════════════════════
        #  AVOID_ROTATE: 회전
        # ════════════════════════════════════════
        if self.state == 'AVOID_ROTATE':
            yaw_err = self.normalize_angle(self.avoid_yaw_target - self.yaw)
            if abs(yaw_err) > 0.10:
                twist.angular.z = ANGULAR_SPEED * (1.0 if yaw_err > 0 else -1.0)
                self.cmd_pub.publish(twist)
            else:
                # 회전 완료 → 웨이포인트 재시도
                self.state = 'ROTATE'
                self.get_logger().info('회피 완료 → 웨이포인트 재시도')
            return

        # ── 웨이포인트 도달 판정 ──
        if dist < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info(
                f'[{self.wp_index+1}/{len(WAYPOINTS)}] ✓ ({tx:.1f}, {ty:.1f}) 도달'
            )
            self.wp_index += 1
            self.state = 'ROTATE'
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

            # 전방 장애물 → 회피 시작
            if self.obstacle_stop:
                self.stop()
                self.avoid_timer = BACKUP_DURATION
                self.state = 'AVOID_BACKUP'
                self.get_logger().warn(
                    f'⚠ 장애물 {self.min_front_dist:.2f}m — 회피 시작 (후진)'
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
