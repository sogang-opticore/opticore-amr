#!/usr/bin/env python3
"""
mapping_tour.py — Opticore AMR 창고 맵핑 자율주행 노드 v4
- /odom 기반 위치 추적
- LiDAR 좌/우 공간 비교 → 넓은 쪽으로 회전
- 후방 감지 시 후진 중단
- 최대 회피 시도 횟수 초과 시 다음 웨이포인트 스킵
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
#  주행 파라미터
# ─────────────────────────────────────────────
LINEAR_SPEED    = 0.25   # m/s
ANGULAR_SPEED   = 0.50   # rad/s
GOAL_TOLERANCE  = 0.35   # m
ANGLE_TOLERANCE = 0.05   # rad

# LiDAR 감지 구역 (도)
FRONT_ANGLE_DEG = 30.0
SIDE_ANGLE_DEG  = 60.0
REAR_ANGLE_DEG  = 30.0   # 후방 180±30도

# 거리 임계값
OBSTACLE_STOP_DIST = 0.55  # m
OBSTACLE_SLOW_DIST = 1.0   # m
REAR_STOP_DIST     = 0.40  # m — 후방 장애물 후진 중단

# 회피 파라미터
BACKUP_SPEED       = -0.12  # m/s
BACKUP_DURATION    = 1.5    # 초
MAX_AVOID_ATTEMPTS = 5      # 초과 시 다음 웨이포인트 스킵


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

        self.dist_front = float('inf')
        self.dist_left  = float('inf')
        self.dist_right = float('inf')
        self.dist_rear  = float('inf')

        self.obstacle_stop = False
        self.obstacle_slow = False

        self.state            = 'ROTATE'
        self.wp_index         = 0
        self.done             = False
        self.avoid_attempts   = 0
        self.avoid_timer      = 0.0
        self.avoid_yaw_target = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('=== Mapping Tour v4 시작 ===')
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
        side_rad  = math.radians(SIDE_ANGLE_DEG)
        rear_rad  = math.radians(REAR_ANGLE_DEG)

        d_front = float('inf')
        d_left  = float('inf')
        d_right = float('inf')
        d_rear  = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.01:
                continue
            angle = msg.angle_min + i * msg.angle_increment

            if abs(angle) <= front_rad:
                d_front = min(d_front, r)
            elif front_rad < angle <= side_rad:
                d_left = min(d_left, r)
            elif -side_rad <= angle < -front_rad:
                d_right = min(d_right, r)

            if abs(angle) >= math.pi - rear_rad:
                d_rear = min(d_rear, r)

        self.dist_front = d_front
        self.dist_left  = d_left
        self.dist_right = d_right
        self.dist_rear  = d_rear

        self.obstacle_stop = d_front < OBSTACLE_STOP_DIST
        self.obstacle_slow = d_front < OBSTACLE_SLOW_DIST

    # ── 각도 정규화 ───────────────────────────
    @staticmethod
    def normalize_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── 회피 방향 결정 (넓은 쪽) ─────────────
    def decide_avoid_direction(self):
        turn_sign = 1.0 if self.dist_left >= self.dist_right else -1.0
        direction = '왼쪽' if turn_sign > 0 else '오른쪽'
        self.get_logger().info(
            f'회피 방향: {direction} '
            f'(좌:{self.dist_left:.2f}m / 우:{self.dist_right:.2f}m)'
        )
        return self.normalize_angle(self.yaw + turn_sign * math.pi / 2)

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

        # ── AVOID_BACKUP: 후진 ───────────────
        if self.state == 'AVOID_BACKUP':
            self.avoid_timer -= 0.05
            if self.dist_rear < REAR_STOP_DIST:
                self.get_logger().warn(
                    f'후방 장애물 {self.dist_rear:.2f}m — 후진 중단, 회전 전환'
                )
                self.avoid_yaw_target = self.decide_avoid_direction()
                self.state = 'AVOID_ROTATE'
                return
            if self.avoid_timer > 0:
                twist.linear.x = BACKUP_SPEED
                self.cmd_pub.publish(twist)
            else:
                self.avoid_yaw_target = self.decide_avoid_direction()
                self.state = 'AVOID_ROTATE'
            return

        # ── AVOID_ROTATE: 회피 회전 ──────────
        if self.state == 'AVOID_ROTATE':
            yaw_err = self.normalize_angle(self.avoid_yaw_target - self.yaw)
            if abs(yaw_err) > 0.08:
                twist.angular.z = ANGULAR_SPEED * (1.0 if yaw_err > 0 else -1.0)
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info(
                    f'회피 완료 ({self.avoid_attempts}/{MAX_AVOID_ATTEMPTS}) → 재시도'
                )
                self.state = 'ROTATE'
            return

        # ── 웨이포인트 도달 ───────────────────
        if dist < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info(
                f'[{self.wp_index+1}/{len(WAYPOINTS)}] ✓ ({tx:.1f}, {ty:.1f}) 도달'
            )
            self.wp_index      += 1
            self.avoid_attempts = 0
            self.state = 'ROTATE'
            return

        # ── ROTATE: 방향 정렬 ─────────────────
        if self.state == 'ROTATE':
            if abs(angle_err) > ANGLE_TOLERANCE:
                twist.angular.z = ANGULAR_SPEED * (1.0 if angle_err > 0 else -1.0)
            else:
                self.state = 'DRIVE'
            self.cmd_pub.publish(twist)
            return

        # ── DRIVE: 직진 ──────────────────────
        if self.state == 'DRIVE':

            if self.obstacle_stop:
                self.stop()
                self.avoid_attempts += 1

                if self.avoid_attempts > MAX_AVOID_ATTEMPTS:
                    self.get_logger().warn(
                        f'웨이포인트 ({tx:.1f},{ty:.1f}) '
                        f'회피 {MAX_AVOID_ATTEMPTS}회 초과 → 스킵'
                    )
                    self.wp_index      += 1
                    self.avoid_attempts = 0
                    self.state = 'ROTATE'
                    return

                self.avoid_timer = BACKUP_DURATION
                self.state = 'AVOID_BACKUP'
                self.get_logger().warn(
                    f'⚠ 전방 {self.dist_front:.2f}m — 회피 #{self.avoid_attempts}'
                )
                return

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
