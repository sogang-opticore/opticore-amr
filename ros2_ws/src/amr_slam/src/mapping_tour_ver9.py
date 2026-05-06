#!/usr/bin/env python3
"""
mapping_tour_ver9.py — Opticore AMR 창고 맵핑 자율주행 노드 v9
변경사항:
  - 정상 주행 5초 경과 시 avoid_attempts 자동 리셋
  - 이전 충돌 이력이 다음 충돌 회피 각도에 영향 안 줌
  - 나머지 로직 ver8 동일
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


# ─────────────────────────────────────────────
#  안전 웨이포인트
#  서x=5.0 / 동x=35.0 / 남y=2.5 / 북y=28.5
# ─────────────────────────────────────────────
WAYPOINTS = [
    # ① 남쪽 외곽 (y=6.0)
    (5.0,  6.0),
    (35.0, 6.0),
    # ② 동쪽 외곽 북상
    (35.0, 28.5),
    # ③ 북쪽 외곽 서진
    (5.0,  28.5),
    # ④ Row3~4 복도 (y=21.5)
    (5.0,  21.5),
    (35.0, 21.5),
    # ⑤ Row2~3 복도 (y=16.5)
    (35.0, 16.5),
    (5.0,  16.5),
    # ⑥ Row1~2 복도 (y=11.5)
    (5.0,  11.5),
    (35.0, 11.5),
    # ⑦ Row1 남쪽 재스캔 (y=6.0)
    (35.0, 6.0),
    (5.0,  6.0),
    # ⑧ 귀환
    (5.0,  15.0),
    (3.0,  15.0),
]

# ─────────────────────────────────────────────
#  주행 파라미터
# ─────────────────────────────────────────────
LINEAR_SPEED    = 0.35
ANGULAR_SPEED   = 0.60
GOAL_TOLERANCE  = 0.40
ANGLE_TOLERANCE = 0.05

FRONT_ANGLE_DEG    = 45.0
OBSTACLE_STOP_DIST = 0.60
OBSTACLE_SLOW_DIST = 1.20

BACKUP_SPEED       = -0.12
BACKUP_DURATION    = 1.5
AVOID_ANGLE_STEP   = 30.0   # 도
MAX_AVOID_ATTEMPTS = 6

AVOID_RESET_SEC    = 5.0    # 정상 주행 이 시간 이상이면 attempts 리셋


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

        self.state            = 'ROTATE'
        self.wp_index         = 0
        self.done             = False
        self.avoid_attempts   = 0
        self.avoid_timer      = 0.0
        self.avoid_yaw_target = 0.0

        # 마지막 충돌 시각 (나노초 → 초 변환용)
        self.last_avoid_time  = 0.0

        self.skipped_wps  = []
        self.retry_mode   = False

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('=== Mapping Tour v9 시작 ===')
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

    # ── 현재 시각 (초) ────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _log_next_wp(self):
        if self.retry_mode and self.skipped_wps:
            tx, ty = self.skipped_wps[0]
            self.get_logger().info(
                f'→ [재시도] ({tx:.1f}, {ty:.1f}) '
                f'남은 {len(self.skipped_wps)}개'
            )
        elif self.wp_index < len(WAYPOINTS):
            tx, ty = WAYPOINTS[self.wp_index]
            self.get_logger().info(
                f'→ [{self.wp_index+1}/{len(WAYPOINTS)}] '
                f'({tx:.1f}, {ty:.1f})'
            )

    def _get_current_target(self):
        if self.retry_mode and self.skipped_wps:
            return self.skipped_wps[0]
        if self.wp_index < len(WAYPOINTS):
            return WAYPOINTS[self.wp_index]
        return None

    def _advance_waypoint(self):
        if self.retry_mode and self.skipped_wps:
            done_wp = self.skipped_wps.pop(0)
            self.get_logger().info(
                f'[재시도 완료] ({done_wp[0]:.1f}, {done_wp[1]:.1f})'
            )
            if not self.skipped_wps:
                self.done = True
                self._finish()
        else:
            self.wp_index += 1
            if self.wp_index >= len(WAYPOINTS):
                if self.skipped_wps:
                    self.retry_mode = True
                    self.get_logger().info(
                        f'=== 메인 투어 완료. '
                        f'스킵 {len(self.skipped_wps)}개 재시도 ==='
                    )
                else:
                    self.done = True
                    self._finish()
                    return
        self.avoid_attempts = 0
        self.state = 'ROTATE'
        self._log_next_wp()

    def _skip_waypoint(self, tx, ty):
        if self.retry_mode:
            failed = self.skipped_wps.pop(0)
            self.get_logger().warn(
                f'[재시도 실패 포기] ({failed[0]:.1f}, {failed[1]:.1f})'
            )
            if not self.skipped_wps:
                self.done = True
                self._finish()
                return
        else:
            self.get_logger().warn(
                f'[스킵 → 재시도 큐] ({tx:.1f}, {ty:.1f})'
            )
            self.skipped_wps.append((tx, ty))
            self.wp_index += 1
            if self.wp_index >= len(WAYPOINTS):
                if self.skipped_wps:
                    self.retry_mode = True
                    self.get_logger().info(
                        f'=== 메인 투어 완료. '
                        f'스킵 {len(self.skipped_wps)}개 재시도 ==='
                    )
                else:
                    self.done = True
                    self._finish()
                    return
        self.avoid_attempts = 0
        self.state = 'ROTATE'
        self._log_next_wp()

    def _finish(self):
        self.stop()
        self.get_logger().info('=== 맵핑 투어 완료! ===')
        self.get_logger().info(
            '맵 저장:\n'
            'ros2 run nav2_map_server map_saver_cli '
            '-f /workspace/github/opticore-amr/ros2_ws/src/'
            'amr_bringup/maps/warehouse_map'
        )

    # ── 제어 루프 (20Hz) ──────────────────────
    def control_loop(self):
        if self.done:
            return

        target = self._get_current_target()
        if target is None:
            self._finish()
            return

        tx, ty     = target
        dx         = tx - self.x
        dy         = ty - self.y
        dist       = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        angle_err  = self.normalize_angle(target_yaw - self.yaw)
        twist      = Twist()

        # ── AVOID_BACKUP: 후진 ───────────────
        if self.state == 'AVOID_BACKUP':
            self.avoid_timer -= 0.05
            if self.avoid_timer > 0:
                twist.linear.x = BACKUP_SPEED
                self.cmd_pub.publish(twist)
            else:
                offset = math.radians(AVOID_ANGLE_STEP * self.avoid_attempts)
                sign   = 1.0 if self.avoid_attempts % 2 == 1 else -1.0
                self.avoid_yaw_target = self.normalize_angle(
                    target_yaw + sign * offset
                )
                self.get_logger().info(
                    f'후진 완료 → 오프셋 '
                    f'{sign * math.degrees(offset):.0f}도 전진 #{self.avoid_attempts}'
                )
                self.state = 'AVOID_ROTATE'
            return

        # ── AVOID_ROTATE: 오프셋 방향 회전 ──
        if self.state == 'AVOID_ROTATE':
            yaw_err = self.normalize_angle(self.avoid_yaw_target - self.yaw)
            if abs(yaw_err) > 0.08:
                twist.angular.z = ANGULAR_SPEED * (1.0 if yaw_err > 0 else -1.0)
                self.cmd_pub.publish(twist)
            else:
                self.state = 'DRIVE_OFFSET'
            return

        # ── DRIVE_OFFSET: 오프셋 방향 전진 ──
        if self.state == 'DRIVE_OFFSET':
            if self.obstacle_stop:
                self.stop()
                self.avoid_attempts += 1
                self.last_avoid_time = self._now()
                if self.avoid_attempts > MAX_AVOID_ATTEMPTS:
                    self._skip_waypoint(tx, ty)
                    return
                self.avoid_timer = BACKUP_DURATION
                self.state = 'AVOID_BACKUP'
                self.get_logger().warn(
                    f'오프셋 전진 중 재차 막힘 → 후진 #{self.avoid_attempts}'
                )
                return

            wp_angle_diff = abs(self.normalize_angle(target_yaw - self.yaw))
            if wp_angle_diff < math.radians(15.0):
                self.state = 'ROTATE'
                return

            offset_err = self.normalize_angle(self.avoid_yaw_target - self.yaw)
            twist.linear.x  = LINEAR_SPEED * 0.8
            twist.angular.z = 1.2 * offset_err
            self.cmd_pub.publish(twist)
            return

        # ── 웨이포인트 도달 ───────────────────
        if dist < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info(
                f'✓ [{self.wp_index+1}/{len(WAYPOINTS)}] '
                f'({tx:.1f}, {ty:.1f}) 도달 '
                f'(실제: {self.x:.2f}, {self.y:.2f})'
            )
            self._advance_waypoint()
            return

        # ── ROTATE: 목표 방향 정렬 ────────────
        if self.state == 'ROTATE':
            if abs(angle_err) > ANGLE_TOLERANCE:
                twist.angular.z = ANGULAR_SPEED * (1.0 if angle_err > 0 else -1.0)
            else:
                self.state = 'DRIVE'
            self.cmd_pub.publish(twist)
            return

        # ── DRIVE: 직진 ──────────────────────
        if self.state == 'DRIVE':

            # ── 핵심: 정상 주행 5초 경과 시 attempts 리셋 ──
            if (self.avoid_attempts > 0
                    and not self.obstacle_stop
                    and not self.obstacle_slow):
                elapsed = self._now() - self.last_avoid_time
                if elapsed > AVOID_RESET_SEC:
                    self.get_logger().info(
                        f'정상 주행 {elapsed:.1f}초 경과 → '
                        f'회피 카운터 리셋 (was {self.avoid_attempts})'
                    )
                    self.avoid_attempts = 0

            if self.obstacle_stop:
                self.stop()
                self.avoid_attempts += 1
                self.last_avoid_time = self._now()
                if self.avoid_attempts > MAX_AVOID_ATTEMPTS:
                    self._skip_waypoint(tx, ty)
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
