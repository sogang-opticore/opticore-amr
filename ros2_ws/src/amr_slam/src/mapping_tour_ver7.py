#!/usr/bin/env python3
"""
mapping_tour_ver7.py — Opticore AMR 창고 맵핑 자율주행 노드 v7
- 비상 회피: 후진 후 attempts * 30도 오프셋으로 전진 (벽 우회)
- 스킵된 웨이포인트 재시도 (전체 투어 완료 후)
- 속도 향상: 0.25 → 0.35 m/s
- /odom 기반 위치 추적
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


# ─────────────────────────────────────────────
#  안전 웨이포인트 (벽 1.5m / 선반 1.2m / 기둥 1.0m 이격)
# ─────────────────────────────────────────────
WAYPOINTS = [
    # ① 남쪽 외곽 (Row1 아래 y=6.0)
    (4.0,  6.0),
    (36.0, 6.0),
    # ② 동쪽 외곽 북상
    (36.0, 27.0),
    # ③ 북쪽 외곽
    (4.0,  27.0),
    # ④ Row4 북쪽 복도 (y=27.0) 재스캔
    (36.0, 27.0),
    # ⑤ Row3~4 복도 중앙 (y=21.5)
    (36.0, 21.5),
    (4.0,  21.5),
    # ⑥ Row2~3 복도 중앙 (y=16.5)
    (4.0,  16.5),
    (36.0, 16.5),
    # ⑦ Row1~2 복도 중앙 (y=11.5)
    (36.0, 11.5),
    (4.0,  11.5),
    # ⑧ Row1 남쪽 복도 (y=6.0) 반대 방향 재스캔
    (4.0,  6.0),
    (36.0, 6.0),
    # ⑨ 귀환
    (36.0, 15.0),
    (4.0,  15.0),
    (3.0,  15.0),
]

# ─────────────────────────────────────────────
#  주행 파라미터
# ─────────────────────────────────────────────
LINEAR_SPEED    = 0.35   # m/s (0.25 → 0.35)
ANGULAR_SPEED   = 0.60   # rad/s
GOAL_TOLERANCE  = 0.40   # m
ANGLE_TOLERANCE = 0.05   # rad

# LiDAR 감지
FRONT_ANGLE_DEG    = 30.0
OBSTACLE_STOP_DIST = 0.50  # m
OBSTACLE_SLOW_DIST = 0.90  # m

# 비상 회피
BACKUP_SPEED        = -0.12   # m/s
BACKUP_DURATION     = 1.5     # 초
AVOID_ANGLE_STEP    = 30.0    # 도 — 회피 시도마다 누적 오프셋
MAX_AVOID_ATTEMPTS  = 6       # 초과 시 스킵 후 재시도 큐에 추가


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

        # 상태머신
        self.state          = 'ROTATE'
        self.wp_index       = 0
        self.done           = False

        # 회피
        self.avoid_attempts   = 0
        self.avoid_timer      = 0.0
        self.avoid_yaw_target = 0.0  # 후진 후 전진할 방향

        # 스킵된 웨이포인트 재시도 큐
        self.skipped_wps     = []   # [(x, y), ...]
        self.retry_mode      = False

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('=== Mapping Tour v7 시작 ===')
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
        if self.retry_mode and self.skipped_wps:
            tx, ty = self.skipped_wps[0]
            self.get_logger().info(
                f'→ [재시도] 스킵 웨이포인트: ({tx:.1f}, {ty:.1f}) '
                f'(남은 재시도: {len(self.skipped_wps)}개)'
            )
        elif self.wp_index < len(WAYPOINTS):
            tx, ty = WAYPOINTS[self.wp_index]
            self.get_logger().info(
                f'→ 다음 목표 [{self.wp_index+1}/{len(WAYPOINTS)}]: '
                f'({tx:.1f}, {ty:.1f})'
            )

    def _get_current_target(self):
        """현재 목표 웨이포인트 반환 (재시도 모드 포함)"""
        if self.retry_mode and self.skipped_wps:
            return self.skipped_wps[0]
        if self.wp_index < len(WAYPOINTS):
            return WAYPOINTS[self.wp_index]
        return None

    def _advance_waypoint(self):
        """웨이포인트 도달 시 다음으로 전진"""
        if self.retry_mode and self.skipped_wps:
            done_wp = self.skipped_wps.pop(0)
            self.get_logger().info(f'[재시도 완료] ({done_wp[0]:.1f}, {done_wp[1]:.1f})')
            if not self.skipped_wps:
                self.get_logger().info('=== 모든 재시도 완료 ===')
                self.done = True
        else:
            self.wp_index += 1
            # 메인 투어 완료 → 재시도 모드로 전환
            if self.wp_index >= len(WAYPOINTS):
                if self.skipped_wps:
                    self.retry_mode = True
                    self.get_logger().info(
                        f'=== 메인 투어 완료. 스킵 웨이포인트 {len(self.skipped_wps)}개 재시도 ==='
                    )
                else:
                    self.done = True

        self.avoid_attempts = 0
        self.state = 'ROTATE'
        self._log_next_wp()

    def _skip_waypoint(self, tx, ty):
        """웨이포인트 스킵 — 재시도 큐에 추가"""
        if self.retry_mode:
            # 재시도도 실패 → 포기
            failed = self.skipped_wps.pop(0)
            self.get_logger().warn(
                f'[재시도 실패 — 포기] ({failed[0]:.1f}, {failed[1]:.1f})'
            )
            if not self.skipped_wps:
                self.done = True
        else:
            self.get_logger().warn(
                f'웨이포인트 ({tx:.1f},{ty:.1f}) {MAX_AVOID_ATTEMPTS}회 초과 '
                f'→ 스킵 (재시도 큐 추가)'
            )
            self.skipped_wps.append((tx, ty))
            self.wp_index += 1
            if self.wp_index >= len(WAYPOINTS):
                if self.skipped_wps:
                    self.retry_mode = True
                    self.get_logger().info(
                        f'=== 메인 투어 완료. 스킵 웨이포인트 {len(self.skipped_wps)}개 재시도 ==='
                    )
                else:
                    self.done = True

        self.avoid_attempts = 0
        self.state = 'ROTATE'
        self._log_next_wp()

    # ── 제어 루프 (20Hz) ──────────────────────
    def control_loop(self):
        if self.done:
            return

        target = self._get_current_target()
        if target is None:
            self.stop()
            self.done = True
            self.get_logger().info('=== 맵핑 투어 완료! ===')
            self.get_logger().info(
                '맵 저장:\nros2 run nav2_map_server map_saver_cli '
                '-f /workspace/github/opticore-amr/ros2_ws/src/'
                'amr_bringup/maps/warehouse_map'
            )
            return

        tx, ty     = target
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
                # 후진 완료 → 웨이포인트 방향 + attempts * 30도 오프셋
                offset = math.radians(
                    AVOID_ANGLE_STEP * self.avoid_attempts
                )
                # 홀수 시도는 왼쪽, 짝수는 오른쪽으로 교대
                sign = 1.0 if self.avoid_attempts % 2 == 1 else -1.0
                self.avoid_yaw_target = self.normalize_angle(
                    target_yaw + sign * offset
                )
                self.get_logger().info(
                    f'후진 완료 → 오프셋 {sign * math.degrees(offset):.0f}도 '
                    f'방향으로 전진 시도 #{self.avoid_attempts}'
                )
                self.state = 'AVOID_ROTATE'
            return

        # ════════════════════════════════════════
        #  AVOID_ROTATE: 오프셋 방향으로 회전
        # ════════════════════════════════════════
        if self.state == 'AVOID_ROTATE':
            yaw_err = self.normalize_angle(self.avoid_yaw_target - self.yaw)
            if abs(yaw_err) > 0.08:
                twist.angular.z = ANGULAR_SPEED * (1.0 if yaw_err > 0 else -1.0)
                self.cmd_pub.publish(twist)
            else:
                # 회전 완료 → 오프셋 방향으로 전진 (DRIVE_OFFSET)
                self.state = 'DRIVE_OFFSET'
                self.get_logger().info('오프셋 방향 전진 시작')
            return

        # ════════════════════════════════════════
        #  DRIVE_OFFSET: 오프셋 방향으로 전진
        #  웨이포인트 방향과 각도 차이가 충분히 줄면 ROTATE로 복귀
        # ════════════════════════════════════════
        if self.state == 'DRIVE_OFFSET':

            # 전방 또 막힘 → 다시 후진
            if self.obstacle_stop:
                self.stop()
                self.avoid_attempts += 1
                if self.avoid_attempts > MAX_AVOID_ATTEMPTS:
                    self._skip_waypoint(tx, ty)
                    return
                self.avoid_timer = BACKUP_DURATION
                self.state = 'AVOID_BACKUP'
                self.get_logger().warn(
                    f'오프셋 전진 중 재차 막힘 → 후진 #{self.avoid_attempts}'
                )
                return

            # 웨이포인트 방향과의 각도 차이가 15도 이하로 줄면
            # 장애물을 충분히 우회한 것으로 판단 → 정상 ROTATE로 복귀
            wp_angle_diff = abs(
                self.normalize_angle(target_yaw - self.yaw)
            )
            if wp_angle_diff < math.radians(15.0):
                self.state = 'ROTATE'
                return

            # 오프셋 방향 유지하며 전진
            offset_err = self.normalize_angle(self.avoid_yaw_target - self.yaw)
            twist.linear.x  = LINEAR_SPEED * 0.8
            twist.angular.z = 1.2 * offset_err
            self.cmd_pub.publish(twist)
            return

        # ── 웨이포인트 도달 판정 ──
        if dist < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info(
                f'✓ ({tx:.1f}, {ty:.1f}) 도달'
            )
            self._advance_waypoint()
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

            if self.obstacle_stop:
                self.stop()
                self.avoid_attempts += 1

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
