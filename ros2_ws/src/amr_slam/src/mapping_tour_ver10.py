#!/usr/bin/env python3
"""
mapping_tour_ver10.py — Opticore AMR 창고 맵핑 자율주행 노드 v10 (v3.1 60×40 월드)

ver9 대비 변경사항:
  [느림 개선]
   - FRONT_ANGLE_DEG 45 → 18  (옆벽이 전방콘에 안 잡히게)
   - OBSTACLE_SLOW_DIST 1.2 → 0.8, slow 계수 0.5 → 0.7
   - LINEAR_SPEED 0.35 → 0.5,  ANGLE_TOLERANCE 0.05 → 0.10
   - DRIVE 중 작은 각오차는 제자리회전 대신 주행하며 조향 (ROTATE 전환 0.20 → 0.55rad)
  [코너 갇힘 개선 = 핵심]
   - 회피 시 '목표방위 ± n·30°' 대신 LiDAR 스캔 전체에서 가장 열린 방향(gap)으로 회전
     → 오목 코너의 local minimum 탈출
   - 측면(좌/우) 섹터 거리 추가 계산
   - stuck 감지: N초간 거의 못 움직이면(전방 미감지여도) 강제 복구
   - BACKUP_DURATION 1.5 → 2.2 (충분히 후진)
  [맵 정합]
   - WAYPOINTS를 v3.1 60×40 (Row A/B/C, 도크 x<10·x>50, pinch 벽 x≈14.5/49.5) 기준 재설계
   - 모든 세로 이동은 rack 칸 사이 'gap'(x=21.5/42.5 등)으로만 → rack 충돌 방지

인터페이스(ver9 동일): pub /cmd_vel(Twist), sub /odom(Odometry), /lidar(LaserScan)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


# ─────────────────────────────────────────────
#  v3.1 60×40 웨이포인트
#  레인(Y): 남4.5 / B-C 12 / A-B 18 / 북22.5 / 도크복도 30
#  서측개방대 x=5, 동측개방대 x=57, rack 사이 gap x=21.5·42.5
#  ⚠ pinch 벽: 좌 B-C 통로 x≈13.5~15.5 / 우 A-B 통로 x≈48.5~50.5
#     → B-C 레인은 x≥21.5에서, A-B 레인은 x≤42.5에서만 주행
# ─────────────────────────────────────────────
WAYPOINTS = [
    (5.0,  15.0),   # ① 시작(서측 개방대, spawn 부근)
    (5.0,   4.5),   # ② 남서 → 남측 레인
    (49.0,  4.5),   # ③ 남측 레인 동진 (남벽·Row C 남면·충전/대기)
    (57.0,  4.5),   # ④ 동측 개방대
    (57.0, 12.0),   # ⑤ 상승 → B-C 레인
    (21.5, 12.0),   # ⑥ B-C 레인 서진 (좌 pinch 앞 gap col2-3에서 정지)
    (21.5, 18.0),   # ⑦ gap 통해 A-B 레인 상승
    (42.5, 18.0),   # ⑧ A-B 레인 동진 (우 pinch 앞 gap col5-6에서 정지)
    (42.5, 22.5),   # ⑨ gap 통해 북측 레인 상승
    (53.0, 22.5),   # ⑩ 북측 레인 동측 끝
    (12.0, 22.5),   # ⑪ 북측 레인 서진 (Row A 북면·기둥)
    (12.0, 30.0),   # ⑫ 메인 복도 상승(서측)
    (49.0, 30.0),   # ⑬ 메인 복도 동진 (도크 전면·북벽)
    (12.0, 30.0),   # ⑭ 메인 복도 복귀
    (5.0,  20.0),   # ⑮ 서측 개방대 하강
    (5.0,  15.0),   # ⑯ spawn 부근 복귀 (loop closure)
]

# ── 주행 파라미터 ──
LINEAR_SPEED    = 0.5
ANGULAR_SPEED   = 0.6
GOAL_TOLERANCE  = 0.40
ANGLE_TOLERANCE = 0.10
DRIVE_STEER_MAX = 0.55      # 이 이하 각오차는 주행하며 조향, 초과 시 제자리회전

# ── 장애물 감지 ──
FRONT_ANGLE_DEG    = 18.0   # 전방콘 ±18°
SIDE_ANGLE_DEG     = 75.0   # 좌/우 섹터 중심
SIDE_HALF_DEG      = 25.0
OBSTACLE_STOP_DIST = 0.60
OBSTACLE_SLOW_DIST = 0.80
SLOW_FACTOR        = 0.7

# ── 회피 / 복구 ──
BACKUP_SPEED       = -0.15
BACKUP_DURATION    = 2.2
GAP_WINDOW_DEG     = 15.0    # gap 탐색 윈도우 반각
MAX_AVOID_ATTEMPTS = 6
AVOID_RESET_SEC    = 5.0
STUCK_DIST         = 0.15    # 이만큼도 못 움직이면
STUCK_SEC          = 4.0     # 이 시간 동안 → stuck


class MappingTour(Node):
    def __init__(self):
        super().__init__('mapping_tour')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar', self.scan_callback, 10)

        # pose (odom 수신 시 갱신; 초기값 = spawn)
        self.x   = 3.0
        self.y   = 15.0
        self.yaw = 0.0

        # scan
        self.dist_front     = float('inf')
        self.dist_left      = float('inf')
        self.dist_right     = float('inf')
        self.obstacle_stop  = False
        self.obstacle_slow  = False
        self.scan_ranges    = []
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0
        self.scan_range_max = 10.0

        # 상태
        self.state            = 'ROTATE'
        self.wp_index         = 0
        self.done             = False
        self.avoid_attempts   = 0
        self.avoid_timer      = 0.0
        self.avoid_yaw_target = 0.0
        self.last_avoid_time  = 0.0

        # stuck 감지
        self.last_prog_x    = self.x
        self.last_prog_y    = self.y
        self.last_prog_time = 0.0

        # 스킵/재시도
        self.skipped_wps = []
        self.retry_mode  = False

        self.timer = self.create_timer(0.05, self.control_loop)   # 20Hz
        self.get_logger().info(f'mapping_tour_ver10 시작 — 웨이포인트 {len(WAYPOINTS)}개')
        self._log_next_wp()

    # ── odom ──
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    # ── LiDAR ──
    def scan_callback(self, msg: LaserScan):
        self.scan_ranges    = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment
        self.scan_range_max = msg.range_max if msg.range_max > 0 else 10.0

        front = math.radians(FRONT_ANGLE_DEG)
        side_c = math.radians(SIDE_ANGLE_DEG)
        side_h = math.radians(SIDE_HALF_DEG)
        d_front = d_left = d_right = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.01:
                continue
            a = msg.angle_min + i * msg.angle_increment
            if abs(a) <= front:
                d_front = min(d_front, r)
            if abs(a - side_c) <= side_h:        # 좌측(+)
                d_left = min(d_left, r)
            if abs(a + side_c) <= side_h:        # 우측(-)
                d_right = min(d_right, r)

        self.dist_front = d_front
        self.dist_left  = d_left
        self.dist_right = d_right
        self.obstacle_stop = d_front < OBSTACLE_STOP_DIST
        self.obstacle_slow = d_front < OBSTACLE_SLOW_DIST

    # ── 스캔 전체에서 가장 열린 방향(상대각) ──
    def best_gap_angle(self):
        rs = self.scan_ranges
        if not rs or self.scan_angle_inc == 0:
            return math.pi   # 데이터 없으면 일단 뒤로
        rmax = self.scan_range_max
        clean = [(r if (math.isfinite(r) and r > 0.01) else rmax) for r in rs]
        n = len(clean)
        w = max(1, int(math.radians(GAP_WINDOW_DEG) / self.scan_angle_inc))
        best_i, best_clear = 0, -1.0
        for i in range(n):
            lo, hi = max(0, i - w), min(n, i + w + 1)
            clear = min(clean[lo:hi])      # 윈도우 내 최소 = 그 방향 통과 가능폭
            if clear > best_clear:
                best_clear, best_i = clear, i
        rel = self.scan_angle_min + best_i * self.scan_angle_inc
        return self.normalize_angle(rel)

    @staticmethod
    def normalize_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _log_next_wp(self):
        if self.retry_mode and self.skipped_wps:
            tx, ty = self.skipped_wps[0]
            self.get_logger().info(f'→ [재시도] ({tx:.1f}, {ty:.1f}) 남은 {len(self.skipped_wps)}개')
        elif self.wp_index < len(WAYPOINTS):
            tx, ty = WAYPOINTS[self.wp_index]
            self.get_logger().info(f'→ [{self.wp_index+1}/{len(WAYPOINTS)}] ({tx:.1f}, {ty:.1f})')

    def _get_current_target(self):
        if self.retry_mode and self.skipped_wps:
            return self.skipped_wps[0]
        if self.wp_index < len(WAYPOINTS):
            return WAYPOINTS[self.wp_index]
        return None

    def _advance_waypoint(self):
        if self.retry_mode and self.skipped_wps:
            self.skipped_wps.pop(0)
            if not self.skipped_wps:
                self.done = True
                self._finish()
                return
        else:
            self.wp_index += 1
            if self.wp_index >= len(WAYPOINTS):
                if self.skipped_wps:
                    self.retry_mode = True
                    self.get_logger().info(f'=== 메인 투어 완료. 스킵 {len(self.skipped_wps)}개 재시도 ===')
                else:
                    self.done = True
                    self._finish()
                    return
        self.state = 'ROTATE'
        self.avoid_attempts = 0
        self._reset_progress()
        self._log_next_wp()

    def _skip_waypoint(self, tx, ty):
        self.get_logger().warn(f'✗ ({tx:.1f}, {ty:.1f}) 도달 실패 → 스킵 (나중에 재시도)')
        if not self.retry_mode:
            self.skipped_wps.append((tx, ty))
        self.avoid_attempts = 0
        self.stop()
        self._advance_waypoint()

    def _finish(self):
        self.stop()
        self.get_logger().info('=== 맵핑 투어 완료 — 로봇 정지 ===')

    def _reset_progress(self):
        self.last_prog_x = self.x
        self.last_prog_y = self.y
        self.last_prog_time = self._now()

    def _enter_backup(self, reason):
        self.avoid_attempts += 1
        self.last_avoid_time = self._now()
        if self.avoid_attempts > MAX_AVOID_ATTEMPTS:
            t = self._get_current_target()
            if t:
                self._skip_waypoint(t[0], t[1])
            return False
        self.stop()
        self.avoid_timer = BACKUP_DURATION
        self.state = 'AVOID_BACKUP'
        self.get_logger().warn(f'{reason} → 후진/복구 #{self.avoid_attempts}')
        return True

    # ── 메인 루프 ──
    def control_loop(self):
        if self.done:
            return
        target = self._get_current_target()
        if target is None:
            self._finish()
            return

        tx, ty     = target
        dx, dy     = tx - self.x, ty - self.y
        dist       = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        angle_err  = self.normalize_angle(target_yaw - self.yaw)
        twist      = Twist()

        # ── AVOID_BACKUP ──
        if self.state == 'AVOID_BACKUP':
            self.avoid_timer -= 0.05
            if self.avoid_timer > 0:
                twist.linear.x = BACKUP_SPEED
                self.cmd_pub.publish(twist)
            else:
                # 핵심: 목표방위가 아니라 '가장 열린 방향'으로 회전
                self.avoid_yaw_target = self.normalize_angle(self.yaw + self.best_gap_angle())
                self.get_logger().info(
                    f'후진 완료 → 최대개방 방향으로 회전 '
                    f'(rel {math.degrees(self.normalize_angle(self.avoid_yaw_target - self.yaw)):.0f}도)')
                self.state = 'AVOID_ROTATE'
                self._reset_progress()
            return

        # ── AVOID_ROTATE ──
        if self.state == 'AVOID_ROTATE':
            yaw_err = self.normalize_angle(self.avoid_yaw_target - self.yaw)
            if abs(yaw_err) > 0.10:
                twist.angular.z = ANGULAR_SPEED * (1.0 if yaw_err > 0 else -1.0)
                self.cmd_pub.publish(twist)
            else:
                self.state = 'DRIVE_OFFSET'
                self._reset_progress()
            return

        # ── DRIVE_OFFSET: 열린 방향으로 빠져나가기 ──
        if self.state == 'DRIVE_OFFSET':
            if self.obstacle_stop:
                self._enter_backup('오프셋 주행 중 재차 막힘')
                return
            # 전방이 트이고 목표쪽으로 향할 수 있으면 정상 복귀
            if (not self.obstacle_slow) and abs(angle_err) < math.radians(40.0):
                self.state = 'ROTATE'
                self.avoid_attempts = 0
                self._reset_progress()
                return
            offset_err = self.normalize_angle(self.avoid_yaw_target - self.yaw)
            twist.linear.x  = LINEAR_SPEED * SLOW_FACTOR
            twist.angular.z = 1.2 * offset_err
            self.cmd_pub.publish(twist)
            self._check_stuck()
            return

        # ── 웨이포인트 도달 ──
        if dist < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info(f'✓ ({tx:.1f}, {ty:.1f}) 도달 (실제 {self.x:.2f}, {self.y:.2f})')
            self._advance_waypoint()
            return

        # ── ROTATE: 목표 방향 정렬(제자리) ──
        if self.state == 'ROTATE':
            if abs(angle_err) > ANGLE_TOLERANCE:
                twist.angular.z = ANGULAR_SPEED * (1.0 if angle_err > 0 else -1.0)
                self.cmd_pub.publish(twist)
            else:
                self.state = 'DRIVE'
                self._reset_progress()
            return

        # ── DRIVE: 주행(조향 동시) ──
        if self.state == 'DRIVE':
            # 정상 주행이 이어지면 회피 카운터 리셋
            if (self.avoid_attempts > 0 and not self.obstacle_slow
                    and self._now() - self.last_avoid_time > AVOID_RESET_SEC):
                self.get_logger().info(f'정상 주행 → 회피 카운터 리셋 (was {self.avoid_attempts})')
                self.avoid_attempts = 0

            if self.obstacle_stop:
                self._enter_backup(f'⚠ 전방 {self.dist_front:.2f}m')
                return

            # 각오차 크면 제자리회전, 작으면 주행하며 조향
            if abs(angle_err) > DRIVE_STEER_MAX:
                self.state = 'ROTATE'
                return

            speed = LINEAR_SPEED * SLOW_FACTOR if self.obstacle_slow else LINEAR_SPEED
            twist.linear.x  = speed
            twist.angular.z = 1.2 * angle_err
            self.cmd_pub.publish(twist)
            self._check_stuck()
            return

    # ── stuck 감지: 전방 미감지여도 안 움직이면 복구 ──
    def _check_stuck(self):
        moved = math.hypot(self.x - self.last_prog_x, self.y - self.last_prog_y)
        if moved > STUCK_DIST:
            self._reset_progress()
            return
        if self._now() - self.last_prog_time > STUCK_SEC:
            self.get_logger().warn(f'{STUCK_SEC:.0f}초간 정체 감지 → 강제 복구')
            self._enter_backup('stuck')

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MappingTour()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 — 로봇 정지')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
