#!/usr/bin/env python3
"""
DWA Local Planner Node — 2026-05-29 상태 머신 리팩토링 (YS)

Opticore AMR — 직접 구현 DWA (Nav2 dwb_local_planner 의존 금지).

═══════════════════════════════════════════════════════════════════════
설계 원칙 (2026-05-25 재설계 핵심, 유지)
═══════════════════════════════════════════════════════════════════════

원칙 1) **DWA 의 모든 내부 계산은 odom_filtered frame 안에서**.
원칙 2) **매 cycle TF lookup 없음** — EKF /odometry/filtered 의 pose+twist 직접 사용.
원칙 3) **global_path 는 받을 때 한 번만** map → odom_filtered 로 변환·캐싱.
원칙 4) AMCL TF freeze / 보정 점프 의 영향이 cycle 단위로 안 들어옴.

═══════════════════════════════════════════════════════════════════════
2026-05-29 리팩토링 (YS)
═══════════════════════════════════════════════════════════════════════

기존 _control_loop 의 암묵적 상태 (if/elif 중첩) → NavState enum 명시화.

NavState 전이 규칙:
    NORMAL  → ALIGN       : |α| > align_thresh
    NORMAL  → EMERGENCY   : collision_imminent
    NORMAL  → SPIN        : is_velocity_blocked (stuck 1.5s)
    ALIGN   → NORMAL      : |α| < align_exit
    SPIN    → NORMAL      : spin 완료 + fwd_clear 확보
    SPIN    → BACKUP      : spin 완료 + fwd_clear 여전히 막힘
    BACKUP  → NORMAL      : backup 완료
    BACKUP  → EMERGENCY   : backup 중 후방 위험
    EMERGENCY → NORMAL    : 외부 트리거 (A* 새 path)
    * → REACHED           : dist_to_goal < tolerance

각 상태 실행 함수:
    _execute_normal()   Pure Pursuit + adaptive velocity
    _execute_align()    In-place PD 회전
    _execute_spin()     Recovery spin (제자리 회전)
    _execute_backup()   Recovery backup (후진)
    _execute_emergency() 완전 정지 + A* 재계획 대기

외부 인터페이스 (토픽, 파라미터) 는 기존과 완전 동일.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException


# ═══════════════════════════════════════════════════════════════════════
# 데이터 구조 — ROS 비의존 순수 자료형
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class RobotState:
    """로봇 상태 (frame 무관 — 호출자가 frame 일관성 책임)."""
    x: float
    y: float
    theta: float
    v: float
    w: float


@dataclass
class DynamicWindow:
    """Dynamic Window — 도달 가능한 (v, w) 영역."""
    v_min: float
    v_max: float
    w_min: float
    w_max: float


@dataclass
class VelocityCommand:
    """평가 결과로 선택된 (v, w) 명령."""
    v: float
    w: float
    score: float = 0.0


class NavState(Enum):
    """명시적 내비게이션 상태 — fleet BT 연결 준비."""
    NORMAL       = "NORMAL"        # Pure Pursuit 정상 추종
    ALIGN        = "ALIGN"         # in-place 회전 (heading 오차 큼)
    SPIN         = "SPIN"          # spin recovery (stuck → 제자리 회전 탈출)
    FORWARD_ONLY = "FORWARD_ONLY"  # spin 완료 후 현재 heading으로 짧게 전진
    BACKUP       = "BACKUP"        # backup recovery (후진)
    EMERGENCY    = "EMERGENCY"     # 충돌 임박 / 전후방 막힘
    REACHED   = "REACHED"     # goal 도달


# ═══════════════════════════════════════════════════════════════════════
# 순수 알고리즘 함수 — 단위 테스트 가능 (기존과 동일)
# ═══════════════════════════════════════════════════════════════════════

def compute_dynamic_window(
    state: RobotState,
    v_max: float,
    v_min: float,
    w_max: float,
    a_max: float,
    alpha_max: float,
    dt: float,
) -> DynamicWindow:
    v_lo_accel = state.v - a_max * dt
    v_hi_accel = state.v + a_max * dt
    w_lo_accel = state.w - alpha_max * dt
    w_hi_accel = state.w + alpha_max * dt
    return DynamicWindow(
        v_min=max(v_min, v_lo_accel),
        v_max=min(v_max, v_hi_accel),
        w_min=max(-w_max, w_lo_accel),
        w_max=min(w_max, w_hi_accel),
    )


def sample_velocities(
    window: DynamicWindow,
    n_v: int,
    n_w: int,
) -> List[Tuple[float, float]]:
    samples: List[Tuple[float, float]] = []
    if n_v < 1 or n_w < 1:
        return samples
    if n_v == 1:
        v_step_values = [(window.v_min + window.v_max) / 2.0]
    else:
        v_step = (window.v_max - window.v_min) / (n_v - 1)
        v_step_values = [window.v_min + i * v_step for i in range(n_v)]
    if n_w == 1:
        w_step_values = [(window.w_min + window.w_max) / 2.0]
    else:
        w_step = (window.w_max - window.w_min) / (n_w - 1)
        w_step_values = [window.w_min + j * w_step for j in range(n_w)]
    for v in v_step_values:
        for w in w_step_values:
            samples.append((v, w))
    return samples


def forward_simulate(
    state: RobotState,
    v: float,
    w: float,
    dt: float,
    sim_time: float,
) -> List[Tuple[float, float, float]]:
    if dt <= 0 or sim_time <= 0:
        return []
    steps = int(sim_time / dt)
    if steps <= 0:
        return []
    traj: List[Tuple[float, float, float]] = []
    x, y, theta = state.x, state.y, state.theta
    for _ in range(steps):
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        theta = math.atan2(math.sin(theta), math.cos(theta))
        traj.append((x, y, theta))
    return traj


def heading_score(traj_end_x: float, traj_end_y: float, traj_end_theta: float,
                  goal_x: float, goal_y: float) -> float:
    """[기존] test_dwa.py 호환을 위해 유지."""
    dx = goal_x - traj_end_x
    dy = goal_y - traj_end_y
    if dx == 0.0 and dy == 0.0:
        return 1.0
    angle_to_goal = math.atan2(dy, dx)
    diff = angle_to_goal - traj_end_theta
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return 1.0 - abs(diff) / math.pi


def heading_score_dist(traj_end_x: float, traj_end_y: float,
                       goal_x: float, goal_y: float,
                       max_dist: float = 1.0) -> float:
    dx = goal_x - traj_end_x
    dy = goal_y - traj_end_y
    dist = math.sqrt(dx * dx + dy * dy)
    return max(0.0, 1.0 - dist / max_dist)


def path_tangent_score(traj_end_theta: float, path_tangent_local: float) -> float:
    diff = path_tangent_local - traj_end_theta
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return 1.0 - abs(diff) / math.pi


def compute_path_tangent_local(path_xy: List[Tuple[float, float]],
                                start_idx: int,
                                look_ahead: int,
                                robot_x: float, robot_y: float,
                                robot_theta: float) -> float:
    if not path_xy or start_idx >= len(path_xy):
        return 0.0
    target_idx = min(start_idx + look_ahead, len(path_xy) - 1)
    if target_idx <= start_idx:
        return 0.0
    px1, py1 = path_xy[start_idx]
    px2, py2 = path_xy[target_idx]
    path_yaw_world = math.atan2(py2 - py1, px2 - px1)
    diff = path_yaw_world - robot_theta
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return diff


def clearance_score(trajectory_xy: List[Tuple[float, float]],
                    obstacle_points: List[Tuple[float, float]],
                    max_clearance: float = 1.0) -> float:
    if not trajectory_xy:
        return 0.0
    if not obstacle_points:
        return 1.0
    min_dist_sq = float("inf")
    for tx, ty in trajectory_xy:
        for ox, oy in obstacle_points:
            d_sq = (tx - ox) * (tx - ox) + (ty - oy) * (ty - oy)
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
    if min_dist_sq == float("inf"):
        return 1.0
    min_dist = math.sqrt(min_dist_sq)
    if max_clearance <= 0.0:
        return 0.0
    return min(min_dist / max_clearance, 1.0)


def velocity_score(v: float, v_max: float) -> float:
    if v_max <= 0.0:
        return 0.0
    return max(0.0, v) / v_max


def min_clearance_distance(trajectory_xy: List[Tuple[float, float]],
                           obstacle_points: List[Tuple[float, float]]) -> float:
    if not trajectory_xy or not obstacle_points:
        return float("inf")
    best = float("inf")
    for tx, ty in trajectory_xy:
        for ox, oy in obstacle_points:
            d_sq = (tx - ox) * (tx - ox) + (ty - oy) * (ty - oy)
            if d_sq < best:
                best = d_sq
    return math.sqrt(best) if best != float("inf") else float("inf")


def find_nearest_idx(
    path_xy: List[Tuple[float, float]],
    robot_xy: Tuple[float, float],
    start_idx: int = 0,
) -> int:
    if not path_xy:
        return 0
    s = max(0, min(start_idx, len(path_xy) - 1))
    rx, ry = robot_xy
    nearest_idx = s
    nearest_d_sq = float("inf")
    for i in range(s, len(path_xy)):
        px, py = path_xy[i]
        d_sq = (px - rx) * (px - rx) + (py - ry) * (py - ry)
        if d_sq < nearest_d_sq:
            nearest_d_sq = d_sq
            nearest_idx = i
    return nearest_idx


def pick_lookahead_point(
    path_xy: List[Tuple[float, float]],
    robot_xy: Tuple[float, float],
    lookahead_dist: float,
    start_idx: int = 0,
) -> Optional[Tuple[float, float]]:
    if not path_xy:
        return None
    nearest_idx = find_nearest_idx(path_xy, robot_xy, start_idx)
    cumulative = 0.0
    prev_x, prev_y = path_xy[nearest_idx]
    for j in range(nearest_idx + 1, len(path_xy)):
        x, y = path_xy[j]
        cumulative += math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
        if cumulative >= lookahead_dist:
            return (x, y)
        prev_x, prev_y = x, y
    return path_xy[-1]


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_local(world_xy: Tuple[float, float],
                   robot: RobotState) -> Tuple[float, float]:
    dx = world_xy[0] - robot.x
    dy = world_xy[1] - robot.y
    cos_t = math.cos(-robot.theta)
    sin_t = math.sin(-robot.theta)
    return (cos_t * dx - sin_t * dy, sin_t * dx + cos_t * dy)


# ═══════════════════════════════════════════════════════════════════════
# DWA Planner Node
# ═══════════════════════════════════════════════════════════════════════

class DwaPlannerNode(Node):
    """DWA Local Planner — NavState 기반 명시적 상태 머신 (2026-05-29 리팩토링).

    구독:
        /odometry/filtered   EKF 출력 (1순위)
        /odom                DiffDrive plugin 출력 (fallback)
        /global_path         A* 의 전역 경로 (frame: map)
        /lidar               장애물 점

    발행:
        /cmd_vel             제어 명령 (Twist)
        /dwa/trajectories    후보 trajectory (MarkerArray, base_link frame)
        /dwa/best_trajectory 선택된 trajectory (Marker, base_link frame)
        /dwa/status          상태 String (1Hz)
    """

    LOCAL_FRAME  = "odom_filtered"
    GLOBAL_FRAME = "map"
    ROBOT_FRAME  = "base_footprint"

    # ───────────────────────────────────────────────────────────────
    # __init__
    # ───────────────────────────────────────────────────────────────
    def __init__(self) -> None:
        super().__init__("dwa_planner")

        # ── 파라미터 선언 (기존과 완전 동일) ──────────────────────────
        self.declare_parameter("v_max", 1.5)
        self.declare_parameter("v_min", -1.0)
        self.declare_parameter("w_max", 1.5)
        self.declare_parameter("a_max", 1.0)
        self.declare_parameter("alpha_max", 1.5)
        self.declare_parameter("a_lat_max", 1.0)
        self.declare_parameter("sample_v_n", 11)
        self.declare_parameter("sample_w_n", 21)
        self.declare_parameter("predict_horizon", 1.0)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("weight_heading", 0.6)
        self.declare_parameter("weight_clearance", 1.2)
        self.declare_parameter("weight_velocity", 0.2)
        self.declare_parameter("weight_path_tangent", 0.8)
        self.declare_parameter("path_tangent_lookahead", 10)
        self.declare_parameter("lookahead_dist", 1.0)
        self.declare_parameter("lookahead_time", 0.7)
        self.declare_parameter("max_clearance", 1.0)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("clearance_slowdown_distance", 0.80)
        self.declare_parameter("clearance_stop_distance", 0.30)
        self.declare_parameter("align_angle_thresh", 0.785)
        self.declare_parameter("align_angle_exit", 0.262)
        self.declare_parameter("align_kp", 1.5)
        self.declare_parameter("align_kd", 0.5)
        self.declare_parameter("align_v_blend_max", 0.3)
        self.declare_parameter("align_cooldown", 1.0)
        self.declare_parameter("w_min_rotate", 0.3)
        self.declare_parameter("allow_backward", False)
        self.declare_parameter("max_path_offset", 2.0)
        self.declare_parameter("stuck_recovery_sec", 1.5)
        self.declare_parameter("backup_velocity", -0.25)
        self.declare_parameter("backup_duration", 1.5)
        self.declare_parameter("backup_turn_gain", 0.8)
        self.declare_parameter("backup_turn_max", 0.45)
        self.declare_parameter("recovery_cooldown", 3.0)
        self.declare_parameter("spin_duration", 2.0)       # spin recovery 지속 시간
        self.declare_parameter("robot_radius", 0.20)
        self.declare_parameter("hard_collision_distance", 0.05)
        self.declare_parameter("safety_distance", 0.30)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("scan_range_max", 25.0)
        self.declare_parameter("lidar_offset_x", 0.25)
        self.declare_parameter("lidar_offset_y", 0.0)
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("odom_fallback_topic", "/odom")
        self.declare_parameter("global_path_topic", "/global_path")
        self.declare_parameter("scan_topic", "/lidar")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("state_log_period", 1.0)
        self.declare_parameter("candidate_log_period", 1.0)
        self.declare_parameter("path_tf_timeout", 1.0)

        self._load_params()

        # ── ROS 입출력 상태 ───────────────────────────────────────────
        self._state: Optional[RobotState] = None
        self._path_local: Optional[Path] = None
        self._latest_scan: Optional[LaserScan] = None
        self._last_odom_time: Optional[float] = None
        self._last_odom_was_fallback = False
        self._path_warn_logged = False

        # ── NavState 머신 ─────────────────────────────────────────────
        self._nav_state: NavState = NavState.NORMAL

        # NORMAL/ALIGN 공통
        self._path_progress_idx = 0
        self._last_kappa: float = 0.0

        # ALIGN 전용
        self._in_align_mode = False      # 하위 호환 (path 콜백에서 리셋)
        self._align_cooldown_until = 0.0

        # SPIN 전용
        self._spin_until = 0.0
        self._spin_direction = 1.0       # +1 왼쪽, -1 오른쪽

        # FORWARD_ONLY 전용 (spin 완료 후 짧은 전진)
        self._forward_only_until = 0.0   # 전진 종료 시각
        self._forward_only_dist  = 0.0   # 목표 전진 거리 [m]
        self._forward_only_start_x = 0.0 # 전진 시작 위치 x
        self._forward_only_start_y = 0.0 # 전진 시작 위치 y

        # BACKUP 전용
        self._backup_until = 0.0

        # RECOVERY 공통 (SPIN/BACKUP 완료 후 cooldown)
        self._recovery_cooldown_until = 0.0
        self._stuck_counter = 0

        # REACHED
        self._reached = False

        # 진단
        self._state_log_counter = 0
        self._candidate_log_counter = 0

        # ── TF ────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ──────────────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=10,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT)
        path_qos = QoSProfile(depth=10,
                              reliability=QoSReliabilityPolicy.RELIABLE)

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, self.p_odom_topic, self._on_odom, 10)
        self.create_subscription(
            Odometry, self.p_odom_fallback_topic, self._on_odom_fallback, 10)
        self.create_subscription(
            Path, self.p_global_path_topic, self._on_global_path, path_qos)
        self.create_subscription(
            LaserScan, self.p_scan_topic, self._on_scan, sensor_qos)

        # ── 발행 ─────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, self.p_cmd_vel_topic, 10)
        self._traj_pub = self.create_publisher(MarkerArray, "/dwa/trajectories", 10)
        self._best_pub = self.create_publisher(Marker, "/dwa/best_trajectory", 10)
        self._status_pub = self.create_publisher(String, "/dwa/status", 10)

        # ── 타이머 ───────────────────────────────────────────────────
        period = 1.0 / max(self.p_control_rate, 1.0)
        self._control_timer = self.create_timer(period, self._control_loop)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"DWA Planner 시작 (NavState 머신 2026-05-29) — "
            f"v_max={self.p_v_max} w_max={self.p_w_max} "
            f"control_rate={self.p_control_rate}Hz"
        )

    # ───────────────────────────────────────────────────────────────
    # 파라미터 로드
    # ───────────────────────────────────────────────────────────────
    def _load_params(self) -> None:
        gp = self.get_parameter
        self.p_v_max                    = gp("v_max").value
        self.p_v_min                    = gp("v_min").value
        self.p_w_max                    = gp("w_max").value
        self.p_a_max                    = gp("a_max").value
        self.p_alpha_max                = gp("alpha_max").value
        self.p_a_lat_max                = gp("a_lat_max").value
        self.p_sample_v_n               = gp("sample_v_n").value
        self.p_sample_w_n               = gp("sample_w_n").value
        self.p_predict_horizon          = gp("predict_horizon").value
        self.p_dt                       = gp("dt").value
        self.p_control_rate             = gp("control_rate").value
        self.p_w_heading                = gp("weight_heading").value
        self.p_w_clearance              = gp("weight_clearance").value
        self.p_w_velocity               = gp("weight_velocity").value
        self.p_w_path_tangent           = gp("weight_path_tangent").value
        self.p_path_tangent_lookahead   = gp("path_tangent_lookahead").value
        self.p_lookahead_dist           = gp("lookahead_dist").value
        self.p_lookahead_time           = gp("lookahead_time").value
        self.p_max_clearance            = gp("max_clearance").value
        self.p_goal_tolerance           = gp("goal_tolerance").value
        self.p_clearance_slowdown_distance = gp("clearance_slowdown_distance").value
        self.p_clearance_stop_distance  = gp("clearance_stop_distance").value
        self.p_align_angle_thresh       = gp("align_angle_thresh").value
        self.p_align_angle_exit         = gp("align_angle_exit").value
        self.p_align_kp                 = gp("align_kp").value
        self.p_align_kd                 = gp("align_kd").value
        self.p_align_v_blend_max        = gp("align_v_blend_max").value
        self.p_align_cooldown           = gp("align_cooldown").value
        self.p_w_min_rotate             = gp("w_min_rotate").value
        self.p_allow_backward           = gp("allow_backward").value
        self.p_max_path_offset          = gp("max_path_offset").value
        self.p_stuck_recovery_sec       = gp("stuck_recovery_sec").value
        self.p_backup_velocity          = gp("backup_velocity").value
        self.p_backup_duration          = gp("backup_duration").value
        self.p_backup_turn_gain         = gp("backup_turn_gain").value
        self.p_backup_turn_max          = gp("backup_turn_max").value
        self.p_recovery_cooldown        = gp("recovery_cooldown").value
        self.p_spin_duration            = gp("spin_duration").value
        self.p_robot_radius             = gp("robot_radius").value
        self.p_hard_collision_distance  = gp("hard_collision_distance").value
        self.p_safety_distance          = gp("safety_distance").value
        self.p_odom_timeout             = gp("odom_timeout").value
        self.p_scan_range_max           = gp("scan_range_max").value
        self.p_lidar_offset_x           = gp("lidar_offset_x").value
        self.p_lidar_offset_y           = gp("lidar_offset_y").value
        self.p_odom_topic               = gp("odom_topic").value
        self.p_odom_fallback_topic      = gp("odom_fallback_topic").value
        self.p_global_path_topic        = gp("global_path_topic").value
        self.p_scan_topic               = gp("scan_topic").value
        self.p_cmd_vel_topic            = gp("cmd_vel_topic").value
        self.p_state_log_period         = gp("state_log_period").value
        self.p_candidate_log_period     = gp("candidate_log_period").value
        self.p_path_tf_timeout          = gp("path_tf_timeout").value

    # ───────────────────────────────────────────────────────────────
    # 콜백 (기존과 동일)
    # ───────────────────────────────────────────────────────────────
    def _on_odom(self, msg: Odometry) -> None:
        self._update_state_from_odom(msg, is_fallback=False)
        self._last_odom_time = self._sec_now()

    def _on_odom_fallback(self, msg: Odometry) -> None:
        now = self._sec_now()
        if self._last_odom_time is None:
            self._update_state_from_odom(msg, is_fallback=True)
            self._last_odom_time = now
            return
        if now - self._last_odom_time > self.p_odom_timeout:
            if not self._last_odom_was_fallback:
                self.get_logger().warn(
                    "EKF /odometry/filtered timeout → /odom fallback 사용")
            self._update_state_from_odom(msg, is_fallback=True)
            self._last_odom_time = now

    def _update_state_from_odom(self, msg: Odometry, *, is_fallback: bool) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        if not is_fallback and msg.header.frame_id != self.LOCAL_FRAME:
            self.get_logger().warn(
                f"/odometry/filtered frame_id='{msg.header.frame_id}' ≠ '{self.LOCAL_FRAME}'",
                throttle_duration_sec=10.0)
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        v = twist.linear.x
        w = twist.angular.z
        if self._state is None:
            self._state = RobotState(x=x, y=y, theta=theta, v=v, w=w)
        else:
            self._state.x = x; self._state.y = y; self._state.theta = theta
            self._state.v = v; self._state.w = w
        self._last_odom_was_fallback = is_fallback

    def _on_global_path(self, msg: Path) -> None:
        if not msg.poses:
            if self._path_local is None or self._path_local.poses:
                self.get_logger().warn("빈 /global_path 수신 — DWA 정지 모드")
            empty = Path()
            empty.header.frame_id = self.LOCAL_FRAME
            empty.header.stamp = self.get_clock().now().to_msg()
            self._path_local = empty
            self._reached = False
            return

        src_frame = msg.header.frame_id or self.GLOBAL_FRAME
        if src_frame == self.LOCAL_FRAME:
            self._path_local = msg
            self._reset_path_state()
            last = msg.poses[-1].pose.position
            self.get_logger().info(
                f"/global_path 수신 ({src_frame}) — "
                f"{len(msg.poses)}점, goal=({last.x:.2f},{last.y:.2f})")
            return

        if src_frame != self.GLOBAL_FRAME:
            self.get_logger().warn(
                f"/global_path frame_id='{src_frame}' — 지원 안 됨. 무시.")
            return

        transformed = self._transform_path_to_local(msg)
        if transformed is None:
            return

        self._path_local = transformed
        self._reset_path_state()
        last = transformed.poses[-1].pose.position
        self.get_logger().info(
            f"/global_path 수신 ({src_frame} → {self.LOCAL_FRAME}) — "
            f"{len(transformed.poses)}점, "
            f"goal=({last.x:.2f},{last.y:.2f}) [{self.LOCAL_FRAME}]")

    def _reset_path_state(self) -> None:
        """새 path 수신 시 추종 상태 리셋."""
        self._reached = False
        self._path_progress_idx = 0
        self._in_align_mode = False
        # EMERGENCY/SPIN/BACKUP 중이어도 새 path 오면 NORMAL 복귀
        if self._nav_state in (NavState.EMERGENCY,):
            self._nav_state = NavState.NORMAL
            self._stuck_counter = 0

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    # ───────────────────────────────────────────────────────────────
    # 메인 제어 루프 — NavState 디스패처
    # ───────────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        # ── 공통 전제조건 ──────────────────────────────────────────
        if self._state is None:
            return
        if (self._last_odom_time is None or
                self._sec_now() - self._last_odom_time > self.p_odom_timeout):
            self._stop_robot("odom_timeout")
            return

        # ── NavState 디스패치 ─────────────────────────────────────
        # SPIN/FORWARD_ONLY/BACKUP 은 path 없이도 실행 (recovery 우선)
        if self._nav_state == NavState.SPIN:
            self._execute_spin()
            return

        if self._nav_state == NavState.FORWARD_ONLY:
            self._execute_forward_only()
            return

        if self._nav_state == NavState.BACKUP:
            self._execute_backup()
            return

        # path 필요한 상태
        if self._path_local is None or not self._path_local.poses:
            self._stop_robot("no_global_path")
            self._publish_status_value("STOPPED")
            return

        if self._nav_state == NavState.REACHED:
            self._stop_robot("reached_hold")
            return

        # 공통 선행 계산 (NORMAL/ALIGN/EMERGENCY 공유)
        ctx = self._compute_context()
        if ctx is None:
            return   # 내부에서 이미 정지 처리

        # REACHED 전이
        if ctx["dist_to_goal"] < self.p_goal_tolerance:
            self._nav_state = NavState.REACHED
            self._reached = True
            self._stop_robot("goal_reached")
            self._goal_reached_logged_once()
            return

        # 상태별 실행
        if self._nav_state == NavState.EMERGENCY:
            self._execute_emergency(ctx)
        elif self._nav_state == NavState.ALIGN:
            self._execute_align(ctx)
        else:
            self._execute_normal(ctx)

    # ───────────────────────────────────────────────────────────────
    # 공통 선행 계산 — NORMAL/ALIGN/EMERGENCY 가 모두 필요한 값
    # ───────────────────────────────────────────────────────────────
    def _compute_context(self) -> Optional[dict]:
        """경로 추종에 필요한 공통 값을 계산해 dict 로 반환.

        실패(lookahead 없음, path_offset 초과 등) 시 None 반환.
        """
        path_xy = self._path_xy(self._path_local)
        gx, gy = path_xy[-1]
        dist_to_goal = math.hypot(gx - self._state.x, gy - self._state.y)

        nearest_idx = find_nearest_idx(
            path_xy, (self._state.x, self._state.y), self._path_progress_idx)
        self._path_progress_idx = nearest_idx

        nx, ny = path_xy[nearest_idx]
        path_offset = math.hypot(nx - self._state.x, ny - self._state.y)
        if path_offset > self.p_max_path_offset:
            self._stop_robot("path_offset_too_large")
            self._publish_status_value("PATH_LOST")
            self.get_logger().warn(
                f"path 와 {path_offset:.2f}m 떨어짐 (max={self.p_max_path_offset}m).",
                throttle_duration_sec=2.0)
            return None

        # Adaptive lookahead (approach scaling)
        approach_scale = min(1.0, dist_to_goal / max(self.p_lookahead_dist, 1e-3))
        effective_lookahead = max(
            self.p_goal_tolerance * 1.5,
            self.p_lookahead_time * abs(self._state.v),
            self.p_lookahead_dist * approach_scale,
        )
        lookahead = pick_lookahead_point(
            path_xy, (self._state.x, self._state.y),
            effective_lookahead, start_idx=nearest_idx)
        if lookahead is None:
            self._stop_robot("no_lookahead")
            return None

        lx, ly = world_to_local(lookahead, self._state)
        L = math.hypot(lx, ly)
        alpha = math.atan2(ly, lx)

        obstacles_local = self._extract_obstacles_from_scan()
        fwd_clear = self._forward_clearance_inline(obstacles_local, kappa=self._last_kappa)

        return {
            "path_xy": path_xy,
            "dist_to_goal": dist_to_goal,
            "nearest_idx": nearest_idx,
            "effective_lookahead": effective_lookahead,
            "lx": lx, "ly": ly, "L": L, "alpha": alpha,
            "obstacles_local": obstacles_local,
            "fwd_clear": fwd_clear,
        }

    # ───────────────────────────────────────────────────────────────
    # 상태 실행 함수들
    # ───────────────────────────────────────────────────────────────
    def _execute_normal(self, ctx: dict) -> None:
        """NavState.NORMAL — Pure Pursuit + adaptive velocity."""
        alpha       = ctx["alpha"]
        lx          = ctx["lx"]
        ly          = ctx["ly"]
        L           = ctx["L"]
        fwd_clear   = ctx["fwd_clear"]
        dist_to_goal= ctx["dist_to_goal"]
        obstacles   = ctx["obstacles_local"]

        period = 1.0 / max(self.p_control_rate, 1.0)
        dv_max = self.p_a_max * period
        dw_max = self.p_alpha_max * period

        # ── ALIGN 전이 판정 ──────────────────────────────────────
        if self._in_align_mode:
            if abs(alpha) < self.p_align_angle_exit:
                self._in_align_mode = False
        else:
            if abs(alpha) > self.p_align_angle_thresh and L > 0.1:
                self._in_align_mode = True

        if self._in_align_mode:
            self._nav_state = NavState.ALIGN
            self._execute_align(ctx)
            return

        # ── Pure Pursuit ─────────────────────────────────────────
        kappa = 2.0 * ly / (L * L) if L >= 1e-3 else 0.0
        self._last_kappa = kappa

        v_target = self.p_v_max

        # (i) 곡률 기반 감속
        if abs(kappa) > 1e-3:
            v_curve = math.sqrt(self.p_a_lat_max / abs(kappa))
            v_target = min(v_target, v_curve)

        # (ii) arc clearance 기반 감속 — trajectory_clearance 만 사용
        # fwd_clear(부채꼴)는 측면 장애물을 전방으로 오인해 불필요한 감속 유발.
        # trajectory_clearance는 실제 진행 arc 위 장애물만 보므로
        # 측면 벽 옆을 지날 때 감속하지 않고, 진행 방향 장애물에만 반응.
        probe_v = max(0.05, min(self.p_v_max, v_target))
        probe_w = kappa * probe_v
        motion_clear = self._trajectory_clearance_margin(obstacles, probe_v, probe_w)
        cf = self.p_clearance_stop_distance
        cc = max(self.p_clearance_slowdown_distance, cf + 1e-3)
        if motion_clear < cc:
            v_clear = self.p_v_max * max(0.0, motion_clear - cf) / max(cc - cf, 1e-3)
            v_target = min(v_target, v_clear)

        # (iii) goal 감속
        v_goal = math.sqrt(2.0 * self.p_a_max *
                           max(0.0, dist_to_goal - self.p_goal_tolerance))
        v_target = min(v_target, v_goal)

        # (iv) heading 감속
        heading_factor = max(0.3, math.cos(alpha))
        v_target *= heading_factor

        # w 계산 (v/w 커플링 해제)
        w_pp = kappa * v_target
        if (self.p_w_min_rotate > 0.0
                and abs(alpha) > self.p_align_angle_exit
                and abs(w_pp) < self.p_w_min_rotate):
            w_target = math.copysign(self.p_w_min_rotate, alpha)
        else:
            w_target = w_pp

        # 한계 + 가속도 제한
        v_target = max(0.0, min(self.p_v_max, v_target))
        w_target = max(-self.p_w_max, min(self.p_w_max, w_target))
        v_cmd = max(self._state.v - dv_max,
                    min(self._state.v + dv_max, v_target))
        w_cmd = max(self._state.w - dw_max,
                    min(self._state.w + dw_max, w_target))

        # 최종 충돌 체크
        sim_state = RobotState(x=0.0, y=0.0, theta=0.0,
                               v=self._state.v, w=self._state.w)
        sim_traj = forward_simulate(sim_state, v_cmd, w_cmd,
                                    self.p_dt, self.p_predict_horizon)
        if sim_traj:
            sim_traj_xy = [(p[0], p[1]) for p in sim_traj[2:]] or \
                          [(p[0], p[1]) for p in sim_traj]
            min_d = min_clearance_distance(sim_traj_xy, obstacles)
        else:
            sim_traj_xy = []; min_d = float("inf")

        collision_imminent = (
            min_d < (self.p_hard_collision_distance + self.p_robot_radius) or
            # fwd_clear 보조: 측면 대각선 동적 장애물 등 arc 밖 위협 감지
            fwd_clear < (self.p_hard_collision_distance + self.p_robot_radius)
        )

        # stuck/velocity blocked 판정
        # motion_clear: arc 위 장애물 (주 기준)
        # fwd_clear: 부채꼴 장애물 (보조 — arc 밖 측면 벽이 v를 낮춘 경우 커버)
        is_velocity_blocked = (abs(v_cmd) < 0.02 and
                               (motion_clear < self.p_clearance_stop_distance or
                                fwd_clear < self.p_clearance_stop_distance))
        in_recovery_cooldown = self._sec_now() < self._recovery_cooldown_until

        if collision_imminent or is_velocity_blocked:
            if collision_imminent:
                self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
                self._publish_status_value("EMERGENCY")
            else:
                self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
                self._publish_status_value("STOPPED_NEAR_WALL")

            if not in_recovery_cooldown:
                self._stuck_counter += 1
                stuck_threshold = int(self.p_control_rate * self.p_stuck_recovery_sec)
                if self._stuck_counter > stuck_threshold:
                    self._trigger_recovery(obstacles, motion_clear)
            self._log_state_throttled()
            if sim_traj:
                self._publish_best_trajectory(sim_traj)
            return

        # 정상 발행
        self._stuck_counter = 0
        self._publish_cmd(VelocityCommand(v=v_cmd, w=w_cmd))
        self._publish_status_value("NORMAL")
        self._log_state_throttled()
        if sim_traj:
            self._publish_best_trajectory(sim_traj)

        # 진단 로그
        self._candidate_log_counter += 1
        target = int(self.p_control_rate * self.p_candidate_log_period)
        if self.p_candidate_log_period > 0.0 and \
           self._candidate_log_counter >= max(1, target):
            self._candidate_log_counter = 0
            self.get_logger().info(
                f"PP[NORMAL]: la=({lx:+.2f},{ly:+.2f}) "
                f"L={L:.2f}(eff={ctx['effective_lookahead']:.2f}) "
                f"α={math.degrees(alpha):+.1f}° κ={kappa:+.2f} "
                f"v={v_cmd:+.2f}/{v_target:.2f} w={w_cmd:+.2f}/{w_target:+.2f} "
                f"clr={motion_clear:.2f} fwd={fwd_clear:.2f} "
                f"d_goal={dist_to_goal:.2f}")

    def _execute_align(self, ctx: dict) -> None:
        """NavState.ALIGN — In-place PD 회전."""
        alpha     = ctx["alpha"]
        fwd_clear = ctx["fwd_clear"]
        obstacles = ctx["obstacles_local"]

        period = 1.0 / max(self.p_control_rate, 1.0)
        dv_max = self.p_a_max * period
        dw_max = self.p_alpha_max * period

        # ALIGN 종료 → NORMAL 전이
        if abs(alpha) < self.p_align_angle_exit:
            self._in_align_mode = False
            self._nav_state = NavState.NORMAL
            self._align_cooldown_until = self._sec_now() + self.p_align_cooldown
            return

        # rotate clearance 체크
        rotate_clear = self._rotation_clearance_inline(obstacles)
        motion_clear = rotate_clear

        v_target = 0.0
        if (self.p_align_v_blend_max > 0.0 and
                rotate_clear > self.p_clearance_slowdown_distance and
                abs(alpha) < math.radians(75.0)):
            blend = 1.0 - (abs(alpha) - self.p_align_angle_exit) / \
                max(math.radians(75.0) - self.p_align_angle_exit, 1e-3)
            v_target = self.p_align_v_blend_max * max(0.0, min(1.0, blend))

        w_target = (self.p_align_kp * alpha - self.p_align_kd * self._state.w)
        w_target = max(-self.p_w_max * 0.7, min(self.p_w_max * 0.7, w_target))

        v_cmd = max(self._state.v - dv_max,
                    min(self._state.v + dv_max, v_target))
        w_cmd = max(self._state.w - dw_max,
                    min(self._state.w + dw_max, w_target))

        # align 중 stuck 체크
        is_velocity_blocked = (abs(v_cmd) < 0.02 and
                               motion_clear < self.p_clearance_stop_distance)
        in_recovery_cooldown = self._sec_now() < self._recovery_cooldown_until

        if is_velocity_blocked and not in_recovery_cooldown:
            self._stuck_counter += 1
            stuck_threshold = int(self.p_control_rate * self.p_stuck_recovery_sec)
            if self._stuck_counter > stuck_threshold:
                self._trigger_recovery(obstacles, motion_clear)
                return

        self._publish_cmd(VelocityCommand(v=v_cmd, w=w_cmd))
        self._publish_status_value("ALIGN")
        self._log_state_throttled()

    def _execute_spin(self) -> None:
        """NavState.SPIN — Recovery spin (제자리 회전으로 탈출 방향 확보).

        종료 조건 분리:
          spin_unsafe  : 회전 arc 자체가 위험 (몸체 충돌) → 즉시 종료
          spin_done    : 직진 arc 가 열렸음 (전진 가능) → 종료
          그 외         : 계속 회전

        "전방이 열렸다"의 기준 = trajectory_clearance(v>0, w=0).
        오른쪽 벽이 가까워도 직진 arc 위에 없으면 열린 것으로 판단.
        측면 벽 때문에 spin이 무한 지속되지 않음.
        """
        now = self._sec_now()
        obstacles = self._extract_obstacles_from_scan()

        # 회전 자체가 안전한가 — 몸체 반경 기준
        spin_probe_v = max(0.05, self.p_v_max * 0.3)
        spin_probe_w = self._spin_direction * self.p_w_max
        arc_clear = self._trajectory_clearance_margin(
            obstacles, spin_probe_v, spin_probe_w)
        spin_unsafe = arc_clear < self.p_robot_radius

        # 전진 가능해졌는가 — 직진 arc 기준 (측면 벽 무시)
        fwd_probe_v = max(0.05, self.p_v_max * 0.3)
        forward_clear = self._trajectory_clearance_margin(
            obstacles, fwd_probe_v, 0.0)
        spin_done = forward_clear > self.p_clearance_stop_distance

        if now < self._spin_until and not spin_unsafe and not spin_done:
            # 회전 안전 + 전방 아직 막힘 → 계속 회전
            w_spin = self._spin_direction * self.p_w_max
            self._publish_cmd(VelocityCommand(v=0.0, w=w_spin))
            self._publish_status_value("RECOVERY")
            self._log_state_throttled()
            return

        # 종료 — 사유 판단
        self._spin_until = 0.0

        if spin_done:
            # 전진 가능 → FORWARD_ONLY로 전환 (현재 heading으로 짧게 전진)
            # 위치가 바뀐 후 A* 재계획 → rack 모서리 벗어난 경로 생성
            self._forward_only_until   = now + 2.0   # 최대 2초
            self._forward_only_dist    = 0.8          # 목표 0.8m 전진
            self._forward_only_start_x = self._state.x
            self._forward_only_start_y = self._state.y
            self._nav_state  = NavState.FORWARD_ONLY
            self._stuck_counter = 0
            self._path_local = None   # 기존 path 무효화
            self._path_progress_idx = 0
            self.get_logger().info(
                f"spin 완료 → FORWARD_ONLY 0.8m 전진 후 A* 재계획 "
                f"(forward_clear={forward_clear:.2f}m)")
        elif spin_unsafe:
            # 회전 자체가 위험 → BACKUP 또는 EMERGENCY
            bwd_clear = self._backward_clearance_inline(obstacles)
            if bwd_clear > self.p_safety_distance:
                self._nav_state = NavState.BACKUP
                self._backup_until = now + self.p_backup_duration
                self._stuck_counter = 0
                self.get_logger().warn(
                    f"spin 중 회전 위험 → BACKUP "
                    f"(arc={arc_clear:.2f}m, bwd={bwd_clear:.2f}m)")
            else:
                self._nav_state = NavState.EMERGENCY
                self._stuck_counter = 0
                self.get_logger().warn(
                    f"spin 중 전후방 막힘 → EMERGENCY")
        else:
            # spin_duration 초과인데 전방 미확보 → BACKUP 시도
            bwd_clear = self._backward_clearance_inline(obstacles)
            if bwd_clear > self.p_safety_distance:
                self._nav_state = NavState.BACKUP
                self._backup_until = now + self.p_backup_duration
                self._stuck_counter = 0
                self.get_logger().warn(
                    f"spin timeout → BACKUP "
                    f"(forward={forward_clear:.2f}m, bwd={bwd_clear:.2f}m)")
            else:
                self._nav_state = NavState.EMERGENCY
                self._stuck_counter = 0
                self.get_logger().warn(
                    f"spin timeout + 후방 막힘 → EMERGENCY")

    def _execute_forward_only(self) -> None:
        """NavState.FORWARD_ONLY — spin 완료 후 현재 heading으로 짧게 전진.

        목적: spin으로 heading을 바꾼 후 위치를 이동시켜야 A*가 다른 경로를 생성.
              제자리에서 재계획하면 A*가 또 같은 rack 방향 경로를 줄 수 있음.
        종료: 목표 거리 도달 or 시간 초과 or 전방 장애물 감지 → A* 재계획 대기
        """
        now = self._sec_now()
        obstacles = self._extract_obstacles_from_scan()

        # 전방 안전 체크
        forward_clear = self._trajectory_clearance_margin(
            obstacles, self.p_v_max * 0.3, 0.0)
        collision_near = forward_clear < self.p_clearance_stop_distance

        # 이동 거리 체크
        dist_moved = math.hypot(
            self._state.x - self._forward_only_start_x,
            self._state.y - self._forward_only_start_y)
        dist_reached = dist_moved >= self._forward_only_dist

        if (now < self._forward_only_until
                and not collision_near
                and not dist_reached):
            # 현재 heading 유지하며 직진
            period = 1.0 / max(self.p_control_rate, 1.0)
            dv_max = self.p_a_max * period
            v_target = self.p_v_max * 0.5   # 절반 속도로 안전하게
            v_cmd = max(self._state.v - dv_max,
                        min(self._state.v + dv_max, v_target))
            self._publish_cmd(VelocityCommand(v=v_cmd, w=0.0))
            self._publish_status_value("RECOVERY")
            self._log_state_throttled()
            return

        # 종료 → A* 재계획 대기
        self._forward_only_until = 0.0
        self._nav_state = NavState.NORMAL
        self._stuck_counter = 0
        self._recovery_cooldown_until = now + self.p_recovery_cooldown

        if dist_reached:
            reason = f"목표 거리 도달 ({dist_moved:.2f}m)"
        elif collision_near:
            reason = f"전방 장애물 (forward_clear={forward_clear:.2f}m)"
        else:
            reason = "시간 초과"

        self.get_logger().info(
            f"FORWARD_ONLY 완료 → A* 재계획 대기 ({reason})")

    def _execute_backup(self) -> None:
        """NavState.BACKUP — Recovery backup (후진)."""
        now = self._sec_now()
        obstacles = self._extract_obstacles_from_scan()

        if now < self._backup_until:
            bwd_clear = self._backward_clearance_inline(obstacles)
            if bwd_clear < self.p_safety_distance / 2.0:
                # 후방 위험 → EMERGENCY
                self._backup_until = 0.0
                self._nav_state = NavState.EMERGENCY
                self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
                self._publish_status_value("EMERGENCY")
                self.get_logger().warn(
                    f"backup 중 후방 위험 → EMERGENCY (bwd={bwd_clear:.2f}m)")
                return

            period = 1.0 / max(self.p_control_rate, 1.0)
            dv_max = self.p_a_max * period
            dw_max = self.p_alpha_max * period
            v_cmd_bk = max(self._state.v - dv_max,
                           min(self._state.v + dv_max, self.p_backup_velocity))
            w_target_bk = self._escape_turn_rate(obstacles)
            w_cmd_bk = max(self._state.w - dw_max,
                           min(self._state.w + dw_max, w_target_bk))
            self._publish_cmd(VelocityCommand(v=v_cmd_bk, w=w_cmd_bk))
            self._publish_status_value("RECOVERY")
            self._log_state_throttled()
            return

        # backup 완료 → NORMAL 복귀
        self._backup_until = 0.0
        self._nav_state = NavState.NORMAL
        self._stuck_counter = 0
        self._recovery_cooldown_until = now + self.p_recovery_cooldown
        self.get_logger().info(
            f"backup recovery 완료 → NORMAL "
            f"(cooldown {self.p_recovery_cooldown}s)")

    def _execute_emergency(self, ctx: dict) -> None:
        """NavState.EMERGENCY — 완전 정지 + A* 재계획 대기."""
        self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
        self._publish_status_value("EMERGENCY")
        self.get_logger().warn(
            "EMERGENCY: 전후방 막힘. A* 재계획 대기.",
            throttle_duration_sec=2.0)
        # 새 path 수신 시 _reset_path_state 에서 NORMAL 복귀

    # ───────────────────────────────────────────────────────────────
    # Recovery 트리거 — stuck 임계치 초과 시 호출
    # ───────────────────────────────────────────────────────────────
    def _trigger_recovery(self, obstacles_local, motion_clear: float) -> None:
        """Recovery 우선순위: SPIN → BACKUP → EMERGENCY."""
        now = self._sec_now()

        # Step 1: spin recovery 시도
        # 제자리 회전은 이동 없음 → 몸체(robot_radius)에 안 닿으면 회전 가능
        rotate_clear = self._rotation_clearance_inline(obstacles_local)
        if rotate_clear > self.p_robot_radius:
            self._spin_direction = (
                1.0 if self._escape_turn_rate(obstacles_local) >= 0.0 else -1.0
            )
            self._spin_until = now + self.p_spin_duration
            self._nav_state = NavState.SPIN
            self._stuck_counter = 0
            self._recovery_cooldown_until = self._spin_until + self.p_recovery_cooldown
            self.get_logger().warn(
                f"stuck → SPIN recovery {self.p_spin_duration}s "
                f"(rotate_clear={rotate_clear:.2f}m, "
                f"dir={'LEFT' if self._spin_direction > 0 else 'RIGHT'})")
            return

        # Step 2: backup 시도
        bwd_clear = self._backward_clearance_inline(obstacles_local)
        if bwd_clear > self.p_safety_distance:
            self._backup_until = now + self.p_backup_duration
            self._nav_state = NavState.BACKUP
            self._stuck_counter = 0
            self.get_logger().warn(
                f"stuck → BACKUP recovery {self.p_backup_duration}s "
                f"(bwd_clear={bwd_clear:.2f}m, motion_clear={motion_clear:.2f}m)")
            return

        # Step 3: 전후방 모두 막힘
        self._nav_state = NavState.EMERGENCY
        self._stuck_counter = 0
        self.get_logger().warn(
            f"stuck + 전후방 막힘 → EMERGENCY "
            f"(rotate_clear={rotate_clear:.2f}m, bwd_clear={bwd_clear:.2f}m). "
            f"A* 재계획 대기.",
            throttle_duration_sec=2.0)

    # ───────────────────────────────────────────────────────────────
    # Clearance 헬퍼들 (기존과 동일)
    # ───────────────────────────────────────────────────────────────
    def _forward_clearance_inline(
        self,
        obstacles_local: List[Tuple[float, float]],
        kappa: float = 0.0,
    ) -> float:
        """진행 arc 방향 기준 동적 부채꼴 (kappa 비례 각도)."""
        if not obstacles_local:
            return float("inf")
        half_angle = min(math.radians(50),
                         math.radians(20) + 0.8 * abs(kappa))
        best = float("inf")
        for ox, oy in obstacles_local:
            if ox < 0.0:
                continue
            ang = math.atan2(oy, ox)
            if abs(ang) > half_angle:
                continue
            d_eff = max(0.0, math.hypot(ox, oy) - self.p_robot_radius)
            if d_eff < best:
                best = d_eff
        return best

    def _trajectory_clearance_margin(
        self,
        obstacles_local: List[Tuple[float, float]],
        v_cmd: float,
        w_cmd: float,
    ) -> float:
        """현재 명령 arc 주변의 최단 여유 거리."""
        if not obstacles_local:
            return float("inf")
        sim_state = RobotState(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
        horizon = max(0.35, self.p_predict_horizon)
        traj = forward_simulate(sim_state, v_cmd, w_cmd, self.p_dt, horizon)
        if not traj:
            return float("inf")
        traj_xy = [(p[0], p[1]) for p in traj[2:]] or [(p[0], p[1]) for p in traj]
        center_dist = min_clearance_distance(traj_xy, obstacles_local)
        if center_dist == float("inf"):
            return float("inf")
        return max(0.0, center_dist - self.p_robot_radius)

    def _backward_clearance_inline(
        self,
        obstacles_local: List[Tuple[float, float]],
        half_angle: float = math.pi / 3.0,
    ) -> float:
        """후방 ±half_angle 부채꼴 최단 장애물 거리."""
        if not obstacles_local:
            return float("inf")
        best = float("inf")
        for ox, oy in obstacles_local:
            if ox > 0.0:
                continue
            ang = math.atan2(oy, -ox)
            if abs(ang) > half_angle:
                continue
            d_eff = max(0.0, math.hypot(ox, oy) - self.p_robot_radius)
            if d_eff < best:
                best = d_eff
        return best

    def _rotation_clearance_inline(
        self,
        obstacles_local: List[Tuple[float, float]],
    ) -> float:
        """제자리 회전 시 로봇 몸체 반경 기준 최단 장애물 거리.

        기존 전방향 safety_distance 체크의 문제:
          옆 벽(0.4m 거리)도 위험으로 판정 → rotate_clear < safety_distance
          → SPIN 불가 → 즉시 BACKUP → 뒤 벽 데드락.

        수정: 제자리 회전은 이동이 없으므로 몸체(robot_radius)에
          실제로 닿는 점만 위험. 옆 벽은 닿지 않으면 회전 가능.
        """
        if not obstacles_local:
            return float("inf")
        # 성능 필터: scan_range_max 대신 합리적 범위만 체크
        # robot_radius(0.2) * 8 = 1.6m — 제자리 회전에 영향 줄 수 있는 범위
        body_range = self.p_robot_radius * 8.0
        best = float("inf")
        for ox, oy in obstacles_local:
            d = math.hypot(ox, oy)
            if d < body_range and d < best:
                best = d
        return max(0.0, best - self.p_robot_radius)

    def _escape_turn_rate(self, obstacles_local: List[Tuple[float, float]]) -> float:
        """Recovery 중 벽 반대쪽 yaw rate."""
        if not obstacles_local:
            return 0.0
        side_bias = 0.0
        for ox, oy in obstacles_local:
            if ox < -0.20:
                continue
            d = math.hypot(ox, oy)
            if d < 1e-3 or d > 1.2:
                continue
            side_bias += (1.0 if oy >= 0.0 else -1.0) / d
        if abs(side_bias) < 1e-3:
            return 0.0
        target = -self.p_backup_turn_gain * side_bias
        return max(-self.p_backup_turn_max, min(self.p_backup_turn_max, target))

    # ───────────────────────────────────────────────────────────────
    # path 변환 (기존과 동일)
    # ───────────────────────────────────────────────────────────────
    def _transform_path_to_local(self, msg: Path) -> Optional[Path]:
        try:
            t = self._tf_buffer.lookup_transform(
                self.LOCAL_FRAME, self.GLOBAL_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.p_path_tf_timeout))
        except (TransformException, tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            if not self._path_warn_logged:
                self.get_logger().warn(
                    f"path 변환 TF lookup 실패: {e} (이전 path 유지)")
                self._path_warn_logged = True
            return None
        if self._path_warn_logged:
            self.get_logger().info("path 변환 TF 복구")
            self._path_warn_logged = False

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        q  = t.transform.rotation
        tyaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        cos_t = math.cos(tyaw)
        sin_t = math.sin(tyaw)

        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.LOCAL_FRAME

        from geometry_msgs.msg import PoseStamped
        for p_in in msg.poses:
            p_out = PoseStamped()
            p_out.header.stamp = out.header.stamp
            p_out.header.frame_id = self.LOCAL_FRAME
            px = p_in.pose.position.x
            py = p_in.pose.position.y
            p_out.pose.position.x = tx + cos_t * px - sin_t * py
            p_out.pose.position.y = ty + sin_t * px + cos_t * py
            p_out.pose.position.z = 0.0
            qi_x = p_in.pose.orientation.x
            qi_y = p_in.pose.orientation.y
            qi_z = p_in.pose.orientation.z
            qi_w = p_in.pose.orientation.w
            in_yaw = yaw_from_quaternion(qi_x, qi_y, qi_z, qi_w)
            out_yaw = in_yaw + tyaw
            p_out.pose.orientation.z = math.sin(out_yaw / 2.0)
            p_out.pose.orientation.w = math.cos(out_yaw / 2.0)
            out.poses.append(p_out)
        return out

    # ───────────────────────────────────────────────────────────────
    # 헬퍼
    # ───────────────────────────────────────────────────────────────
    def _sec_now(self) -> float:
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    @staticmethod
    def _path_xy(path_msg: Path) -> List[Tuple[float, float]]:
        return [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]

    def _extract_obstacles_from_scan(self) -> List[Tuple[float, float]]:
        scan = self._latest_scan
        if scan is None:
            return []
        points: List[Tuple[float, float]] = []
        angle = scan.angle_min
        increment = scan.angle_increment
        range_max = min(scan.range_max, float(self.p_scan_range_max))
        effective_min = max(scan.range_min, self.p_robot_radius)
        if scan.range_min <= 0:
            effective_min = max(0.05, self.p_robot_radius)
        for r in scan.ranges:
            if r != r or r < effective_min or r > range_max:
                angle += increment
                continue
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            points.append((lx + self.p_lidar_offset_x,
                           ly + self.p_lidar_offset_y))
            angle += increment
        return points

    # ───────────────────────────────────────────────────────────────
    # 진단 로그
    # ───────────────────────────────────────────────────────────────
    def _log_state_throttled(self) -> None:
        if self.p_state_log_period <= 0.0:
            return
        self._state_log_counter += 1
        target = int(self.p_control_rate * self.p_state_log_period)
        if self._state_log_counter >= max(1, target):
            self._state_log_counter = 0
            self.get_logger().info(
                f"DWA state [{self.LOCAL_FRAME}] [{self._nav_state.value}]: "
                f"({self._state.x:.2f}, {self._state.y:.2f}, "
                f"yaw={math.degrees(self._state.theta):.1f}°), "
                f"v={self._state.v:+.2f} w={self._state.w:+.2f}")

    def _goal_reached_logged_once(self) -> None:
        if not getattr(self, "_goal_logged", False):
            self.get_logger().info("goal 도착 — 정지")
            self._goal_logged = True

    # ───────────────────────────────────────────────────────────────
    # 시각화
    # ───────────────────────────────────────────────────────────────
    def _publish_best_trajectory(self,
                                 traj: List[Tuple[float, float, float]]) -> None:
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "dwa_best"; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 1.0
        m.lifetime.nanosec = int(0.3 * 1e9)
        for x, y, _ in traj:
            pt = Point(); pt.x = float(x); pt.y = float(y); pt.z = 0.02
            m.points.append(pt)
        self._best_pub.publish(m)

    # ───────────────────────────────────────────────────────────────
    # cmd_vel / status 발행
    # ───────────────────────────────────────────────────────────────
    def _publish_cmd(self, cmd: VelocityCommand) -> None:
        twist = Twist()
        twist.linear.x = float(cmd.v)
        twist.angular.z = float(cmd.w)
        self._cmd_pub.publish(twist)

    def _stop_robot(self, reason: str) -> None:
        cmd = Twist()
        self._cmd_pub.publish(cmd)
        self.get_logger().debug(f"정지: {reason}")

    def _publish_status_value(self, value: str) -> None:
        msg = String(); msg.data = value
        self._status_pub.publish(msg)

    def _publish_status(self) -> None:
        """1Hz 정기 status."""
        msg = String()
        if self._state is None:
            msg.data = "WAITING_ODOM"
        elif self._nav_state == NavState.REACHED:
            msg.data = "REACHED"
        elif self._path_local is None or not self._path_local.poses:
            msg.data = "STOPPED"
        else:
            msg.data = self._nav_state.value
        self._status_pub.publish(msg)


# ═══════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DwaPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
