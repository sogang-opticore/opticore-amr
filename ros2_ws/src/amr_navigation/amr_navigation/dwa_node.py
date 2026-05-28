#!/usr/bin/env python3
"""
DWA Local Planner Node — 2026-05-25 완전 재설계 (SW)

Opticore AMR — 직접 구현 DWA (Nav2 dwb_local_planner 의존 금지).

═══════════════════════════════════════════════════════════════════════
설계 원칙 (2026-05-25 재설계 핵심)
═══════════════════════════════════════════════════════════════════════

원칙 1) **DWA 의 모든 내부 계산은 odom_filtered frame 안에서**.
원칙 2) **매 cycle TF lookup 없음** — EKF /odometry/filtered 의 pose+twist 직접 사용.
원칙 3) **global_path 는 받을 때 한 번만** map → odom_filtered 로 변환·캐싱.
원칙 4) AMCL TF freeze / 보정 점프 의 영향이 cycle 단위로 안 들어옴.

이전 spiral bug 의 근본 원인:
    매 cycle lookup_transform(map → base_footprint) 가
    chain stale 또는 AMCL 보정 점프 영향으로 _state 가 들쭉날쭉 →
    world_to_local 에서 lookahead 점이 REAR 로 매핑 → 후진 trajectory 1등 → spiral.

새 설계의 효과:
    1) self._state.x/y/theta 는 EKF 의 50Hz 부드러운 값 → 점프 없음.
    2) 캐싱된 path 와 self._state 간의 lookahead 계산은 안정적.
    3) AMCL 의 큰 보정은 다음 A* 재발행 때 path 변환에 한 번만 반영.

비유:
    AMCL 은 매장 외부 GPS, EKF 는 차량 내부 속도계.
    GPS 가 가끔 끊겨도 속도계는 부드럽게 돌아감.
    DWA 는 속도계(EKF) 만 보고 운전, GPS(AMCL) 는 새 길 안내(A*) 받을 때만 참고.

═══════════════════════════════════════════════════════════════════════
순수 함수 (ROS 비의존, test_dwa.py 가 import)
═══════════════════════════════════════════════════════════════════════
    RobotState, DynamicWindow, VelocityCommand,
    compute_dynamic_window, sample_velocities, forward_simulate,
    heading_score, clearance_score, velocity_score,
    min_clearance_distance, pick_lookahead_point

설계 원칙 (sw_context.md):
    1. Nav2 의 dwb_local_planner 를 import 해서 쓰지 말 것.
    2. 미확정 값은 # TODO 명시 (예: alpha_max).
    3. 코드 주석은 한국어 우선.
    4. 파라미터는 declare_parameter 로 외부화.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
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
    x: float          # [m]
    y: float          # [m]
    theta: float      # [rad], yaw
    v: float          # [m/s], 본체 선속도
    w: float          # [rad/s], 본체 각속도


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


# ═══════════════════════════════════════════════════════════════════════
# 순수 알고리즘 함수 — 단위 테스트 가능
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
    """동역학 한계 ∩ 가속도 한계.

    비유: 자전거를 타는데 "다음 dt 초 동안 페달과 핸들로 만들 수 있는 속도/회전 범위".
    """
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
    """Dynamic Window 안에서 (v, w) 격자 샘플링."""
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
    """Unicycle model 로 (v, w) 유지 시 sim_time 동안 trajectory 적분."""
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
    """[기존] trajectory 끝점 yaw 와 goal 방향 정렬도 [0, 1].

    ⚠ DEPRECATED for DWA — test_dwa.py 호환을 위해 유지.
    회전 trajectory 우대 → 큰 회전 누적 문제. heading_score_dist 사용 권장.
    """
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
    """trajectory 끝점이 goal 에 얼마나 가까운지 (거리 기반)."""
    dx = goal_x - traj_end_x
    dy = goal_y - traj_end_y
    dist = math.sqrt(dx * dx + dy * dy)
    return max(0.0, 1.0 - dist / max_dist)


def path_tangent_score(traj_end_theta: float, path_tangent_local: float) -> float:
    """trajectory 끝점 yaw 가 path 진행 방향 (tangent) 과 정렬됐는지 [0, 1].

    [2026-05-25] 회전 trajectory 누적 문제의 결정타.

    문제: heading_score_dist (위치 정렬) 만으로는 trajectory 가 path 옆길로
    빠지는 회전 trajectory 를 막을 수 없음. lookahead 위치는 비슷해도 path
    진행 방향과 안 맞으면 다음 cycle 에 path 와 더 어긋남 → 누적 회전.

    해결: trajectory 끝점 yaw 가 path 진행 방향 (tangent) 과 일치하면 1.0.
    Pure Pursuit / Stanley controller 의 표준 heading control 항목.

    Args:
        traj_end_theta: trajectory 끝점 yaw (base_link frame)
        path_tangent_local: path 진행 방향 (base_link frame, atan2 형식)

    Returns:
        float [0, 1]. 둘이 정렬되면 1.0, 정반대면 0.
    """
    diff = path_tangent_local - traj_end_theta
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return 1.0 - abs(diff) / math.pi


def compute_path_tangent_local(path_xy: List[Tuple[float, float]],
                                start_idx: int,
                                look_ahead: int,
                                robot_x: float, robot_y: float,
                                robot_theta: float) -> float:
    """path 의 start_idx 점에서 look_ahead 점 앞까지의 진행 방향 (base_link frame).

    Args:
        path_xy: path 점 리스트 (odom_filtered frame, = self._path_local 좌표계)
        start_idx: 시작 idx (보통 nearest_idx)
        look_ahead: 몇 점 앞을 볼지 (5~10 권장)
        robot_x, robot_y, robot_theta: robot pose (odom_filtered frame)

    Returns:
        path tangent 방향 (base_link frame, [-pi, pi])
    """
    if not path_xy or start_idx >= len(path_xy):
        return 0.0
    target_idx = min(start_idx + look_ahead, len(path_xy) - 1)
    if target_idx <= start_idx:
        return 0.0
    px1, py1 = path_xy[start_idx]
    px2, py2 = path_xy[target_idx]
    # odom_filtered frame 진행 방향
    path_yaw_world = math.atan2(py2 - py1, px2 - px1)
    # base_link frame 으로 (yaw 차이)
    diff = path_yaw_world - robot_theta
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return diff


def clearance_score(trajectory_xy: List[Tuple[float, float]],
                    obstacle_points: List[Tuple[float, float]],
                    max_clearance: float = 1.0) -> float:
    """trajectory 위 점들에서 가장 가까운 장애물까지의 최소 거리 정규화 [0, 1]."""
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
    """선속도가 빠를수록 높은 점수 (정지 회피)."""
    if v_max <= 0.0:
        return 0.0
    return max(0.0, v) / v_max


def min_clearance_distance(trajectory_xy: List[Tuple[float, float]],
                           obstacle_points: List[Tuple[float, float]]) -> float:
    """trajectory 위 점들 중 장애물까지 최단 거리 [m]."""
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
    """robot_xy 와 가장 가까운 path 점의 idx. start_idx 이전 점은 검색 제외.

    monotonic forward progress 컨벤션: 한 번 지나간 path 점은 nearest 후보 X.
    이게 없으면 catmull-rom smoothing 또는 우회로 path 에서 자기 뒤 점이
    더 가까워지는 순간 lookahead 가 뒤로 점프 → REAR 매핑 → spiral.

    비유: 등산 가이드가 "지나온 길은 빼고, 앞으로 갈 길에서 제일 가까운 점이 어디?" 라고 묻는 것.
    """
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
    """Path 에서 lookahead_dist 만큼 앞쪽 점을 선택.

    Args:
        start_idx: 이 idx 이전 점은 nearest 후보에서 제외 (forward progress).
                   기본 0 — 기존 호출자(테스트) 호환.

    Returns:
        lookahead (x, y) 또는 None.
    """
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
    """quaternion → yaw (2D)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_local(world_xy: Tuple[float, float],
                   robot: RobotState) -> Tuple[float, float]:
    """world(=odom_filtered) frame 점 → robot 의 base_link frame.

    robot 의 위치/yaw 가 world frame 기준이라는 전제.
    """
    dx = world_xy[0] - robot.x
    dy = world_xy[1] - robot.y
    cos_t = math.cos(-robot.theta)
    sin_t = math.sin(-robot.theta)
    return (cos_t * dx - sin_t * dy, sin_t * dx + cos_t * dy)


# ═══════════════════════════════════════════════════════════════════════
# DWA Planner Node
# ═══════════════════════════════════════════════════════════════════════

class DwaPlannerNode(Node):
    """DWA Local Planner — odom_filtered frame 기반 설계 (2026-05-25 재설계).

    구독:
        /odometry/filtered   EKF 출력 (1순위)
        /odom                DiffDrive plugin 출력 (fallback, odom→odom_filtered 가정 정합)
        /global_path         A* 의 전역 경로 (frame: map)
        /lidar               장애물 점

    발행:
        /cmd_vel             제어 명령 (Twist)
        /dwa/trajectories    후보 trajectory (MarkerArray, base_link frame)
        /dwa/best_trajectory 선택된 trajectory (Marker, base_link frame)
        /dwa/status          상태 String (1Hz)
    """

    # ── 내부 frame 컨벤션 ──────────────────────────────────────────────
    # 모든 내부 상태/계산은 이 frame 으로 정규화.
    # EKF 가 발행하는 odometry 의 frame_id 와 일치해야 함.
    LOCAL_FRAME = "odom_filtered"
    GLOBAL_FRAME = "map"
    ROBOT_FRAME = "base_footprint"

    # ───────────────────────────────────────────────────────────────
    # __init__
    # ───────────────────────────────────────────────────────────────
    def __init__(self) -> None:
        super().__init__("dwa_planner")

        # ── 파라미터 선언 ─────────────────────────────────────────────
        # 로봇 동역학 한계 (Notion 명세 §4.1)
        # TODO(디스코드 N-2): v_max 1.5 임시값. 명세는 2.0. 팀 합의 후 갱신.
        self.declare_parameter("v_max", 1.5)
        self.declare_parameter("v_min", -1.0)
        self.declare_parameter("w_max", 1.5)
        self.declare_parameter("a_max", 1.0)
        # TODO: alpha_max 명세 미명시. 시뮬 측정 후 갱신.
        self.declare_parameter("alpha_max", 1.5)
        # 횡가속 한계 (Pure Pursuit v_curve 캐핑용)
        # 명세 미명시. 기본 = a_max (보수적). 시뮬 측정 후 a_max 의 1.5~2배 가능.
        self.declare_parameter("a_lat_max", 1.0)

        # 샘플링 / 시뮬레이션
        self.declare_parameter("sample_v_n", 11)
        self.declare_parameter("sample_w_n", 21)
        self.declare_parameter("predict_horizon", 1.0)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("control_rate", 20.0)

        # 평가함수 가중치
        self.declare_parameter("weight_heading", 0.6)        # heading_score_dist (위치 정렬)
        self.declare_parameter("weight_clearance", 1.2)
        self.declare_parameter("weight_velocity", 0.2)
        # 2026-05-25 추가: path tangent 정렬 (회전 trajectory 누적 방지)
        # trajectory 끝점 yaw 가 path 진행 방향과 정렬되면 점수 가산.
        # Pure Pursuit / Stanley controller 의 heading control 항목.
        self.declare_parameter("weight_path_tangent", 0.8)
        # path tangent 계산 시 nearest_idx 부터 몇 점 앞을 볼지
        self.declare_parameter("path_tangent_lookahead", 10)

        # 추종 / 평가 보조
        self.declare_parameter("lookahead_dist", 1.0)
        # 17차 (Adaptive Lookahead, 표준 Pure Pursuit 확장 — Nav2 regulated PP 패턴):
        # L = max(lookahead_dist, lookahead_time · |v|)
        # 정지 = L_min, 직진 1.5m/s = 0.7·1.5 = 1.05m → path 평균 방향 따라감.
        # 비유: 고속도로일수록 멀리 봐야 함. 잔 굴곡에 핸들 휙휙 안 함.
        self.declare_parameter("lookahead_time", 0.7)   # s, k = 시간 상수
        self.declare_parameter("max_clearance", 1.0)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("clearance_slowdown_distance", 0.80)
        self.declare_parameter("clearance_stop_distance", 0.30)

        # In-place rotation 모드 (2026-05-25, 13차 overshoot stop + cooldown)
        # 표준 Pure Pursuit 의 forward velocity ramp + heading deadband + overshoot stop.
        self.declare_parameter("align_angle_thresh", 0.785)  # rad ≈ 45° (진입)
        self.declare_parameter("align_angle_exit", 0.262)    # rad ≈ 15° (종료, hysteresis)
        self.declare_parameter("align_kp", 1.5)              # P gain
        self.declare_parameter("align_kd", 0.5)              # D gain
        self.declare_parameter("align_v_blend_max", 0.3)     # m/s, blending v 상한
        # Cooldown: align mode 종료 후 이 시간 동안 재진입 금지.
        # normal DWA 가 직진 trajectory 안정적으로 선택할 시간 확보 → 진동 차단.
        self.declare_parameter("align_cooldown", 1.0)        # s

        # 후진 / Path offset 안전장치 (2026-05-25 추가, CLAUDE.md §0.1 원칙 8 발동)
        # 먼 거리 시나리오 (20m+) 에서 발견된 두 문제:
        #   (a) heading_score 가 후진 trajectory 도 만점 평가 → 모든 전진이 충돌일 때
        #       후진이 1등 선택 → path 옆길로 빠짐. allow_backward=false 로 후진 자체
        #       sample 에서 제외. EMERGENCY 가 정직하게 표시되어 A* 재계획 유도.
        #   (b) path 옆길로 한 번 빠지면 lookahead 가 멀어지고 lateral error 누적 →
        #       max_path_offset 으로 안전 정지. 표준 path follower (Pure Pursuit) 의
        #       cross-track error 안전장치.
        self.declare_parameter("allow_backward", False)
        self.declare_parameter("max_path_offset", 2.0)   # m, path 와 이 이상 떨어지면 정지

        # Stuck recovery (16차, 표준 Nav2 BackUp 패턴)
        # v ≈ 0 + fwd_clear < safety 가 stuck_recovery_sec 이상 지속되면
        # 후방 clearance 체크 후 backup_duration 동안 backup_velocity 로 후진.
        # allow_backward 와 별개 (recovery 는 항상 가능).
        self.declare_parameter("stuck_recovery_sec", 1.5)   # s, stuck 판정 시간
        self.declare_parameter("backup_velocity", -0.25)    # m/s, 후진 속도 (음수)
        self.declare_parameter("backup_duration", 1.5)      # s, 후진 지속 시간
        self.declare_parameter("backup_turn_gain", 0.8)     # rad/s, 벽 반대쪽 회전 강도
        self.declare_parameter("backup_turn_max", 0.45)     # rad/s, recovery 중 최대 회전
        # 18차 추가: backup 완료 후 stuck 재판정 안 하는 시간.
        # A* 1Hz 재계획 + PP 새 path 시도 + adaptive lookahead 안정화 시간 확보.
        # 비유: 막다른 골목 후진 후 "내비 재탐색" 기다림. 바로 또 박지 말 것.
        self.declare_parameter("recovery_cooldown", 3.0)    # s, recovery 완료 후 cooldown

        # 안전
        self.declare_parameter("robot_radius", 0.20)
        self.declare_parameter("hard_collision_distance", 0.05)
        self.declare_parameter("safety_distance", 0.30)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("scan_range_max", 25.0)

        # LiDAR mount offset (URDF lidar_link → base_link, 2026-05-25 추가)
        # URDF: lidar_link 가 base_link 의 (+0.25, 0, +h) 에 yaw=0 으로 mount.
        # _extract_obstacles_from_scan() 이 lidar frame 점을 base_link frame 으로
        # 변환하려면 +x 평행이동 필요. 미반영 시 장애물이 실제보다 0.25m 가깝게
        # 인식되어 trajectory reject 가 너무 보수적 → stuck.
        # 비유: 자(尺) 길이를 잘못 알고 있어 1m 벽을 0.5m 라고 착각하는 운전자.
        self.declare_parameter("lidar_offset_x", 0.25)   # m, URDF lidar_x 와 일치
        self.declare_parameter("lidar_offset_y", 0.0)    # m, lidar y 오프셋 (=0)

        # 토픽명
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("odom_fallback_topic", "/odom")
        self.declare_parameter("global_path_topic", "/global_path")
        self.declare_parameter("scan_topic", "/lidar")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # 로그 throttle (cycle 단위)
        self.declare_parameter("state_log_period", 1.0)
        self.declare_parameter("candidate_log_period", 1.0)

        # path 변환 TF lookup 의 timeout (받을 때만 사용, cycle 안에서는 안 함)
        self.declare_parameter("path_tf_timeout", 1.0)

        self._load_params()

        # ── 상태 변수 ────────────────────────────────────────────────
        # _state: 로봇 pose+twist (LOCAL_FRAME = odom_filtered 기준)
        self._state: Optional[RobotState] = None
        # _path_local: global_path 를 LOCAL_FRAME 으로 변환한 캐싱본
        self._path_local: Optional[Path] = None
        self._latest_scan: Optional[LaserScan] = None
        self._last_odom_time: Optional[float] = None
        self._last_odom_was_fallback = False

        # 진단/플래그
        self._reached = False
        self._state_log_counter = 0
        self._candidate_log_counter = 0
        self._path_warn_logged = False

        # path following monotonic forward progress
        # 매 cycle 의 nearest_idx 가 이 값 이전으로 안 가도록.
        # 새 path 받으면 0 으로 리셋.
        self._path_progress_idx = 0

        # align mode 의 hysteresis 상태 — 한 번 진입하면 exit_thresh 까지 유지
        # ⚠ 14차 (2026-05-25, Pure Pursuit 전환) 이후 unused — 호환 위해 잔존.
        #   Pure Pursuit 의 곡률 공식 κ=2y/L² 가 lateral error 0 일 때 w=0 자동 보장.
        self._in_align_mode = False
        # align mode 종료 후 cooldown 만료 시각 (sim_time)
        self._align_cooldown_until = 0.0

        # 14차 (Pure Pursuit 전환): stuck recovery 카운터
        # control_loop 가 collision imminent 로 정지할 때마다 +1, 정상 cmd 발행 시 0 리셋.
        # stuck_recovery_cycles 이상 누적되면 backup recovery 발동.
        self._stuck_counter = 0

        # 16차 (벽 stuck 탈출, 표준 Nav2 Backup 패턴):
        #   stuck 시 후방 free 면 후진 N초 → A* 재계획 자동 수신 → 새 path 재시도.
        #   0 = backup 비활성. > 0 = sim_time 기준 backup 종료 시각.
        self._backup_until = 0.0

        # 18차 (무한 recovery loop 차단): backup 완료 후 stuck 재판정 안 하는 시간.
        # A* 재계획 + PP 새 path 시도 + adaptive lookahead 안정화 시간.
        self._recovery_cooldown_until = 0.0

        # ── TF buffer (path 변환에만 사용) ──────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ──────────────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=10,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT)
        # 2026-05-24 통합: HU astar_node 는 /global_path 를 RELIABLE/VOLATILE 로 발행.
        # README 계약은 TRANSIENT_LOCAL 권장이지만, 호환 위해 RELIABLE 로.
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
        self._cmd_pub = self.create_publisher(
            Twist, self.p_cmd_vel_topic, 10)
        self._traj_pub = self.create_publisher(
            MarkerArray, "/dwa/trajectories", 10)
        self._best_pub = self.create_publisher(
            Marker, "/dwa/best_trajectory", 10)
        self._status_pub = self.create_publisher(String, "/dwa/status", 10)

        # ── 타이머 ───────────────────────────────────────────────────
        period = 1.0 / max(self.p_control_rate, 1.0)
        self._control_timer = self.create_timer(period, self._control_loop)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"DWA Planner 시작 (재설계 2026-05-25) — "
            f"v_max={self.p_v_max} w_max={self.p_w_max} a_max={self.p_a_max} "
            f"control_rate={self.p_control_rate}Hz "
            f"LOCAL_FRAME={self.LOCAL_FRAME}"
        )

    # ───────────────────────────────────────────────────────────────
    # 파라미터 로드
    # ───────────────────────────────────────────────────────────────
    def _load_params(self) -> None:
        gp = self.get_parameter
        self.p_v_max = gp("v_max").value
        self.p_v_min = gp("v_min").value
        self.p_w_max = gp("w_max").value
        self.p_a_max = gp("a_max").value
        self.p_alpha_max = gp("alpha_max").value
        self.p_a_lat_max = gp("a_lat_max").value

        self.p_sample_v_n = gp("sample_v_n").value
        self.p_sample_w_n = gp("sample_w_n").value
        self.p_predict_horizon = gp("predict_horizon").value
        self.p_dt = gp("dt").value
        self.p_control_rate = gp("control_rate").value

        self.p_w_heading = gp("weight_heading").value
        self.p_w_clearance = gp("weight_clearance").value
        self.p_w_velocity = gp("weight_velocity").value
        self.p_w_path_tangent = gp("weight_path_tangent").value
        self.p_path_tangent_lookahead = gp("path_tangent_lookahead").value

        self.p_lookahead_dist = gp("lookahead_dist").value
        self.p_lookahead_time = gp("lookahead_time").value
        self.p_max_clearance = gp("max_clearance").value
        self.p_goal_tolerance = gp("goal_tolerance").value
        self.p_clearance_slowdown_distance = gp("clearance_slowdown_distance").value
        self.p_clearance_stop_distance = gp("clearance_stop_distance").value
        self.p_align_angle_thresh = gp("align_angle_thresh").value
        self.p_align_angle_exit = gp("align_angle_exit").value
        self.p_align_kp = gp("align_kp").value
        self.p_align_kd = gp("align_kd").value
        self.p_align_v_blend_max = gp("align_v_blend_max").value
        self.p_align_cooldown = gp("align_cooldown").value
        self.p_allow_backward = gp("allow_backward").value
        self.p_max_path_offset = gp("max_path_offset").value
        self.p_stuck_recovery_sec = gp("stuck_recovery_sec").value
        self.p_backup_velocity = gp("backup_velocity").value
        self.p_backup_duration = gp("backup_duration").value
        self.p_backup_turn_gain = gp("backup_turn_gain").value
        self.p_backup_turn_max = gp("backup_turn_max").value
        self.p_recovery_cooldown = gp("recovery_cooldown").value
        self.p_lidar_offset_x = gp("lidar_offset_x").value
        self.p_lidar_offset_y = gp("lidar_offset_y").value

        self.p_robot_radius = gp("robot_radius").value
        self.p_hard_collision_distance = gp("hard_collision_distance").value
        self.p_safety_distance = gp("safety_distance").value
        self.p_odom_timeout = gp("odom_timeout").value
        self.p_scan_range_max = gp("scan_range_max").value

        self.p_odom_topic = gp("odom_topic").value
        self.p_odom_fallback_topic = gp("odom_fallback_topic").value
        self.p_global_path_topic = gp("global_path_topic").value
        self.p_scan_topic = gp("scan_topic").value
        self.p_cmd_vel_topic = gp("cmd_vel_topic").value

        self.p_state_log_period = gp("state_log_period").value
        self.p_candidate_log_period = gp("candidate_log_period").value
        self.p_path_tf_timeout = gp("path_tf_timeout").value

    # ───────────────────────────────────────────────────────────────
    # 콜백
    # ───────────────────────────────────────────────────────────────
    def _on_odom(self, msg: Odometry) -> None:
        """EKF /odometry/filtered — pose+twist 직접 사용 (LOCAL_FRAME 기준)."""
        self._update_state_from_odom(msg, is_fallback=False)
        self._last_odom_time = self._sec_now()

    def _on_odom_fallback(self, msg: Odometry) -> None:
        """/odom — EKF 타임아웃 시에만 사용.

        ⚠ frame 차이: /odom 은 frame=odom, /odometry/filtered 는 odom_filtered.
        둘은 같은 origin 에서 시작했고 EKF 가 다리만 보강한 거라 정합. fallback 으로
        들어와도 trajectory 계산은 가능하지만, AMCL TF (map → odom_filtered) 가
        제대로 매핑돼야 path 변환이 정확. fallback 모드에서 path 변환은 위험.
        """
        now = self._sec_now()
        if self._last_odom_time is None:
            self._update_state_from_odom(msg, is_fallback=True)
            self._last_odom_time = now
            return
        if now - self._last_odom_time > self.p_odom_timeout:
            if not self._last_odom_was_fallback:
                self.get_logger().warn(
                    "EKF /odometry/filtered timeout → /odom fallback 사용 "
                    "(주의: AMCL path 변환은 odom_filtered frame 가정)"
                )
            self._update_state_from_odom(msg, is_fallback=True)
            self._last_odom_time = now

    def _update_state_from_odom(self, msg: Odometry, *, is_fallback: bool) -> None:
        """Odometry → RobotState 변환.

        pose 와 twist 둘 다 odometry 메시지에서 직접 읽음. TF lookup 없음.
        odometry 의 pose 는 odometry.header.frame_id 기준 (= odom_filtered 또는 odom).
        twist 는 child_frame_id (= base_footprint) 기준의 본체 속도.

        주의: frame_id 검증.
        """
        pose = msg.pose.pose
        twist = msg.twist.twist

        # EKF 의 frame_id 가 LOCAL_FRAME 과 다르면 한 번 WARN
        if not is_fallback and msg.header.frame_id != self.LOCAL_FRAME:
            self.get_logger().warn(
                f"/odometry/filtered frame_id='{msg.header.frame_id}' ≠ '{self.LOCAL_FRAME}' "
                f"— path 변환 좌표계 불일치 가능. ekf.yaml 의 world_frame 확인.",
                throttle_duration_sec=10.0
            )

        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        v = twist.linear.x
        w = twist.angular.z

        if self._state is None:
            self._state = RobotState(x=x, y=y, theta=theta, v=v, w=w)
        else:
            self._state.x = x
            self._state.y = y
            self._state.theta = theta
            self._state.v = v
            self._state.w = w

        self._last_odom_was_fallback = is_fallback

    def _on_global_path(self, msg: Path) -> None:
        """global_path 받음 — map → LOCAL_FRAME 으로 한 번 변환 후 캐싱.

        매 cycle 변환하지 않고 받을 때 한 번만 변환 → cycle 안 TF lookup 0번.
        """
        # 빈 path = 실패 컨벤션 (README §5)
        if not msg.poses:
            if self._path_local is None or self._path_local.poses:
                self.get_logger().warn("빈 /global_path 수신 — DWA 정지 모드")
            empty = Path()
            empty.header.frame_id = self.LOCAL_FRAME
            empty.header.stamp = self.get_clock().now().to_msg()
            self._path_local = empty
            self._reached = False
            return

        # frame_id 확인
        src_frame = msg.header.frame_id or self.GLOBAL_FRAME
        if src_frame == self.LOCAL_FRAME:
            # 이미 LOCAL_FRAME 이면 그대로 캐싱
            self._path_local = msg
            self._reached = False
            self._path_progress_idx = 0   # 새 path 받을 때 progress 리셋
            self._in_align_mode = False   # align mode 도 리셋
            last = msg.poses[-1].pose.position
            self.get_logger().info(
                f"/global_path 수신 ({src_frame}, 변환 불필요) — "
                f"{len(msg.poses)}점, goal=({last.x:.2f},{last.y:.2f})"
            )
            return

        if src_frame != self.GLOBAL_FRAME:
            self.get_logger().warn(
                f"/global_path frame_id='{src_frame}' "
                f"— '{self.GLOBAL_FRAME}' 또는 '{self.LOCAL_FRAME}' 만 지원. 무시.")
            return

        # map → LOCAL_FRAME 변환 (한 번만)
        transformed = self._transform_path_to_local(msg)
        if transformed is None:
            # 변환 실패 — 이전 캐싱본 유지 (또는 초기엔 None)
            return

        self._path_local = transformed
        self._reached = False
        self._path_progress_idx = 0   # 새 path 받을 때 progress 리셋
        self._in_align_mode = False   # align mode 도 리셋
        last = transformed.poses[-1].pose.position
        self.get_logger().info(
            f"/global_path 수신 ({src_frame} → {self.LOCAL_FRAME} 변환) — "
            f"{len(transformed.poses)}점, "
            f"goal=({last.x:.2f},{last.y:.2f}) [{self.LOCAL_FRAME}]"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    # ───────────────────────────────────────────────────────────────
    # path 변환 — map → odom_filtered
    # ───────────────────────────────────────────────────────────────
    def _transform_path_to_local(self, msg: Path) -> Optional[Path]:
        """nav_msgs/Path 의 모든 point 를 map → LOCAL_FRAME 으로 변환.

        한 번의 TF lookup 만 수행 (path 의 모든 점은 같은 transform 적용).
        timeout 까지 새 TF 를 기다림 — AMCL publish 가 조금 늦어도 OK.

        실패 시 None 반환 (caller 가 이전 캐싱본 유지 결정).
        """
        try:
            t = self._tf_buffer.lookup_transform(
                self.LOCAL_FRAME,           # target
                self.GLOBAL_FRAME,           # source
                rclpy.time.Time(),           # latest
                timeout=rclpy.duration.Duration(seconds=self.p_path_tf_timeout),
            )
        except (TransformException,
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            if not self._path_warn_logged:
                self.get_logger().warn(
                    f"path 변환 TF lookup 실패 "
                    f"({self.GLOBAL_FRAME} → {self.LOCAL_FRAME}): {e} "
                    f"(이전 path 유지)"
                )
                self._path_warn_logged = True
            return None

        if self._path_warn_logged:
            self.get_logger().info("path 변환 TF 복구")
            self._path_warn_logged = False

        # TF 의 translation/rotation
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        q = t.transform.rotation
        tyaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        cos_t = math.cos(tyaw)
        sin_t = math.sin(tyaw)

        # 변환된 path 생성
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.LOCAL_FRAME

        from geometry_msgs.msg import PoseStamped
        for p_in in msg.poses:
            p_out = PoseStamped()
            p_out.header.stamp = out.header.stamp
            p_out.header.frame_id = self.LOCAL_FRAME

            # 점 좌표만 변환 (DWA 는 점만 보고 lookahead 결정)
            px = p_in.pose.position.x
            py = p_in.pose.position.y
            p_out.pose.position.x = tx + cos_t * px - sin_t * py
            p_out.pose.position.y = ty + sin_t * px + cos_t * py
            p_out.pose.position.z = 0.0
            # orientation 도 변환 (path 끝점의 방향 정보 보존)
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
        """LaserScan → base_link 기준 (x, y) 점 리스트.

        URDF: lidar_link 가 base_link 의 +0.25m (lidar_offset_x) 에 yaw=0 으로 mount.
        lidar frame 의 점을 base_link frame 으로 변환:
            base_link_x = lidar_x + lidar_offset_x
            base_link_y = lidar_y + lidar_offset_y
        (yaw=0 이라 회전 변환 불필요)

        ★ 2026-05-25 정정 (CLAUDE.md §0.1 원칙 8 발동):
        이전 코드는 offset 미반영. lidar 가 1m 앞 점 잡으면 코드가 "base_link 1m 앞"
        으로 해석 — 실제는 1.25m 앞. → trajectory reject 가 보수적 → stuck.
        """
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
            # lidar frame 의 점
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            # base_link frame 으로 변환 (lidar 가 base_link 의 +offset 에 yaw=0 mount)
            bx = lx + self.p_lidar_offset_x
            by = ly + self.p_lidar_offset_y
            points.append((bx, by))
            angle += increment
        return points

    # ───────────────────────────────────────────────────────────────
    # 메인 제어 루프 — Pure Pursuit + Adaptive Velocity Profile
    # (2026-05-25 14차, 표준 modern AMR 컨트롤러로 완전 전환)
    # ───────────────────────────────────────────────────────────────
    #
    # ╔══════════════════════════════════════════════════════════════╗
    # ║ 왜 이 컨트롤러로 바꿨나                                       ║
    # ╚══════════════════════════════════════════════════════════════╝
    # 1~13차 patch chain (DWA score-tuning, align mode PD, hysteresis,
    # overshoot stop, cooldown) 가 모두 같은 증상을 다른 각도에서 패치하다
    # 진동·과회전·wall stuck 를 만들었다 (CLAUDE.md §0.1 원칙 8 발동 2회째).
    #
    # 표준 AMR motion controller (Coulter 1992, 이후 ROS Nav1/Nav2,
    # autoware, Stanley 등 모든 산업 구현의 뼈대) 의 단일 구조로 통일:
    #
    #     w = κ · v         (Pure Pursuit 곡률 추종)
    #     κ = 2y / L²       (표준 공식)
    #     v = min(v_max, v_curve, v_clearance, v_goal, v_heading)
    #
    # 이 구조의 결정적 장점 — 사용자 3개 요구가 알고리즘 자체에 내재:
    #
    # 1) "정렬되면 회전 자동 정지":
    #    lookahead 의 lateral error y → 0 이면 κ → 0 이고 w = κ·v = 0.
    #    PD/hysteresis/overshoot/cooldown 같은 "정지 시키는 로직" 불필요.
    #    수학적으로 y=0 인 순간 w=0 보장.
    #
    # 2) "최대한 빠르게 경로 도달":
    #    v 는 직선 구간에선 v_max, 코너에선 횡가속 한계로 자동 감속,
    #    장애물 가까이엔 clearance 비례 감속, goal 가까이엔 정확한
    #    제동거리 공식 (v = √(2·a·d)) 으로 감속.
    #    "필요한 만큼만 늦추고 나머지는 풀가속" — 표준 velocity profile.
    #
    # 3) "벽 부딪혀 stuck":
    #    forward clearance scaling 으로 벽 가까이에서 v 자동 감속,
    #    DWA-style 적분 collision check 로 임박한 충돌 정지,
    #    stuck_counter 누적 시 EMERGENCY status → A* 재계획 유도.
    #
    # ╔══════════════════════════════════════════════════════════════╗
    # ║ 비유                                                          ║
    # ╚══════════════════════════════════════════════════════════════╝
    # 자동차 운전자:
    #   - 핸들 = Pure Pursuit (멀리 점 보고 그쪽으로 자동 조향)
    #   - 액셀 = adaptive v (직선 풀가속, 코너 감속, 도착 감속)
    #   - 브레이크 = collision check (충돌 직전 무조건 정지)
    #   - 후진/탈출 = stuck recovery (막다른 길에서 A* 재계획 요청)
    # ───────────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        # ── 1) state 검증 ──────────────────────────────────────────
        if self._state is None:
            return
        if (self._last_odom_time is None or
                self._sec_now() - self._last_odom_time > self.p_odom_timeout):
            self._stop_robot("odom_timeout")
            return

        # ── 1.5) Backup Recovery 처리 (다른 모든 로직 우선) ───────
        # 16차 추가. stuck recovery 활성 중이면 다른 모든 로직 무시하고 후진.
        # backup 완료 후 A* 의 1Hz 재계획이 새 path 발행 → 자동 재시도.
        # 비유: 막다른 골목에 코박았을 때 운전자가 다른 모든 결정 무시하고 후진하는 것.
        if self._backup_until > 0.0:
            if self._sec_now() < self._backup_until:
                # 후진 중 — 후방 collision check 도 계속
                obstacles_local_bk = self._extract_obstacles_from_scan()
                bwd_clear = self._backward_clearance_inline(obstacles_local_bk)
                if bwd_clear < self.p_safety_distance / 2.0:
                    # 후방 위험 → backup 즉시 중단
                    self._backup_until = 0.0
                    self._stuck_counter = 0
                    self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
                    self._publish_status_value("EMERGENCY")
                    self.get_logger().warn(
                        f"backup 중단: 후방 가까워짐 (bwd_clear={bwd_clear:.2f}m)",
                        throttle_duration_sec=2.0
                    )
                    return
                # 후진 명령 (가속도 한계)
                period = 1.0 / max(self.p_control_rate, 1.0)
                dv_max = self.p_a_max * period
                v_cmd_bk = max(self._state.v - dv_max,
                               min(self._state.v + dv_max,
                                   self.p_backup_velocity))
                w_target_bk = self._escape_turn_rate(obstacles_local_bk)
                dw_max = self.p_alpha_max * period
                w_cmd_bk = max(self._state.w - dw_max,
                               min(self._state.w + dw_max,
                                   w_target_bk))
                self._publish_cmd(VelocityCommand(v=v_cmd_bk, w=w_cmd_bk))
                self._publish_status_value("RECOVERY")
                self._log_state_throttled()
                return
            else:
                # backup 만료 → 다음 cycle 부터 정상 동작
                self._backup_until = 0.0
                self._stuck_counter = 0
                # 18차: cooldown 시작. 이 시간 동안 stuck 재판정 안 함 →
                # A* 재계획 1Hz × cooldown 횟수만큼 새 path 시도 가능.
                self._recovery_cooldown_until = (self._sec_now()
                                                 + self.p_recovery_cooldown)
                self.get_logger().info(
                    f"backup recovery 완료 — cooldown {self.p_recovery_cooldown}s "
                    f"동안 path 재시도 (A* 재계획 대기)"
                )

        # ── 2) path 검증 ───────────────────────────────────────────
        if self._path_local is None or not self._path_local.poses:
            self._stop_robot("no_global_path")
            return

        path_xy = self._path_xy(self._path_local)

        # ── 3) 도착 판정 + REACHED hold ───────────────────────────
        # dist_to_goal jitter (관성+EKF noise) 로 reached 가 깜빡거리면
        # path[-1] 이 자기 뒤로 매핑돼 다시 회전 → 한 번 reached 면 새 path
        # 도착 전까지 stop hold. (5차 진단 결과 유지)
        gx, gy = path_xy[-1]
        dist_to_goal = math.hypot(gx - self._state.x, gy - self._state.y)

        if self._reached:
            self._stop_robot("reached_hold")
            return

        if dist_to_goal < self.p_goal_tolerance:
            self._stop_robot("goal_reached")
            self._reached = True
            self._goal_reached_logged_once()
            return

        # ── 4) nearest_idx (monotonic) + path lateral offset 안전 ─
        nearest_idx = find_nearest_idx(
            path_xy, (self._state.x, self._state.y), self._path_progress_idx)
        self._path_progress_idx = nearest_idx

        nx, ny = path_xy[nearest_idx]
        path_offset = math.hypot(nx - self._state.x, ny - self._state.y)
        if path_offset > self.p_max_path_offset:
            self._stop_robot("path_offset_too_large")
            self._publish_status_value("PATH_LOST")
            self.get_logger().warn(
                f"path 와 {path_offset:.2f}m 떨어짐 (max={self.p_max_path_offset}m). "
                f"정지. 새 goal/path 발행 필요.",
                throttle_duration_sec=2.0
            )
            return

        # ── 5) Lookahead 점 선택 + base_link frame 변환 ───────────
        # 17차: Adaptive Lookahead (표준 Pure Pursuit 확장)
        #   L = max(L_min, k·|v|). v 빠르면 멀리 봐서 path 평균 방향 따라감.
        #   Nav2 의 regulated_pure_pursuit_controller 표준 패턴.
        effective_lookahead = max(
            self.p_lookahead_dist,
            self.p_lookahead_time * abs(self._state.v)
        )
        lookahead = pick_lookahead_point(
            path_xy, (self._state.x, self._state.y),
            effective_lookahead, start_idx=nearest_idx)
        if lookahead is None:
            self._stop_robot("no_lookahead")
            return

        lx, ly = world_to_local(lookahead, self._state)
        L = math.hypot(lx, ly)
        alpha = math.atan2(ly, lx)   # lookahead 방위각 [-π, π]

        # ── 6) 장애물 + forward clearance ─────────────────────────
        obstacles_local = self._extract_obstacles_from_scan()
        fwd_clear = self._forward_clearance_inline(obstacles_local)
        motion_clear = fwd_clear

        # ── 7) 제어 분기 ──────────────────────────────────────────
        # 큰 heading 오차 (|α| > thresh) 면 Pure Pursuit 부적합 →
        # 정지하고 in-place 회전. thresh 이내면 표준 Pure Pursuit.
        period = 1.0 / max(self.p_control_rate, 1.0)
        dv_max = self.p_a_max * period
        dw_max = self.p_alpha_max * period

        # ── 7) 모드 결정 — Hysteresis + PD (15차 추가, oscillation 차단) ──
        # 14차 P-only ROTATE 의 oscillation 분석 (시나리오 (0, 5) 진단):
        #   α ≈ ±90° 부근에서 lookahead 점의 좌/우 부호가 wraparound →
        #   P 출력 부호 휙휙 → 가속도 한계로 정지 못함 → overshoot →
        #   반대방향 가속 → 또 wraparound → oscillation.
        #
        # 15차 해결:
        #   1) PD 의 D 항 (kd·w) 가 회전 관성 잡음 → critical damping.
        #      α 부호 바뀌어도 현재 w 가 크면 D 가 P 출력 상쇄 → 매끄러운 감속.
        #   2) Hysteresis (45° in, 15° out): 진입 후 |α| < exit 까지 ROTATE 유지.
        #      모드 자체가 휙휙 바뀌는 것 방지. 13차 align_angle_exit 재사용.
        #
        # 비유: P-only = 핸들 휙휙. PD + hysteresis = 핸들 + ABS 브레이크 + 데드존.
        if self._in_align_mode:
            # 종료: |α| 가 exit 이하면 mode 해제 → PP 인계
            if abs(alpha) < self.p_align_angle_exit:
                self._in_align_mode = False
        else:
            # 진입: |α| > thresh + 의미 있는 lookahead 거리
            if abs(alpha) > self.p_align_angle_thresh and L > 0.1:
                self._in_align_mode = True

        if self._in_align_mode:
            # ── 7a) In-place rotation (PD 제어) ───────────────────
            # PP 의 κ 공식은 |y| 크고 |x| 작을 때 unstable → 이 영역만 정지 회전.
            v_target = 0.0
            if (self.p_align_v_blend_max > 0.0 and
                    fwd_clear > self.p_clearance_slowdown_distance and
                    abs(alpha) < math.radians(75.0)):
                # 큰 heading error 라도 앞이 충분히 열려 있으면 아주 천천히 전진.
                # 완전 정지 회전만 반복할 때보다 path 인계가 빨라진다.
                blend = 1.0 - (abs(alpha) - self.p_align_angle_exit) / \
                    max(math.radians(75.0) - self.p_align_angle_exit, 1e-3)
                v_target = self.p_align_v_blend_max * max(0.0, min(1.0, blend))
            # PD: P × α  -  D × w_current  (회전 관성 잡기)
            w_target = (self.p_align_kp * alpha
                        - self.p_align_kd * self._state.w)
            w_target = max(-self.p_w_max * 0.7,
                           min(self.p_w_max * 0.7, w_target))
            kappa_dbg = 0.0
        else:
            # ── 7b) Pure Pursuit (정상 추종) ──────────────────────
            # 표준 공식: 곡률 κ = 2y / L²
            # 비유: 자전거가 멀리 한 점을 보고 그 점에 정확히 닿는 호의 반경.
            if L < 1e-3:
                kappa = 0.0
            else:
                kappa = 2.0 * ly / (L * L)
            kappa_dbg = kappa

            # ── adaptive velocity profile ─────────────────────────
            v_target = self.p_v_max

            # (i) 곡률 기반 (횡가속 한계 안에서)
            #     a_lat = κ · v² 가 한계 이하 → v ≤ √(a_lat_max / |κ|)
            #     비유: 시속 100km 로 급커브 못 돈다 — 횡력 한계.
            #     15차: a_lat_max yaml 외부화 (기본 = a_max 보수적).
            if abs(kappa) > 1e-3:
                v_curve = math.sqrt(self.p_a_lat_max / abs(kappa))
                v_target = min(v_target, v_curve)

            # (ii) 실제 추종 arc clearance 기반
            #      기존 ±60도 전방 최단점은 옆 벽까지 정면 장애물처럼 보아
            #      좁은 통로에서 불필요하게 감속했다. 이제 현재 κ로 실제 갈
            #      arc를 먼저 그려 보고 그 주변 clearance만 속도 제한에 쓴다.
            probe_v = max(0.05, min(self.p_v_max, v_target))
            probe_w = kappa * probe_v
            motion_clear = self._trajectory_clearance_margin(
                obstacles_local, probe_v, probe_w)

            cf = self.p_clearance_stop_distance
            cc = max(self.p_clearance_slowdown_distance, cf + 1e-3)
            if motion_clear < cc:
                v_clear = self.p_v_max * max(0.0, motion_clear - cf) / \
                          max(cc - cf, 1e-3)
                v_target = min(v_target, v_clear)

            # (iii) goal 감속 (운동량 공식 v = √(2·a·d))
            #       정확히 goal_tolerance 에서 v=0 되도록 감속거리 계산.
            v_goal = math.sqrt(2.0 * self.p_a_max *
                               max(0.0, dist_to_goal - self.p_goal_tolerance))
            v_target = min(v_target, v_goal)

            # (iv) heading 오차 감속
            #      α 가 작아도 0 이 아니면 살짝 감속해 부드러운 회전.
            #      α=0 (완전 정렬) → 1.0 (풀가속), α=±90° → 0.3 cap.
            heading_factor = max(0.3, math.cos(alpha))
            v_target *= heading_factor

            # ── Pure Pursuit ω = κ · v ────────────────────────────
            w_target = kappa * v_target

        # ── 8) 한계 + 가속도 제한 ─────────────────────────────────
        v_target = max(0.0, min(self.p_v_max, v_target))
        w_target = max(-self.p_w_max, min(self.p_w_max, w_target))
        v_cmd = max(self._state.v - dv_max,
                    min(self._state.v + dv_max, v_target))
        w_cmd = max(self._state.w - dw_max,
                    min(self._state.w + dw_max, w_target))

        # ── 9) 최종 충돌 체크 (DWA-style safety net) ──────────────
        # (v_cmd, w_cmd) 로 predict_horizon 적분 후 hard collision 검사.
        # 회전+전진 결합 trajectory 가 실제로 안전한지 마지막 확인.
        # 비유: 운전자가 핸들/액셀 결정한 뒤 ABS 시스템이 최종 점검.
        sim_state = RobotState(x=0.0, y=0.0, theta=0.0,
                               v=self._state.v, w=self._state.w)
        sim_traj = forward_simulate(
            sim_state, v_cmd, w_cmd, self.p_dt, self.p_predict_horizon)
        if sim_traj:
            sim_traj_xy = [(p[0], p[1]) for p in sim_traj[2:]] or \
                          [(p[0], p[1]) for p in sim_traj]
            min_d = min_clearance_distance(sim_traj_xy, obstacles_local) \
                    if sim_traj_xy else float("inf")
        else:
            sim_traj_xy = []
            min_d = float("inf")

        collision_imminent = min_d < (self.p_hard_collision_distance +
                                       self.p_robot_radius)

        # 16차: stuck 판정 — collision 또는 v_clear cap 으로 정지 trap
        # Pure Pursuit w=κ·v 가 v=0 이면 w=0. 회전조차 못해 무한 정지.
        # 이걸 collision 과 같은 stuck 으로 잡아 recovery 발동.
        # 18차: recovery cooldown 중이면 stuck 판정 무시 (무한 backup loop 차단).
        is_velocity_blocked = (abs(v_cmd) < 0.02 and
                               motion_clear < self.p_clearance_stop_distance)
        in_recovery_cooldown = (self._sec_now() < self._recovery_cooldown_until)
        is_stuck = ((collision_imminent or is_velocity_blocked)
                    and not in_recovery_cooldown)

        if is_stuck:
            # 정지 명령 + status 발행
            self._publish_cmd(VelocityCommand(v=0.0, w=0.0))
            if collision_imminent:
                self._publish_status_value("EMERGENCY")
            else:
                self._publish_status_value("STOPPED_NEAR_WALL")
            self._stuck_counter += 1

            # Backup recovery 진입 (stuck_recovery_sec 이상 지속 시)
            stuck_threshold = int(self.p_control_rate *
                                   self.p_stuck_recovery_sec)
            if self._stuck_counter > stuck_threshold:
                bwd_clear = self._backward_clearance_inline(obstacles_local)
                if bwd_clear > self.p_safety_distance:
                    # 후방 free → backup 시작
                    self._backup_until = (self._sec_now()
                                          + self.p_backup_duration)
                    self._stuck_counter = 0
                    self.get_logger().warn(
                        f"stuck {self.p_stuck_recovery_sec}s 초과 → "
                        f"backup {self.p_backup_duration}s 시작 "
                        f"(bwd_clear={bwd_clear:.2f}m, "
                        f"motion_clear={motion_clear:.2f}m, min_d={min_d:.2f}m)"
                    )
                else:
                    # 후방도 막힘 → 사람 개입 대기
                    self.get_logger().warn(
                        f"stuck + 후방 막힘 (bwd_clear={bwd_clear:.2f}m). "
                        f"A* 재계획/사람 개입 대기.",
                        throttle_duration_sec=2.0
                    )
            self._log_state_throttled()
            if sim_traj:
                self._publish_best_trajectory(sim_traj)
            return

        # stuck 안 → counter 리셋
        self._stuck_counter = 0

        # ── 10) cmd_vel 발행 ─────────────────────────────────────
        self._publish_cmd(VelocityCommand(v=v_cmd, w=w_cmd))

        # ── 11) 진단 로그 (throttle) ─────────────────────────────
        self._log_state_throttled()
        self._candidate_log_counter += 1
        target = int(self.p_control_rate * self.p_candidate_log_period)
        if self.p_candidate_log_period > 0.0 and \
           self._candidate_log_counter >= max(1, target):
            self._candidate_log_counter = 0
            mode = "ROTATE" if (abs(alpha) > self.p_align_angle_thresh
                                 and L > 0.1) else "PP"
            self.get_logger().info(
                f"PP[{mode}]: la=({lx:+.2f},{ly:+.2f}) "
                f"L={L:.2f}(eff={effective_lookahead:.2f}) "
                f"α={math.degrees(alpha):+.1f}° κ={kappa_dbg:+.2f} "
                f"v={v_cmd:+.2f}/{v_target:.2f} w={w_cmd:+.2f}/{w_target:+.2f} "
                f"clr={motion_clear:.2f} fwd={fwd_clear:.2f} d_goal={dist_to_goal:.2f}"
            )

        # ── 12) 시각화 (Pure Pursuit best trajectory) ─────────────
        if sim_traj:
            self._publish_best_trajectory(sim_traj)

    # ───────────────────────────────────────────────────────────────
    # Forward clearance — 전방 부채꼴 안 최단 장애물 거리
    # ───────────────────────────────────────────────────────────────
    def _forward_clearance_inline(
        self,
        obstacles_local: List[Tuple[float, float]],
        half_angle: float = math.pi / 3.0,   # ±60° 부채꼴
    ) -> float:
        """base_link 정면 (±half_angle) 부채꼴 안의 최단 장애물 거리.

        adaptive velocity profile 의 v_clearance 항 입력.
        Pure Pursuit 의 lateral 방향 장애물은 자체 회피 못 하므로,
        전방 부채꼴만 보고 감속 결정. (회전 중 후방 장애물 무시 OK)

        Args:
            obstacles_local: base_link frame 의 (x, y) 점들
            half_angle: 부채꼴 반각 [rad]

        Returns:
            최단 거리 [m]. 부채꼴 안 점 없으면 +inf.
        """
        if not obstacles_local:
            return float("inf")
        best = float("inf")
        for ox, oy in obstacles_local:
            # 자기 뒤 점은 무시
            if ox < 0.0:
                continue
            ang = math.atan2(oy, ox)
            if abs(ang) > half_angle:
                continue
            d = math.hypot(ox, oy)
            # 로봇 반경 빼서 "여유 거리" 로 환산
            d_eff = max(0.0, d - self.p_robot_radius)
            if d_eff < best:
                best = d_eff
        return best

    def _trajectory_clearance_margin(
        self,
        obstacles_local: List[Tuple[float, float]],
        v_cmd: float,
        w_cmd: float,
    ) -> float:
        """현재 명령 arc 주변의 최단 여유 거리.

        전방 부채꼴 최단점 대신 실제로 로봇 중심이 지나갈 arc만 본다.
        옆 벽과 나란히 달릴 때는 감속하지 않고, 진행 arc 위 장애물에는
        빠르게 반응한다.
        """
        if not obstacles_local:
            return float("inf")

        sim_state = RobotState(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
        horizon = max(0.35, self.p_predict_horizon)
        traj = forward_simulate(sim_state, v_cmd, w_cmd, self.p_dt, horizon)
        if not traj:
            return float("inf")

        # 첫 점들은 로봇 몸체 내부/바로 옆 scan에 과민하므로 조금 건너뛴다.
        traj_xy = [(p[0], p[1]) for p in traj[2:]] or [(p[0], p[1]) for p in traj]
        center_dist = min_clearance_distance(traj_xy, obstacles_local)
        if center_dist == float("inf"):
            return float("inf")
        return max(0.0, center_dist - self.p_robot_radius)

    # ───────────────────────────────────────────────────────────────
    # Backward clearance — Backup Recovery 안전 판단용 (16차)
    # ───────────────────────────────────────────────────────────────
    def _backward_clearance_inline(
        self,
        obstacles_local: List[Tuple[float, float]],
        half_angle: float = math.pi / 3.0,   # ±60° 후방 부채꼴
    ) -> float:
        """base_link 후방 (±half_angle) 부채꼴 안의 최단 장애물 거리.

        Backup recovery 진입 전 안전 체크. 후방이 막혔으면 recovery 못 함.

        Args:
            obstacles_local: base_link frame 의 (x, y) 점들 (ox<0 이 후방)
            half_angle: 부채꼴 반각 [rad]

        Returns:
            최단 거리 [m]. 부채꼴 안 점 없으면 +inf.
        """
        if not obstacles_local:
            return float("inf")
        best = float("inf")
        for ox, oy in obstacles_local:
            # 자기 앞 점은 무시 (후방 부채꼴)
            if ox > 0.0:
                continue
            # ox < 0: 후방. -ox 로 후방 정면 각도 계산
            ang = math.atan2(oy, -ox)
            if abs(ang) > half_angle:
                continue
            d = math.hypot(ox, oy)
            d_eff = max(0.0, d - self.p_robot_radius)
            if d_eff < best:
                best = d_eff
        return best

    def _escape_turn_rate(self, obstacles_local: List[Tuple[float, float]]) -> float:
        """Recovery 후진 중 벽 반대쪽으로 살짝 틀기 위한 yaw rate."""
        if not obstacles_local:
            return 0.0

        side_bias = 0.0
        for ox, oy in obstacles_local:
            # 주로 전방/측방의 가까운 장애물만 탈출 방향 판단에 사용.
            if ox < -0.20:
                continue
            d = math.hypot(ox, oy)
            if d < 1e-3 or d > 1.2:
                continue
            side_bias += (1.0 if oy >= 0.0 else -1.0) / d

        if abs(side_bias) < 1e-3:
            return 0.0

        # 장애물이 왼쪽(+)에 많으면 오른쪽(-)으로, 오른쪽이면 왼쪽으로.
        target = -self.p_backup_turn_gain * side_bias
        return max(-self.p_backup_turn_max,
                   min(self.p_backup_turn_max, target))


    # ───────────────────────────────────────────────────────────────
    # 진단 로그 (throttle)
    # ───────────────────────────────────────────────────────────────
    def _log_state_throttled(self) -> None:
        if self.p_state_log_period <= 0.0:
            return
        self._state_log_counter += 1
        target = int(self.p_control_rate * self.p_state_log_period)
        if self._state_log_counter >= max(1, target):
            self._state_log_counter = 0
            self.get_logger().info(
                f"DWA state [{self.LOCAL_FRAME}]: "
                f"({self._state.x:.2f}, {self._state.y:.2f}, "
                f"yaw={math.degrees(self._state.theta):.1f}°), "
                f"v={self._state.v:+.2f} w={self._state.w:+.2f}"
            )

    def _log_best_candidate_throttled(
        self,
        best_cmd: VelocityCommand,
        best_traj: List[Tuple[float, float, float]],
        local_goal: Tuple[float, float],
        n_cands: int,
    ) -> None:
        if self.p_candidate_log_period <= 0.0:
            return
        self._candidate_log_counter += 1
        target = int(self.p_control_rate * self.p_candidate_log_period)
        if self._candidate_log_counter < max(1, target):
            return
        self._candidate_log_counter = 0
        end_x, end_y, end_theta = best_traj[-1]
        h_best = heading_score_dist(end_x, end_y,
                                    local_goal[0], local_goal[1],
                                    max_dist=self.p_max_clearance)
        c_best = clearance_score(
            [(p[0], p[1]) for p in best_traj[2:]] or
            [(p[0], p[1]) for p in best_traj],
            self._extract_obstacles_from_scan(), self.p_max_clearance)
        v_best = velocity_score(best_cmd.v, self.p_v_max)
        lg_dir = "FRONT" if local_goal[0] >= 0 else "REAR"
        self.get_logger().info(
            f"DWA best: v={best_cmd.v:+.2f} w={best_cmd.w:+.2f} "
            f"score={best_cmd.score:.2f} "
            f"(h={h_best:.2f} c={c_best:.2f} vel={v_best:.2f}) "
            f"local_goal=({local_goal[0]:+.2f},{local_goal[1]:+.2f}) {lg_dir} "
            f"n_cands={n_cands}"
        )

    def _goal_reached_logged_once(self) -> None:
        if not getattr(self, "_goal_logged", False):
            self.get_logger().info("goal 도착 — 정지")
            self._goal_logged = True

    # ───────────────────────────────────────────────────────────────
    # 시각화
    # ───────────────────────────────────────────────────────────────
    def _publish_trajectories(self,
                              trajectories: List[List[Tuple[float, float, float]]]
                              ) -> None:
        array = MarkerArray()
        for i, traj in enumerate(trajectories):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "dwa_candidates"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.01
            m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.5, 0.5, 0.4
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(0.2 * 1e9)
            for x, y, _ in traj:
                pt = Point()
                pt.x = float(x); pt.y = float(y); pt.z = 0.01
                m.points.append(pt)
            array.markers.append(m)
        self._traj_pub.publish(array)

    def _publish_best_trajectory(self,
                                 traj: List[Tuple[float, float, float]]
                                 ) -> None:
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "dwa_best"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 1.0
        m.lifetime.sec = 0
        m.lifetime.nanosec = int(0.3 * 1e9)
        for x, y, _ in traj:
            pt = Point()
            pt.x = float(x); pt.y = float(y); pt.z = 0.02
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
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self._cmd_pub.publish(cmd)
        self.get_logger().debug(f"정지: {reason}")

    def _publish_status_value(self, value: str) -> None:
        msg = String()
        msg.data = value
        self._status_pub.publish(msg)

    def _publish_status(self) -> None:
        """1Hz 정기 status — README §3.3 4상태."""
        msg = String()
        if self._state is None:
            msg.data = "WAITING_ODOM"
        elif self._reached:
            msg.data = "REACHED"
        elif self._path_local is None or not self._path_local.poses:
            msg.data = "STOPPED"
        else:
            msg.data = "PLANNING"
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
