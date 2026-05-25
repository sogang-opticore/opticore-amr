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
    """trajectory 끝점에서 goal 방향 정렬도 [0, 1]."""
    dx = goal_x - traj_end_x
    dy = goal_y - traj_end_y
    if dx == 0.0 and dy == 0.0:
        return 1.0
    angle_to_goal = math.atan2(dy, dx)
    diff = angle_to_goal - traj_end_theta
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return 1.0 - abs(diff) / math.pi


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

        # 샘플링 / 시뮬레이션
        self.declare_parameter("sample_v_n", 11)
        self.declare_parameter("sample_w_n", 21)
        self.declare_parameter("predict_horizon", 1.0)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("control_rate", 20.0)

        # 평가함수 가중치
        self.declare_parameter("weight_heading", 0.6)
        self.declare_parameter("weight_clearance", 1.2)
        self.declare_parameter("weight_velocity", 0.2)

        # 추종 / 평가 보조
        self.declare_parameter("lookahead_dist", 1.0)
        self.declare_parameter("max_clearance", 1.0)
        self.declare_parameter("goal_tolerance", 0.20)

        # In-place rotation 모드 (2026-05-25 추가)
        # lookahead 의 base_link 각도가 이 값 이상 차이면 v=0, w 만으로 정렬.
        # DWA 의 1초 horizon 안에 큰 yaw 차이 (>86° at w_max=1.5) 를 못 좁히는 문제 해결.
        # 비유: 동쪽 보고 있는데 북쪽 가야 함 → "회전만" 한 후 출발. 회전+전진 동시는
        #       1초 horizon 안에선 항상 부족해서 누적 오차 발생.
        self.declare_parameter("align_angle_thresh", 1.05)  # rad ≈ 60°
        self.declare_parameter("align_kp", 1.5)             # P 제어 gain

        # 안전
        self.declare_parameter("robot_radius", 0.20)
        self.declare_parameter("hard_collision_distance", 0.05)
        self.declare_parameter("safety_distance", 0.30)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("scan_range_max", 25.0)

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

        self.p_sample_v_n = gp("sample_v_n").value
        self.p_sample_w_n = gp("sample_w_n").value
        self.p_predict_horizon = gp("predict_horizon").value
        self.p_dt = gp("dt").value
        self.p_control_rate = gp("control_rate").value

        self.p_w_heading = gp("weight_heading").value
        self.p_w_clearance = gp("weight_clearance").value
        self.p_w_velocity = gp("weight_velocity").value

        self.p_lookahead_dist = gp("lookahead_dist").value
        self.p_max_clearance = gp("max_clearance").value
        self.p_goal_tolerance = gp("goal_tolerance").value
        self.p_align_angle_thresh = gp("align_angle_thresh").value
        self.p_align_kp = gp("align_kp").value

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

        URDF: lidar_link 가 base_link 의 +x=0.25 m 에 mount. 회전은 0.
        엄밀히는 lidar_link → base_link TF 변환 필요하지만, 단순화로 lidar 점들이
        base_link 기준이라고 가정 (offset 0.25m 는 robot_radius 0.20m 안이라 무시 가능).
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
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y))
            angle += increment
        return points

    # ───────────────────────────────────────────────────────────────
    # 메인 제어 루프 — cycle 안 TF lookup 0번
    # ───────────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        # 1) state 검증
        if self._state is None:
            return
        if (self._last_odom_time is None or
                self._sec_now() - self._last_odom_time > self.p_odom_timeout):
            self._stop_robot("odom_timeout")
            return

        # 2) path 검증
        if self._path_local is None or not self._path_local.poses:
            self._stop_robot("no_global_path")
            return

        path_xy = self._path_xy(self._path_local)

        # 3) 도착 판정 (LOCAL_FRAME 안에서) + hysteresis
        #
        # 2026-05-25 (5차 진단) — REACHED state hold:
        #   이전 코드는 dist_to_goal jitter (관성 + EKF noise) 로 reached 가
        #   깜빡거려, 도착 직후 path[-1] 이 자기 뒤로 매핑 → align mode 무한 회전.
        #   해결: 한 번 reached 가 True 면 새 path 도착할 때까지 stop hold.
        #         새 path 받으면 _on_global_path 에서 self._reached=False 리셋.
        #
        # 비유: 식당 도착했으면 새 약속 잡기 전까진 자리 유지. 0.2m 지나쳤다고
        #       또 돌아오느라 빙빙 돌지 말 것.
        gx, gy = path_xy[-1]
        dist_to_goal = math.hypot(gx - self._state.x, gy - self._state.y)

        if self._reached:
            # REACHED hold — 새 path 가 self._reached 를 False 로 리셋할 때까지 정지
            self._stop_robot("reached_hold")
            return

        if dist_to_goal < self.p_goal_tolerance:
            self._stop_robot("goal_reached")
            self._reached = True
            self._goal_reached_logged_once()
            return

        # 4) lookahead 점 선택 (LOCAL_FRAME)
        #    monotonic forward progress: 이전 nearest_idx 이전 점은 후보 제외.
        #    catmull-rom / 우회로 path 에서 자기 뒤 점이 nearest 로 잡혀 REAR 매핑 되는 것 차단.
        nearest_idx = find_nearest_idx(
            path_xy, (self._state.x, self._state.y), self._path_progress_idx)
        self._path_progress_idx = nearest_idx   # 단조 증가 갱신
        lookahead = pick_lookahead_point(
            path_xy, (self._state.x, self._state.y),
            self.p_lookahead_dist, start_idx=nearest_idx)
        if lookahead is None:
            self._stop_robot("no_lookahead")
            return

        # 5) base_link frame 변환 — 단순 회전+평행이동 (TF lookup 불필요)
        #    DWA 내부 trajectory 평가는 base_link 안에서 (가상 (0,0,0) 시작).
        local_state = RobotState(
            x=0.0, y=0.0, theta=0.0,
            v=self._state.v, w=self._state.w,
        )
        local_goal = world_to_local(lookahead, self._state)

        # 5.5) In-place rotation 모드 (2026-05-25 추가, 5차 진단에서 안전장치 추가)
        #      lookahead 가 ±60° 이상 빗나가 있으면 DWA 의 1초 horizon 안에 못 잡힘.
        #      회전+전진 trade-off 에 의해 매 cycle 부분 회전 누적 → path 와 어긋남.
        #      → v=0 으로 회전만 (P 제어). 정렬되면 normal DWA 재개.
        #
        #      안전장치: 도착 영역 (1.5 × tolerance) 안에선 align mode 비활성.
        #      도착 직후 jitter 로 lookahead 가 자기 뒤를 가리킬 때 무한 회전 방지.
        local_goal_angle = math.atan2(local_goal[1], local_goal[0])
        align_safe_dist = self.p_goal_tolerance * 1.5
        if dist_to_goal > align_safe_dist and \
           abs(local_goal_angle) > self.p_align_angle_thresh:
            w_cmd = self.p_align_kp * local_goal_angle
            # w_max 클램프
            w_cmd = max(-self.p_w_max, min(self.p_w_max, w_cmd))
            # 가속도 한계 (이전 w 에서 alpha_max * dt 만큼만 변경 가능)
            dw_max = self.p_alpha_max * (1.0 / max(self.p_control_rate, 1.0))
            w_cmd = max(self._state.w - dw_max,
                        min(self._state.w + dw_max, w_cmd))
            self._publish_cmd(VelocityCommand(v=0.0, w=w_cmd))

            # 진단 로그 throttle
            self._candidate_log_counter += 1
            target = int(self.p_control_rate * self.p_candidate_log_period)
            if self.p_candidate_log_period > 0.0 and \
               self._candidate_log_counter >= max(1, target):
                self._candidate_log_counter = 0
                self.get_logger().info(
                    f"DWA align mode: goal_angle={math.degrees(local_goal_angle):+.1f}° "
                    f"→ v=0.00 w={w_cmd:+.2f} "
                    f"(thresh={math.degrees(self.p_align_angle_thresh):.0f}°)"
                )
            # state 로그도 같이
            self._log_state_throttled()
            # 시각화는 normal DWA 모드 때만
            return

        # 6) Dynamic Window
        window = compute_dynamic_window(
            state=local_state,
            v_max=self.p_v_max, v_min=self.p_v_min,
            w_max=self.p_w_max, a_max=self.p_a_max,
            alpha_max=self.p_alpha_max, dt=self.p_dt,
        )

        # 7) 속도 샘플링
        samples = sample_velocities(
            window, self.p_sample_v_n, self.p_sample_w_n)
        if not samples:
            self._stop_robot("no_samples")
            return

        # 8) 장애물 (base_link frame)
        obstacles_local = self._extract_obstacles_from_scan()

        # 9) 각 후보 평가
        candidates: List[Tuple[VelocityCommand, List[Tuple[float, float, float]]]] = []
        for v, w in samples:
            traj = forward_simulate(
                local_state, v, w, self.p_dt, self.p_predict_horizon)
            if not traj:
                continue
            # 첫 2 step 은 base_link 원점 근처 (자체 lidar 가까이) → 충돌 검사 제외
            traj_xy = [(p[0], p[1]) for p in traj[2:]] or \
                      [(p[0], p[1]) for p in traj]
            min_d = min_clearance_distance(traj_xy, obstacles_local)
            if min_d < (self.p_hard_collision_distance + self.p_robot_radius):
                continue

            end_x, end_y, end_theta = traj[-1]
            h = heading_score(end_x, end_y, end_theta,
                              local_goal[0], local_goal[1])
            c = clearance_score(traj_xy, obstacles_local, self.p_max_clearance)
            vel_s = velocity_score(v, self.p_v_max)
            score = (
                self.p_w_heading * h
                + self.p_w_clearance * c
                + self.p_w_velocity * vel_s
            )
            candidates.append((VelocityCommand(v=v, w=w, score=score), traj))

        # 10) 모든 후보 차단 → EMERGENCY
        if not candidates:
            self._stop_robot("all_candidates_blocked")
            self._publish_status_value("EMERGENCY")
            return

        # 11) 최고 점수 선택
        candidates.sort(key=lambda c: c[0].score, reverse=True)
        best_cmd, best_traj = candidates[0]
        self._publish_cmd(best_cmd)

        # 12) 진단 로그 (throttle)
        self._log_state_throttled()
        self._log_best_candidate_throttled(
            best_cmd, best_traj, local_goal, len(candidates))

        # 13) 시각화
        self._publish_trajectories([t for _, t in candidates])
        self._publish_best_trajectory(best_traj)

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
        h_best = heading_score(end_x, end_y, end_theta,
                               local_goal[0], local_goal[1])
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
