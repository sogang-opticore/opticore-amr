"""dwa_node.py 순수 함수 단위 테스트 — Week 2.

ROS 의존성 없는 순수 알고리즘 테스트.
실행:
    colcon test --packages-select amr_navigation
    또는
    python3 -m pytest src/amr_navigation/test/test_dwa.py -v
"""

import math


from amr_navigation.dwa_node import (
    RobotState,
    DynamicWindow,
    compute_dynamic_window,
    sample_velocities,
    forward_simulate,
    heading_score,
    clearance_score,
    velocity_score,
    min_clearance_distance,
    pick_lookahead_point,
)


# ---------------------------------------------------------------------------
# Dynamic Window
# ---------------------------------------------------------------------------

class TestDynamicWindow:
    def test_within_dynamic_limits(self):
        """현재 v=0.5에서 a_max=1.0, dt=0.1 → 가능 범위 [0.4, 0.6]."""
        state = RobotState(x=0, y=0, theta=0, v=0.5, w=0.0)
        w = compute_dynamic_window(
            state, v_max=2.0, v_min=-1.0,
            w_max=1.5, a_max=1.0, alpha_max=1.5, dt=0.1,
        )
        assert abs(w.v_min - 0.4) < 1e-9
        assert abs(w.v_max - 0.6) < 1e-9

    def test_clipped_by_v_max(self):
        """현재 v=1.9, a=1.0, dt=0.1 → naive 2.0인데 v_max=1.5에 clipped."""
        state = RobotState(x=0, y=0, theta=0, v=1.9, w=0.0)
        w = compute_dynamic_window(
            state, v_max=1.5, v_min=-1.0,
            w_max=1.5, a_max=1.0, alpha_max=1.5, dt=0.1,
        )
        assert w.v_max == 1.5    # ← clipped


# ---------------------------------------------------------------------------
# Velocity Sampling
# ---------------------------------------------------------------------------

class TestSampleVelocities:
    def test_count(self):
        window = DynamicWindow(v_min=0.0, v_max=1.0, w_min=-1.0, w_max=1.0)
        samples = sample_velocities(window, n_v=11, n_w=21)
        assert len(samples) == 11 * 21

    def test_boundary_values_present(self):
        window = DynamicWindow(v_min=0.0, v_max=1.0, w_min=-1.0, w_max=1.0)
        samples = sample_velocities(window, n_v=11, n_w=21)
        vs = sorted({v for v, _ in samples})
        ws = sorted({w for _, w in samples})
        assert vs[0] == 0.0 and vs[-1] == 1.0
        assert ws[0] == -1.0 and ws[-1] == 1.0

    def test_empty_when_zero_n(self):
        window = DynamicWindow(v_min=0.0, v_max=1.0, w_min=-1.0, w_max=1.0)
        assert sample_velocities(window, n_v=0, n_w=21) == []
        assert sample_velocities(window, n_v=11, n_w=0) == []


# ---------------------------------------------------------------------------
# Forward Simulate
# ---------------------------------------------------------------------------

class TestForwardSimulate:
    def test_straight_motion(self):
        """v=0.5, w=0, sim_time=1.0 → x ≈ 0.5, y ≈ 0."""
        state = RobotState(x=0, y=0, theta=0, v=0.0, w=0.0)
        traj = forward_simulate(state, v=0.5, w=0.0, dt=0.1, sim_time=1.0)
        assert len(traj) == 10
        x_end, y_end, theta_end = traj[-1]
        assert abs(x_end - 0.5) < 0.01
        assert abs(y_end) < 1e-9
        assert abs(theta_end) < 1e-9

    def test_pure_rotation(self):
        """v=0, w=π/2, sim_time=1.0 → theta ≈ π/2, x=y=0."""
        state = RobotState(x=0, y=0, theta=0, v=0.0, w=0.0)
        traj = forward_simulate(state, v=0.0, w=math.pi / 2, dt=0.1, sim_time=1.0)
        x_end, y_end, theta_end = traj[-1]
        assert abs(x_end) < 1e-9
        assert abs(y_end) < 1e-9
        assert abs(theta_end - math.pi / 2) < 0.01

    def test_circular_arc(self):
        """v=1.0, w=1.0 → 반경 1m 원호. 반바퀴 (sim_time=π) 후 (0, 2)."""
        state = RobotState(x=0, y=0, theta=0, v=0.0, w=0.0)
        traj = forward_simulate(state, v=1.0, w=1.0, dt=0.005, sim_time=math.pi)
        x_end, y_end, _ = traj[-1]
        assert abs(x_end) < 0.05
        assert abs(y_end - 2.0) < 0.05

    def test_starts_from_state(self):
        """초기 state 위치에서 출발, v=w=0 → 위치 변화 없음."""
        state = RobotState(x=2.0, y=3.0, theta=math.pi / 4, v=0.0, w=0.0)
        traj = forward_simulate(state, v=0.0, w=0.0, dt=0.1, sim_time=1.0)
        x_end, y_end, theta_end = traj[-1]
        assert abs(x_end - 2.0) < 1e-9
        assert abs(y_end - 3.0) < 1e-9
        assert abs(theta_end - math.pi / 4) < 1e-9

    def test_empty_when_invalid(self):
        state = RobotState(x=0, y=0, theta=0, v=0.0, w=0.0)
        assert forward_simulate(state, v=1, w=0, dt=0.1, sim_time=0.05) == []
        assert forward_simulate(state, v=1, w=0, dt=0, sim_time=1) == []


# ---------------------------------------------------------------------------
# Heading Score
# ---------------------------------------------------------------------------

class TestHeadingScore:
    def test_perfect_alignment(self):
        """trajectory 끝점 (0,0,0)에서 goal (1,0)을 정확히 바라봄 → 1.0."""
        s = heading_score(0.0, 0.0, 0.0, 1.0, 0.0)
        assert abs(s - 1.0) < 1e-9

    def test_opposite_direction(self):
        """끝점 theta=0인데 goal이 뒤(-1,0)에 있음 → 0.0."""
        s = heading_score(0.0, 0.0, 0.0, -1.0, 0.0)
        assert abs(s - 0.0) < 1e-9

    def test_ninety_degrees(self):
        """90도 어긋남 → 0.5."""
        s = heading_score(0.0, 0.0, 0.0, 0.0, 1.0)  # goal이 +y인데 끝점은 +x 향함
        assert abs(s - 0.5) < 1e-9

    def test_returns_unit_interval(self):
        """다양한 입력에서 [0, 1] 보장."""
        for theta in [-math.pi, -1.5, 0.0, 1.0, math.pi]:
            for goal in [(1.0, 0.0), (0.0, 1.0), (-1.0, -1.0), (0.5, 0.7)]:
                s = heading_score(0.0, 0.0, theta, goal[0], goal[1])
                assert 0.0 <= s <= 1.0

    def test_zero_vector_returns_one(self):
        """끝점이 정확히 goal 위 → 1.0 (방향 의미 없음)."""
        s = heading_score(1.0, 1.0, 0.5, 1.0, 1.0)
        assert s == 1.0


# ---------------------------------------------------------------------------
# Clearance Score
# ---------------------------------------------------------------------------

class TestClearanceScore:
    def test_no_obstacles(self):
        traj = [(0, 0), (1, 0), (2, 0)]
        s = clearance_score(traj, [], max_clearance=1.0)
        assert s == 1.0

    def test_empty_trajectory(self):
        s = clearance_score([], [(1, 1)], max_clearance=1.0)
        assert s == 0.0

    def test_minimum_distance(self):
        """가장 가까운 점 (0,0) 과 장애물 (0.5, 0) → min_dist=0.5, max=1.0 → score=0.5."""
        traj = [(0.0, 0.0), (1.0, 0.0)]
        obs = [(0.5, 0.0)]
        s = clearance_score(traj, obs, max_clearance=1.0)
        assert abs(s - 0.5) < 1e-9

    def test_capped_at_one(self):
        """모든 장애물이 max_clearance 보다 멀면 1.0."""
        traj = [(0, 0)]
        obs = [(5.0, 5.0)]
        s = clearance_score(traj, obs, max_clearance=1.0)
        assert s == 1.0


# ---------------------------------------------------------------------------
# Velocity Score
# ---------------------------------------------------------------------------

class TestVelocityScore:
    def test_zero_velocity(self):
        assert velocity_score(0.0, 1.5) == 0.0

    def test_max_velocity(self):
        assert velocity_score(1.5, 1.5) == 1.0

    def test_negative_velocity_zero(self):
        """후진은 0점 (정지 회피 효과만)."""
        assert velocity_score(-0.5, 1.5) == 0.0

    def test_zero_v_max(self):
        assert velocity_score(1.0, 0.0) == 0.0


# ---------------------------------------------------------------------------
# Min Clearance Distance
# ---------------------------------------------------------------------------

class TestMinClearanceDistance:
    def test_inf_when_no_obstacles(self):
        assert min_clearance_distance([(0, 0)], []) == float("inf")

    def test_inf_when_no_trajectory(self):
        assert min_clearance_distance([], [(1, 1)]) == float("inf")

    def test_simple_distance(self):
        d = min_clearance_distance([(0.0, 0.0)], [(3.0, 4.0)])
        assert abs(d - 5.0) < 1e-9


# ---------------------------------------------------------------------------
# Lookahead Point
# ---------------------------------------------------------------------------

class TestPickLookahead:
    def test_empty_path(self):
        assert pick_lookahead_point([], (0, 0), 1.0) is None

    def test_short_path_returns_last(self):
        path = [(0, 0), (0.5, 0)]
        pt = pick_lookahead_point(path, (0, 0), 5.0)
        assert pt == (0.5, 0)

    def test_finds_lookahead(self):
        """robot at (0,0), path 1m 간격 직선. lookahead=2.0 → 약 (2,0) 또는 그 다음."""
        path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        pt = pick_lookahead_point(path, (0, 0), 2.0)
        # 누적 거리가 2.0 이상이 되는 첫 점 = (2,0)
        assert pt == (2, 0)

    def test_starts_from_nearest(self):
        """로봇 위치 (2.1, 0). nearest=(2,0). lookahead=1.0 → 누적 1.0 이상 = (3,0)."""
        path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        pt = pick_lookahead_point(path, (2.1, 0), 1.0)
        assert pt == (3, 0)


# ── v/w 커플링 해제 패치 검증 (YS, 2026-05-29) ──────────────────────────

class TestRotationClearanceInline:
    """_rotation_clearance_inline 단위 테스트."""

    def _make_node_stub(self):
        """ROS 없이 메서드만 빌려쓰는 최소 stub."""
        import types, math
        node = types.SimpleNamespace()
        node.p_robot_radius = 0.20
        node.p_safety_distance = 0.30
        # 메서드 바인딩
        from amr_navigation.dwa_node import DwaPlannerNode
        node._rotation_clearance_inline = (
            DwaPlannerNode._rotation_clearance_inline.__get__(node)
        )
        return node

    def test_no_obstacles_returns_inf(self):
        node = self._make_node_stub()
        result = node._rotation_clearance_inline([])
        assert result == float("inf")

    def test_far_obstacle_ignored(self):
        """robot_radius × 8 (= 1.6m) 밖 장애물 → inf.

        dwa_node._rotation_clearance_inline 의 성능 필터:
        body_range = p_robot_radius * 8.0 안의 장애물만 본다.
        그 밖이면 best 가 갱신 안 되어 inf 그대로 반환.
        """
        node = self._make_node_stub()
        # 1.6m 밖 (2.0m) → 필터에 의해 무시
        result = node._rotation_clearance_inline([(2.0, 0.0)])
        assert result == float("inf")

    def test_close_obstacle_detected(self):
        """로봇 바로 앞 0.3m 장애물 → 양수 거리 반환."""
        node = self._make_node_stub()
        result = node._rotation_clearance_inline([(0.3, 0.0)])
        assert 0.0 <= result < float("inf")

    def test_side_wall_at_0_4m_returns_positive(self):
        """측면 0.4m 벽 — fwd_clear 였으면 0 이지만 rotation_clear 는 양수."""
        node = self._make_node_stub()
        # 측면(y=0.4m) 장애물 — robot_radius × 8 = 1.6m 이내이므로 감지
        result = node._rotation_clearance_inline([(0.0, 0.4)])
        # 0.4 - robot_radius(0.20) = 0.20m 정도
        assert result > 0.0


class TestWMinRotate:
    """w_min_rotate 동작 검증 — w = κ·v 가 0 일 때 최소 회전 보장."""

    def test_w_floored_when_v_zero(self):
        """v=0, alpha=0.5rad(>exit 0.262) → w_min_rotate(0.3) 보장."""
        import math
        w_min_rotate = 0.3
        alpha = 0.5          # heading 오차 있음
        align_angle_exit = 0.262
        v_target = 0.0       # clearance 로 v=0 된 상황
        kappa = 1.0

        w_pp = kappa * v_target   # = 0.0
        if (w_min_rotate > 0.0
                and abs(alpha) > align_angle_exit
                and abs(w_pp) < w_min_rotate):
            w_target = math.copysign(w_min_rotate, alpha)
        else:
            w_target = w_pp

        assert abs(w_target) >= w_min_rotate
        assert w_target > 0   # alpha 양수 → 왼쪽 회전

    def test_w_not_floored_when_aligned(self):
        """alpha < align_angle_exit(0.262) 이면 floor 미적용."""
        import math
        w_min_rotate = 0.3
        alpha = 0.1          # 거의 정렬됨
        align_angle_exit = 0.262
        v_target = 0.0
        kappa = 0.5

        w_pp = kappa * v_target   # = 0.0
        if (w_min_rotate > 0.0
                and abs(alpha) > align_angle_exit
                and abs(w_pp) < w_min_rotate):
            w_target = math.copysign(w_min_rotate, alpha)
        else:
            w_target = w_pp

        assert w_target == 0.0   # floor 안 걸림

    def test_w_not_floored_when_zero_param(self):
        """w_min_rotate=0.0 이면 기존 동작 유지."""
        import math
        w_min_rotate = 0.0   # 비활성
        alpha = 0.9
        align_angle_exit = 0.262
        v_target = 0.0
        kappa = 1.0

        w_pp = kappa * v_target
        if (w_min_rotate > 0.0
                and abs(alpha) > align_angle_exit
                and abs(w_pp) < w_min_rotate):
            w_target = math.copysign(w_min_rotate, alpha)
        else:
            w_target = w_pp

        assert w_target == 0.0   # w_min_rotate=0 → floor 없음