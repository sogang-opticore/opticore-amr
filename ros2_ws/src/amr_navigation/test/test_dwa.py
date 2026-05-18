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
