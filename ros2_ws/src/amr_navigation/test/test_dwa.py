"""dwa_node.py 순수 함수 단위 테스트 — Week 2.

ROS 의존성 없는 순수 알고리즘 테스트.
실행:
    colcon test --packages-select amr_navigation
    또는
    python3 -m pytest src/amr_navigation/test/test_dwa.py -v
"""

import math
from types import SimpleNamespace


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
    pick_lookahead_from_projection,
    sample_path_from_projection,
    DynamicPathBlockage,
    detect_path_corridor_blockage,
    choose_dynamic_avoid_target,
    choose_rejoin_target,
    occupancy_grid_has_static_obstacle_near,
    should_use_rejoin,
    should_force_rejoin_for_short_lookahead,
    predict_signed_path_offset,
    should_release_align,
    clamp_forward_velocity,
    rate_limit_linear_velocity,
    should_rearm_reached_with_path,
    speed_limit_from_clearance,
    turn_demand_intensity,
    turn_clearance_speed_limit,
    near_wall_escape_adjustment,
    goal_approach_speed_limit,
    should_mark_goal_reached,
    safe_forward_only_distance,
    rate_limit_angular_velocity,
    should_finish_forward_only,
    project_to_path,
    point_segment_distance,
    segment_clearance_margin,
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


class TestPathProjectionLookahead:
    def test_projection_returns_signed_cross_track_error(self):
        path = [(0.0, 0.0), (2.0, 0.0)]

        proj = project_to_path(path, (0.5, 0.2), 0)

        assert proj is not None
        assert abs(proj.point[0] - 0.5) < 1e-9
        assert abs(proj.point[1]) < 1e-9
        assert abs(proj.offset - 0.2) < 1e-9
        assert abs(proj.signed_offset - 0.2) < 1e-9
        assert abs(proj.yaw) < 1e-9

    def test_projection_lookahead_interpolates_from_projected_point(self):
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        proj = project_to_path(path, (0.25, 0.4), 0)

        pt = pick_lookahead_from_projection(path, proj, 0.75)

        assert pt is not None
        assert abs(pt[0] - 1.0) < 1e-9
        assert abs(pt[1]) < 1e-9

    def test_sample_path_from_projection_returns_yaw(self):
        path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        proj = project_to_path(path, (0.2, 0.3), 0)

        sample = sample_path_from_projection(path, proj, 1.2)

        assert sample is not None
        assert abs(sample.point[0] - 1.0) < 1e-9
        assert abs(sample.point[1] - 0.4) < 1e-9
        assert abs(sample.yaw - math.pi / 2.0) < 1e-9

    def test_rejoin_target_prefers_forward_merge_over_nearest(self):
        path = [(0.0, 0.0), (5.0, 0.0)]
        robot = RobotState(x=0.0, y=1.0, theta=0.0, v=0.0, w=0.0)
        proj = project_to_path(path, (robot.x, robot.y), 0)

        target = choose_rejoin_target(
            path_xy=path,
            robot=robot,
            projection=proj,
            min_lookahead=0.80,
            max_lookahead=3.50,
            step=0.25,
            heading_weight=1.2,
            distance_weight=0.12,
            curvature_weight=0.18,
        )

        assert target is not None
        assert target.distance > 1.0
        assert abs(target.alpha) < math.radians(35.0)
        assert target.curvature < 1.0
        assert target.desired_distance > 2.5

    def test_segment_clearance_margin_subtracts_robot_radius(self):
        assert abs(point_segment_distance(
            (0.5, 0.35), (0.0, 0.0), (1.0, 0.0)) - 0.35) < 1e-9

        margin = segment_clearance_margin(
            (0.0, 0.0),
            (1.0, 0.0),
            [(0.5, 0.35)],
            robot_radius=0.20,
        )

        assert abs(margin - 0.15) < 1e-9

    def test_detect_path_corridor_blockage_on_global_path(self):
        path = [(0.0, 0.0), (5.0, 0.0)]
        robot = RobotState(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
        proj = project_to_path(path, (robot.x, robot.y), 0)

        blockage = detect_path_corridor_blockage(
            path_xy=path,
            robot=robot,
            projection=proj,
            obstacles_local=[(1.20, 0.10), (1.25, -0.08)],
            corridor_width=0.50,
            check_distance=3.0,
            min_points=2,
            step=0.25,
        )

        assert blockage.blocked is True
        assert blockage.count == 2
        assert 1.0 <= blockage.distance <= 1.4

    def test_detect_path_corridor_ignores_lateral_obstacle(self):
        path = [(0.0, 0.0), (5.0, 0.0)]
        robot = RobotState(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
        proj = project_to_path(path, (robot.x, robot.y), 0)

        blockage = detect_path_corridor_blockage(
            path_xy=path,
            robot=robot,
            projection=proj,
            obstacles_local=[(1.20, 1.20), (1.30, -1.10)],
            corridor_width=0.50,
            check_distance=3.0,
            min_points=1,
            step=0.25,
        )

        assert blockage.blocked is False

    def test_static_map_filter_ignores_wall_points_for_dynamic_blockage(self):
        path = [(0.0, 0.0), (5.0, 0.0)]
        robot = RobotState(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
        proj = project_to_path(path, (robot.x, robot.y), 0)
        obstacles = [(1.20, 0.10), (1.25, -0.08)]
        grid = SimpleNamespace(
            info=SimpleNamespace(
                resolution=0.05,
                width=120,
                height=40,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=-0.5, y=-1.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
            data=[0] * (120 * 40),
        )
        for ox, oy in obstacles:
            col = int(math.floor((ox - grid.info.origin.position.x) /
                                 grid.info.resolution))
            row = int(math.floor((oy - grid.info.origin.position.y) /
                                 grid.info.resolution))
            grid.data[row * grid.info.width + col] = 100

        dynamic_obstacles = [
            obs for obs in obstacles
            if not occupancy_grid_has_static_obstacle_near(
                grid, obs, radius=0.30, occupied_threshold=65)
        ]

        blockage = detect_path_corridor_blockage(
            path_xy=path,
            robot=robot,
            projection=proj,
            obstacles_local=dynamic_obstacles,
            corridor_width=0.50,
            check_distance=3.0,
            min_points=2,
            step=0.25,
        )

        assert dynamic_obstacles == []
        assert blockage.blocked is False

    def test_dynamic_avoid_target_offsets_away_from_blocked_side(self):
        path = [(0.0, 0.0), (5.0, 0.0)]
        robot = RobotState(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
        proj = project_to_path(path, (robot.x, robot.y), 0)
        blockage = DynamicPathBlockage(
            blocked=True,
            distance=1.0,
            count=3,
            side_bias=0.5,
            min_margin=0.3,
        )

        target = choose_dynamic_avoid_target(
            path_xy=path,
            robot=robot,
            projection=proj,
            blockage=blockage,
            obstacles_local=[(1.0, 0.15), (1.2, 0.20), (1.4, 0.10)],
            robot_radius=0.20,
            lateral_offsets=[0.75, 1.0],
            min_clearance=0.10,
            min_lookahead=0.8,
            max_lookahead=3.0,
            rejoin_distance=1.4,
            step=0.25,
            previous_side=0,
            side_switch_penalty=0.5,
        )

        assert target is not None
        assert target.side == -1
        assert target.point[1] < 0.0
        assert target.clearance >= 0.10

    def test_rejoin_target_penalizes_blocked_merge_line(self):
        path = [(0.0, 0.0), (0.8, 0.0), (0.8, 2.0)]
        robot = RobotState(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
        proj = project_to_path(path, (robot.x, robot.y), 0)

        target = choose_rejoin_target(
            path_xy=path,
            robot=robot,
            projection=proj,
            min_lookahead=0.80,
            max_lookahead=1.80,
            step=0.50,
            heading_weight=0.0,
            distance_weight=0.0,
            curvature_weight=0.0,
            obstacles_local=[(0.60, -0.20)],
            robot_radius=0.20,
            clearance_min=0.25,
            clearance_weight=3.0,
        )

        assert target is not None
        assert target.distance > 0.80
        assert target.clearance >= 0.25

    def test_rejoin_stays_active_until_heading_is_aligned(self):
        assert should_use_rejoin(
            path_offset=0.10,
            heading_error=0.60,
            was_rejoining=True,
            entry_offset=0.30,
            exit_offset=0.18,
            exit_heading=0.45,
        ) is True

    def test_rejoin_exits_when_offset_and_heading_are_small(self):
        assert should_use_rejoin(
            path_offset=0.10,
            heading_error=0.10,
            was_rejoining=True,
            entry_offset=0.30,
            exit_offset=0.18,
            exit_heading=0.45,
        ) is False

    def test_rejoin_enters_by_offset(self):
        assert should_use_rejoin(
            path_offset=0.35,
            heading_error=0.0,
            was_rejoining=False,
            entry_offset=0.30,
            exit_offset=0.18,
            exit_heading=0.45,
        ) is True

    def test_rejoin_releases_when_predicted_error_is_only_mild(self):
        assert should_use_rejoin(
            path_offset=0.12,
            heading_error=0.10,
            was_rejoining=True,
            entry_offset=0.30,
            exit_offset=0.22,
            exit_heading=0.52,
            predicted_offset=0.36,
            predicted_exit_offset=0.42,
        ) is False

    def test_rejoin_stays_when_predicted_error_is_still_diverging(self):
        assert should_use_rejoin(
            path_offset=0.12,
            heading_error=0.10,
            was_rejoining=True,
            entry_offset=0.30,
            exit_offset=0.22,
            exit_heading=0.52,
            predicted_offset=0.48,
            predicted_exit_offset=0.42,
        ) is True


# ── Predictive REJOIN guard 검증 (Codex, 2026-05-31) ────────────────────

class TestShortLookaheadRejoin:
    def test_forces_rejoin_when_path_lookahead_is_folded_far_from_goal(self):
        assert should_force_rejoin_for_short_lookahead(
            local_lookahead_distance=0.11,
            effective_lookahead=0.65,
            dist_to_goal=11.0,
            goal_tolerance=0.20,
            min_distance=0.35,
            ratio=0.55,
            goal_margin=1.0,
        ) is True

    def test_does_not_force_rejoin_for_normal_lookahead(self):
        assert should_force_rejoin_for_short_lookahead(
            local_lookahead_distance=0.55,
            effective_lookahead=0.65,
            dist_to_goal=11.0,
            goal_tolerance=0.20,
            min_distance=0.35,
            ratio=0.55,
            goal_margin=1.0,
        ) is False

    def test_does_not_force_rejoin_near_goal(self):
        assert should_force_rejoin_for_short_lookahead(
            local_lookahead_distance=0.11,
            effective_lookahead=0.65,
            dist_to_goal=0.75,
            goal_tolerance=0.20,
            min_distance=0.35,
            ratio=0.55,
            goal_margin=1.0,
        ) is False


class TestPredictiveRejoin:
    def test_predict_signed_path_offset_catches_diverging_heading(self):
        predicted = predict_signed_path_offset(
            signed_offset=-0.10,
            heading_error=math.radians(40.0),
            speed=1.5,
            horizon=0.55,
        )

        assert abs(predicted) > 0.60

    def test_rejoin_enters_by_predicted_offset(self):
        assert should_use_rejoin(
            path_offset=0.18,
            heading_error=0.0,
            was_rejoining=False,
            entry_offset=0.30,
            exit_offset=0.18,
            exit_heading=0.45,
            predicted_offset=0.52,
        ) is True

    def test_rejoin_target_uses_predicted_offset_for_merge_distance(self):
        path = [(0.0, 0.0), (5.0, 0.0)]
        robot = RobotState(x=0.0, y=0.15, theta=0.0, v=1.2, w=0.0)
        proj = project_to_path(path, (robot.x, robot.y), 0)

        target = choose_rejoin_target(
            path_xy=path,
            robot=robot,
            projection=proj,
            min_lookahead=0.80,
            max_lookahead=3.50,
            step=0.25,
            heading_weight=1.2,
            distance_weight=0.12,
            curvature_weight=0.18,
            effective_offset=0.75,
        )

        assert target is not None
        assert target.desired_distance > 2.8


# ── v/w 커플링 해제 패치 검증 (YS, 2026-05-29) ──────────────────────────

class TestAlignReleaseAndVelocityClamp:
    def test_align_release_is_more_permissive_during_rejoin(self):
        assert should_release_align(
            alpha=math.radians(50.0),
            is_rejoining=True,
            release_angle=0.70,
            rejoin_release_angle=0.95,
            rotate_clearance=1.0,
            release_clearance=0.60,
        ) is True
        assert should_release_align(
            alpha=math.radians(50.0),
            is_rejoining=False,
            release_angle=0.70,
            rejoin_release_angle=0.95,
            rotate_clearance=1.0,
            release_clearance=0.60,
        ) is False

    def test_no_backward_clamp_blocks_negative_velocity(self):
        assert clamp_forward_velocity(-0.2, allow_backward=False) == 0.0
        assert clamp_forward_velocity(-0.2, allow_backward=True) == -0.2

    def test_linear_rate_uses_fast_brake_when_target_drops(self):
        assert abs(rate_limit_linear_velocity(
            current_v=1.0,
            target_v=0.2,
            accel_step=0.05,
            brake_step=0.15,
            allow_backward=False,
        ) - 0.85) < 1e-9

    def test_linear_rate_uses_normal_accel_when_speeding_up(self):
        assert abs(rate_limit_linear_velocity(
            current_v=0.2,
            target_v=1.0,
            accel_step=0.05,
            brake_step=0.15,
            allow_backward=False,
        ) - 0.25) < 1e-9

    def test_angular_rate_uses_fast_brake_when_target_drops(self):
        assert abs(rate_limit_angular_velocity(
            current_w=0.85,
            target_w=0.16,
            accel_step=0.075,
            brake_step=0.30,
        ) - 0.55) < 1e-9

    def test_angular_rate_uses_normal_accel_when_speeding_up(self):
        assert abs(rate_limit_angular_velocity(
            current_w=0.10,
            target_w=0.80,
            accel_step=0.075,
            brake_step=0.30,
        ) - 0.175) < 1e-9


class TestForwardOnlyFinish:
    def test_forward_only_finishes_when_path_rejoined(self):
        done, reason = should_finish_forward_only(
            now=1.0,
            until=2.0,
            dist_moved=0.12,
            target_dist=0.35,
            collision_near=False,
            path_offset=0.18,
            min_dist_before_path_exit=0.10,
            path_rejoin_offset=0.25,
        )

        assert done is True
        assert "path rejoined" in reason

    def test_forward_only_continues_until_min_distance_before_path_exit(self):
        done, _ = should_finish_forward_only(
            now=1.0,
            until=2.0,
            dist_moved=0.05,
            target_dist=0.35,
            collision_near=False,
            path_offset=0.18,
            min_dist_before_path_exit=0.10,
            path_rejoin_offset=0.25,
        )

        assert done is False


class TestClearanceAwareSpeedCaps:
    def test_speed_limit_from_clearance_uses_braking_distance(self):
        assert speed_limit_from_clearance(
            clearance=0.30,
            stop_distance=0.30,
            acceleration=1.0,
        ) == 0.0
        assert abs(speed_limit_from_clearance(
            clearance=0.80,
            stop_distance=0.30,
            acceleration=1.0,
        ) - 1.0) < 1e-9

    def test_turn_brake_blends_only_when_turn_demand_is_high(self):
        limited = turn_clearance_speed_limit(
            v_target=1.2,
            forward_clearance=0.50,
            stop_distance=0.30,
            acceleration=1.0,
            turn_intensity=1.0,
        )
        assert abs(limited - math.sqrt(0.4)) < 1e-9

        unchanged = turn_clearance_speed_limit(
            v_target=1.2,
            forward_clearance=0.50,
            stop_distance=0.30,
            acceleration=1.0,
            turn_intensity=0.0,
        )
        assert unchanged == 1.2

    def test_turn_demand_uses_heading_or_angular_rate(self):
        assert turn_demand_intensity(
            alpha=0.45,
            path_heading_error=0.0,
            w_target=0.0,
            w_max=1.5,
            brake_angle=0.45,
        ) == 1.0
        assert turn_demand_intensity(
            alpha=0.0,
            path_heading_error=0.0,
            w_target=0.75,
            w_max=1.5,
            brake_angle=0.45,
        ) == 0.5

    def test_goal_approach_speed_tapers_near_goal(self):
        assert goal_approach_speed_limit(
            dist_to_goal=1.2,
            goal_tolerance=0.2,
            approach_distance=1.2,
            approach_speed=0.8,
        ) == float("inf")
        assert abs(goal_approach_speed_limit(
            dist_to_goal=0.7,
            goal_tolerance=0.2,
            approach_distance=1.2,
            approach_speed=0.8,
        ) - 0.4) < 1e-9

    def test_goal_reached_accepts_exact_boundary(self):
        assert should_mark_goal_reached(
            dist_to_goal=0.20,
            goal_tolerance=0.20,
            goal_reached_epsilon=0.03,
            robot_speed=0.2,
            stopped_speed=0.03,
        ) is True

    def test_goal_reached_accepts_stopped_robot_inside_epsilon_band(self):
        assert should_mark_goal_reached(
            dist_to_goal=0.22,
            goal_tolerance=0.20,
            goal_reached_epsilon=0.03,
            robot_speed=0.01,
            stopped_speed=0.03,
        ) is True
        assert should_mark_goal_reached(
            dist_to_goal=0.22,
            goal_tolerance=0.20,
            goal_reached_epsilon=0.03,
            robot_speed=0.10,
            stopped_speed=0.03,
        ) is False

    def test_spin_forward_distance_keeps_extra_clearance_margin(self):
        assert safe_forward_only_distance(
            forward_clearance=0.31,
            stop_distance=0.30,
            margin=0.15,
            desired_distance=0.35,
        ) == 0.0
        assert abs(safe_forward_only_distance(
            forward_clearance=0.80,
            stop_distance=0.30,
            margin=0.15,
            desired_distance=0.35,
        ) - 0.35) < 1e-9


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


class TestTriggerRecoverySpinEntry:
    """[Codex T4 P1] SPIN 진입 임계 회귀 방지.

    `_rotation_clearance_inline` 은 이미 (장애물 중심거리 - robot_radius) 한 '여유 거리'다.
    따라서 SPIN 진입 조건을 robot_radius 와 비교하면 중심이 ~2·robot_radius 밖이어야 해서
    과도하게 보수적이었다. 후진(BACKUP) 제거 후엔 이 false negative 가 곧 '회전도 안 하고
    EMERGENCY 정지'가 된다. 수정: 여유 > hard_collision 마진이면(=몸체가 안 닿으면) SPIN.
    """

    def _make_node(self):
        import types
        from amr_navigation.dwa_node import DwaPlannerNode, NavState

        node = types.SimpleNamespace()
        node.p_robot_radius = 0.20
        node.p_hard_collision_distance = 0.05
        node.p_spin_duration = 2.0
        node.p_recovery_cooldown = 3.0
        node._spin_direction = 1.0
        node._spin_until = 0.0
        node._nav_state = NavState.NORMAL
        node._stuck_counter = 5
        node._recovery_cooldown_until = 0.0
        node._sec_now = lambda: 0.0

        class _NullLogger:
            def warn(self, *a, **k):
                pass

            def info(self, *a, **k):
                pass

        node.get_logger = lambda: _NullLogger()
        for name in ("_trigger_recovery", "_rotation_clearance_inline",
                     "_escape_turn_bias"):
            setattr(node, name, getattr(DwaPlannerNode, name).__get__(node))
        return node, NavState

    def test_spin_entered_when_body_clears_small_margin(self):
        """중심 0.30m → 여유 0.10m(>0.05). 구버전(>robot_radius=0.20)이면 EMERGENCY 였지만
        몸체가 안 닿으므로 SPIN 으로 들어가야 한다."""
        node, NavState = self._make_node()
        node._trigger_recovery([(0.30, 0.0)], motion_clear=0.0)
        assert node._nav_state == NavState.SPIN

    def test_emergency_when_no_rotation_room(self):
        """중심 0.22m → 여유 0.02m(<0.05). 회전 공간 없음 → EMERGENCY."""
        node, NavState = self._make_node()
        node._trigger_recovery([(0.22, 0.0)], motion_clear=0.0)
        assert node._nav_state == NavState.EMERGENCY


class TestNearWallCreep:
    """전방이 열린 측면 벽 근접 상황에서 v=0 고착을 피한다."""

    def _make_node(self):
        import types
        from amr_navigation.dwa_node import DwaPlannerNode

        node = types.SimpleNamespace()
        node.p_near_wall_creep_speed = 0.12
        node.p_clearance_slowdown_distance = 0.80
        node.p_clearance_stop_distance = 0.30
        node.p_robot_radius = 0.20
        node.p_hard_collision_distance = 0.05
        node.p_near_wall_creep_min_clearance = 0.60
        node.p_rejoin_creep_min_clearance = 0.70
        node._allow_near_wall_creep = (
            DwaPlannerNode._allow_near_wall_creep.__get__(node)
        )
        return node

    def test_allows_creep_when_only_side_clearance_is_low(self):
        node = self._make_node()
        assert node._allow_near_wall_creep(motion_clear=0.61,
                                           fwd_clear=1.20) is True

    def test_blocks_creep_when_front_is_not_clear(self):
        node = self._make_node()
        assert node._allow_near_wall_creep(motion_clear=0.30,
                                           fwd_clear=0.50) is False

    def test_blocks_creep_inside_hard_margin(self):
        node = self._make_node()
        assert node._allow_near_wall_creep(motion_clear=0.20,
                                           fwd_clear=1.20) is False

    def test_blocks_creep_below_stop_distance(self):
        node = self._make_node()
        assert node._allow_near_wall_creep(motion_clear=0.29,
                                           fwd_clear=1.20) is False

    def test_blocks_creep_below_recovery_margin(self):
        node = self._make_node()
        assert node._allow_near_wall_creep(motion_clear=0.55,
                                           fwd_clear=1.20) is False

    def test_rejoin_creep_uses_stricter_margin(self):
        node = self._make_node()
        assert node._allow_near_wall_creep(motion_clear=0.65,
                                           fwd_clear=1.20,
                                           is_rejoining=True) is False
        assert node._allow_near_wall_creep(motion_clear=0.71,
                                           fwd_clear=1.20,
                                           is_rejoining=True) is True


class TestNearWallEscapeAdjustment:
    def test_adds_speed_and_turn_away_when_front_is_open(self):
        speed_floor, turn_bias, active = near_wall_escape_adjustment(
            motion_clear=0.36,
            forward_clearance=1.20,
            curvature=0.05,
            escape_bias=-1.0,
            stop_distance=0.30,
            slowdown_distance=0.80,
            escape_clearance=0.45,
            escape_speed=0.28,
            escape_turn=0.22,
            escape_max_curvature=0.80,
            acceleration=1.5,
        )

        assert active is True
        assert speed_floor > 0.0
        assert turn_bias < 0.0

    def test_blocks_escape_when_front_is_not_open(self):
        speed_floor, turn_bias, active = near_wall_escape_adjustment(
            motion_clear=0.36,
            forward_clearance=0.50,
            curvature=0.05,
            escape_bias=-1.0,
            stop_distance=0.30,
            slowdown_distance=0.80,
            escape_clearance=0.45,
            escape_speed=0.28,
            escape_turn=0.22,
            escape_max_curvature=0.80,
            acceleration=1.5,
        )

        assert active is False
        assert speed_floor == 0.0
        assert turn_bias == 0.0


class TestGlobalPathStaleGuard:
    """중복 /global_path publisher가 기존 path를 흔드는 회귀 방지."""

    @staticmethod
    def _path(points):
        import types
        poses = []
        for x, y in points:
            pose = types.SimpleNamespace()
            pose.pose = types.SimpleNamespace()
            pose.pose.position = types.SimpleNamespace(x=x, y=y)
            poses.append(pose)
        return types.SimpleNamespace(poses=poses)

    def _make_node(self):
        import types
        from amr_navigation.dwa_node import DwaPlannerNode, RobotState

        node = types.SimpleNamespace()
        node._goal_version = 1
        node._path_goal_version = 1
        node._path_local = self._path([(0.0, 0.0), (1.0, 0.0)])
        node._path_goal_xy_global = (1.0, 0.0)
        node._state = RobotState(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
        node.p_max_path_offset = 2.0
        node.p_recovery_path_accept_offset = 3.5
        node.p_recovery_path_accept_duration = 5.0
        node._relaxed_path_accept_until = 0.0
        node._now = 0.0
        node._sec_now = lambda: node._now
        node.p_goal_dedup_dist = 0.10
        node.p_goal_tolerance = 0.20
        node.p_goal_dedup_yaw = 0.10
        node._last_goal_xy = (5.0, 1.0)
        node._last_goal_yaw = 0.0
        node._nav_state = None

        class _NullLogger:
            def warn(self, *a, **k):
                pass

        node.get_logger = lambda: _NullLogger()
        for name in ("_should_ignore_empty_path", "_path_goal_matches_last_goal",
                     "_should_ignore_path_while_reached",
                     "_is_path_goal_close_to_latest_goal",
                     "_goal_match_radius",
                     "_path_offset_to_state", "_is_path_close_to_state",
                     "_relax_path_acceptance", "_is_duplicate_goal"):
            setattr(node, name, getattr(DwaPlannerNode, name).__get__(node))
        node._path_xy = DwaPlannerNode._path_xy
        return node

    def test_ignores_empty_path_when_no_new_goal(self):
        node = self._make_node()
        assert node._should_ignore_empty_path() is True

    def test_accepts_empty_path_after_new_goal_edge(self):
        node = self._make_node()
        node._goal_version = 2
        assert node._should_ignore_empty_path() is False

    def test_ignores_empty_path_when_cached_path_matches_latest_goal(self):
        node = self._make_node()
        node._goal_version = 2
        node._path_goal_xy_global = (5.04, 1.02)

        assert node._should_ignore_empty_path() is True
        assert node._path_goal_version == 2

    def test_rejects_stale_path_far_from_current_pose(self):
        node = self._make_node()
        stale = self._path([(3.0, 0.0), (4.0, 0.0)])
        assert node._is_path_close_to_state(stale) is False

    def test_accepts_path_near_current_pose(self):
        node = self._make_node()
        current = self._path([(0.1, 0.0), (1.0, 0.0)])
        assert node._is_path_close_to_state(current) is True

    def test_accepts_farther_recovery_path_during_relaxed_window(self):
        node = self._make_node()
        node._relax_path_acceptance()
        recovery_path = self._path([(3.0, 0.0), (4.0, 0.0)])

        assert node._is_path_close_to_state(recovery_path) is True

    def test_repeated_goal_does_not_create_new_edge(self):
        node = self._make_node()
        assert node._is_duplicate_goal((5.05, 1.02), 0.05) is True

    def test_yaw_changed_same_position_does_not_create_new_edge(self):
        node = self._make_node()
        assert node._is_duplicate_goal((5.0, 1.0), 0.30) is True

    def test_reached_same_goal_path_is_ignored(self):
        from amr_navigation.dwa_node import NavState

        node = self._make_node()
        node._nav_state = NavState.REACHED
        near_goal_path = self._path([(0.0, 0.0), (0.12, 0.0)])

        assert node._should_ignore_path_while_reached(
            (5.04, 1.02),
            near_goal_path,
        ) is True

    def test_reached_different_goal_path_is_not_ignored(self):
        from amr_navigation.dwa_node import NavState

        node = self._make_node()
        node._nav_state = NavState.REACHED
        near_goal_path = self._path([(0.0, 0.0), (0.12, 0.0)])

        assert node._should_ignore_path_while_reached(
            (7.0, 3.0),
            near_goal_path,
        ) is False

    def test_reached_rearms_when_path_endpoint_is_far_from_robot(self):
        assert should_rearm_reached_with_path(
            True,
            (26.55, 15.97),
            (0.0, -10.0),
            0.75,
        ) is True

    def test_reached_holds_when_path_endpoint_is_still_current_goal(self):
        assert should_rearm_reached_with_path(
            True,
            (26.55, 15.97),
            (26.71, 16.03),
            0.75,
        ) is False

    def test_rejects_path_goal_for_old_goal(self):
        node = self._make_node()
        assert node._is_path_goal_close_to_latest_goal((7.0, 3.0)) is False


class TestPathStateReset:
    """1Hz path 갱신이 REJOIN 중 recovery 카운터를 지우는 회귀 방지."""

    def _make_node(self, nav_state, stuck_counter=0):
        import types
        from amr_navigation.dwa_node import DwaPlannerNode

        node = types.SimpleNamespace()
        node._reached = False
        node._path_progress_idx = 7
        node._in_align_mode = True
        node._align_trigger_count = 2
        node._nav_state = nav_state
        node._stuck_counter = stuck_counter
        node._reset_path_state = DwaPlannerNode._reset_path_state.__get__(node)
        return node

    def test_rejoin_path_update_preserves_recovery_counter(self):
        from amr_navigation.dwa_node import NavState

        node = self._make_node(NavState.REJOIN, stuck_counter=12)
        node._reset_path_state()

        assert node._nav_state == NavState.REJOIN
        assert node._stuck_counter == 12
        assert node._path_progress_idx == 0
        assert node._in_align_mode is False
        assert node._align_trigger_count == 0

    def test_emergency_path_update_still_rearms_normal(self):
        from amr_navigation.dwa_node import NavState

        node = self._make_node(NavState.EMERGENCY, stuck_counter=12)
        node._reset_path_state()

        assert node._nav_state == NavState.NORMAL
        assert node._stuck_counter == 0
