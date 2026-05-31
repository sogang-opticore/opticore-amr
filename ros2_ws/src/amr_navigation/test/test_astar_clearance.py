"""A* clearance-aware 경로 선택 회귀 테스트."""

import types

import numpy as np

from amr_navigation.astar_node import (
    AstarPlanner,
    clearance_preference_cost,
    safety_hysteresis_should_retain_previous,
    status_allows_path_hysteresis,
    status_allows_path_safety_retention,
)


def _bind(node, *names):
    for name in names:
        setattr(node, name, getattr(AstarPlanner, name).__get__(node))


class TestAstarClearanceCost:
    def test_clearance_cost_penalizes_wall_edge_cells(self):
        node = types.SimpleNamespace()
        node.clearance_grid = np.array([[0.55, 1.00]], dtype=np.float32)
        node.preferred_clearance = 1.00
        node.clearance_cost_weight = 6.0
        _bind(node, "_clearance_at_cell", "_clearance_cost")

        assert node._clearance_cost((0, 0)) > 0.0
        assert node._clearance_cost((0, 1)) == 0.0

    def test_barrier_cost_strongly_discourages_inflation_edge_cells(self):
        edge_cost = clearance_preference_cost(
            0.55,
            preferred_clearance=1.20,
            clearance_cost_weight=8.0,
            inflation_radius=0.50,
            wall_avoid_clearance=0.85,
            wall_avoid_cost_weight=1.5,
            wall_avoid_min_margin=0.05,
        )
        safer_cost = clearance_preference_cost(
            0.70,
            preferred_clearance=1.20,
            clearance_cost_weight=8.0,
            inflation_radius=0.50,
            wall_avoid_clearance=0.85,
            wall_avoid_cost_weight=1.5,
            wall_avoid_min_margin=0.05,
        )
        open_cost = clearance_preference_cost(
            1.20,
            preferred_clearance=1.20,
            clearance_cost_weight=8.0,
            inflation_radius=0.50,
            wall_avoid_clearance=0.85,
            wall_avoid_cost_weight=1.5,
            wall_avoid_min_margin=0.05,
        )

        assert edge_cost > safer_cost * 10.0
        assert open_cost == 0.0


class TestAstarPathHysteresisStatus:
    def test_hysteresis_is_allowed_in_stable_tracking_states(self):
        stable = {"NORMAL", "ALIGN"}

        assert status_allows_path_hysteresis("NORMAL", stable) is True
        assert status_allows_path_hysteresis("ALIGN", stable) is True

    def test_hysteresis_is_disabled_while_recovering_or_stuck(self):
        stable = {"NORMAL", "ALIGN"}

        assert status_allows_path_hysteresis("STOPPED_NEAR_WALL", stable) is False
        assert status_allows_path_hysteresis("RECOVERY", stable) is False
        assert status_allows_path_hysteresis("REJOIN", stable) is False

    def test_safety_retention_allows_rejoin_but_not_recovery(self):
        safety = {"NORMAL", "ALIGN", "REJOIN"}

        assert status_allows_path_safety_retention("REJOIN", safety) is True
        assert status_allows_path_safety_retention("RECOVERY", safety) is False
        assert status_allows_path_safety_retention("EMERGENCY", safety) is False


class TestAstarSafetyHysteresis:
    def test_retains_safer_previous_path_when_candidate_is_only_modestly_shorter(self):
        assert safety_hysteresis_should_retain_previous(
            previous_min_clearance=1.05,
            candidate_min_clearance=0.70,
            candidate_length_improvement=0.80,
            bad_clearance=0.90,
            min_clearance_loss=0.15,
            max_length_sacrifice=1.60,
        ) is True

    def test_allows_short_candidate_when_length_gain_is_large(self):
        assert safety_hysteresis_should_retain_previous(
            previous_min_clearance=1.05,
            candidate_min_clearance=0.70,
            candidate_length_improvement=1.80,
            bad_clearance=0.90,
            min_clearance_loss=0.15,
            max_length_sacrifice=1.60,
        ) is False

    def test_allows_candidate_when_clearance_loss_is_small(self):
        assert safety_hysteresis_should_retain_previous(
            previous_min_clearance=0.95,
            candidate_min_clearance=0.82,
            candidate_length_improvement=0.50,
            bad_clearance=0.90,
            min_clearance_loss=0.15,
            max_length_sacrifice=1.60,
        ) is False


class TestAstarNeighbors:
    def test_diagonal_corner_cut_is_rejected(self):
        node = types.SimpleNamespace()
        node.allow_diagonal = True
        node.inflated_grid = np.zeros((3, 3), dtype=np.uint8)
        node.inflated_grid[1, 2] = 1
        node.inflated_grid[2, 1] = 1
        _bind(node, "_is_free_cell", "_get_neighbors")

        neighbors = node._get_neighbors((1, 1))

        assert (2, 2) not in neighbors


class TestAstarSnap:
    def test_snap_prefers_safer_free_cell_within_radius(self):
        node = types.SimpleNamespace()
        node.goal_snap_radius = 2.0
        node.preferred_clearance = 1.0
        node.inflated_grid = np.ones((5, 5), dtype=np.uint8)
        node.inflated_grid[2, 3] = 0
        node.inflated_grid[2, 4] = 0
        node.clearance_grid = np.zeros((5, 5), dtype=np.float32)
        node.clearance_grid[2, 3] = 0.55
        node.clearance_grid[2, 4] = 0.95
        node.map_data = types.SimpleNamespace(
            info=types.SimpleNamespace(resolution=1.0)
        )
        _bind(node, "_clearance_at_cell", "_snap_to_nearest_free")

        assert node._snap_to_nearest_free((2, 2)) == (2, 4)


class TestAstarSmoothingSafety:
    def test_segment_rejects_diagonal_corner_cut(self):
        node = types.SimpleNamespace()
        node.inflated_grid = np.zeros((3, 3), dtype=np.uint8)
        node.inflated_grid[1, 2] = 1
        node.inflated_grid[2, 1] = 1
        _bind(node, "_is_free_cell", "_segment_is_free")

        assert node._segment_is_free((1, 1), (2, 2)) is False

    def test_smoothing_segment_rejects_low_clearance_shortcut(self):
        node = types.SimpleNamespace()
        node.inflated_grid = np.zeros((3, 3), dtype=np.uint8)
        node.clearance_grid = np.ones((3, 3), dtype=np.float32)
        node.clearance_grid[1, 1] = 0.70
        node.smoothing_min_clearance = 0.80
        _bind(
            node,
            "_is_free_cell",
            "_clearance_at_cell",
            "_segment_is_free",
            "_segment_min_clearance",
            "_segment_is_safe_for_smoothing",
        )

        assert node._segment_is_safe_for_smoothing((1, 0), (1, 2)) is False


class TestAstarGoalDirectPath:
    def test_direct_goal_path_is_used_when_near_and_clear(self):
        node = types.SimpleNamespace()
        node.goal_direct_distance = 2.0
        node.goal_direct_min_clearance = 0.55
        node.inflated_grid = np.zeros((5, 5), dtype=np.uint8)
        node.clearance_grid = np.ones((5, 5), dtype=np.float32)
        _bind(
            node,
            "_is_free_cell",
            "_segment_is_free",
            "_clearance_at_cell",
            "_path_min_clearance",
            "_try_goal_direct_path",
        )

        direct = node._try_goal_direct_path(
            start_cell=(2, 0),
            goal_cell=(2, 4),
            start_world=(0.0, 0.0),
            goal_world=(0.2, 0.0),
        )

        assert direct == [(2, 0), (2, 1), (2, 2), (2, 3), (2, 4)]

    def test_direct_goal_path_rejects_low_clearance_segment(self):
        node = types.SimpleNamespace()
        node.goal_direct_distance = 2.0
        node.goal_direct_min_clearance = 0.55
        node.inflated_grid = np.zeros((5, 5), dtype=np.uint8)
        node.clearance_grid = np.ones((5, 5), dtype=np.float32)
        node.clearance_grid[2, 2] = 0.50
        _bind(
            node,
            "_is_free_cell",
            "_segment_is_free",
            "_clearance_at_cell",
            "_path_min_clearance",
            "_try_goal_direct_path",
        )

        direct = node._try_goal_direct_path(
            start_cell=(2, 0),
            goal_cell=(2, 4),
            start_world=(0.0, 0.0),
            goal_world=(0.2, 0.0),
        )

        assert direct is None

    def test_direct_goal_path_rejects_wall_hugging_final_approach(self):
        node = types.SimpleNamespace()
        node.goal_direct_distance = 2.0
        node.goal_direct_min_clearance = 0.80
        node.inflated_grid = np.zeros((5, 5), dtype=np.uint8)
        node.clearance_grid = np.ones((5, 5), dtype=np.float32)
        node.clearance_grid[2, 2] = 0.75
        _bind(
            node,
            "_is_free_cell",
            "_segment_is_free",
            "_clearance_at_cell",
            "_path_min_clearance",
            "_try_goal_direct_path",
        )

        direct = node._try_goal_direct_path(
            start_cell=(2, 0),
            goal_cell=(2, 4),
            start_world=(0.0, 0.0),
            goal_world=(0.2, 0.0),
        )

        assert direct is None
