"""astar_node.py 재계획 실패 처리 회귀 테스트.

ROS 노드를 띄우지 않고, AstarPlanner의 실패 처리 헬퍼만 바인딩해 검증한다.
"""

import types

from amr_navigation.astar_node import (
    AstarPlanner,
    cell_path_length,
    cells_on_segment,
    clearance_switch_should_replace_previous,
    remaining_path_metrics,
    should_apply_path_hysteresis,
    should_retain_previous_path,
)


class _FakeLogger:
    def __init__(self):
        self.warns = []
        self.infos = []

    def warn(self, msg, **_kwargs):
        self.warns.append(msg)

    def info(self, msg, **_kwargs):
        self.infos.append(msg)


def _make_node(has_valid_path):
    node = types.SimpleNamespace()
    node._has_valid_path_for_goal = has_valid_path
    node.published_empty = 0
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    node._publish_empty_path = lambda: setattr(
        node, "published_empty", node.published_empty + 1)
    node._handle_plan_failure = AstarPlanner._handle_plan_failure.__get__(node)
    return node


class TestAstarReplanFailure:
    def test_replan_failure_keeps_existing_valid_path(self):
        node = _make_node(has_valid_path=True)

        node._handle_plan_failure("경로 없음", clear_on_failure=False)

        assert node.published_empty == 0
        assert node._has_valid_path_for_goal is True
        assert "기존 /global_path 유지" in node.logger.warns[-1]

    def test_new_goal_failure_clears_path(self):
        node = _make_node(has_valid_path=True)

        node._handle_plan_failure("경로 없음", clear_on_failure=True)

        assert node.published_empty == 1
        assert node._has_valid_path_for_goal is False

    def test_replan_failure_without_cache_publishes_empty_path(self):
        node = _make_node(has_valid_path=False)

        node._handle_plan_failure("경로 없음", clear_on_failure=False)

        assert node.published_empty == 1
        assert node._has_valid_path_for_goal is False


def _make_status_node():
    node = types.SimpleNamespace()
    node.goal = object()
    node.map_data = object()
    node.status_replan_states = {"EMERGENCY", "PATH_LOST", "RECOVERY_DONE"}
    node.dynamic_status_replan_states = {"DYNAMIC_BLOCKED", "INSIDE_DYNAMIC_ZONE"}
    node.status_replan_after_states = {"FORWARD_ONLY", "RECOVERY"}
    node.status_replan_reset_states = {
        "NORMAL", "ALIGN", "STOPPED", "REACHED", "GOAL_REACHED"}
    node.status_replan_cooldown = 2.0
    node.dynamic_status_replan_cooldown = 0.5
    node._status_replan_armed = True
    node._last_status_replan_time = -float("inf")
    node._last_dynamic_status_replan_time = -float("inf")
    node._last_dwa_status = None
    node.now = 10.0
    node.plan_calls = []
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    node._sec_now = lambda: node.now
    node._plan = lambda **kwargs: (
        node.plan_calls.append(kwargs) or True
    )
    node._request_status_replan = AstarPlanner._request_status_replan.__get__(node)
    node._on_dwa_status = AstarPlanner._on_dwa_status.__get__(node)
    return node


class TestAstarStatusEventReplan:
    def test_emergency_triggers_once_until_normal_rearms(self):
        node = _make_status_node()

        node._on_dwa_status(types.SimpleNamespace(data="EMERGENCY"))
        node._on_dwa_status(types.SimpleNamespace(data="EMERGENCY"))

        assert len(node.plan_calls) == 1

        node.now = 13.0
        node._on_dwa_status(types.SimpleNamespace(data="NORMAL"))
        node._on_dwa_status(types.SimpleNamespace(data="EMERGENCY"))

        assert len(node.plan_calls) == 2

    def test_forward_only_to_normal_triggers_replan_after_recovery(self):
        node = _make_status_node()

        node._on_dwa_status(types.SimpleNamespace(data="FORWARD_ONLY"))
        node.now = 13.0
        node._on_dwa_status(types.SimpleNamespace(data="NORMAL"))

        assert len(node.plan_calls) == 1

    def test_recovery_done_triggers_immediate_replan(self):
        node = _make_status_node()

        node._on_dwa_status(types.SimpleNamespace(data="RECOVERY_DONE"))

        assert len(node.plan_calls) == 1

    def test_dynamic_status_uses_short_cooldown_without_disarming_regular_events(self):
        node = _make_status_node()
        node.status_replan_states.add("INSIDE_DYNAMIC_ZONE")

        node._on_dwa_status(types.SimpleNamespace(data="INSIDE_DYNAMIC_ZONE"))
        node.now = 10.3
        node._on_dwa_status(types.SimpleNamespace(data="INSIDE_DYNAMIC_ZONE"))
        node.now = 10.6
        node._on_dwa_status(types.SimpleNamespace(data="INSIDE_DYNAMIC_ZONE"))
        node.now = 10.7
        node._on_dwa_status(types.SimpleNamespace(data="EMERGENCY"))

        assert len(node.plan_calls) == 3

    def test_recovery_to_stopped_triggers_replan_fallback(self):
        node = _make_status_node()

        node._on_dwa_status(types.SimpleNamespace(data="RECOVERY"))
        node.now = 13.0
        node._on_dwa_status(types.SimpleNamespace(data="STOPPED"))

        assert len(node.plan_calls) == 1

    def test_stopped_near_wall_does_not_trigger_path_churn(self):
        node = _make_status_node()

        node._on_dwa_status(types.SimpleNamespace(data="STOPPED_NEAR_WALL"))

        assert node.plan_calls == []


class TestAstarPathHysteresis:
    def test_new_goal_force_publish_temporarily_bypasses_hysteresis(self):
        assert should_apply_path_hysteresis(
            True,
            False,
            now=12.0,
            force_publish_until=15.0,
        ) is False

    def test_path_hysteresis_rearms_after_force_publish_window(self):
        assert should_apply_path_hysteresis(
            True,
            False,
            now=15.0,
            force_publish_until=15.0,
        ) is True

    def test_retain_previous_when_candidate_improvement_is_tiny(self):
        previous = [(0, 0), (0, 10), (5, 10)]
        candidate = [(0, 0), (0, 9), (5, 9)]

        keep, improvement, prev_len, cand_len, offset = should_retain_previous_path(
            previous_cells=previous,
            candidate_cells=candidate,
            start_cell=(0, 0),
            resolution=0.05,
            switch_hysteresis=0.35,
            max_start_offset=0.80,
        )

        assert keep is True
        assert improvement < 0.35
        assert prev_len > cand_len
        assert offset == 0.0

    def test_candidate_wins_when_improvement_exceeds_hysteresis(self):
        previous = [(0, 0), (0, 30), (10, 30)]
        candidate = [(0, 0), (0, 8), (10, 8)]

        keep, improvement, *_ = should_retain_previous_path(
            previous_cells=previous,
            candidate_cells=candidate,
            start_cell=(0, 0),
            resolution=0.05,
            switch_hysteresis=0.35,
            max_start_offset=0.80,
        )

        assert keep is False
        assert improvement > 0.35

    def test_hysteresis_releases_when_robot_far_from_previous_path(self):
        previous = [(0, 0), (0, 10)]
        candidate = [(20, 0), (20, 10)]

        keep, *_ = should_retain_previous_path(
            previous_cells=previous,
            candidate_cells=candidate,
            start_cell=(20, 0),
            resolution=0.05,
            switch_hysteresis=0.35,
            max_start_offset=0.80,
        )

        assert keep is False

    def test_clearance_switch_replaces_wall_hugging_previous_path(self):
        assert clearance_switch_should_replace_previous(
            previous_min_clearance=0.55,
            candidate_min_clearance=0.90,
            extra_length=2.4,
            bad_clearance=0.80,
            min_clearance_gain=0.25,
            max_extra_length=3.0,
        ) is True

    def test_clearance_switch_keeps_candidate_from_detouring_too_far(self):
        assert clearance_switch_should_replace_previous(
            previous_min_clearance=0.55,
            candidate_min_clearance=1.10,
            extra_length=3.5,
            bad_clearance=0.80,
            min_clearance_gain=0.25,
            max_extra_length=3.0,
        ) is False

    def test_path_min_clearance_ahead_skips_shared_start_wall(self):
        node = types.SimpleNamespace()
        node.map_data = types.SimpleNamespace(
            info=types.SimpleNamespace(resolution=0.05))
        clearances = {
            (0, 0): 0.30,
            (0, 5): 0.35,
            (0, 20): 1.10,
            (0, 30): 1.20,
        }
        node._clearance_at_cell = lambda c: clearances[c]
        node._path_min_clearance = AstarPlanner._path_min_clearance.__get__(node)
        node._path_min_clearance_ahead = (
            AstarPlanner._path_min_clearance_ahead.__get__(node))

        result = node._path_min_clearance_ahead(
            [(0, 0), (0, 5), (0, 20), (0, 30)],
            start_cell=(0, 0),
            skip_distance=0.75,
        )

        assert result == 1.10

    def test_remaining_path_metrics_uses_nearest_projection_cell(self):
        remaining, offset = remaining_path_metrics(
            cells=[(0, 0), (0, 10), (0, 20)],
            start_cell=(1, 10),
            resolution=0.05,
        )

        assert abs(offset - 0.05) < 1e-9
        assert abs(remaining - 0.55) < 1e-9

    def test_cells_on_segment_returns_dense_unique_cells(self):
        cells = cells_on_segment((0, 0), (0, 3))

        assert cells == [(0, 0), (0, 1), (0, 2), (0, 3)]
        assert abs(cell_path_length(cells, 0.05) - 0.15) < 1e-9
