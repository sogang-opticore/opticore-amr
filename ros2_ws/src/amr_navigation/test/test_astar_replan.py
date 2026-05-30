"""astar_node.py 재계획 실패 처리 회귀 테스트.

ROS 노드를 띄우지 않고, AstarPlanner의 실패 처리 헬퍼만 바인딩해 검증한다.
"""

import types

from amr_navigation.astar_node import AstarPlanner


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
    node.status_replan_after_states = {"FORWARD_ONLY", "RECOVERY"}
    node.status_replan_reset_states = {
        "NORMAL", "ALIGN", "STOPPED", "REACHED", "GOAL_REACHED"}
    node.status_replan_cooldown = 2.0
    node._status_replan_armed = True
    node._last_status_replan_time = -float("inf")
    node._last_dwa_status = None
    node.now = 10.0
    node.plan_calls = []
    node.logger = _FakeLogger()
    node.get_logger = lambda: node.logger
    node._sec_now = lambda: node.now
    node._plan = lambda clear_on_failure=False: (
        node.plan_calls.append(clear_on_failure) or True
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
