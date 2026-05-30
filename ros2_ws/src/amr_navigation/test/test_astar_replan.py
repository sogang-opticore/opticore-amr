"""astar_node.py 재계획 실패 처리 회귀 테스트.

ROS 노드를 띄우지 않고, AstarPlanner의 실패 처리 헬퍼만 바인딩해 검증한다.
"""

import types

from amr_navigation.astar_node import AstarPlanner


class _FakeLogger:
    def __init__(self):
        self.warns = []

    def warn(self, msg, **_kwargs):
        self.warns.append(msg)


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
