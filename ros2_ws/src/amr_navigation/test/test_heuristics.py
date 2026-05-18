"""heuristics.py 단위 테스트 — Notion 명세 §10 커버리지 70% 목표 시작점."""

from math import sqrt, isclose

import pytest

from amr_navigation.heuristics import (
    GridInfo,
    euclidean_distance,
    get_heuristic,
    grid_to_world,
    grid_value,
    in_bounds,
    manhattan_distance,
    neighbors,
    octile_distance,
    step_cost,
    world_to_grid,
)


# ---------------------------------------------------------------------------
# 휴리스틱
# ---------------------------------------------------------------------------

class TestHeuristics:
    def test_manhattan_known(self):
        assert manhattan_distance((0, 0), (3, 4)) == 7

    def test_euclidean_known(self):
        assert isclose(euclidean_distance((0, 0), (3, 4)), 5.0)

    def test_octile_known(self):
        # max=4, min=3 → 4 + 0.4142*3 ≈ 5.2426
        expected = 4 + (sqrt(2) - 1) * 3
        assert isclose(octile_distance((0, 0), (3, 4)), expected, rel_tol=1e-9)

    def test_zero_distance(self):
        for h in (manhattan_distance, euclidean_distance, octile_distance):
            assert h((5, 7), (5, 7)) == 0

    def test_admissibility_ordering(self):
        # 8-conn에서: euclidean ≤ octile ≤ manhattan(과대평가)
        a, b = (0, 0), (5, 3)
        eucl = euclidean_distance(a, b)
        oct = octile_distance(a, b)
        man = manhattan_distance(a, b)
        assert eucl <= oct <= man

    def test_get_heuristic_registry(self):
        assert get_heuristic("manhattan") is manhattan_distance
        assert get_heuristic("euclidean") is euclidean_distance
        assert get_heuristic("octile") is octile_distance

    def test_get_heuristic_unknown_raises(self):
        with pytest.raises(KeyError):
            get_heuristic("astar_magic")


# ---------------------------------------------------------------------------
# 좌표 변환
# ---------------------------------------------------------------------------

class TestCoordTransform:
    @pytest.fixture
    def info(self):
        return GridInfo(width=20, height=20, resolution=0.05,
                        origin_x=-0.5, origin_y=-0.5)

    def test_world_to_grid_origin(self, info):
        # origin 자체는 grid (0, 0)
        assert world_to_grid((-0.5, -0.5), info) == (0, 0)

    def test_grid_to_world_center(self, info):
        # 셀 중심이라 origin + 0.5*res
        x, y = grid_to_world((0, 0), info)
        assert isclose(x, -0.5 + 0.025)
        assert isclose(y, -0.5 + 0.025)

    def test_round_trip(self, info):
        cell = (10, 7)
        world = grid_to_world(cell, info)
        recovered = world_to_grid(world, info)
        assert recovered == cell

    def test_in_bounds(self, info):
        assert in_bounds((0, 0), info)
        assert in_bounds((19, 19), info)
        assert not in_bounds((20, 0), info)
        assert not in_bounds((-1, 0), info)


# ---------------------------------------------------------------------------
# 이웃
# ---------------------------------------------------------------------------

class TestNeighbors:
    @pytest.fixture
    def info(self):
        return GridInfo(width=10, height=10, resolution=0.05,
                        origin_x=0.0, origin_y=0.0)

    def test_4_connected_count(self, info):
        ns = list(neighbors((5, 5), info, allow_diagonal=False))
        assert len(ns) == 4

    def test_8_connected_count(self, info):
        ns = list(neighbors((5, 5), info, allow_diagonal=True))
        assert len(ns) == 8

    def test_corner_clipped(self, info):
        # 좌하단 코너에서 4-conn은 2개, 8-conn은 3개
        ns4 = list(neighbors((0, 0), info, allow_diagonal=False))
        assert len(ns4) == 2
        ns8 = list(neighbors((0, 0), info, allow_diagonal=True))
        assert len(ns8) == 3


# ---------------------------------------------------------------------------
# step_cost
# ---------------------------------------------------------------------------

class TestStepCost:
    def test_orthogonal_free(self):
        assert isclose(step_cost((0, 0), (1, 0), 0), 1.0)

    def test_diagonal_free(self):
        assert isclose(step_cost((0, 0), (1, 1), 0), sqrt(2.0))

    def test_occupied_is_inf(self):
        assert step_cost((0, 0), (1, 0), 100) == float("inf")

    def test_unknown_is_inf(self):
        assert step_cost((0, 0), (1, 0), -1) == float("inf")

    def test_inflation_penalty_increases_cost(self):
        free = step_cost((0, 0), (1, 0), 0)
        near_wall = step_cost((0, 0), (1, 0), 50)
        assert near_wall > free


# ---------------------------------------------------------------------------
# grid_value
# ---------------------------------------------------------------------------

class TestGridValue:
    def test_row_major_indexing(self):
        # 3x2 그리드: data = [c0r0, c1r0, c2r0, c0r1, c1r1, c2r1]
        info = GridInfo(width=3, height=2, resolution=1.0,
                        origin_x=0.0, origin_y=0.0)
        data = [10, 20, 30, 40, 50, 60]
        assert grid_value(data, (0, 0), info) == 10
        assert grid_value(data, (2, 0), info) == 30
        assert grid_value(data, (0, 1), info) == 40
        assert grid_value(data, (2, 1), info) == 60

    def test_out_of_bounds_returns_unknown(self):
        info = GridInfo(width=3, height=2, resolution=1.0,
                        origin_x=0.0, origin_y=0.0)
        data = [10, 20, 30, 40, 50, 60]
        assert grid_value(data, (-1, 0), info) == -1
        assert grid_value(data, (5, 5), info) == -1
