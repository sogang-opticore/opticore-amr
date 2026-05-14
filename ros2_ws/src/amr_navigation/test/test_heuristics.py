"""
test_heuristics.py — heuristics.py 단위 테스트
ROS 없이 순수 Python으로 실행 가능.

실행 방법:
    cd amr_navigation
    python3 -m pytest test/test_heuristics.py -v
"""

import math
import pytest
from amr_navigation.heuristics import heuristic, movement_cost


# ══════════════════════════════════════════════════════════════════
# heuristic() 테스트
# ══════════════════════════════════════════════════════════════════

class TestManhattan:
    def test_same_cell(self):
        """같은 셀이면 비용 0."""
        assert heuristic((0, 0), (0, 0), 'manhattan') == 0.0

    def test_straight(self):
        """직선 이동 — 행 방향 5칸."""
        assert heuristic((0, 0), (5, 0), 'manhattan') == 5.0

    def test_diagonal(self):
        """대각 방향 — manhattan은 dr+dc."""
        assert heuristic((0, 0), (3, 4), 'manhattan') == 7.0

    def test_symmetry(self):
        """a→b 와 b→a 비용은 같아야 함."""
        a, b = (2, 3), (7, 1)
        assert heuristic(a, b, 'manhattan') == heuristic(b, a, 'manhattan')


class TestEuclidean:
    def test_same_cell(self):
        assert heuristic((0, 0), (0, 0), 'euclidean') == 0.0

    def test_straight(self):
        assert heuristic((0, 0), (3, 4), 'euclidean') == pytest.approx(5.0)

    def test_symmetry(self):
        a, b = (1, 2), (5, 6)
        assert heuristic(a, b, 'euclidean') == pytest.approx(heuristic(b, a, 'euclidean'))

    def test_admissible_vs_movement(self):
        """
        Euclidean은 항상 실제 이동 비용 이하여야 함 (admissible).
        대각 1칸 이동: 실제 비용 √2, euclidean도 √2 → 같거나 작아야 함.
        """
        cost = heuristic((0, 0), (1, 1), 'euclidean')
        assert cost <= math.sqrt(2) + 1e-9


class TestOctile:
    def test_same_cell(self):
        assert heuristic((0, 0), (0, 0), 'octile') == 0.0

    def test_straight(self):
        """직선 이동 — octile은 manhattan과 동일."""
        assert heuristic((0, 0), (5, 0), 'octile') == pytest.approx(5.0)

    def test_pure_diagonal(self):
        """순수 대각 이동 — 비용은 √2 * n."""
        n = 4
        assert heuristic((0, 0), (n, n), 'octile') == pytest.approx(math.sqrt(2) * n)

    def test_mixed(self):
        """대각 + 직선 혼합 — octile 공식 검증."""
        # dr=3, dc=5 → max=5, min=3 → 5 + (√2-1)*3
        expected = 5 + (math.sqrt(2) - 1) * 3
        assert heuristic((0, 0), (3, 5), 'octile') == pytest.approx(expected)

    def test_symmetry(self):
        a, b = (2, 7), (9, 3)
        assert heuristic(a, b, 'octile') == pytest.approx(heuristic(b, a, 'octile'))

    def test_admissible_vs_movement(self):
        """
        Octile은 8-connected 실제 이동 비용과 정확히 일치해야 함.
        대각 1칸: 비용 √2, octile도 √2.
        """
        assert heuristic((0, 0), (1, 1), 'octile') == pytest.approx(math.sqrt(2))
        assert heuristic((0, 0), (1, 0), 'octile') == pytest.approx(1.0)


class TestInvalidMode:
    def test_invalid_mode(self):
        """잘못된 mode는 ValueError 발생."""
        with pytest.raises(ValueError):
            heuristic((0, 0), (1, 1), 'invalid')


# ══════════════════════════════════════════════════════════════════
# movement_cost() 테스트
# ══════════════════════════════════════════════════════════════════

class TestMovementCost:
    def test_straight_up(self):
        """위쪽 직선 이동."""
        assert movement_cost((1, 0), (0, 0)) == pytest.approx(1.0)

    def test_straight_right(self):
        """오른쪽 직선 이동."""
        assert movement_cost((0, 0), (0, 1)) == pytest.approx(1.0)