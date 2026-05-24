"""
test_frontier_explorer.py — frontier_explorer.py 순수 함수 단위 테스트

ROS 의존성 없이 순수 알고리즘 검증.

실행:
    colcon test --packages-select amr_navigation
    또는
    python3 -m pytest src/amr_navigation/test/test_frontier_explorer.py -v
"""

from amr_navigation.frontier_explorer import (
    find_frontier_cells,
    cluster_frontiers,
    cluster_centroid,
    select_best_cluster,
)


# ---------------------------------------------------------------------------
# find_frontier_cells
# ---------------------------------------------------------------------------

class TestFindFrontier:
    """frontier 셀 검출 — free 셀 중 unknown(8-이웃) 있는 셀."""

    def test_3x3_known_pattern(self):
        """
        3x3 grid:
            row0: 0 -1 -1
            row1: 0  0 -1
            row2: 0  0  0
        free 셀 6개 중 frontier = (0,0),(0,1),(1,1),(1,2),(2,2) 5개.
        (0,2)는 8-이웃 어디에도 unknown이 없으므로 제외.
        """
        data = [0, -1, -1,
                0,  0, -1,
                0,  0,  0]
        fc = find_frontier_cells(data, 3, 3)
        assert sorted(fc) == sorted([(0, 0), (0, 1), (1, 1), (1, 2), (2, 2)])

    def test_no_unknown_no_frontier(self):
        """unknown이 없는 그리드 → frontier 0개."""
        data = [0, 0, 0, 0]
        assert find_frontier_cells(data, 2, 2) == []

    def test_all_unknown_no_frontier(self):
        """모두 unknown → free 셀 자체가 없어 frontier 0개."""
        data = [-1, -1, -1, -1]
        assert find_frontier_cells(data, 2, 2) == []

    def test_occupied_not_frontier(self):
        """점유(>=50) 셀은 frontier 아님."""
        data = [100, -1,
                100, -1]
        assert find_frontier_cells(data, 2, 2) == []

    def test_free_threshold(self):
        """free_threshold 미만이어야 free로 인정."""
        # v=30이고 옆이 unknown이면, threshold=50에서 frontier로 잡힘
        data = [30, -1]
        assert find_frontier_cells(data, 2, 1, free_threshold=50) == [(0, 0)]
        # threshold=20이면 30도 free 아님
        assert find_frontier_cells(data, 2, 1, free_threshold=20) == []


# ---------------------------------------------------------------------------
# cluster_frontiers
# ---------------------------------------------------------------------------

class TestClusterFrontiers:
    """BFS로 연결된 frontier 셀 묶기."""

    def test_single_cluster(self):
        """연결된 셀들은 1 cluster."""
        cells = [(0, 0), (0, 1), (1, 0), (1, 1)]
        clusters = cluster_frontiers(cells, min_size=1)
        assert len(clusters) == 1
        assert len(clusters[0]) == 4

    def test_two_disjoint_clusters(self):
        """떨어진 셀들은 별개 cluster."""
        cells = [(0, 0), (0, 1), (10, 10), (10, 11)]
        clusters = cluster_frontiers(cells, min_size=1)
        assert len(clusters) == 2

    def test_min_size_filter(self):
        """min_size 미만 cluster는 제외."""
        cells = [(0, 0), (0, 1), (10, 10)]
        # (10,10)은 혼자라서 size=1 → 제외
        clusters = cluster_frontiers(cells, min_size=2)
        assert len(clusters) == 1
        assert len(clusters[0]) == 2

    def test_8_connectivity(self):
        """대각선도 연결로 본다 (8-conn)."""
        # 대각 인접
        cells = [(0, 0), (1, 1)]
        clusters = cluster_frontiers(cells, min_size=1)
        assert len(clusters) == 1


# ---------------------------------------------------------------------------
# cluster_centroid
# ---------------------------------------------------------------------------

class TestCentroid:
    def test_single_cell(self):
        assert cluster_centroid([(3, 7)]) == (3.0, 7.0)

    def test_average(self):
        cx, cy = cluster_centroid([(0, 0), (2, 2), (4, 0)])
        assert abs(cx - 2.0) < 1e-9
        assert abs(cy - 2.0 / 3) < 1e-9

    def test_empty(self):
        assert cluster_centroid([]) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# select_best_cluster
# ---------------------------------------------------------------------------

class TestSelectBest:
    """점수 = size / distance — 크고 가까운 cluster를 고르되 visited는 제외."""

    def test_empty_returns_none(self):
        assert select_best_cluster([], (0, 0), [], 0.5, 0.5) is None

    def test_size_distance_tradeoff(self):
        """가까운 small cluster vs 멀고 큰 cluster — 점수 더 높은 쪽."""
        big_far = [(10, 10), (10, 11), (11, 10), (11, 11), (12, 10)]  # size 5
        small_near = [(1, 1), (1, 2), (2, 1)]                          # size 3
        # robot (0,0) → small_near centroid ≈ (1.33, 1.33), dist ≈ 1.88, score ≈ 1.6
        #             big_far centroid ≈ (10.8, 10.4), dist ≈ 15, score ≈ 0.33
        b = select_best_cluster([big_far, small_near], (0, 0),
                                visited_centroids=[],
                                min_dist_cells=0.5,
                                visit_radius_cells=0.5)
        assert b is not None
        assert b[1] in [(1, 1), (1, 2), (2, 1)]   # small_near centroid 근처

    def test_min_dist_filter(self):
        """너무 가까운 cluster는 제외."""
        c = [(0, 0), (0, 1), (1, 0)]  # robot 바로 옆
        b = select_best_cluster([c], (0, 0),
                                visited_centroids=[],
                                min_dist_cells=5.0,
                                visit_radius_cells=0.5)
        assert b is None

    def test_visited_skipped(self):
        """visited 영역 내 cluster centroid는 스킵."""
        big = [(10, 10), (10, 11), (11, 10), (11, 11), (12, 10)]
        small = [(1, 1), (1, 2), (2, 1)]
        # big centroid (10.8, 10.4) — visited (11, 10) 와 거리 ~0.4 → 스킵
        # small centroid (1.33, 1.33) 살아남음
        b = select_best_cluster([big, small], (0, 0),
                                visited_centroids=[(11, 10)],
                                min_dist_cells=0.5,
                                visit_radius_cells=3.0)
        assert b is not None
        assert b[1] in [(1, 1), (1, 2), (2, 1)]

    def test_all_visited_returns_none(self):
        c = [(10, 10), (10, 11), (11, 10)]
        b = select_best_cluster([c], (0, 0),
                                visited_centroids=[(10, 10)],
                                min_dist_cells=0.5,
                                visit_radius_cells=3.0)
        assert b is None
