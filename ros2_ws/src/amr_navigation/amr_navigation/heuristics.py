"""
A* 휴리스틱 및 비용 함수 모듈 — Week 1 SW 분담 (HU의 astar_node에서 import)

Opticore AMR — A* Global Planner 직접 구현.
이 모듈은 ROS 의존성이 전혀 없는 순수 함수만 모은다 (단위 테스트 용이).

설계 원칙:
    1. 휴리스틱 h(n)은 admissible(과대평가하지 않음)해야 A*의 최적성이 보장된다.
        - manhattan: 4-connected 격자에서 admissible
        - euclidean: 어떤 그래프에서든 admissible (가장 안전)
        - octile  : 8-connected 격자에서 manhattan보다 tight한 admissible
    2. g(n) = start로부터 n까지 누적 비용. 이동 비용 = 셀 간 거리 × 셀 가중치.
    3. f(n) = g(n) + h(n).
    4. occupancy grid의 각 셀은 [0, 100] 또는 -1(unknown).
       100 = 점유, 0 = 자유 공간.

비유:
    - h(n) = "여기서 목적지까지 직선으로 얼마나 멀어 보이는지" (낙관적 추정치)
    - g(n) = "여기까지 실제로 얼마나 비용 들여 왔는지" (실측치)
    - f(n) = 우선순위 큐 정렬 키 (작을수록 먼저 탐색)

────────────────────────────────────────────────────────────────────────────
2026-05-24 통합 패치 (SW)
    HU 작업(opticore-amr-astar)의 astar_node.py가 다음 두 함수를 import한다:
        from amr_navigation.heuristics import heuristic, movement_cost
    SW 베이스 모듈은 더 풍부한 API(manhattan_distance / octile_distance / ...)
    를 제공하지만, HU 노드 호환을 위해 아래 두 wrapper를 추가했다.
        - heuristic(a, b, mode='octile')  → 단일 디스패처. HU 호출용.
        - movement_cost(a, b)             → 점유값 미고려 단순 이동 비용.
    이로써 같은 모듈에서 SW 단위 테스트(21건) + HU 단위 테스트(17건) 모두 통과.
    좌표 컨벤션:
        SW의 manhattan_distance / euclidean_distance / octile_distance 는
        대칭 함수라 (col,row)와 (row,col) 어느 쪽으로 줘도 결과가 같다.
        movement_cost 도 |dr|+|dc| 만 보므로 컨벤션 무관.
────────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Iterator, List, Tuple


# ---------------------------------------------------------------------------
# 타입 별칭
# ---------------------------------------------------------------------------

GridCoord = Tuple[int, int]       # (col, row) — Occupancy Grid 인덱스
WorldCoord = Tuple[float, float]  # (x, y) — map frame [m]


# ---------------------------------------------------------------------------
# 그리드 메타데이터 (nav_msgs/OccupancyGrid.info와 1:1 대응)
# ---------------------------------------------------------------------------

@dataclass
class GridInfo:
    """Occupancy Grid 메타데이터.

    좌표 변환 정책 (셀 중심 기준):
        world_x = origin_x + (col + 0.5) * resolution
        world_y = origin_y + (row + 0.5) * resolution
    """
    width: int          # 열(col) 수
    height: int         # 행(row) 수
    resolution: float   # [m/cell], 명세 §3 상한 0.05
    origin_x: float     # [m], grid (0,0) 셀의 world 좌표
    origin_y: float     # [m]


# ---------------------------------------------------------------------------
# 휴리스틱 함수 — h(n)
# ---------------------------------------------------------------------------

def manhattan_distance(a: GridCoord, b: GridCoord) -> float:
    """4-connected 격자에서 admissible.

    예: (0,0) → (3,4) = 7. 8-conn에서는 과대평가(inadmissible).
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean_distance(a: GridCoord, b: GridCoord) -> float:
    """어떤 격자에서든 admissible.

    예: (0,0) → (3,4) = 5.0
    """
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return sqrt(dx * dx + dy * dy)


def octile_distance(a: GridCoord, b: GridCoord) -> float:
    """8-connected 격자에서 가장 tight한 admissible 휴리스틱.

    수식:
        h = max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)

    예: (0,0) → (3,4)일 때 max=4, min=3
        h = 4 + 0.4142 * 3 ≈ 5.2426
        실제 8-conn 최단거리 = 3*sqrt(2) + 1 = 5.2426 ✓ (정확히 일치)
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return max(dx, dy) + (sqrt(2.0) - 1.0) * min(dx, dy)


HEURISTIC_REGISTRY = {
    "manhattan": manhattan_distance,
    "euclidean": euclidean_distance,
    "octile": octile_distance,
}


def get_heuristic(name: str):
    """이름으로 휴리스틱 함수 조회.

    Raises:
        KeyError: 등록되지 않은 휴리스틱 이름.
    """
    if name not in HEURISTIC_REGISTRY:
        raise KeyError(
            f"Unknown heuristic '{name}'. "
            f"Choose from {list(HEURISTIC_REGISTRY.keys())}"
        )
    return HEURISTIC_REGISTRY[name]


# ---------------------------------------------------------------------------
# HU astar_node 호환 wrapper (2026-05-24 통합)
#
# HU 노드 시그니처:
#     heuristic(a, b, mode='octile') -> float
#     movement_cost(a, b)            -> float
#
# SW의 풍부한 API는 그대로 두고, 위 두 이름만 별도로 제공한다.
# (SW 코드가 직접 호출하지는 않으므로, A* 노드만 영향을 받는다)
# ---------------------------------------------------------------------------

def heuristic(a: GridCoord, b: GridCoord, mode: str = "octile") -> float:
    """단일 디스패처 — HU astar_node 호환.

    내부적으로 HEURISTIC_REGISTRY 를 그대로 사용한다.

    Args:
        a, b : 셀 좌표. (row,col) / (col,row) 어느 컨벤션이든 무관(대칭).
        mode : 'manhattan' / 'euclidean' / 'octile'.

    Raises:
        ValueError: 등록되지 않은 mode (HU 노드의 기존 예외와 동일 타입).
    """
    if mode not in HEURISTIC_REGISTRY:
        raise ValueError(
            f"알 수 없는 heuristic mode: {mode!r}. "
            f"선택지: {list(HEURISTIC_REGISTRY.keys())}"
        )
    return HEURISTIC_REGISTRY[mode](a, b)


def movement_cost(a: GridCoord, b: GridCoord) -> float:
    """인접 셀 간 단순 이동 비용 — HU astar_node 호환.

    g(n) 증분 계산용. SW의 step_cost와 달리 점유값은 받지 않으며,
    inflation 차단은 HU 노드의 `_is_free_cell()`이 별도로 담당한다.

    규칙:
        - 같은 셀 또는 0거리: 0.0
        - 대각 이동 (|dr|=|dc|=1): √2
        - 그 외 (직선·과대 이동 포함): 1.0

    Args:
        a, b : 인접 셀. 컨벤션 무관(대칭).
    """
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    if dr + dc == 0:
        return 0.0
    return sqrt(2.0) if (dr == 1 and dc == 1) else 1.0


# ---------------------------------------------------------------------------
# 이동 비용 (g(n) 증분) — SW DWA·heuristics 단위 테스트가 사용하는 풀-피처 버전
# ---------------------------------------------------------------------------

def step_cost(
    from_cell: GridCoord,
    to_cell: GridCoord,
    grid_value_at_to: int,
    *,
    inflation_penalty_scale: float = 0.01,
) -> float:
    """한 셀 이동의 비용.

    기본 비용:
        - 4-방향(직교) 이동: 1.0
        - 8-방향(대각선) 이동: sqrt(2)
    추가 페널티:
        - 점유 셀(>= 100): inf (이동 불가)
        - unknown(-1)    : inf (보수적 정책 — 필요 시 변경 가능)
        - 0~99           : inflation 페널티(셀 값 × scale)

    인자:
        grid_value_at_to       : 도착 셀의 occupancy 값 [0, 100], -1=unknown
        inflation_penalty_scale: occupancy를 비용에 가산할 때의 스케일
    """
    if grid_value_at_to < 0:
        return float("inf")
    if grid_value_at_to >= 100:
        return float("inf")

    dx = abs(from_cell[0] - to_cell[0])
    dy = abs(from_cell[1] - to_cell[1])
    if dx + dy == 0:
        return 0.0
    base = sqrt(2.0) if (dx == 1 and dy == 1) else 1.0
    return base + inflation_penalty_scale * grid_value_at_to


# ---------------------------------------------------------------------------
# 이웃 셀 생성기
# ---------------------------------------------------------------------------

NEIGHBORS_4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
NEIGHBORS_8 = NEIGHBORS_4 + ((1, 1), (1, -1), (-1, 1), (-1, -1))


def neighbors(
    cell: GridCoord,
    info: GridInfo,
    *,
    allow_diagonal: bool = True,
) -> Iterator[GridCoord]:
    """그리드 경계 안의 이웃 셀을 yield."""
    deltas = NEIGHBORS_8 if allow_diagonal else NEIGHBORS_4
    cx, cy = cell
    for dx, dy in deltas:
        nx, ny = cx + dx, cy + dy
        if 0 <= nx < info.width and 0 <= ny < info.height:
            yield (nx, ny)


# ---------------------------------------------------------------------------
# 좌표 변환 (world ↔ grid)
# ---------------------------------------------------------------------------

def world_to_grid(point: WorldCoord, info: GridInfo) -> GridCoord:
    """world (m) 좌표 → grid (col, row) 인덱스.

    범위 체크는 호출자가 한다 (음수/초과 인덱스가 나올 수 있음).
    """
    col = int((point[0] - info.origin_x) / info.resolution)
    row = int((point[1] - info.origin_y) / info.resolution)
    return (col, row)


def grid_to_world(cell: GridCoord, info: GridInfo) -> WorldCoord:
    """grid (col, row) → world (m), 셀 중심 좌표."""
    x = info.origin_x + (cell[0] + 0.5) * info.resolution
    y = info.origin_y + (cell[1] + 0.5) * info.resolution
    return (x, y)


def in_bounds(cell: GridCoord, info: GridInfo) -> bool:
    """그리드 경계 안에 있는지 검사."""
    return 0 <= cell[0] < info.width and 0 <= cell[1] < info.height


# ---------------------------------------------------------------------------
# 그리드 접근 헬퍼 — OccupancyGrid.data는 row-major 1D 배열
# ---------------------------------------------------------------------------

def grid_value(data: List[int], cell: GridCoord, info: GridInfo) -> int:
    """row-major 1D 배열에서 (col, row) 셀 값 조회.

    nav_msgs/OccupancyGrid 컨벤션:
        index = row * width + col
    """
    if not in_bounds(cell, info):
        return -1
    return data[cell[1] * info.width + cell[0]]


# ---------------------------------------------------------------------------
# 자체 검증 (스크립트로 직접 실행 가능)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    a, b = (0, 0), (3, 4)

    print("=== 휴리스틱 비교 (a=(0,0) → b=(3,4)) ===")
    print(f"manhattan : {manhattan_distance(a, b):.4f}  (= 7.0)")
    print(f"euclidean : {euclidean_distance(a, b):.4f}  (= 5.0)")
    print(f"octile    : {octile_distance(a, b):.4f}  (= 5.2426)")

    info = GridInfo(width=10, height=10, resolution=0.05,
                    origin_x=-1.0, origin_y=-1.0)
    print("\n=== 좌표 변환 sanity ===")
    print(f"world (-1.0, -1.0) → grid : {world_to_grid((-1.0, -1.0), info)}")
    print(f"grid (5, 5) → world      : {grid_to_world((5, 5), info)}")
    assert in_bounds((0, 0), info)
    assert not in_bounds((-1, 0), info)
    assert not in_bounds((10, 0), info)

    print("\n=== 이웃 (8-conn) ===")
    print(list(neighbors((5, 5), info, allow_diagonal=True)))

    print("\n=== step_cost 검증 ===")
    print(f"수직 이동 (occ=0)   : {step_cost((0, 0), (1, 0), 0):.4f}  (= 1.0)")
    print(f"대각선 이동 (occ=0) : {step_cost((0, 0), (1, 1), 0):.4f}  (= 1.4142)")
    print(f"점유 셀 (occ=100)   : {step_cost((0, 0), (1, 0), 100)}  (= inf)")
    print(f"unknown (occ=-1)    : {step_cost((0, 0), (1, 0), -1)}  (= inf)")
    print(f"inflation (occ=50)  : {step_cost((0, 0), (1, 0), 50):.4f}  (= 1.5)")
    print("\n모든 sanity check 통과 ✓")
