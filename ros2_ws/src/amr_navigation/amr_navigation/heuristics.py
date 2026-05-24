"""
heuristics.py — A* 휴리스틱 / 비용 함수 모듈
ROS 의존성 없는 순수 Python — 단위 테스트 용이
담당: SW (휴리스틱 분담)
"""

import math


def heuristic(a: tuple, b: tuple, mode: str = 'octile') -> float:
    """
    두 셀 사이 휴리스틱 거리 계산.

    A*에서 h(n) — 현재 셀에서 goal까지의 추정 비용.
    실제 비용보다 과대추정하면 최적 경로를 놓칠 수 있으므로
    admissible(절대 과대추정 안 함) 휴리스틱을 사용한다.

    Args:
        a: 현재 셀 (row, col)
        b: 목표 셀 (row, col)
        mode: 'manhattan' | 'euclidean' | 'octile'

    Returns:
        float: 휴리스틱 비용 (작을수록 goal에 가까움)
    """
    dr = abs(a[0] - b[0])  # 행 방향 거리
    dc = abs(a[1] - b[1])  # 열 방향 거리

    if mode == 'manhattan':
        # 4-connected 그리드에서 최적.
        # 대각 이동이 없으면 이게 정확한 비용.
        # 8-connected에서 쓰면 과소추정 → 최적이지만 느림.
        return float(dr + dc)

    elif mode == 'euclidean':
        # 직선 거리. 항상 admissible이지만
        # 그리드 이동 비용(1.0 or √2)보다 작아서 과소추정 경향.
        # 탐색 노드가 많아져 느려질 수 있음.
        return math.sqrt(dr * dr + dc * dc)

    elif mode == 'octile':
        # 8-connected 그리드에서 최적 휴리스틱.
        # 대각 이동(√2) + 직선 이동(1.0)을 정확히 반영.
        # 우리 환경(8-connected, allow_diagonal=True) 기본값.
        return max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)

    else:
        raise ValueError(f'알 수 없는 heuristic mode: {mode}')


def movement_cost(a: tuple, b: tuple) -> float:
    """
    인접 셀 간 실제 이동 비용 — g(n) 계산에 사용.

    직선 이동(상하좌우): 1.0
    대각 이동: √2 ≈ 1.414

    Args:
        a: 출발 셀 (row, col)
        b: 도착 셀 (row, col)

    Returns:
        float: 이동 비용
    """
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    # 두 방향 모두 1칸 차이면 대각 이동
    return math.sqrt(2) if (dr == 1 and dc == 1) else 1.0