from __future__ import annotations

import heapq
import math

from amr_lite.envs.map import WarehouseMap


def astar(warehouse_map: WarehouseMap, start: tuple[float, float], goal: tuple[float, float],
          resolution: float = 0.2, inflation: float = 0.4) -> list[tuple[float, float]]:
    cols, rows = int(math.ceil(warehouse_map.width / resolution)), int(math.ceil(warehouse_map.height / resolution))
    source = warehouse_map.world_to_grid(*start, resolution)
    target = warehouse_map.world_to_grid(*goal, resolution)
    if warehouse_map.occupied(*start, inflation) or warehouse_map.occupied(*goal, inflation):
        return []
    frontier: list[tuple[float, tuple[int, int]]] = [(0.0, source)]
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {source: None}
    cost = {source: 0.0}
    neighbors = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
                 (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0))]
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == target:
            break
        for dc, dr, step_cost in neighbors:
            nxt = current[0] + dc, current[1] + dr
            if not (0 <= nxt[0] < cols and 0 <= nxt[1] < rows):
                continue
            wx, wy = warehouse_map.grid_to_world(*nxt, resolution)
            if warehouse_map.occupied(wx, wy, inflation):
                continue
            tentative = cost[current] + step_cost
            if tentative >= cost.get(nxt, math.inf):
                continue
            cost[nxt] = tentative
            heuristic = math.hypot(nxt[0] - target[0], nxt[1] - target[1])
            heapq.heappush(frontier, (tentative + heuristic, nxt))
            came_from[nxt] = current
    if target not in came_from:
        return []
    cells, cursor = [], target
    while cursor is not None:
        cells.append(cursor)
        cursor = came_from[cursor]
    points = [warehouse_map.grid_to_world(*cell, resolution) for cell in reversed(cells)]
    points[0], points[-1] = start, goal
    return simplify_path(points, warehouse_map, inflation)


def simplify_path(path: list[tuple[float, float]], warehouse_map: WarehouseMap,
                  inflation: float) -> list[tuple[float, float]]:
    if len(path) < 3:
        return path
    result, anchor = [path[0]], 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1 and not _visible(path[anchor], path[candidate], warehouse_map, inflation):
            candidate -= 1
        result.append(path[candidate])
        anchor = candidate
    return result


def _visible(a: tuple[float, float], b: tuple[float, float], warehouse_map: WarehouseMap,
             inflation: float) -> bool:
    distance = math.dist(a, b)
    count = max(2, int(distance / 0.08))
    return all(not warehouse_map.occupied(a[0] + (b[0] - a[0]) * i / count,
                                            a[1] + (b[1] - a[1]) * i / count, inflation)
               for i in range(count + 1))

