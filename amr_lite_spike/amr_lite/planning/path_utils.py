from __future__ import annotations

import math
from dataclasses import dataclass

from amr_lite.envs.dynamics import RobotState
from amr_lite.envs.geometry import point_segment_projection, wrap_angle


@dataclass(frozen=True)
class PathState:
    projection: tuple[float, float]
    segment_index: int
    segment_t: float
    cross_track_error: float
    heading_error: float
    local_goal: tuple[float, float]
    rho_goal: float
    alpha_goal: float


def path_state(path: list[tuple[float, float]], robot: RobotState, lookahead: float) -> PathState:
    if len(path) < 2:
        goal = path[0] if path else (robot.x, robot.y)
        return PathState(goal, 0, 0.0, 0.0, 0.0, goal, math.dist((robot.x, robot.y), goal), 0.0)
    best = None
    for index, (a, b) in enumerate(zip(path, path[1:])):
        qx, qy, t = point_segment_projection(robot.x, robot.y, *a, *b)
        distance = math.hypot(robot.x - qx, robot.y - qy)
        if best is None or distance < best[0]:
            best = (distance, index, qx, qy, t)
    assert best is not None
    _, index, qx, qy, t = best
    ax, ay = path[index]
    bx, by = path[index + 1]
    heading = math.atan2(by - ay, bx - ax)
    # Positive means the robot is to the left of the directed path segment.
    signed = math.cos(heading) * (robot.y - qy) - math.sin(heading) * (robot.x - qx)
    local_goal = sample_ahead(path, index, (qx, qy), lookahead)
    dx, dy = local_goal[0] - robot.x, local_goal[1] - robot.y
    return PathState((qx, qy), index, t, signed, wrap_angle(heading - robot.theta),
                     local_goal, math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - robot.theta))


def sample_ahead(path: list[tuple[float, float]], segment_index: int,
                 projection: tuple[float, float], distance: float) -> tuple[float, float]:
    current = projection
    remaining = max(0.0, distance)
    for index in range(segment_index, len(path) - 1):
        end = path[index + 1]
        length = math.dist(current, end)
        if length >= remaining and length > 1e-12:
            ratio = remaining / length
            return current[0] + ratio * (end[0] - current[0]), current[1] + ratio * (end[1] - current[1])
        remaining -= length
        current = end
    return path[-1]


def remaining_path_distance(path: list[tuple[float, float]], state: PathState) -> float:
    """Arc length from the robot's path projection to the path endpoint."""
    if len(path) < 2:
        return state.rho_goal
    remaining = math.dist(state.projection, path[state.segment_index + 1])
    remaining += sum(math.dist(a, b) for a, b in zip(path[state.segment_index + 1:],
                                                      path[state.segment_index + 2:]))
    return remaining


def world_to_robot(point: tuple[float, float], robot: RobotState) -> tuple[float, float]:
    dx, dy = point[0] - robot.x, point[1] - robot.y
    c, s = math.cos(robot.theta), math.sin(robot.theta)
    return c * dx + s * dy, -s * dx + c * dy
