from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class Rectangle:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def inflated(self, amount: float) -> "Rectangle":
        return Rectangle(self.x_min - amount, self.y_min - amount,
                         self.x_max + amount, self.y_max + amount)

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


def point_segment_projection(px: float, py: float, ax: float, ay: float,
                             bx: float, by: float) -> tuple[float, float, float]:
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    t = 0.0 if denom <= 1e-12 else clamp(((px - ax) * dx + (py - ay) * dy) / denom, 0.0, 1.0)
    return ax + t * dx, ay + t * dy, t


def circle_rectangle_clearance(x: float, y: float, radius: float, rect: Rectangle) -> float:
    nx = clamp(x, rect.x_min, rect.x_max)
    ny = clamp(y, rect.y_min, rect.y_max)
    return math.hypot(x - nx, y - ny) - radius


def ray_rectangle_distance(ox: float, oy: float, dx: float, dy: float,
                           rect: Rectangle, max_range: float) -> float:
    t_min, t_max = -math.inf, math.inf
    for origin, direction, low, high in ((ox, dx, rect.x_min, rect.x_max),
                                          (oy, dy, rect.y_min, rect.y_max)):
        if abs(direction) < 1e-12:
            if origin < low or origin > high:
                return max_range
            continue
        a, b = (low - origin) / direction, (high - origin) / direction
        if a > b:
            a, b = b, a
        t_min, t_max = max(t_min, a), min(t_max, b)
        if t_min > t_max:
            return max_range
    hit = t_min if t_min >= 0.0 else t_max
    return hit if 0.0 <= hit <= max_range else max_range


def ray_circle_distance(ox: float, oy: float, dx: float, dy: float,
                        cx: float, cy: float, radius: float, max_range: float) -> float:
    qx, qy = ox - cx, oy - cy
    b = 2.0 * (qx * dx + qy * dy)
    c = qx * qx + qy * qy - radius * radius
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return max_range
    root = math.sqrt(disc)
    candidates = [t for t in ((-b - root) / 2.0, (-b + root) / 2.0) if t >= 0.0]
    return min(candidates, default=max_range)

