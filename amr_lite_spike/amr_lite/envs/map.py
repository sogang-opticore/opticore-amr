from __future__ import annotations

import math
from dataclasses import dataclass, field

from .geometry import Rectangle, circle_rectangle_clearance


@dataclass
class WarehouseMap:
    width: float = 12.0
    height: float = 8.0
    obstacles: list[Rectangle] = field(default_factory=list)

    def all_rectangles(self) -> list[Rectangle]:
        t = 0.05
        return self.obstacles + [Rectangle(-t, -t, 0.0, self.height + t),
                                 Rectangle(self.width, -t, self.width + t, self.height + t),
                                 Rectangle(0.0, -t, self.width, 0.0),
                                 Rectangle(0.0, self.height, self.width, self.height + t)]

    def clearance(self, x: float, y: float, radius: float = 0.0) -> float:
        return min(circle_rectangle_clearance(x, y, radius, rect) for rect in self.all_rectangles())

    def collides(self, x: float, y: float, radius: float) -> bool:
        return self.clearance(x, y, radius) <= 0.0

    def occupied(self, x: float, y: float, inflation: float = 0.0) -> bool:
        return self.collides(x, y, inflation)

    def world_to_grid(self, x: float, y: float, resolution: float) -> tuple[int, int]:
        return int(x / resolution), int(y / resolution)

    def grid_to_world(self, col: int, row: int, resolution: float) -> tuple[float, float]:
        return (col + 0.5) * resolution, (row + 0.5) * resolution

