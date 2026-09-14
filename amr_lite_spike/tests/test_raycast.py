import math

import numpy as np
import pytest

from amr_lite.envs.dynamics import RobotState
from amr_lite.envs.geometry import Rectangle
from amr_lite.envs.map import WarehouseMap
from amr_lite.envs.raycast import lidar_scan
from amr_lite.envs.scenarios import DynamicObstacle


def test_empty_room_and_front_wall_ordering():
    world = WarehouseMap(width=20, height=20)
    scan = lidar_scan(RobotState(10, 10, 0), world, [], 72, math.radians(270), 8.0)
    assert scan.shape == (72,)
    assert np.all(scan <= 8.0)
    wall = WarehouseMap(width=20, height=20, obstacles=[Rectangle(11, 5, 11.1, 15)])
    scan = lidar_scan(RobotState(10, 10, 0), wall, [], 73, math.radians(270), 8.0)
    assert scan[36] == pytest.approx(1.0, abs=0.02)


def test_circle_is_first_hit_and_rotation_changes_scan():
    world = WarehouseMap(width=20, height=20, obstacles=[Rectangle(14, 5, 14.1, 15)])
    obstacle = DynamicObstacle(12, 10, 0, 0, radius=0.5)
    forward = lidar_scan(RobotState(10, 10, 0), world, [obstacle], 73, math.radians(270), 8.0)
    rotated = lidar_scan(RobotState(10, 10, math.pi / 2), world, [obstacle], 73, math.radians(270), 8.0)
    assert forward[36] == pytest.approx(1.5, abs=0.02)
    assert not np.allclose(forward, rotated)

