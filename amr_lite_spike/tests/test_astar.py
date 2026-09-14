import math

from amr_lite.envs.dynamics import RobotState
from amr_lite.envs.geometry import Rectangle
from amr_lite.envs.map import WarehouseMap
from amr_lite.planning.astar import astar
from amr_lite.planning.path_utils import path_state, remaining_path_distance
from amr_lite.envs import WarehouseEnv


def test_astar_avoids_obstacle():
    world = WarehouseMap(obstacles=[Rectangle(5, 2, 6, 6)])
    path = astar(world, (1, 4), (11, 4), inflation=0.4)
    assert path[0] == (1, 4) and path[-1] == (11, 4)
    assert all(not world.occupied(x, y, 0.4) for x, y in path)


def test_projection_and_local_goal():
    state = path_state([(0, 0), (4, 0), (4, 4)], RobotState(1, 1, 0), 1.0)
    assert state.projection == (1.0, 0.0)
    assert state.local_goal == (2.0, 0.0)
    assert state.cross_track_error == 1.0
    assert state.heading_error == 0.0
    assert remaining_path_distance([(0, 0), (4, 0), (4, 4)], state) == 7.0


def test_no_path_is_classified():
    env = WarehouseEnv(scenario_id="S7")
    env.reset(seed=1, options={"scenario_id": "S7"})
    _, _, terminated, _, info = env.step([0.0, 0.0])
    assert terminated and info["failure_type"] == "NO_PATH"
