from amr_lite.envs.geometry import Rectangle
from amr_lite.envs.map import WarehouseMap
from amr_lite.envs import WarehouseEnv
from amr_lite.config import load_config


def test_overlap_corner_boundary_and_thin_wall():
    world = WarehouseMap(obstacles=[Rectangle(2.0, 1.0, 2.03, 3.0)])
    assert world.collides(2.0, 2.0, 0.2)
    assert world.collides(1.8, 1.0, 0.2)
    assert world.collides(1.8, 2.0, 0.2)  # exact footprint boundary
    assert not world.collides(1.7, 2.0, 0.2)


def test_physics_substeps_prevent_high_speed_tunneling():
    config = load_config("env", {"robot_radius": 0.05, "v_max": 10.0,
                                 "accel_max": 1000.0, "shield_enabled": False})
    env = WarehouseEnv(config, "S0")
    env.scenario.warehouse_map = WarehouseMap(obstacles=[Rectangle(1.22, 3.0, 1.23, 5.0)])
    _, _, terminated, _, info = env.step([1.0, 0.0])
    assert terminated and info["collision"]
