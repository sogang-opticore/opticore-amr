from amr_lite.config import load_config
from amr_lite.envs.dynamics import RobotState
from amr_lite.envs.geometry import Rectangle
from amr_lite.envs.map import WarehouseMap
from amr_lite.planning.safety_shield import SafetyShield
from amr_lite.envs.scenarios import DynamicObstacle


def test_emergency_stop_or_clamp_before_wall():
    config = load_config("env")
    shield = SafetyShield(config)
    result = shield.apply(RobotState(1.0, 1.0, 0.0, 1.0, 0.0), 1.5, 0.0,
                          WarehouseMap(obstacles=[Rectangle(1.4, 0.0, 1.5, 2.0)]), [], True)
    assert result.mode in {"CLAMP", "STOP"}
    assert result.unsafe_without_shield


def test_nan_action_stops():
    config = load_config("env")
    result = SafetyShield(config).apply(RobotState(2, 2, 0), float("nan"), 0, WarehouseMap(), [], True)
    assert (result.v, result.w, result.mode) == (0.0, 0.0, "STOP")


def test_approaching_obstacle_is_predicted():
    config = load_config("env", {"shield_horizon": 1.0})
    obstacle = DynamicObstacle(2.3, 2.0, -1.0, 0.0)
    result = SafetyShield(config).apply(RobotState(1.0, 2.0, 0.0, 1.0, 0.0),
                                        1.0, 0.0, WarehouseMap(), [obstacle], True, 0.0)
    assert result.unsafe_without_shield
    assert result.mode in {"CLAMP", "STOP"}
