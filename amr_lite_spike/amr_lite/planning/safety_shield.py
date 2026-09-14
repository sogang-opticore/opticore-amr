from __future__ import annotations

import math
from dataclasses import dataclass

from amr_lite.envs.dynamics import RobotState, integrate, limit_velocity
from amr_lite.envs.map import WarehouseMap
from amr_lite.envs.scenarios import DynamicObstacle


@dataclass(frozen=True)
class ShieldResult:
    v: float
    w: float
    mode: str
    unsafe_without_shield: bool


class SafetyShield:
    def __init__(self, config: dict):
        self.config = config

    def apply(self, state: RobotState, target_v: float, target_w: float,
              warehouse_map: WarehouseMap, dynamics: list[DynamicObstacle], enabled: bool = True,
              current_time: float = 0.0) -> ShieldResult:
        if not math.isfinite(target_v) or not math.isfinite(target_w):
            return ShieldResult(0.0, 0.0, "STOP", True)
        v, w = limit_velocity(state.v, state.w, target_v, target_w,
                              self.config["policy_dt"], self.config["v_max"], self.config["w_max"],
                              self.config["accel_max"], self.config["decel_max"], self.config["angular_accel_max"])
        unsafe = self._unsafe(state, v, w, warehouse_map, dynamics, current_time)
        if not enabled or not unsafe:
            return ShieldResult(v, w, "PASS", unsafe)
        for scale in (0.5, 0.2):
            if not self._unsafe(state, v * scale, w, warehouse_map, dynamics, current_time):
                return ShieldResult(v * scale, w, "CLAMP", True)
        # A stopped robot can still be hit by an approaching obstacle. Try a
        # bounded steering escape before falling back to STOP.
        for direction in (1.0, -1.0):
            evade_v, evade_w = limit_velocity(
                state.v, state.w, min(v, 0.45 * self.config["v_max"]),
                direction * 0.65 * self.config["w_max"], self.config["policy_dt"],
                self.config["v_max"], self.config["w_max"], self.config["accel_max"],
                self.config["decel_max"], self.config["angular_accel_max"])
            if not self._unsafe(state, evade_v, evade_w, warehouse_map, dynamics, current_time):
                return ShieldResult(evade_v, evade_w, "CLAMP", True)
        return ShieldResult(0.0, 0.0, "STOP", True)

    def _unsafe(self, state: RobotState, v: float, w: float, warehouse_map: WarehouseMap,
                dynamics: list[DynamicObstacle], current_time: float = 0.0) -> bool:
        probe = RobotState(state.x, state.y, state.theta, v, w)
        dt = 0.05
        for index in range(max(1, int(self.config["shield_horizon"] / dt))):
            probe = integrate(probe, v, w, dt)
            if warehouse_map.clearance(probe.x, probe.y, self.config["robot_radius"]) < 0.03:
                return True
            for obstacle in dynamics:
                future_time = current_time + (index + 1) * dt
                motion_end = min(future_time, obstacle.stop_time if obstacle.stop_time is not None else future_time)
                moving_duration = max(0.0, motion_end - max(current_time, obstacle.start_delay))
                predicted_x = obstacle.x + obstacle.vx * moving_duration
                predicted_y = obstacle.y + obstacle.vy * moving_duration
                if math.hypot(probe.x - predicted_x, probe.y - predicted_y) <= self.config["robot_radius"] + obstacle.radius + 0.03:
                    return True
        return False
