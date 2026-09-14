from __future__ import annotations

import math

import numpy as np

from amr_lite.config import load_config
from amr_lite.learning.action import normalize_command


class RuleBasedPlanner:
    name = "lite_rule_baseline"

    def __init__(self, env_config: dict | None = None, config: dict | None = None):
        self.env_config = env_config or load_config("env")
        self.config = config or load_config("baseline")

    def predict(self, observation: np.ndarray, deterministic: bool = True) -> np.ndarray:
        del deterministic
        scan = observation[216:288] * self.env_config["lidar_max_range"]
        goal_distance, sin_goal, cos_goal = map(float, observation[288:291])
        cte = float(observation[293]) * self.env_config["max_cross_track_error"]
        sin_heading, cos_heading = map(float, observation[294:296])
        alpha = math.atan2(sin_goal, cos_goal)
        heading_error = math.atan2(sin_heading, cos_heading)
        lookahead = max(0.2, goal_distance * self.env_config["goal_distance_scale"])
        curvature = 2.0 * math.sin(alpha) / lookahead
        v_max = self.env_config["v_max"]
        curve_limit = math.sqrt(self.config["curvature_accel_limit"] / max(abs(curvature), 1e-5))
        heading_factor = max(0.12, 1.0 - abs(heading_error) / self.config["heading_slowdown"])
        path_factor = max(0.2, 1.0 - abs(cte) / self.config["path_error_slowdown"])
        front = scan[24:48]
        clearance = float(np.min(front))
        clearance_factor = np.clip((clearance - self.config["clearance_stop"]) /
                                   (self.config["clearance_slow"] - self.config["clearance_stop"]), 0.0, 1.0)
        goal_m = goal_distance * self.env_config["goal_distance_scale"]
        braking = math.sqrt(max(0.0, 2.0 * self.env_config["decel_max"] * max(0.0, goal_m - 0.1)))
        v = min(v_max, curve_limit, braking) * heading_factor * path_factor * float(clearance_factor)
        correction = 0.65 * heading_error - 0.8 * cte
        w = curvature * v + correction
        if clearance < self.config["clearance_stop"]:
            v, w = 0.0, self.config["recovery_turn_rate"] * (-1.0 if alpha < 0.0 else 1.0)
        return normalize_command(v, w, v_max, self.env_config["w_max"])

