from __future__ import annotations

import math

import numpy as np

from amr_lite.envs.dynamics import RobotState
from amr_lite.planning.path_utils import PathState


OBSERVATION_SIZE = 296


def build_observation(scan_history: list[np.ndarray], path: PathState, robot: RobotState,
                      lidar_max_range: float, goal_distance_scale: float,
                      v_max: float, w_max: float, max_cross_track_error: float) -> np.ndarray:
    if len(scan_history) != 4 or any(scan.shape != (72,) for scan in scan_history):
        raise ValueError("Observation contract requires four 72-ray scans")
    scans = np.concatenate([np.clip(scan, 0.0, lidar_max_range) / lidar_max_range for scan in scan_history])
    tail = np.asarray([
        np.clip(path.rho_goal / goal_distance_scale, 0.0, 1.0),
        math.sin(path.alpha_goal), math.cos(path.alpha_goal),
        np.clip(robot.v / v_max, 0.0, 1.0), np.clip(robot.w / w_max, -1.0, 1.0),
        np.clip(path.cross_track_error / max_cross_track_error, -1.0, 1.0),
        math.sin(path.heading_error), math.cos(path.heading_error),
    ], dtype=np.float32)
    observation = np.concatenate((scans.astype(np.float32), tail)).astype(np.float32)
    if observation.shape != (OBSERVATION_SIZE,) or not np.isfinite(observation).all():
        raise ValueError("Invalid observation")
    return observation

