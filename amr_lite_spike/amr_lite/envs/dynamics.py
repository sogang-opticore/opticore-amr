from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import clamp, wrap_angle


@dataclass
class RobotState:
    x: float
    y: float
    theta: float
    v: float = 0.0
    w: float = 0.0


def limit_velocity(current_v: float, current_w: float, target_v: float, target_w: float,
                   dt: float, v_max: float, w_max: float, accel_max: float,
                   decel_max: float, angular_accel_max: float) -> tuple[float, float]:
    target_v = clamp(target_v, 0.0, v_max)
    target_w = clamp(target_w, -w_max, w_max)
    dv = target_v - current_v
    dv_limit = (accel_max if dv >= 0.0 else decel_max) * dt
    dw = clamp(target_w - current_w, -angular_accel_max * dt, angular_accel_max * dt)
    return clamp(current_v + clamp(dv, -dv_limit, dv_limit), 0.0, v_max), clamp(current_w + dw, -w_max, w_max)


def integrate(state: RobotState, v: float, w: float, dt: float) -> RobotState:
    if abs(w) < 1e-9:
        x = state.x + v * math.cos(state.theta) * dt
        y = state.y + v * math.sin(state.theta) * dt
    else:
        theta2 = state.theta + w * dt
        x = state.x + v / w * (math.sin(theta2) - math.sin(state.theta))
        y = state.y - v / w * (math.cos(theta2) - math.cos(state.theta))
    return RobotState(x, y, wrap_angle(state.theta + w * dt), v, w)

