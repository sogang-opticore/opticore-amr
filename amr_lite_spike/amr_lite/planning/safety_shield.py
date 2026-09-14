from __future__ import annotations

import math
from dataclasses import dataclass

from amr_lite.envs.dynamics import RobotState, integrate, limit_velocity
from amr_lite.envs.map import WarehouseMap
from amr_lite.envs.scenarios import DynamicObstacle


@dataclass(frozen=True)
class RiskAssessment:
    unsafe: bool
    source: str = "NONE"
    time_to_collision: float = math.inf
    cpa_distance: float = math.inf
    minimum_clearance: float = math.inf


@dataclass(frozen=True)
class ShieldResult:
    v: float
    w: float
    mode: str
    unsafe_without_shield: bool
    action_type: str = "PASS"
    risk_source: str = "NONE"
    predicted_ttc: float = math.inf
    cpa_distance: float = math.inf
    predicted_minimum_clearance: float = math.inf
    false_positive: bool = False
    false_negative: bool = False
    tracking_speed_error: float = math.nan


@dataclass
class ObstacleTrack:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    observed_at: float


class SafetyShield:
    def __init__(self, config: dict):
        self.config = config
        self._tracks: dict[int, ObstacleTrack] = {}

    def reset(self) -> None:
        self._tracks = {}

    def apply(self, state: RobotState, target_v: float, target_w: float,
              warehouse_map: WarehouseMap, dynamics: list[DynamicObstacle], enabled: bool = True,
              current_time: float = 0.0) -> ShieldResult:
        tracks, speed_error = self._update_tracks(dynamics, current_time)
        if not math.isfinite(target_v) or not math.isfinite(target_w):
            return ShieldResult(0.0, 0.0, "STOP", True, "STOP", "INVALID_ACTION",
                                false_negative=True, tracking_speed_error=speed_error)
        strategy = str(self.config.get("shield_strategy", "legacy"))
        if strategy == "predictive":
            return self._apply_predictive(
                state, target_v, target_w, warehouse_map, dynamics, tracks,
                enabled, current_time, speed_error
            )
        return self._apply_legacy(
            state, target_v, target_w, warehouse_map, dynamics,
            enabled, current_time, speed_error
        )

    def _update_tracks(self, dynamics: list[DynamicObstacle],
                       current_time: float) -> tuple[list[ObstacleTrack], float]:
        alpha = float(self.config.get("shield_tracker_alpha", 0.65))
        new_tracks: dict[int, ObstacleTrack] = {}
        errors = []
        for index, obstacle in enumerate(dynamics):
            previous = self._tracks.get(index)
            if previous is None or current_time <= previous.observed_at + 1e-9:
                vx = vy = 0.0
            else:
                dt = current_time - previous.observed_at
                measured_vx = (obstacle.x - previous.x) / dt
                measured_vy = (obstacle.y - previous.y) / dt
                vx = alpha * measured_vx + (1.0 - alpha) * previous.vx
                vy = alpha * measured_vy + (1.0 - alpha) * previous.vy
            track = ObstacleTrack(
                obstacle.x, obstacle.y, vx, vy, obstacle.radius, current_time
            )
            new_tracks[index] = track
            active = (current_time >= obstacle.start_delay
                      and (obstacle.stop_time is None or current_time < obstacle.stop_time))
            truth_vx, truth_vy = (obstacle.vx, obstacle.vy) if active else (0.0, 0.0)
            errors.append(math.hypot(vx - truth_vx, vy - truth_vy))
        self._tracks = new_tracks
        return list(new_tracks.values()), (sum(errors) / len(errors) if errors else math.nan)

    def _limit(self, state: RobotState, target_v: float, target_w: float,
               dt: float | None = None, allow_reverse: bool = False) -> tuple[float, float]:
        reverse_max = float(self.config.get("shield_reverse_max", 0.0)) if allow_reverse else 0.0
        return limit_velocity(
            state.v, state.w, target_v, target_w,
            float(self.config["policy_dt"] if dt is None else dt),
            float(self.config["v_max"]), float(self.config["w_max"]),
            float(self.config["accel_max"]), float(self.config["decel_max"]),
            float(self.config["angular_accel_max"]), v_min=-reverse_max,
        )

    def _apply_legacy(self, state: RobotState, target_v: float, target_w: float,
                      warehouse_map: WarehouseMap, dynamics: list[DynamicObstacle],
                      enabled: bool, current_time: float, speed_error: float) -> ShieldResult:
        v, w = self._limit(state, target_v, target_w)
        unsafe = self._unsafe_legacy(state, v, w, warehouse_map, dynamics, current_time)
        if not enabled or not unsafe:
            return ShieldResult(v, w, "PASS", unsafe, "PASS", "LEGACY",
                                tracking_speed_error=speed_error)
        for scale in (0.5, 0.2):
            if not self._unsafe_legacy(
                    state, v * scale, w, warehouse_map, dynamics, current_time):
                return ShieldResult(v * scale, w, "CLAMP", True, "SLOW", "LEGACY",
                                    tracking_speed_error=speed_error)
        for direction in (1.0, -1.0):
            evade_v, evade_w = self._limit(
                state, min(v, 0.45 * self.config["v_max"]),
                direction * 0.65 * self.config["w_max"]
            )
            if not self._unsafe_legacy(
                    state, evade_v, evade_w, warehouse_map, dynamics, current_time):
                return ShieldResult(evade_v, evade_w, "CLAMP", True, "EVADE", "LEGACY",
                                    tracking_speed_error=speed_error)
        return ShieldResult(0.0, 0.0, "STOP", True, "STOP", "LEGACY",
                            tracking_speed_error=speed_error)

    def _apply_predictive(self, state: RobotState, target_v: float, target_w: float,
                          warehouse_map: WarehouseMap, dynamics: list[DynamicObstacle],
                          tracks: list[ObstacleTrack], enabled: bool, current_time: float,
                          speed_error: float) -> ShieldResult:
        raw = self._assess_target(
            state, target_v, target_w, warehouse_map, tracks,
            static_horizon=float(self.config.get("shield_static_horizon", 0.45)),
            dynamic_horizon=float(self.config.get("shield_dynamic_horizon", 2.5)),
            static_margin=float(self.config.get("shield_static_margin", 0.01)),
            dynamic_margin=float(self.config.get("shield_dynamic_margin", 0.08)),
            use_cpa=True,
        )
        truth_tracks = self._truth_tracks(dynamics, current_time)
        audit = self._assess_target(
            state, target_v, target_w, warehouse_map, truth_tracks,
            static_horizon=float(self.config.get("shield_audit_static_horizon", 0.45)),
            dynamic_horizon=float(self.config.get("shield_audit_dynamic_horizon", 2.5)),
            static_margin=0.0,
            dynamic_margin=0.0,
            use_cpa=True,
        )
        first_v, first_w = self._limit(state, target_v, target_w)
        if not enabled or not raw.unsafe:
            return ShieldResult(
                first_v, first_w, "PASS", raw.unsafe, "PASS", raw.source,
                raw.time_to_collision, raw.cpa_distance, raw.minimum_clearance,
                false_negative=bool(enabled and audit.unsafe),
                tracking_speed_error=speed_error,
            )

        candidate_targets: list[tuple[str, float, float, bool]] = []
        for scale in self.config.get("shield_slow_scales", [0.5, 0.2]):
            candidate_targets.append(("SLOW", target_v * float(scale), target_w, False))
        evade_v = min(
            max(0.0, target_v),
            float(self.config.get("shield_evade_v", 0.55)),
        )
        evade_w = float(self.config.get("shield_evade_w", 1.45))
        for direction in (1.0, -1.0):
            candidate_targets.append(("EVADE", evade_v, direction * evade_w, False))
        if bool(self.config.get("shield_reverse_enabled", True)):
            reverse_v = -float(self.config.get("shield_reverse_max", 0.35))
            reverse_w = float(self.config.get("shield_reverse_w", 0.8))
            candidate_targets.extend([
                ("REVERSE", reverse_v, 0.0, True),
                ("REVERSE", reverse_v, reverse_w, True),
                ("REVERSE", reverse_v, -reverse_w, True),
            ])
        candidate_targets.append(("STOP", 0.0, 0.0, False))

        assessed = []
        for action_type, candidate_v, candidate_w, allow_reverse in candidate_targets:
            risk = self._assess_target(
                state, candidate_v, candidate_w, warehouse_map, tracks,
                static_horizon=float(self.config.get("shield_static_horizon", 0.45)),
                dynamic_horizon=float(self.config.get("shield_dynamic_horizon", 2.5)),
                static_margin=float(self.config.get("shield_static_margin", 0.01)),
                dynamic_margin=float(self.config.get("shield_dynamic_margin", 0.08)),
                use_cpa=action_type in {"SLOW", "STOP"},
                allow_reverse=allow_reverse,
            )
            assessed.append((action_type, candidate_v, candidate_w, allow_reverse, risk))
            if not risk.unsafe:
                command_v, command_w = self._limit(
                    state, candidate_v, candidate_w, allow_reverse=allow_reverse
                )
                mode = "STOP" if action_type == "STOP" else "CLAMP"
                return ShieldResult(
                    command_v, command_w, mode, True, action_type, raw.source,
                    raw.time_to_collision, raw.cpa_distance, raw.minimum_clearance,
                    false_positive=not audit.unsafe,
                    tracking_speed_error=speed_error,
                )

        # No candidate is completely safe. Execute the candidate with the largest
        # predicted clearance instead of defaulting to a stationary collision.
        action_type, candidate_v, candidate_w, allow_reverse, _ = max(
            assessed,
            key=lambda item: (
                item[4].minimum_clearance,
                item[4].time_to_collision,
                item[0] != "STOP",
            ),
        )
        command_v, command_w = self._limit(
            state, candidate_v, candidate_w, allow_reverse=allow_reverse
        )
        mode = "STOP" if action_type == "STOP" else "CLAMP"
        return ShieldResult(
            command_v, command_w, mode, True, action_type, raw.source,
            raw.time_to_collision, raw.cpa_distance, raw.minimum_clearance,
            false_positive=not audit.unsafe,
            tracking_speed_error=speed_error,
        )

    def _assess_target(self, state: RobotState, target_v: float, target_w: float,
                       warehouse_map: WarehouseMap, tracks: list[ObstacleTrack],
                       *, static_horizon: float, dynamic_horizon: float,
                       static_margin: float, dynamic_margin: float, use_cpa: bool,
                       allow_reverse: bool = False) -> RiskAssessment:
        cpa_ttc, cpa_distance = self._closest_approach(
            state, target_v, tracks, dynamic_margin
        )
        cpa_unsafe = use_cpa and cpa_ttc <= dynamic_horizon
        dt = float(self.config.get("shield_sim_dt", 0.05))
        horizon = max(static_horizon, dynamic_horizon)
        steps = max(1, int(math.ceil(horizon / dt)))
        probe = RobotState(state.x, state.y, state.theta, state.v, state.w)
        minimum_clearance = math.inf
        first_source = "DYNAMIC_CPA" if cpa_unsafe else "NONE"
        first_ttc = cpa_ttc if cpa_unsafe else math.inf
        for index in range(steps):
            elapsed = (index + 1) * dt
            next_v, next_w = self._limit(
                probe, target_v, target_w, dt=dt, allow_reverse=allow_reverse
            )
            probe = integrate(probe, next_v, next_w, dt)
            if elapsed <= static_horizon + 1e-9:
                clearance = warehouse_map.clearance(
                    probe.x, probe.y, float(self.config["robot_radius"])
                )
                minimum_clearance = min(minimum_clearance, clearance)
                if clearance <= static_margin and elapsed < first_ttc:
                    first_source, first_ttc = "STATIC_ROLLOUT", elapsed
            if elapsed <= dynamic_horizon + 1e-9:
                for track in tracks:
                    obstacle_x = track.x + track.vx * elapsed
                    obstacle_y = track.y + track.vy * elapsed
                    clearance = (
                        math.hypot(probe.x - obstacle_x, probe.y - obstacle_y)
                        - float(self.config["robot_radius"]) - track.radius
                    )
                    minimum_clearance = min(minimum_clearance, clearance)
                    if clearance <= dynamic_margin and elapsed < first_ttc:
                        first_source, first_ttc = "DYNAMIC_ROLLOUT", elapsed
        unsafe = first_source != "NONE"
        return RiskAssessment(
            unsafe, first_source, first_ttc, cpa_distance, minimum_clearance
        )

    def _closest_approach(self, state: RobotState, target_v: float,
                          tracks: list[ObstacleTrack],
                          margin: float) -> tuple[float, float]:
        best_ttc, best_cpa = math.inf, math.inf
        robot_vx = target_v * math.cos(state.theta)
        robot_vy = target_v * math.sin(state.theta)
        for track in tracks:
            px, py = track.x - state.x, track.y - state.y
            vx, vy = track.vx - robot_vx, track.vy - robot_vy
            threshold = float(self.config["robot_radius"]) + track.radius + margin
            speed_sq = vx * vx + vy * vy
            if speed_sq <= 1e-10:
                if math.hypot(px, py) <= threshold:
                    return 0.0, math.hypot(px, py)
                continue
            t_cpa = max(0.0, -(px * vx + py * vy) / speed_sq)
            cpa = math.hypot(px + vx * t_cpa, py + vy * t_cpa)
            a = speed_sq
            b = 2.0 * (px * vx + py * vy)
            c = px * px + py * py - threshold * threshold
            discriminant = b * b - 4.0 * a * c
            ttc = math.inf
            if discriminant >= 0.0:
                first = (-b - math.sqrt(discriminant)) / (2.0 * a)
                second = (-b + math.sqrt(discriminant)) / (2.0 * a)
                nonnegative = [value for value in (first, second) if value >= 0.0]
                if nonnegative:
                    ttc = min(nonnegative)
            if ttc < best_ttc:
                best_ttc = ttc
            best_cpa = min(best_cpa, cpa)
        return best_ttc, best_cpa

    @staticmethod
    def _truth_tracks(dynamics: list[DynamicObstacle],
                      current_time: float) -> list[ObstacleTrack]:
        tracks = []
        for obstacle in dynamics:
            active = (current_time >= obstacle.start_delay
                      and (obstacle.stop_time is None or current_time < obstacle.stop_time))
            vx, vy = (obstacle.vx, obstacle.vy) if active else (0.0, 0.0)
            tracks.append(ObstacleTrack(
                obstacle.x, obstacle.y, vx, vy, obstacle.radius, current_time
            ))
        return tracks

    def _unsafe_legacy(self, state: RobotState, v: float, w: float,
                       warehouse_map: WarehouseMap, dynamics: list[DynamicObstacle],
                       current_time: float = 0.0) -> bool:
        probe = RobotState(state.x, state.y, state.theta, v, w)
        dt = 0.05
        for index in range(max(1, int(self.config["shield_horizon"] / dt))):
            probe = integrate(probe, v, w, dt)
            if warehouse_map.clearance(
                    probe.x, probe.y, self.config["robot_radius"]) < 0.03:
                return True
            for obstacle in dynamics:
                future_time = current_time + (index + 1) * dt
                motion_end = min(
                    future_time,
                    obstacle.stop_time if obstacle.stop_time is not None else future_time,
                )
                moving_duration = max(
                    0.0, motion_end - max(current_time, obstacle.start_delay)
                )
                predicted_x = obstacle.x + obstacle.vx * moving_duration
                predicted_y = obstacle.y + obstacle.vy * moving_duration
                if math.hypot(
                        probe.x - predicted_x, probe.y - predicted_y
                ) <= self.config["robot_radius"] + obstacle.radius + 0.03:
                    return True
        return False
