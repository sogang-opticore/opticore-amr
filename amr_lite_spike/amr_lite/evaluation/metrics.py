from __future__ import annotations

import csv
import math
import statistics
import zlib
from collections import defaultdict
from pathlib import Path
from typing import Callable

import numpy as np


BINARY_METRICS = {
    "success_rate": "success",
    "collision_rate": "collision",
    "timeout_rate": "truncated",
}
CONTINUOUS_METRICS = {
    "mean_minimum_clearance": "minimum_clearance",
    "mean_episode_time": "episode_time",
}


def write_csv(path: str | Path, rows: list[dict]) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError("Cannot write empty result table")
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return target


def wilson_interval(successes: int, count: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if count <= 0:
        return math.nan, math.nan
    rate = successes / count
    denominator = 1.0 + z * z / count
    centre = (rate + z * z / (2.0 * count)) / denominator
    margin = z * math.sqrt(rate * (1.0 - rate) / count + z * z / (4.0 * count * count)) / denominator
    return max(0.0, centre - margin), min(1.0, centre + margin)


def _training_seed(row: dict) -> str:
    value = row.get("training_seed", "")
    return "" if value is None else str(value)


def _resample_stratified(values: list[dict], rng: np.random.Generator) -> list[dict]:
    """Resample training seeds, then episodes within each fixed scenario stratum."""
    training_seeds = sorted({seed for seed in map(_training_seed, values) if seed != ""})
    if training_seeds:
        sampled_training_seeds: list[str | None] = list(
            rng.choice(training_seeds, size=len(training_seeds), replace=True)
        )
    else:
        sampled_training_seeds = [None]
    sample: list[dict] = []
    for training_seed in sampled_training_seeds:
        seed_values = values if training_seed is None else [
            row for row in values if _training_seed(row) == training_seed
        ]
        scenarios = sorted({str(row["scenario_id"]) for row in seed_values})
        for scenario in scenarios:
            stratum = [row for row in seed_values if str(row["scenario_id"]) == scenario]
            indices = rng.integers(0, len(stratum), size=len(stratum))
            sample.extend(stratum[int(index)] for index in indices)
    return sample


def _bootstrap_ci(values: list[dict], value_fn: Callable[[dict], float], resamples: int,
                  seed: int) -> tuple[float, float]:
    if not values:
        return math.nan, math.nan
    if resamples <= 0:
        point = float(np.mean([value_fn(row) for row in values]))
        return point, point
    rng = np.random.default_rng(seed)
    estimates = np.empty(resamples, dtype=np.float64)
    for index in range(resamples):
        sample = _resample_stratified(values, rng)
        estimates[index] = np.mean([value_fn(row) for row in sample])
    low, high = np.quantile(estimates, [0.025, 0.975])
    return float(low), float(high)


def _summary_group(values: list[dict], bootstrap_resamples: int, bootstrap_seed: int) -> dict:
    count = len(values)
    total_steps = max(1, sum(int(item["steps"]) for item in values))
    output = {
        "episodes": count,
        "training_seeds": len({seed for seed in map(_training_seed, values) if seed != ""}),
        "evaluation_seeds": len({int(item["seed"]) for item in values}),
        "scenarios": len({str(item["scenario_id"]) for item in values}),
    }
    for metric, field in BINARY_METRICS.items():
        successes = sum(int(item[field]) for item in values)
        output[metric] = successes / count
        wilson_low, wilson_high = wilson_interval(successes, count)
        output[f"{metric}_wilson_low"] = wilson_low
        output[f"{metric}_wilson_high"] = wilson_high
        low, high = _bootstrap_ci(values, lambda row, key=field: float(row[key]), bootstrap_resamples,
                                  bootstrap_seed + zlib.crc32(metric.encode("utf-8")))
        output[f"{metric}_ci_low"] = low
        output[f"{metric}_ci_high"] = high
    for metric, field in CONTINUOUS_METRICS.items():
        output[metric] = statistics.mean(float(item[field]) for item in values)
        low, high = _bootstrap_ci(values, lambda row, key=field: float(row[key]), bootstrap_resamples,
                                  bootstrap_seed + zlib.crc32(metric.encode("utf-8")))
        output[f"{metric}_ci_low"] = low
        output[f"{metric}_ci_high"] = high
    output.update({
        "median_episode_time": statistics.median(float(item["episode_time"]) for item in values),
        "mean_path_length_ratio": statistics.mean(float(item["path_length_ratio"]) for item in values),
        "minimum_clearance": min(float(item["minimum_clearance"]) for item in values),
        "mean_cross_track_error": statistics.mean(float(item["mean_cross_track_error"]) for item in values),
        "mean_safety_cost": statistics.mean(float(item["safety_cost"]) for item in values),
        "shield_interventions_per_episode": statistics.mean(int(item["shield_interventions"]) for item in values),
        "shield_pass_ratio": sum(int(item["shield_pass"]) for item in values) / total_steps,
        "shield_clamp_ratio": sum(int(item["shield_clamp"]) for item in values) / total_steps,
        "shield_stop_ratio": sum(int(item["shield_stop"]) for item in values) / total_steps,
        "unsafe_without_shield_per_episode": statistics.mean(int(item["unsafe_without_shield"]) for item in values),
        "shield_slow_per_episode": statistics.mean(int(item.get("shield_slow", 0)) for item in values),
        "shield_evade_per_episode": statistics.mean(int(item.get("shield_evade", 0)) for item in values),
        "shield_reverse_per_episode": statistics.mean(int(item.get("shield_reverse", 0)) for item in values),
        "shield_false_positive_per_episode": statistics.mean(
            int(item.get("shield_false_positive", 0)) for item in values
        ),
        "shield_false_negative_per_episode": statistics.mean(
            int(item.get("shield_false_negative", 0)) for item in values
        ),
        "shield_actual_false_negative_per_episode": statistics.mean(
            int(item.get("shield_actual_false_negative", 0)) for item in values
        ),
        "shield_ineffective_intervention_per_episode": statistics.mean(
            int(item.get("shield_ineffective_intervention", 0)) for item in values
        ),
        "tracking_speed_error_mean": statistics.mean(
            float(item.get("tracking_speed_error_mean", 0.0)) for item in values
        ),
        "max_shield_sustained_time": max(float(item["shield_sustained_time"]) for item in values),
        "mean_inference_ms": statistics.mean(float(item["inference_ms"]) for item in values),
        "mean_simulation_steps_per_second": statistics.mean(
            float(item["simulation_steps_per_second"]) for item in values
        ),
    })
    return output


def summarize(rows: list[dict], bootstrap_resamples: int = 2000,
              bootstrap_seed: int = 20260914) -> list[dict]:
    groups: dict[tuple[str, str, bool], list[dict]] = defaultdict(list)
    for row in rows:
        family = str(row.get("policy_family", row["policy"]))
        role = str(row.get("checkpoint_role", "fixed"))
        groups[(family, role, bool(row["shield_enabled"]))].append(row)
    output = []
    for (family, role, shield), values in sorted(groups.items()):
        group_seed = bootstrap_seed + zlib.crc32(f"{family}:{role}:{shield}".encode("utf-8"))
        output.append({
            "policy": family,
            "checkpoint_role": role,
            "shield_enabled": shield,
            **_summary_group(values, bootstrap_resamples, group_seed),
        })
    return output


def summarize_by_training_seed(rows: list[dict]) -> list[dict]:
    groups: dict[tuple[str, str, str, bool], list[dict]] = defaultdict(list)
    for row in rows:
        family = str(row.get("policy_family", row["policy"]))
        role = str(row.get("checkpoint_role", "fixed"))
        groups[(family, role, _training_seed(row), bool(row["shield_enabled"]))].append(row)
    output = []
    for (family, role, training_seed, shield), values in sorted(groups.items()):
        output.append({
            "policy": family,
            "checkpoint_role": role,
            "training_seed": training_seed,
            "shield_enabled": shield,
            **_summary_group(values, 0, 0),
        })
    return output


def summarize_by_scenario(rows: list[dict], bootstrap_resamples: int = 2000,
                          bootstrap_seed: int = 20260914) -> list[dict]:
    groups: dict[tuple[str, str, bool, str], list[dict]] = defaultdict(list)
    for row in rows:
        family = str(row.get("policy_family", row["policy"]))
        role = str(row.get("checkpoint_role", "fixed"))
        groups[(family, role, bool(row["shield_enabled"]), str(row["scenario_id"]))].append(row)
    output = []
    for (family, role, shield, scenario), values in sorted(groups.items()):
        group_seed = bootstrap_seed + zlib.crc32(
            f"{family}:{role}:{shield}:{scenario}".encode("utf-8")
        )
        output.append({
            "policy": family,
            "checkpoint_role": role,
            "shield_enabled": shield,
            "scenario_id": scenario,
            **_summary_group(values, bootstrap_resamples, group_seed),
        })
    return output


def _paired_summary(values: list[dict], bootstrap_resamples: int, bootstrap_seed: int) -> dict:
    output = {
        "pairs": len(values),
        "training_seeds": len({seed for seed in map(_training_seed, values) if seed != ""}),
        "evaluation_seeds": len({int(row["seed"]) for row in values}),
        "scenarios": len({str(row["scenario_id"]) for row in values}),
    }
    for metric in ("success", "collision", "timeout", "minimum_clearance"):
        field = f"{metric}_delta"
        output[field] = statistics.mean(float(row[field]) for row in values)
        low, high = _bootstrap_ci(
            values,
            lambda row, key=field: float(row[key]),
            bootstrap_resamples,
            bootstrap_seed + zlib.crc32(field.encode("utf-8")),
        )
        output[f"{field}_ci_low"] = low
        output[f"{field}_ci_high"] = high
    return output


def paired_comparisons(rows: list[dict], reference_family: str = "lite_rule_baseline",
                       bootstrap_resamples: int = 2000,
                       bootstrap_seed: int = 20260914) -> list[dict]:
    output_rows: list[dict] = []
    reference = {}
    for row in rows:
        family = str(row.get("policy_family", row["policy"]))
        if family == reference_family:
            reference[(str(row["scenario_id"]), int(row["seed"]), bool(row["shield_enabled"]))] = row
    for row in rows:
        family = str(row.get("policy_family", row["policy"]))
        if family == reference_family:
            continue
        key = (str(row["scenario_id"]), int(row["seed"]), bool(row["shield_enabled"]))
        if key not in reference:
            continue
        baseline = reference[key]
        output_rows.append({
            "comparison": "policy_minus_rule",
            "policy": family,
            "checkpoint_role": str(row.get("checkpoint_role", "fixed")),
            "training_seed": _training_seed(row),
            "shield_enabled": bool(row["shield_enabled"]),
            "scenario_id": row["scenario_id"],
            "seed": row["seed"],
            "success_delta": int(row["success"]) - int(baseline["success"]),
            "collision_delta": int(row["collision"]) - int(baseline["collision"]),
            "timeout_delta": int(row["truncated"]) - int(baseline["truncated"]),
            "minimum_clearance_delta": (
                float(row["minimum_clearance"]) - float(baseline["minimum_clearance"])
            ),
        })
    off_rows = {}
    for row in rows:
        if not bool(row["shield_enabled"]):
            key = (
                str(row.get("policy_family", row["policy"])),
                str(row.get("checkpoint_role", "fixed")),
                _training_seed(row),
                str(row["scenario_id"]),
                int(row["seed"]),
            )
            off_rows[key] = row
    for row in rows:
        if not bool(row["shield_enabled"]):
            continue
        key = (
            str(row.get("policy_family", row["policy"])),
            str(row.get("checkpoint_role", "fixed")),
            _training_seed(row),
            str(row["scenario_id"]),
            int(row["seed"]),
        )
        if key not in off_rows:
            continue
        baseline = off_rows[key]
        output_rows.append({
            "comparison": "shield_on_minus_off",
            "policy": key[0],
            "checkpoint_role": key[1],
            "training_seed": key[2],
            "shield_enabled": True,
            "scenario_id": row["scenario_id"],
            "seed": row["seed"],
            "success_delta": int(row["success"]) - int(baseline["success"]),
            "collision_delta": int(row["collision"]) - int(baseline["collision"]),
            "timeout_delta": int(row["truncated"]) - int(baseline["truncated"]),
            "minimum_clearance_delta": (
                float(row["minimum_clearance"]) - float(baseline["minimum_clearance"])
            ),
        })
    groups: dict[tuple[str, str, str, bool], list[dict]] = defaultdict(list)
    for row in output_rows:
        groups[(row["comparison"], row["policy"], row["checkpoint_role"],
                row["shield_enabled"])].append(row)
    summaries = []
    for key, values in sorted(groups.items()):
        seed = bootstrap_seed + zlib.crc32(":".join(map(str, key)).encode("utf-8"))
        summaries.append({
            "comparison": key[0],
            "policy": key[1],
            "checkpoint_role": key[2],
            "shield_enabled": key[3],
            **_paired_summary(values, bootstrap_resamples, seed),
        })
    return summaries
