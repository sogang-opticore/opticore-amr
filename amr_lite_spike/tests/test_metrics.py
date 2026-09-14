from amr_lite.evaluation.metrics import summarize, wilson_interval


def _row(seed: int, training_seed: int, success: int, collision: int, clearance: float) -> dict:
    return {
        "policy": f"ppo_{training_seed}",
        "policy_family": "ppo",
        "checkpoint_role": "best",
        "training_seed": training_seed,
        "shield_enabled": False,
        "scenario_id": "S0" if seed % 2 == 0 else "D1",
        "seed": seed,
        "success": success,
        "collision": collision,
        "truncated": int(not success and not collision),
        "steps": 10,
        "minimum_clearance": clearance,
        "episode_time": 1.0,
        "path_length_ratio": 1.0,
        "mean_cross_track_error": 0.0,
        "safety_cost": 0.0,
        "shield_interventions": 0,
        "shield_pass": 10,
        "shield_clamp": 0,
        "shield_stop": 0,
        "unsafe_without_shield": 0,
        "shield_sustained_time": 0.0,
        "inference_ms": 0.1,
        "simulation_steps_per_second": 1000.0,
    }


def test_wilson_interval_contains_observed_rate():
    low, high = wilson_interval(7, 8)
    assert 0.0 <= low <= 7 / 8 <= high <= 1.0


def test_summary_has_reproducible_hierarchical_confidence_intervals():
    rows = [
        _row(30000, 7, 1, 0, 0.20),
        _row(30001, 7, 0, 1, -0.01),
        _row(30000, 17, 1, 0, 0.30),
        _row(30001, 17, 1, 0, 0.10),
    ]
    first = summarize(rows, bootstrap_resamples=200, bootstrap_seed=9)
    second = summarize(rows, bootstrap_resamples=200, bootstrap_seed=9)
    assert first == second
    summary = first[0]
    assert summary["training_seeds"] == 2
    assert summary["success_rate_ci_low"] <= summary["success_rate"] <= summary["success_rate_ci_high"]
    assert (summary["mean_minimum_clearance_ci_low"]
            <= summary["mean_minimum_clearance"]
            <= summary["mean_minimum_clearance_ci_high"])
