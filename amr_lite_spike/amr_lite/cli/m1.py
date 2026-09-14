from __future__ import annotations

import argparse
import json
import sys
from copy import deepcopy
from pathlib import Path

import torch

from amr_lite.config import ROOT, load_config
from amr_lite.evaluation import evaluate
from amr_lite.experiments.manifest import (
    build_manifest,
    dataset_snapshot,
    sha256_file,
    write_manifest,
)
from amr_lite.learning.networks import load_policy
from amr_lite.learning.train_ppo import train as train_ppo
from amr_lite.planning import RuleBasedPlanner


def _resolve(path: str | Path) -> Path:
    value = Path(path)
    return value if value.is_absolute() else ROOT / value


def _evaluation_seeds(config: dict) -> list[int]:
    if config.get("seeds") is not None:
        return [int(seed) for seed in config["seeds"]]
    return [int(config["seed"]) + offset
            for offset in range(int(config["episodes_per_scenario"]))]


def _checkpoint_training_seed(path: Path) -> int | str:
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    return checkpoint.get("config", {}).get("seed", "")


def _annotate(policy, family: str, role: str, training_seed: int | str,
              checkpoint: Path | None = None):
    policy.policy_family = family
    policy.checkpoint_role = role
    policy.training_seed = training_seed
    policy.checkpoint_sha256 = sha256_file(checkpoint) if checkpoint is not None else ""
    return policy


def _can_resume(path: Path, seed: int, total_timesteps: int) -> bool:
    if not path.exists():
        return False
    try:
        checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    except Exception:
        return False
    return (int(checkpoint.get("config", {}).get("seed", -1)) == seed
            and int(checkpoint.get("total_timesteps", -1)) == total_timesteps)


def _format_rate(row: dict, metric: str) -> str:
    return (f"{100.0 * float(row[metric]):.1f}% "
            f"[{100.0 * float(row[f'{metric}_ci_low']):.1f}, "
            f"{100.0 * float(row[f'{metric}_ci_high']):.1f}]")


def _write_report(path: Path, summary: list[dict], experiment: dict) -> None:
    lines = [
        "# M1 Reproducible Baseline",
        "",
        f"- Experiment: {experiment['experiment_version']}",
        f"- Training seeds: {experiment['training_seeds']}",
        f"- Selection config: {experiment['selection_config']}",
        f"- Test config: {experiment['test_config']}",
        "",
        "Binary and clearance intervals are 95% scenario-stratified bootstrap intervals. "
        "Wilson intervals are also available in summary.csv.",
        "",
        "| Policy | Checkpoint | Shield | Episodes | Success 95% CI | Collision 95% CI | "
        "Timeout 95% CI | Mean clearance 95% CI (m) |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary:
        clearance = (
            f"{float(row['mean_minimum_clearance']):.3f} "
            f"[{float(row['mean_minimum_clearance_ci_low']):.3f}, "
            f"{float(row['mean_minimum_clearance_ci_high']):.3f}]"
        )
        lines.append(
            f"| {row['policy']} | {row['checkpoint_role']} | {row['shield_enabled']} | "
            f"{row['episodes']} | {_format_rate(row, 'success_rate')} | "
            f"{_format_rate(row, 'collision_rate')} | {_format_rate(row, 'timeout_rate')} | "
            f"{clearance} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_m1(*, output: str | Path | None = None, full: bool = True,
           training_seeds: list[int] | None = None, test_episodes: int | None = None,
           resume: bool = True) -> dict:
    experiment = load_config("m1")
    seeds = [int(seed) for seed in (training_seeds or experiment["training_seeds"])]
    experiment["training_seeds"] = seeds
    output_root = _resolve(output or experiment["output"])
    output_root.mkdir(parents=True, exist_ok=True)
    checkpoint_dir = output_root / "checkpoints"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    ppo_config = load_config(str(experiment["ppo_config"]))
    selection_config = load_config(str(experiment["selection_config"]))
    test_config = load_config(str(experiment["test_config"]))
    if test_episodes is not None:
        test_config["episodes_per_scenario"] = int(test_episodes)
    if not full:
        selection_config.update({
            "scenarios": ["S0", "S3", "D0"],
            "episodes_per_scenario": 1,
            "bootstrap_resamples": 100,
        })
        test_config.update({
            "scenarios": ["S0", "S3", "D0"],
            "episodes_per_scenario": int(test_episodes or 1),
            "bootstrap_resamples": 100,
        })
        ppo_config["evaluation_interval_updates"] = 0
    selection_seeds = set(_evaluation_seeds(selection_config))
    test_seeds = set(_evaluation_seeds(test_config))
    overlap = selection_seeds & test_seeds
    if overlap:
        raise ValueError(f"Selection and test seeds overlap: {sorted(overlap)}")

    dataset_config = experiment["dataset"]
    dataset_path = _resolve(dataset_config["path"])
    if not dataset_path.exists():
        raise FileNotFoundError(f"Teacher dataset not found: {dataset_path}")
    resolved = {
        "experiment": experiment,
        "ppo": ppo_config,
        "selection": selection_config,
        "test": test_config,
    }
    (output_root / "resolved_config.json").write_text(
        json.dumps(resolved, indent=2, sort_keys=True), encoding="utf-8"
    )

    total_timesteps = int(
        ppo_config["total_timesteps_full"] if full else ppo_config["total_timesteps_smoke"]
    )
    training_results = []
    trained_checkpoints: list[Path] = []
    print(f"M1 training seeds: {seeds}", flush=True)
    for index, seed in enumerate(seeds, start=1):
        seed_config = deepcopy(ppo_config)
        seed_config["seed"] = seed
        seed_config["selection_config"] = experiment["selection_config"]
        final_path = checkpoint_dir / f"custom_stable_ppo_seed_{seed}_final.pt"
        print(f"[train {index}/{len(seeds)}] seed={seed}", flush=True)
        if resume and _can_resume(final_path, seed, total_timesteps):
            metrics_path = final_path.with_suffix(".metrics.json")
            result = json.loads(metrics_path.read_text(encoding="utf-8"))
            print(f"  resumed {final_path}", flush=True)
        else:
            result = train_ppo(
                final_path,
                smoke=not full,
                config=seed_config,
                verbose=True,
                warm_start_dataset=dataset_path,
                stable_architecture=True,
            )
        best_path = Path(result["best_checkpoint"]) if result.get("best_checkpoint") else final_path
        if not best_path.is_absolute():
            best_path = ROOT / best_path
        if full and not best_path.exists():
            raise FileNotFoundError(f"Best checkpoint missing for seed {seed}: {best_path}")
        result["training_seed"] = seed
        result["final_checkpoint"] = str(final_path)
        result["best_checkpoint"] = str(best_path)
        training_results.append(result)
        trained_checkpoints.extend([final_path, best_path])

    policies = {}
    policies["lite_rule_baseline"] = _annotate(
        RuleBasedPlanner(), "lite_rule_baseline", "fixed", ""
    )

    bc_path = _resolve(experiment["bc_checkpoint"])
    if bc_path.exists():
        bc_seed = _checkpoint_training_seed(bc_path)
        policies["bc_full"] = _annotate(
            load_policy(str(bc_path)), "bc", "fixed", bc_seed, bc_path
        )
    legacy_path = _resolve(experiment["legacy_ppo_checkpoint"])
    if legacy_path.exists():
        legacy_seed = _checkpoint_training_seed(legacy_path)
        policies["legacy_stable_ppo"] = _annotate(
            load_policy(str(legacy_path)), "legacy_stable_ppo", "legacy_best",
            legacy_seed, legacy_path
        )

    for result in training_results:
        seed = int(result["training_seed"])
        final_path = Path(result["final_checkpoint"])
        best_path = Path(result["best_checkpoint"])
        policies[f"custom_stable_ppo_seed_{seed}_final"] = _annotate(
            load_policy(str(final_path)), "custom_stable_ppo", "final", seed, final_path
        )
        policies[f"custom_stable_ppo_seed_{seed}_best"] = _annotate(
            load_policy(str(best_path)), "custom_stable_ppo", "best", seed, best_path
        )

    print(f"[evaluate] policies={len(policies)} test_seeds={len(test_seeds)}", flush=True)
    evaluation_dir = output_root / "test"
    _, summary = evaluate(policies, evaluation_dir, test_config)
    dataset = dataset_snapshot(dataset_path, str(dataset_config["version"]))
    manifest = build_manifest(
        experiment={**experiment, "mode": "full" if full else "smoke",
                    "training_results": training_results},
        dataset=dataset,
        evaluation={"selection": selection_config, "test": test_config},
        checkpoints=[*trained_checkpoints, bc_path, legacy_path],
        command=sys.argv,
    )
    manifest_path = write_manifest(output_root / "manifest.json", manifest)
    report_path = output_root / "REPORT.md"
    _write_report(report_path, summary, experiment)
    result = {
        "output": str(output_root),
        "manifest": str(manifest_path),
        "report": str(report_path),
        "summary": str(evaluation_dir / "summary.csv"),
        "training_seeds": seeds,
        "selection_seeds": sorted(selection_seeds),
        "test_seeds": sorted(test_seeds),
        "training_results": training_results,
    }
    (output_root / "run_result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True), encoding="utf-8"
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the M1 reproducible evaluation baseline")
    parser.add_argument("--output")
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--training-seeds", type=int, nargs="+")
    parser.add_argument("--test-episodes", type=int)
    parser.add_argument("--no-resume", action="store_true")
    args = parser.parse_args()
    print(json.dumps(run_m1(
        output=args.output,
        full=not args.smoke,
        training_seeds=args.training_seeds,
        test_episodes=args.test_episodes,
        resume=not args.no_resume,
    ), indent=2))


if __name__ == "__main__":
    main()
