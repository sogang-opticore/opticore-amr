from __future__ import annotations

import argparse
import json

from amr_lite.config import ROOT, load_config
from amr_lite.evaluation import evaluate
from amr_lite.evaluation.plots import plot_training
from amr_lite.learning.collect_teacher import collect
from amr_lite.learning.networks import load_policy
from amr_lite.learning.train_bc import train as train_bc
from amr_lite.learning.train_ppo import train as train_ppo
from amr_lite.planning import RuleBasedPlanner

from .demo import run_demo


def smoke_pipeline() -> dict:
    dataset = ROOT / "artifacts/datasets/teacher"
    bc_checkpoint = ROOT / "artifacts/checkpoints/bc_smoke.pt"
    ppo_checkpoint = ROOT / "artifacts/checkpoints/ppo_smoke.pt"
    smoke_root = ROOT / "artifacts/smoke"
    print("[1/5] Collecting teacher episodes", flush=True)
    chunks = collect(dataset, episodes=6, max_steps=360, seed=7)
    print("[2/5] Training BC smoke checkpoint", flush=True)
    bc_metrics = train_bc(dataset, bc_checkpoint, smoke=True)
    print("[3/5] Training PPO smoke checkpoint", flush=True)
    ppo_metrics = train_ppo(ppo_checkpoint, smoke=True, verbose=True)
    training_plot = plot_training(ppo_checkpoint.with_suffix(".training.csv"), smoke_root / "plots/training_curve.png")
    policies = {"lite_rule_baseline": RuleBasedPlanner(), "bc": load_policy(str(bc_checkpoint)),
                "ppo": load_policy(str(ppo_checkpoint))}
    evaluation = load_config("evaluation")
    evaluation.update({"scenarios": ["S0", "S3", "D0"], "episodes_per_scenario": 1})
    print("[4/5] Evaluating smoke policies", flush=True)
    _, summary = evaluate(policies, smoke_root / "results", evaluation)
    demos = {}
    print("[5/5] Rendering smoke demos", flush=True)
    for name, policy in policies.items():
        target = smoke_root / f"videos/{name}_S3.png"
        demos[name] = run_demo(policy, "S3", 42, target)
    return {"dataset_chunks": len(chunks), "bc": bc_metrics, "ppo": ppo_metrics,
            "training_plot": str(training_plot), "evaluation": summary,
            "demos": {key: {"success": value.get("success"), "failure_type": value.get("failure_type")}
                      for key, value in demos.items()}}


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the complete Phase 0 smoke pipeline")
    parser.parse_args()
    print(json.dumps(smoke_pipeline(), indent=2))


if __name__ == "__main__":
    main()
