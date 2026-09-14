from __future__ import annotations

import argparse
import json

from amr_lite.config import ROOT, load_config
from amr_lite.evaluation import evaluate
from amr_lite.learning.networks import load_policy
from amr_lite.planning import RuleBasedPlanner


def main() -> None:
    parser = argparse.ArgumentParser(description="Paired evaluation for rule, BC, and PPO")
    parser.add_argument("--bc", default=str(ROOT / "artifacts/checkpoints/bc_smoke.pt"))
    parser.add_argument("--ppo", default=str(ROOT / "artifacts/checkpoints/ppo_smoke.pt"))
    parser.add_argument("--dagger", default=str(ROOT / "artifacts/checkpoints/bc_dagger_smoke.pt"))
    parser.add_argument("--output", default=str(ROOT / "artifacts/results"))
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    policies = {"lite_rule_baseline": RuleBasedPlanner(), "bc": load_policy(args.bc), "ppo": load_policy(args.ppo)}
    from pathlib import Path
    if Path(args.dagger).exists():
        policies["bc_dagger"] = load_policy(args.dagger)
    config = load_config("evaluation")
    if args.quick:
        config.update({"scenarios": ["S0", "S3", "D0"], "episodes_per_scenario": 1, "max_steps": 360})
    _, summary = evaluate(policies, args.output, config)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
