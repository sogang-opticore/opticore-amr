from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from amr_lite.config import ROOT
from amr_lite.envs import WarehouseEnv
from amr_lite.learning.networks import load_policy
from amr_lite.planning import RuleBasedPlanner


def run_demo(policy: Any, scenario: str, seed: int, output: str | Path, show: bool = False) -> dict:
    env = WarehouseEnv(scenario_id=scenario, render_mode="human" if show else "rgb_array")
    env.policy_name = getattr(policy, "name", policy.__class__.__name__)
    observation, _ = env.reset(seed=seed, options={"scenario_id": scenario})
    info = {}
    for _ in range(500):
        observation, _, terminated, truncated, info = env.step(policy.predict(observation))
        if show:
            env.render()
        if terminated or truncated:
            break
    env.save_render(Path(output))
    env.close()
    return info


def main() -> None:
    parser = argparse.ArgumentParser(description="Run one AMR-Lite episode and save a final-frame demo")
    parser.add_argument("--policy", choices=["rule", "bc", "ppo"], default="rule")
    parser.add_argument("--checkpoint")
    parser.add_argument("--scenario", default="S3")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default=str(ROOT / "artifacts/videos/demo.png"))
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    if args.policy == "rule":
        policy = RuleBasedPlanner()
    else:
        checkpoint = args.checkpoint or str(ROOT / f"artifacts/checkpoints/{args.policy}_smoke.pt")
        policy = load_policy(checkpoint)
    info = run_demo(policy, args.scenario, args.seed, args.output, args.show)
    print(f"policy={args.policy} scenario={args.scenario} success={info.get('success')} failure={info.get('failure_type')} output={args.output}")


if __name__ == "__main__":
    main()
