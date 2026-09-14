from __future__ import annotations

import argparse

from amr_lite.config import ROOT

from .collect_teacher import collect
from .networks import load_policy


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect risk states visited by BC and relabel with teacher")
    parser.add_argument("--checkpoint", default=str(ROOT / "artifacts/checkpoints/bc_smoke.pt"))
    parser.add_argument("--output", default=str(ROOT / "artifacts/datasets/dagger"))
    parser.add_argument("--episodes", type=int, default=4)
    parser.add_argument("--seed", type=int, default=200)
    args = parser.parse_args()
    files = collect(args.output, args.episodes, seed=args.seed, learner=load_policy(args.checkpoint), dagger_risk_only=True)
    print(f"created {len(files)} DAgger-lite chunks")


if __name__ == "__main__":
    main()
