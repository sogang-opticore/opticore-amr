from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from amr_lite.config import ROOT, load_config
from amr_lite.envs import WarehouseEnv
from amr_lite.planning import RuleBasedPlanner

from .dataset import save_episode


def collect(output_dir: str | Path, episodes: int = 8, max_steps: int = 360,
            seed: int = 7, scenarios: list[str] | None = None,
            learner=None, dagger_risk_only: bool = False) -> list[Path]:
    output = Path(output_dir)
    env_config = load_config("env")
    teacher = RuleBasedPlanner(env_config)
    scenario_ids = scenarios or ["S0", "S1", "S2", "S3", "S4", "S5"]
    created = []
    for episode in range(episodes):
        episode_seed = seed + episode
        scenario = scenario_ids[episode % len(scenario_ids)]
        env = WarehouseEnv(env_config, scenario)
        observation, _ = env.reset(seed=episode_seed,
                                   options={"scenario_id": scenario, "pose_perturbation": 0.08})
        observations, actions = [], []
        metadata = {key: [] for key in ("episode_id", "step_index", "scenario_id", "map_id", "seed",
                                         "robot_pose", "goal_distance", "path_error", "minimum_clearance",
                                         "collision", "success", "terminated", "truncated")}
        for step in range(max_steps):
            teacher_action = teacher.predict(observation)
            execution_action = learner.predict(observation) if learner is not None else teacher_action
            next_observation, _, terminated, truncated, info = env.step(execution_action)
            risky = info["minimum_clearance"] < 0.7 or abs(info["mean_cross_track_error"]) > 0.25 or terminated or truncated
            if not dagger_risk_only or risky:
                observations.append(observation.copy())
                actions.append(teacher_action.copy())
                values = (episode_seed, step, scenario, scenario, episode_seed, info["robot_pose"],
                          info["goal_distance"], info["mean_cross_track_error"], info["minimum_clearance"],
                          info["collision"], info["success"], terminated, truncated)
                for key, value in zip(metadata, values):
                    metadata[key].append(value)
            observation = next_observation
            if terminated or truncated:
                break
        if observations:
            path = output / f"episode_{episode:04d}_{scenario}.npz"
            created.append(save_episode(path, observations, actions, metadata))
        env.close()
    return created


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect lite_rule_baseline teacher episodes")
    parser.add_argument("--output", default=str(ROOT / "artifacts/datasets/teacher"))
    parser.add_argument("--episodes", type=int, default=8)
    parser.add_argument("--max-steps", type=int, default=360)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    files = collect(args.output, args.episodes, args.max_steps, args.seed)
    print(f"created {len(files)} episode chunks in {args.output}")


if __name__ == "__main__":
    main()
