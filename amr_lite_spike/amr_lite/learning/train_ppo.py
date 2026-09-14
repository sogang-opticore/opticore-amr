from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import random
from pathlib import Path

import numpy as np
import torch
from torch.distributions import Normal
from torch.utils.data import DataLoader, TensorDataset

from amr_lite.config import ROOT, deep_merge, load_config
from amr_lite.envs import WarehouseEnv

from .networks import ActorCritic, StableActorCritic
from .dataset import load_dataset


def _log_prob(distribution: Normal, latent: torch.Tensor) -> torch.Tensor:
    return distribution.log_prob(latent).sum(-1)


class RunningReturnStats:
    def __init__(self) -> None:
        self.mean = 0.0
        self.var = 1.0
        self.count = 1e-4

    @property
    def std(self) -> float:
        return math.sqrt(self.var + 1e-8)

    def update(self, values: np.ndarray) -> None:
        batch_mean, batch_var, batch_count = float(np.mean(values)), float(np.var(values)), len(values)
        delta = batch_mean - self.mean
        total = self.count + batch_count
        self.mean += delta * batch_count / total
        self.var = (self.var * self.count + batch_var * batch_count
                    + delta * delta * self.count * batch_count / total) / total
        self.count = total

    def normalize(self, values: np.ndarray) -> np.ndarray:
        return (values - self.mean) / self.std

    def denormalize_scalar(self, value: torch.Tensor) -> float:
        return float((value * self.std + self.mean).item())

    def as_dict(self) -> dict[str, float]:
        return {"mean": self.mean, "var": self.var, "count": self.count}


def _actor_parameters(model: ActorCritic | StableActorCritic) -> list[torch.nn.Parameter]:
    body = model.actor_body if isinstance(model, StableActorCritic) else model.body
    return [*body.parameters(), *model.actor_mean.parameters(), model.log_std]


def _critic_parameters(model: ActorCritic | StableActorCritic) -> list[torch.nn.Parameter]:
    if isinstance(model, StableActorCritic):
        return [*model.critic_body.parameters(), *model.critic.parameters()]
    return list(model.critic.parameters())


def _warm_start_actor(model: ActorCritic | StableActorCritic, dataset_dir: str | Path, cfg: dict,
                      seed: int, verbose: bool) -> float:
    data = load_dataset(dataset_dir)
    loader = DataLoader(TensorDataset(torch.from_numpy(data.observations), torch.from_numpy(data.actions)),
                        batch_size=cfg["minibatch_size"], shuffle=True,
                        generator=torch.Generator().manual_seed(seed))
    parameters = _actor_parameters(model)
    optimizer = torch.optim.Adam(parameters, lr=cfg.get("warm_start_learning_rate", 0.001))
    loss_value = 0.0
    model.train()
    for _ in range(int(cfg.get("warm_start_epochs", 15))):
        for observations, teacher_actions in loader:
            mean, _, _ = model(observations)
            loss = torch.mean((torch.tanh(mean) - teacher_actions) ** 2)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            loss_value = float(loss.item())
    if verbose:
        print(f"BC warm start complete | samples={len(data.observations)} | final_batch_mse={loss_value:.6f}", flush=True)
    return loss_value


@torch.no_grad()
def _evaluate_actor(model: ActorCritic | StableActorCritic, env_config: dict,
                    evaluation_config: dict) -> dict[str, float]:
    was_training = model.training
    model.eval()
    local_env_config = deep_merge(env_config, evaluation_config.get("environment_overrides"))
    seeds = ([int(value) for value in evaluation_config["seeds"]]
             if evaluation_config.get("seeds") is not None
             else [int(evaluation_config["seed"]) + index
                   for index in range(int(evaluation_config.get("episodes_per_scenario", 1)))])
    successes = collisions = timeouts = 0
    elapsed_times, clearances = [], []
    for scenario in evaluation_config["scenarios"]:
        for seed in seeds:
            env = WarehouseEnv(local_env_config, scenario)
            observation, _ = env.reset(seed=seed, options={"scenario_id": scenario})
            info = {}
            truncated = False
            for _ in range(int(evaluation_config.get(
                    "max_steps", local_env_config["episode_timeout"] / local_env_config["policy_dt"]))):
                tensor = torch.as_tensor(observation, dtype=torch.float32).unsqueeze(0)
                mean, _, _ = model(tensor)
                observation, _, terminated, truncated, info = env.step(
                    torch.tanh(mean).squeeze(0).numpy()
                )
                if terminated or truncated:
                    break
            successes += int(info.get("success", False))
            collisions += int(info.get("collision", False))
            timeouts += int(truncated)
            elapsed_times.append(float(info.get("elapsed_time", local_env_config["episode_timeout"])))
            clearances.append(float(info.get("minimum_clearance_episode", 0.0)))
            env.close()
    if was_training:
        model.train()
    count = max(1, len(evaluation_config["scenarios"]) * len(seeds))
    return {"eval_success_rate": successes / count, "eval_collision_rate": collisions / count,
            "eval_timeout_rate": timeouts / count,
            "eval_mean_clearance": float(np.mean(clearances)),
            "eval_mean_time": float(np.mean(elapsed_times)),
            "eval_episodes": count,
            "eval_split": str(evaluation_config.get("split", "selection"))}


def _selection_config(cfg: dict, training_seed: int) -> dict:
    name = cfg.get("selection_config")
    if name:
        return load_config(str(name))
    return {
        "split": "legacy-selection",
        "seed": training_seed + 50_000,
        "episodes_per_scenario": int(cfg.get("best_eval_episodes_per_scenario", 1)),
        "scenarios": list(cfg.get("best_eval_scenarios", ["S0", "S1", "S3"])),
        "max_steps": 360,
    }


def _save_checkpoint(path: Path, model: ActorCritic | StableActorCritic, cfg: dict, timesteps: int,
                     warm_start_dataset: str | Path | None, evaluation: dict | None = None,
                     value_stats: RunningReturnStats | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"kind": model.checkpoint_kind, "hidden_size": cfg["hidden_size"], "model_state": model.state_dict(),
                "config": cfg, "total_timesteps": timesteps,
                "warm_start_dataset": str(warm_start_dataset) if warm_start_dataset else None,
                "evaluation": evaluation, "value_stats": value_stats.as_dict() if value_stats else None}, path)


def train(output: str | Path, smoke: bool = True, config: dict | None = None,
          verbose: bool = False, warm_start_dataset: str | Path | None = None,
          warm_start_only: bool = False, stable_architecture: bool = False) -> dict:
    cfg = config or load_config("ppo")
    torch.set_num_threads(1)
    seed = int(cfg["seed"])
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    env_config = deep_merge(load_config("env"), cfg.get("environment_overrides"))
    selection_config = _selection_config(cfg, seed)
    env = WarehouseEnv(env_config, "S0")
    observation, _ = env.reset(seed=seed)
    model: ActorCritic | StableActorCritic = (StableActorCritic(hidden_size=cfg["hidden_size"])
                                               if stable_architecture else ActorCritic(hidden_size=cfg["hidden_size"]))
    target = Path(output)
    warm_start_mse = (_warm_start_actor(model, warm_start_dataset, cfg, seed, verbose)
                      if warm_start_dataset is not None else None)
    reference_model = None
    if warm_start_dataset is not None:
        model.log_std.data.fill_(float(cfg.get("warm_start_log_std", -1.5)))
        reference_model = copy.deepcopy(model).eval()
    if warm_start_only:
        if warm_start_dataset is None:
            raise ValueError("warm_start_only requires warm_start_dataset")
        _save_checkpoint(target, model, cfg, 0, warm_start_dataset)
        result = {"checkpoint": str(target), "total_timesteps": 0,
                  "warm_start_dataset": str(warm_start_dataset), "warm_start_mse": warm_start_mse,
                  "note": "actor diagnostic before PPO updates"}
        target.with_suffix(".metrics.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        env.close()
        return result
    policy_learning_rate = (cfg.get("fine_tune_learning_rate", cfg["learning_rate"])
                            if warm_start_dataset is not None else cfg["learning_rate"])
    optimizer = torch.optim.Adam(model.parameters(), lr=policy_learning_rate)
    critic_optimizer = torch.optim.Adam(_critic_parameters(model), lr=cfg["learning_rate"])
    value_stats = RunningReturnStats()
    normalize_values = bool(stable_architecture and cfg.get("value_normalization", False))
    rollout_steps = cfg["rollout_steps_smoke"] if smoke else cfg["rollout_steps_full"]
    total_timesteps = cfg["total_timesteps_smoke"] if smoke else cfg["total_timesteps_full"]
    logs: list[dict] = []
    best_path = target.with_name(f"{target.stem}_best{target.suffix}")
    best_score = (-1.0, -1.0, -1.0, -math.inf, -math.inf)
    best_evaluation = None
    if warm_start_dataset is not None and not smoke:
        best_evaluation = _evaluate_actor(model, env_config, selection_config)
        best_score = (best_evaluation["eval_success_rate"], -best_evaluation["eval_collision_rate"],
                      -best_evaluation["eval_timeout_rate"], best_evaluation["eval_mean_clearance"],
                      -best_evaluation["eval_mean_time"])
        _save_checkpoint(best_path, model, cfg, 0, warm_start_dataset, best_evaluation, value_stats)
        if verbose:
            print(f"Initial eval | success={best_evaluation['eval_success_rate']:.3f} "
                  f"| collision={best_evaluation['eval_collision_rate']:.3f}", flush=True)
    episode_reward, episode_length, episode_number = 0.0, 0, 0
    update_total = math.ceil(total_timesteps / rollout_steps)
    report_every = max(1, update_total // 10)
    for update_index, rollout_start in enumerate(range(0, total_timesteps, rollout_steps), start=1):
        observations, latents, old_log_probs, rewards, dones, values = [], [], [], [], [], []
        episode_stats = []
        for _ in range(min(rollout_steps, total_timesteps - rollout_start)):
            tensor = torch.as_tensor(observation, dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                mean, log_std, value = model(tensor)
                distribution = Normal(mean, log_std.exp())
                latent = distribution.sample()
                action = torch.tanh(latent)
                log_prob = _log_prob(distribution, latent)
            next_observation, reward, terminated, truncated, info = env.step(action.squeeze(0).numpy())
            observations.append(observation)
            latents.append(latent.squeeze(0).numpy())
            old_log_probs.append(float(log_prob.item()))
            values.append(value_stats.denormalize_scalar(value) if normalize_values else float(value.item()))
            rewards.append(reward)
            done = terminated or truncated
            dones.append(float(done))
            episode_reward += reward
            episode_length += 1
            observation = next_observation
            if done:
                episode_stats.append({"episode_reward": episode_reward, "episode_length": episode_length,
                                      "success": int(info["success"]), "collision": int(info["collision"]),
                                      "safety_cost": float(info.get("safety_cost", 0.0)),
                                      "minimum_clearance": float(info["minimum_clearance_episode"]),
                                      "shield_interventions": info["shield_counts"]["CLAMP"] + info["shield_counts"]["STOP"]})
                episode_number += 1
                progress_fraction = (rollout_start + len(rewards)) / total_timesteps
                if (not smoke and cfg.get("curriculum_enabled", False)
                        and progress_fraction < cfg.get("curriculum_straight_fraction", 0.3)):
                    scenario = "S0"
                elif (not smoke and cfg.get("curriculum_enabled", False)
                      and progress_fraction < cfg.get("curriculum_corner_fraction", 0.6)):
                    scenario = ["S0", "S1"][episode_number % 2]
                else:
                    scenario = ["S0", "S1", "S3"][episode_number % 3]
                env = WarehouseEnv(env_config, scenario)
                observation, _ = env.reset(seed=seed + episode_number, options={"scenario_id": scenario})
                episode_reward, episode_length = 0.0, 0
        with torch.no_grad():
            _, _, next_value_tensor = model(torch.as_tensor(observation, dtype=torch.float32).unsqueeze(0))
        advantages = np.zeros(len(rewards), dtype=np.float32)
        last_gae = 0.0
        next_value = (value_stats.denormalize_scalar(next_value_tensor)
                      if normalize_values else float(next_value_tensor.item()))
        for index in reversed(range(len(rewards))):
            nonterminal = 1.0 - dones[index]
            delta = rewards[index] + cfg["gamma"] * next_value * nonterminal - values[index]
            last_gae = delta + cfg["gamma"] * cfg["gae_lambda"] * nonterminal * last_gae
            advantages[index] = last_gae
            next_value = values[index]
        returns = advantages + np.asarray(values, dtype=np.float32)
        obs_t = torch.as_tensor(np.asarray(observations), dtype=torch.float32)
        latent_t = torch.as_tensor(np.asarray(latents), dtype=torch.float32)
        old_log_t = torch.as_tensor(old_log_probs, dtype=torch.float32)
        if normalize_values:
            value_stats.update(returns)
            return_targets = value_stats.normalize(returns)
        else:
            return_targets = returns
        return_t = torch.as_tensor(return_targets, dtype=torch.float32)
        advantage_t = torch.as_tensor(advantages, dtype=torch.float32)
        advantage_t = (advantage_t - advantage_t.mean()) / (advantage_t.std() + 1e-8)
        policy_loss = value_loss = entropy_value = approx_kl_value = 0.0
        generator = torch.Generator().manual_seed(seed + rollout_start)
        stop_for_kl = False
        for _ in range(cfg["epochs"]):
            permutation = torch.randperm(len(obs_t), generator=generator)
            for start in range(0, len(obs_t), cfg["minibatch_size"]):
                idx = permutation[start:start + cfg["minibatch_size"]]
                if (warm_start_dataset is not None
                        and rollout_start < int(cfg.get("critic_only_steps", 0))):
                    if isinstance(model, StableActorCritic):
                        predicted_value = model.critic(model.critic_body(obs_t[idx])).squeeze(-1)
                    else:
                        with torch.no_grad():
                            frozen_hidden = model.body(obs_t[idx])
                        predicted_value = model.critic(frozen_hidden).squeeze(-1)
                    critic_loss = torch.mean((predicted_value - return_t[idx]) ** 2)
                    critic_optimizer.zero_grad()
                    critic_loss.backward()
                    critic_optimizer.step()
                    policy_loss, value_loss, entropy_value = 0.0, float(critic_loss.item()), 0.0
                    continue
                mean, log_std, predicted_value = model(obs_t[idx])
                distribution = Normal(mean, log_std.exp())
                new_log = _log_prob(distribution, latent_t[idx])
                approx_kl = torch.mean(old_log_t[idx] - new_log)
                approx_kl_value = float(approx_kl.item())
                if approx_kl_value > float(cfg.get("target_kl", math.inf)):
                    stop_for_kl = True
                    break
                ratio = torch.exp(new_log - old_log_t[idx])
                unclipped = ratio * advantage_t[idx]
                clipped = torch.clamp(ratio, 1.0 - cfg["clip_ratio"], 1.0 + cfg["clip_ratio"]) * advantage_t[idx]
                actor_loss = -torch.min(unclipped, clipped).mean()
                critic_loss = torch.mean((predicted_value - return_t[idx]) ** 2)
                entropy = distribution.entropy().sum(-1).mean()
                anchor_loss = torch.tensor(0.0)
                if reference_model is not None:
                    with torch.no_grad():
                        reference_mean, _, _ = reference_model(obs_t[idx])
                    anchor_loss = torch.mean((torch.tanh(mean) - torch.tanh(reference_mean)) ** 2)
                entropy_coef = (cfg.get("fine_tune_entropy_coef", cfg["entropy_coef"])
                                if reference_model is not None else cfg["entropy_coef"])
                loss = (actor_loss + cfg["value_coef"] * critic_loss - entropy_coef * entropy
                        + cfg.get("bc_anchor_coef", 0.0) * anchor_loss)
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 0.5)
                optimizer.step()
                policy_loss, value_loss, entropy_value = float(actor_loss.item()), float(critic_loss.item()), float(entropy.item())
            if stop_for_kl:
                break
        summary = episode_stats[-1] if episode_stats else {"episode_reward": math.nan, "episode_length": 0,
                                                          "success": 0, "collision": 0, "safety_cost": 0.0,
                                                          "minimum_clearance": math.nan, "shield_interventions": 0}
        completed_steps = min(total_timesteps, rollout_start + len(rewards))
        evaluation = {"eval_success_rate": math.nan, "eval_collision_rate": math.nan,
                      "eval_timeout_rate": math.nan, "eval_mean_clearance": math.nan,
                      "eval_mean_time": math.nan, "eval_episodes": 0, "eval_split": "selection"}
        interval = int(cfg.get("evaluation_interval_updates", 0))
        if not smoke and interval > 0 and (update_index % interval == 0 or update_index == update_total):
            evaluation = _evaluate_actor(model, env_config, selection_config)
            score = (evaluation["eval_success_rate"], -evaluation["eval_collision_rate"],
                     -evaluation["eval_timeout_rate"], evaluation["eval_mean_clearance"],
                     -evaluation["eval_mean_time"])
            if score > best_score:
                best_score, best_evaluation = score, evaluation
                _save_checkpoint(best_path, model, cfg, completed_steps, warm_start_dataset, evaluation, value_stats)
        logs.append({"total_timesteps": completed_steps, **summary,
                     "policy_loss": policy_loss, "value_loss": value_loss, "entropy": entropy_value,
                     "approx_kl": approx_kl_value, **evaluation})
        if verbose and (update_index == 1 or update_index == update_total or update_index % report_every == 0):
            print(f"PPO update {update_index}/{update_total} | steps={logs[-1]['total_timesteps']} "
                  f"| latest_reward={logs[-1]['episode_reward']:.3f}", flush=True)
    _save_checkpoint(target, model, cfg, total_timesteps, warm_start_dataset, value_stats=value_stats)
    log_path = target.with_suffix(".training.csv")
    with log_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(logs[0]))
        writer.writeheader()
        writer.writerows(logs)
    result = {"checkpoint": str(target), "training_log": str(log_path), "total_timesteps": total_timesteps,
              "updates": len(logs), "warm_start_dataset": str(warm_start_dataset) if warm_start_dataset else None,
              "warm_start_mse": warm_start_mse,
              "best_checkpoint": str(best_path) if best_path.exists() else None,
              "best_evaluation": best_evaluation,
              "selection_config": selection_config,
              "stable_architecture": stable_architecture,
              "note": "smoke run validates execution; it is not a convergence claim" if smoke else "full config run"}
    target.with_suffix(".metrics.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    env.close()
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Train the compact PyTorch PPO policy")
    parser.add_argument("--output", default=str(ROOT / "artifacts/checkpoints/ppo_smoke.pt"))
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--warm-start-dataset")
    parser.add_argument("--warm-start-only", action="store_true")
    parser.add_argument("--stable", action="store_true", help="use separate actor/critic encoders, value normalization, and KL stop")
    args = parser.parse_args()
    print(json.dumps(train(args.output, not args.full, verbose=True,
                           warm_start_dataset=args.warm_start_dataset,
                           warm_start_only=args.warm_start_only,
                           stable_architecture=args.stable), indent=2))


if __name__ == "__main__":
    main()
