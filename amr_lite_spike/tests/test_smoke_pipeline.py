from pathlib import Path

from amr_lite.evaluation import evaluate
from amr_lite.learning.collect_teacher import collect
from amr_lite.learning.dataset import load_dataset
from amr_lite.learning.networks import load_policy
from amr_lite.learning.train_bc import train as train_bc
from amr_lite.learning.train_ppo import train as train_ppo


def test_teacher_bc_ppo_and_evaluation_artifacts(tmp_path: Path):
    dataset = tmp_path / "dataset"
    assert collect(dataset, episodes=2, max_steps=12, seed=10, scenarios=["S0"])
    bc_path = tmp_path / "bc.pt"
    metrics = train_bc(dataset, bc_path, smoke=True,
                       config={"seed": 1, "hidden_size": 16, "batch_size": 8, "learning_rate": 0.001,
                               "epochs_smoke": 1, "epochs_full": 1, "validation_fraction": 0.5})
    assert bc_path.exists() and metrics["mse"] >= 0
    ppo_path = tmp_path / "ppo.pt"
    ppo = train_ppo(ppo_path, smoke=True,
                    config={"seed": 1, "hidden_size": 16, "learning_rate": 0.0003, "gamma": 0.99,
                            "gae_lambda": 0.95, "clip_ratio": 0.2, "entropy_coef": 0.005,
                            "value_coef": 0.5, "rollout_steps_smoke": 16, "total_timesteps_smoke": 32,
                            "rollout_steps_full": 16, "total_timesteps_full": 32, "epochs": 1,
                            "minibatch_size": 8})
    assert ppo_path.exists() and ppo["total_timesteps"] == 32
    stable_path = tmp_path / "ppo_stable_initial.pt"
    stable = train_ppo(
        stable_path,
        smoke=False,
        config={"seed": 1, "hidden_size": 16, "learning_rate": 0.0003,
                "minibatch_size": 8, "warm_start_learning_rate": 0.001,
                "warm_start_epochs": 1, "warm_start_log_std": -1.5},
        warm_start_dataset=dataset,
        warm_start_only=True,
        stable_architecture=True,
    )
    sample_observation = load_dataset(dataset).observations[0]
    assert stable_path.exists() and stable["total_timesteps"] == 0
    assert load_policy(str(stable_path)).predict(sample_observation).shape == (2,)
    policies = {"bc": load_policy(str(bc_path)), "ppo": load_policy(str(ppo_path))}
    results = tmp_path / "results"
    rows, summary = evaluate(policies, results,
                             config={"seed": 2, "episodes_per_scenario": 1, "scenarios": ["S0"],
                                     "max_steps": 4, "shield_modes": [True]})
    assert rows and summary
    assert (results / "episodes.csv").exists() and (results / "summary.csv").exists()
