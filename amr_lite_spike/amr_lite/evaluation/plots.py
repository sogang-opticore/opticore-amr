from __future__ import annotations

import csv
import os
from pathlib import Path

from amr_lite.config import ROOT


def make_plots(summary_csv: str | Path, output_dir: str | Path) -> list[Path]:
    cache = ROOT / "artifacts/.matplotlib"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache))
    import matplotlib.pyplot as plt

    with Path(summary_csv).open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    labels = [f"{row['policy']}\nshield={row['shield_enabled']}" for row in rows]
    plots = []
    for metric, filename, ylabel in (("success_rate", "success_rate.png", "Success rate"),
                                      ("collision_rate", "collision_rate.png", "Collision rate"),
                                      ("mean_episode_time", "episode_time.png", "Mean episode time (s)")):
        fig, ax = plt.subplots(figsize=(max(6, len(rows) * 1.5), 4))
        ax.bar(labels, [float(row[metric]) for row in rows], color="#3b82f6")
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.25)
        fig.tight_layout()
        target = output / filename
        fig.savefig(target, dpi=140)
        plt.close(fig)
        plots.append(target)
    return plots


def plot_training(training_csv: str | Path, output: str | Path) -> Path:
    cache = ROOT / "artifacts/.matplotlib"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache))
    import matplotlib.pyplot as plt

    with Path(training_csv).open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot([int(row["total_timesteps"]) for row in rows], [float(row["episode_reward"]) for row in rows], marker="o")
    run_name = "full" if "full" in Path(training_csv).name else "smoke"
    ax.set(xlabel="Total timesteps", ylabel="Latest completed episode reward", title=f"PPO training {run_name} curve")
    ax.grid(alpha=0.25)
    target = Path(output)
    target.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(target, dpi=140)
    plt.close(fig)
    return target
