from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class TeacherData:
    observations: np.ndarray
    actions: np.ndarray
    episode_ids: np.ndarray


def save_episode(path: str | Path, observations: list[np.ndarray], actions: list[np.ndarray],
                 metadata: dict[str, list | np.ndarray]) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    arrays = {"observation": np.asarray(observations, dtype=np.float32),
              "teacher_action": np.asarray(actions, dtype=np.float32)}
    arrays.update({key: np.asarray(value) for key, value in metadata.items()})
    np.savez_compressed(target, **arrays)
    return target


def load_dataset(directory: str | Path | list[str | Path]) -> TeacherData:
    directories = directory if isinstance(directory, list) else [directory]
    files = sorted(path for item in directories for path in Path(item).glob("*.npz"))
    if not files:
        raise FileNotFoundError(f"No dataset chunks in {directory}")
    observations, actions, episodes = [], [], []
    for path in files:
        with np.load(path) as chunk:
            observations.append(chunk["observation"])
            actions.append(chunk["teacher_action"])
            episodes.append(chunk["episode_id"])
    return TeacherData(np.concatenate(observations), np.concatenate(actions), np.concatenate(episodes))


def episode_split(data: TeacherData, validation_fraction: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
    unique = np.unique(data.episode_ids)
    rng = np.random.default_rng(seed)
    rng.shuffle(unique)
    count = max(1, int(round(len(unique) * validation_fraction))) if len(unique) > 1 else 0
    validation_episodes = set(unique[:count].tolist())
    validation = np.asarray([episode in validation_episodes for episode in data.episode_ids])
    return np.flatnonzero(~validation), np.flatnonzero(validation)
