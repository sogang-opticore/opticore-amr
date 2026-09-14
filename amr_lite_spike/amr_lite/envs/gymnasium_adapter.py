from __future__ import annotations

from typing import Any

import numpy as np

from .warehouse_env import WarehouseEnv

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # The reference PPO dependency is optional until M3.
    gym = None
    spaces = None


_GymBase = gym.Env if gym is not None else object


class GymnasiumWarehouseEnv(_GymBase):
    """Gymnasium adapter used by checked third-party RL implementations."""

    metadata = WarehouseEnv.metadata

    def __init__(self, config: dict[str, Any] | None = None, scenario_id: str | None = None,
                 render_mode: str | None = None):
        if gym is None or spaces is None:
            raise RuntimeError(
                "Gymnasium is not installed. Install the optional reference-rl dependencies."
            )
        self.inner = WarehouseEnv(config, scenario_id, render_mode)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(296,), dtype=np.float32)
        self.render_mode = render_mode

    def reset(self, *, seed: int | None = None,
              options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        return self.inner.reset(seed=seed, options=options)

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        return self.inner.step(action)

    def render(self) -> np.ndarray | None:
        return self.inner.render()

    def close(self) -> None:
        self.inner.close()

