from __future__ import annotations

import numpy as np
import torch
from torch import nn


class ActorMLP(nn.Module):
    def __init__(self, input_size: int = 296, hidden_size: int = 256):
        super().__init__()
        self.model = nn.Sequential(nn.Linear(input_size, hidden_size), nn.ReLU(),
                                   nn.Linear(hidden_size, hidden_size), nn.ReLU(),
                                   nn.Linear(hidden_size, 2), nn.Tanh())

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        return self.model(value)


class ActorCritic(nn.Module):
    checkpoint_kind = "ppo"

    def __init__(self, input_size: int = 296, hidden_size: int = 128):
        super().__init__()
        self.body = nn.Sequential(nn.Linear(input_size, hidden_size), nn.Tanh(),
                                  nn.Linear(hidden_size, hidden_size), nn.Tanh())
        self.actor_mean = nn.Linear(hidden_size, 2)
        self.critic = nn.Linear(hidden_size, 1)
        self.log_std = nn.Parameter(torch.full((2,), -0.5))

    def forward(self, observation: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        hidden = self.body(observation)
        return self.actor_mean(hidden), self.log_std.expand_as(self.actor_mean(hidden)), self.critic(hidden).squeeze(-1)


class StableActorCritic(nn.Module):
    """PPO network with independent actor and critic representations."""

    checkpoint_kind = "ppo_stable"

    def __init__(self, input_size: int = 296, hidden_size: int = 128):
        super().__init__()
        self.actor_body = nn.Sequential(nn.Linear(input_size, hidden_size), nn.Tanh(),
                                        nn.Linear(hidden_size, hidden_size), nn.Tanh())
        self.critic_body = nn.Sequential(nn.Linear(input_size, hidden_size), nn.Tanh(),
                                         nn.Linear(hidden_size, hidden_size), nn.Tanh())
        self.actor_mean = nn.Linear(hidden_size, 2)
        self.critic = nn.Linear(hidden_size, 1)
        self.log_std = nn.Parameter(torch.full((2,), -0.5))

    def forward(self, observation: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        actor_hidden = self.actor_body(observation)
        critic_hidden = self.critic_body(observation)
        mean = self.actor_mean(actor_hidden)
        return mean, self.log_std.expand_as(mean), self.critic(critic_hidden).squeeze(-1)


class TorchPolicy:
    def __init__(self, model: nn.Module, kind: str):
        self.model, self.kind = model.eval(), kind
        self.name = kind

    @torch.no_grad()
    def predict(self, observation: np.ndarray, deterministic: bool = True) -> np.ndarray:
        tensor = torch.as_tensor(observation, dtype=torch.float32).unsqueeze(0)
        if self.kind == "bc":
            action = self.model(tensor)
        else:
            mean, log_std, _ = self.model(tensor)
            latent = mean if deterministic else mean + torch.randn_like(mean) * log_std.exp()
            action = torch.tanh(latent)
        return action.squeeze(0).cpu().numpy().astype(np.float32)


def load_policy(path: str) -> TorchPolicy:
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    kind = checkpoint["kind"]
    hidden = int(checkpoint["hidden_size"])
    if kind == "bc":
        model: nn.Module = ActorMLP(hidden_size=hidden)
    elif kind == "ppo_stable":
        model = StableActorCritic(hidden_size=hidden)
    else:
        model = ActorCritic(hidden_size=hidden)
    model.load_state_dict(checkpoint["model_state"])
    return TorchPolicy(model, kind)
