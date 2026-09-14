from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]


def deep_merge(base: dict[str, Any], overrides: dict[str, Any] | None) -> dict[str, Any]:
    """Return a recursive copy of *base* updated with *overrides*."""
    merged = deepcopy(base)
    for key, value in (overrides or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def load_config(name: str, overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    path = ROOT / "configs" / (name if name.endswith(".yaml") else f"{name}.yaml")
    with path.open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    return deep_merge(config, {key: value for key, value in (overrides or {}).items() if value is not None})
