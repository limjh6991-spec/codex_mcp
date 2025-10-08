"""Utility to load RewardConfig from YAML/Dict."""
from __future__ import annotations
from typing import Any, Dict
import yaml
from .reward import RewardConfig


def load_reward_config(path: str | None = None, override: Dict[str, Any] | None = None) -> RewardConfig:
    data: Dict[str, Any] = {}
    if path:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    if override:
        data.update({k: v for k, v in override.items() if v is not None})
    return RewardConfig(**{k: data[k] for k in data if k in RewardConfig().__dict__.keys()})