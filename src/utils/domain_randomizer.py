from __future__ import annotations
"""Domain randomization loader & applier.

YAML 스키마 (예시):
physics:
  dt_jitter: [0.0, 0.0005]
robot:
  joint_friction_scale: [0.8, 1.2]

단순화 구현: 범위(list 길이 2) → uniform 샘플, scalar → 그대로.
환경 객체(env)에 속성 dict로 `env.randomization` 주입.
"""
from dataclasses import dataclass
from typing import Any, Dict
import yaml
import random

@dataclass
class DomainRandomizer:
    config: Dict[str, Any]

    @classmethod
    def from_yaml(cls, path: str) -> "DomainRandomizer":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls(config=data)

    def sample(self) -> Dict[str, Any]:
        return self._sample_recursive(self.config)

    def _sample_recursive(self, node):
        if isinstance(node, dict):
            return {k: self._sample_recursive(v) for k, v in node.items()}
        if isinstance(node, list) and len(node) == 2 and all(isinstance(x, (int, float)) for x in node):
            lo, hi = node
            return random.uniform(lo, hi)
        return node

    def apply(self, env):
        sampled = self.sample()
        setattr(env, "randomization", sampled)
        return sampled
