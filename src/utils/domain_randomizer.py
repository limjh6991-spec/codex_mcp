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
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
import yaml
import random

@dataclass
class DomainRandomizer:
    config: Dict[str, Any]
    _last_sample: Optional[Dict[str, Any]] = field(default=None, init=False, repr=False)

    @classmethod
    def from_yaml(cls, path: str) -> "DomainRandomizer":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls(config=data)

    def sample(self, force: bool = False) -> Dict[str, Any]:
        if self._last_sample is None or force:
            self._last_sample = self._sample_recursive(self.config)
        return self._last_sample

    def last_sample(self) -> Optional[Dict[str, Any]]:
        return self._last_sample

    def _sample_recursive(self, node):
        if isinstance(node, dict):
            return {k: self._sample_recursive(v) for k, v in node.items()}
        if isinstance(node, list) and len(node) == 2 and all(isinstance(x, (int, float)) for x in node):
            lo, hi = node
            return random.uniform(lo, hi)
        return node

    def apply(self, env, force: bool = False):
        sampled = self.sample(force=force)
        setattr(env, "randomization", sampled)
        return sampled
