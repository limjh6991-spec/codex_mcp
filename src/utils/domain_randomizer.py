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
from pathlib import Path
import json
import yaml
import random

@dataclass
class DomainRandomizer:
    config: Dict[str, Any]
    log_path: Optional[str] = None  # JSONL path for coverage logging (each new sample appended)
    _last_sample: Optional[Dict[str, Any]] = field(default=None, init=False, repr=False)

    @classmethod
    def from_yaml(cls, path: str) -> "DomainRandomizer":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls(config=data)

    def sample(self, force: bool = False) -> Dict[str, Any]:
        """Return (and lazily generate) a randomization sample.

        If log_path is specified and a new sample is generated (either first
        time or force=True), append it as a JSON line for coverage analysis.
        """
        regenerated = False
        if self._last_sample is None or force:
            self._last_sample = self._sample_recursive(self.config)
            regenerated = True
        if regenerated and self.log_path:
            try:
                p = Path(self.log_path)
                p.parent.mkdir(parents=True, exist_ok=True)
                with p.open("a", encoding="utf-8") as f:
                    json.dump(self._last_sample, f, ensure_ascii=False)
                    f.write("\n")
            except Exception:  # pragma: no cover - logging best-effort
                pass
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
