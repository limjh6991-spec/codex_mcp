"""Joint limit & scaling utilities.

Provides:
  - load_joint_limits(yaml_path)
  - clamp_to_limits(q, lower, upper)
  - apply_delta_with_limits(q, delta, lower, upper)
  - normalize(q, lower, upper) and denormalize(qn, lower, upper)
"""
from __future__ import annotations
from typing import Tuple, Dict, List
import yaml
import numpy as np

def load_joint_limits(path: str, joint_order: List[str]) -> Tuple[np.ndarray, np.ndarray]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    limits = data.get("limits", {})
    lowers = []
    uppers = []
    for j in joint_order:
        rng = limits.get(j, [-1.0, 1.0])
        lo, hi = float(rng[0]), float(rng[1])
        if hi <= lo:
            hi = lo + 1.0  # fallback widen
        lowers.append(lo)
        uppers.append(hi)
    return np.array(lowers, dtype=float), np.array(uppers, dtype=float)

def clamp_to_limits(q: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(q, lower), upper)

def apply_delta_with_limits(q: np.ndarray, delta: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    target = q + delta
    return clamp_to_limits(target, lower, upper)

def normalize(q: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    span = upper - lower
    return 2.0 * (q - lower) / span - 1.0

def denormalize(qn: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    span = upper - lower
    return (qn + 1.0) * 0.5 * span + lower
