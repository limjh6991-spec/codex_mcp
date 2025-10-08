"""Reward utilities for RoArm M3 tasks.

Phase B: Initial shaping terms (placeholder values until real articulation states available).

Design:
    - tracking_error: L2 distance between goal joint positions and current positions.
    - smoothness_penalty: L2 norm of joint velocity vector.
    - goal_tolerance_bonus: Small positive reward when within tolerance.

All terms returned so caller can log/decompose.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Protocol, Callable, Any
import numpy as np

@dataclass
class RewardConfig:
    tracking_weight: float = 1.0
    smoothness_weight: float = 0.01
    action_penalty_weight: float = 0.0
    goal_tolerance: float = 0.02
    goal_bonus: float = 0.5

class RewardTerm(Protocol):
    def __call__(self, q: np.ndarray, dq: np.ndarray, goal: np.ndarray, action: np.ndarray | None, cfg: RewardConfig) -> Dict[str, float]:...

def term_tracking(q: np.ndarray, dq: np.ndarray, goal: np.ndarray, action: np.ndarray | None, cfg: RewardConfig) -> Dict[str, float]:
    err_vec = goal - q
    tracking_error = float(np.linalg.norm(err_vec))
    value = -cfg.tracking_weight * tracking_error
    return {"tracking_error": tracking_error, "tracking_term": value}

def term_smoothness(q: np.ndarray, dq: np.ndarray, goal: np.ndarray, action: np.ndarray | None, cfg: RewardConfig) -> Dict[str, float]:
    smoothness = float(np.linalg.norm(dq))
    value = -cfg.smoothness_weight * smoothness
    return {"smoothness": smoothness, "smoothness_term": value}

def term_action_penalty(q: np.ndarray, dq: np.ndarray, goal: np.ndarray, action: np.ndarray | None, cfg: RewardConfig) -> Dict[str, float]:
    if action is None or cfg.action_penalty_weight <= 0.0:
        return {"action_penalty_term": 0.0}
    act_mag = float(np.linalg.norm(action))
    value = -cfg.action_penalty_weight * act_mag
    return {"action_magnitude": act_mag, "action_penalty_term": value}

def term_goal_bonus(q: np.ndarray, dq: np.ndarray, goal: np.ndarray, action: np.ndarray | None, cfg: RewardConfig) -> Dict[str, float]:
    tracking_error = float(np.linalg.norm(goal - q))
    within = tracking_error < cfg.goal_tolerance
    return {"within_tol": float(within), "goal_bonus_term": cfg.goal_bonus if within else 0.0}

class RewardComposer:
    def __init__(self, cfg: RewardConfig | None = None, terms: List[RewardTerm] | None = None):
        self.cfg = cfg or RewardConfig()
        self.terms = terms or [term_tracking, term_smoothness, term_action_penalty, term_goal_bonus]
        self._term_keys_cache: List[str] | None = None

    def compute(self, q: np.ndarray, dq: np.ndarray, goal: np.ndarray, action: np.ndarray | None = None) -> Dict[str, float]:
        out: Dict[str, float] = {}
        total = 0.0
        for t in self.terms:
            comp = t(q, dq, goal, action, self.cfg)
            out.update(comp)
        # Sum all *_term keys
        for k, v in out.items():
            if k.endswith("_term"):
                total += v
        out["reward"] = float(total)
        # Cache term key order on first compute
        if self._term_keys_cache is None:
            self._term_keys_cache = [k for k in out.keys() if k.endswith("_term")]
        return out

    # --- Introspection Helpers ---
    def get_term_keys(self) -> List[str]:
        if self._term_keys_cache is not None:
            return list(self._term_keys_cache)
        # Fallback: generate using zero dummy arrays (length heuristic: assume 1)
        dummy = np.zeros(1, dtype=float)
        _ = self.compute(dummy, dummy, dummy, action=None)
        return list(self._term_keys_cache or [])

    def get_weight_map(self) -> Dict[str, float]:
        return {
            "tracking_term": self.cfg.tracking_weight,
            "smoothness_term": self.cfg.smoothness_weight,
            "action_penalty_term": self.cfg.action_penalty_weight,
            "goal_bonus_term": self.cfg.goal_bonus,
        }

def compute_reward(q: np.ndarray, dq: np.ndarray, goal: np.ndarray, cfg: RewardConfig | None = None) -> Dict[str, float]:  # backward compat (no action)
    composer = RewardComposer(cfg)
    return composer.compute(q, dq, goal, action=None)
