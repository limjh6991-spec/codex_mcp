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
from typing import Dict
import numpy as np

@dataclass
class RewardConfig:
    tracking_weight: float = 1.0
    smoothness_weight: float = 0.01
    goal_tolerance: float = 0.02
    goal_bonus: float = 0.5

def compute_reward(q: np.ndarray, dq: np.ndarray, goal: np.ndarray, cfg: RewardConfig | None = None) -> Dict[str, float]:
    if cfg is None:
        cfg = RewardConfig()
    err = goal - q
    tracking_error = float(np.linalg.norm(err))
    smoothness = float(np.linalg.norm(dq))
    within_tol = tracking_error < cfg.goal_tolerance
    reward = -cfg.tracking_weight * tracking_error - cfg.smoothness_weight * smoothness
    if within_tol:
        reward += cfg.goal_bonus
    return {
        "reward": float(reward),
        "tracking_error": tracking_error,
        "smoothness": smoothness,
        "within_tol": float(within_tol),
    }
