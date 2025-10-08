"""Rollout logging utility.

Design goals:
  * Minimal overhead
  * Episode-wise NPZ files + one meta.json
  * Append-safe (creates timestamped directory once)
"""
from __future__ import annotations
import os
import json
from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Optional
import numpy as np
import time
import hashlib
import subprocess


def _git_commit() -> str:
    try:
        out = subprocess.check_output(["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL, timeout=2)
        return out.decode().strip()[:10]
    except Exception:
        return "unknown"


def _timestamp_dir(root: str) -> str:
    ts = time.strftime("%Y-%m-%dT%H-%M-%S")
    path = os.path.join(root, ts)
    os.makedirs(path, exist_ok=True)
    return path

@dataclass
class RolloutMeta:
    env: Dict[str, Any]
    reward_terms: List[str]
    git: str
    created: str
    randomization: Optional[Dict[str, Any]] = None

class RolloutLogger:
    def __init__(self, root: str = "rollouts", env_info: Dict[str, Any] | None = None, reward_terms: List[str] | None = None, randomization: Dict[str, Any] | None = None):
        self.root = root
        self.dir = _timestamp_dir(root)
        self.episodes = 0
        meta = RolloutMeta(
            env=env_info or {},
            reward_terms=reward_terms or [],
            git=_git_commit(),
            created=time.strftime("%Y-%m-%dT%H:%M:%S"),
            randomization=randomization,
        )
        with open(os.path.join(self.dir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(asdict(meta), f, ensure_ascii=False, indent=2)
        self._reset_buffers()

    def _reset_buffers(self):
        self._obs: List[np.ndarray] = []
        self._actions: List[np.ndarray] = []
        self._rewards: List[float] = []
        self._dones: List[bool] = []
        self._info_store: Dict[str, List[float]] = {}

    def begin_episode(self, first_obs: np.ndarray):
        self._reset_buffers()
        self._obs.append(first_obs.copy())

    def record_step(self, obs: np.ndarray, action: np.ndarray, reward: float, done: bool, info: Dict[str, Any]):
        self._actions.append(action.copy())
        self._rewards.append(float(reward))
        self._dones.append(bool(done))
        # Store selective info numeric values
        for k, v in info.items():
            if isinstance(v, (int, float)):
                self._info_store.setdefault(k, []).append(float(v))
        self._obs.append(obs.copy())

    def end_episode(self):
        ep_idx = self.episodes + 1
        ep_name = f"ep_{ep_idx:05d}.npz"
        arr_info = {k: np.array(v, dtype=float) for k, v in self._info_store.items()}
        np.savez_compressed(
            os.path.join(self.dir, ep_name),
            obs=np.stack(self._obs, axis=0),
            actions=np.stack(self._actions, axis=0) if self._actions else np.zeros((0,)),
            rewards=np.array(self._rewards, dtype=float),
            dones=np.array(self._dones, dtype=bool),
            **arr_info,
        )
        self.episodes += 1
        self._reset_buffers()
        return ep_name

