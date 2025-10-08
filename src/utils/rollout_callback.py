from __future__ import annotations
from typing import Any, Dict, List, Optional
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
from .rollout_logger import RolloutLogger

class RolloutCallback(BaseCallback):
    """Episode-level rollout saver using RolloutLogger.

    Notes:
      * Assumes single env (VecEnv length 1). For multi-env, extend to aggregate.
      * Collects env randomization & physics_report only once at training start.
    """
    def __init__(self, reward_terms: Optional[List[str]] = None, verbose: int = 0):
        super().__init__(verbose)
        self.reward_terms = reward_terms or []
        self.logger_obj: Optional[RolloutLogger] = None
        self._episode_start_obs: Optional[np.ndarray] = None

    def _on_training_start(self) -> None:
        env0 = self.training_env.envs[0]
        inner_env = getattr(env0, 'env', env0)  # unwrap GymAdapter if present
        rand = getattr(inner_env, 'randomization', None)
        physics_report = getattr(inner_env, 'physics_report', None)
        env_info = {
            'obs_dim': getattr(inner_env, 'OBS_DIM', None),
            'action_dim': getattr(inner_env, 'ACTION_DIM', None),
            'env_class': inner_env.__class__.__name__,
        }
        self.logger_obj = RolloutLogger(env_info=env_info, reward_terms=self.reward_terms, randomization=rand, physics_report=physics_report)
        # capture initial obs
        obs = self.training_env.reset()
        self._episode_start_obs = obs[0] if isinstance(obs, (list, tuple)) else obs
        self.logger_obj.begin_episode(self._episode_start_obs)

    def _on_step(self) -> bool:
        # Single env assumption
        if self.logger_obj is None:
            return True
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [])
        rewards = self.locals.get('rewards', [])
        actions = self.locals.get('actions', None)
        new_obs = self.locals.get('new_obs', None)
        if new_obs is None or actions is None:
            return True
        # shape normalize
        act_arr = actions[0] if isinstance(actions, (list, tuple)) else actions
        rew = float(rewards[0]) if isinstance(rewards, (list, tuple)) else float(rewards)
        done = bool(dones[0]) if isinstance(dones, (list, tuple)) else bool(dones)
        info = infos[0] if isinstance(infos, (list, tuple)) else (infos or {})
        self.logger_obj.record_step(new_obs[0] if isinstance(new_obs, (list, tuple)) else new_obs, act_arr, rew, done, info)
        if done:
            self.logger_obj.end_episode()
            # next episode start obs already in new_obs (post-step state)
            self.logger_obj.begin_episode(new_obs[0] if isinstance(new_obs, (list, tuple)) else new_obs)
        return True

    def _on_training_end(self) -> None:
        # ensure graceful closure: if an episode in progress with steps recorded but not ended
        # we leave it since partial episode may not represent full trajectory
        pass
