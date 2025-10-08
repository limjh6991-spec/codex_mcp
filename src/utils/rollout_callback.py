from __future__ import annotations
from typing import Any, Dict, List, Optional
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
from .rollout_logger import RolloutLogger

class RolloutCallback(BaseCallback):
        """Episode-level rollout saver supporting single or multi-env VecEnv.

        Behaviors:
            * Captures initial randomization & physics_report only from env[0] (heuristic) at start.
            * For multi-env, maintains per-env buffers and writes separate ep files with suffix _env{i}.
            * Reward terms list is metadata only (actual term logging handled externally if needed).
        """
        def __init__(self, reward_terms: Optional[List[str]] = None, verbose: int = 0):
                super().__init__(verbose)
                self.reward_terms = reward_terms or []
                self.logger_obj: Optional[RolloutLogger] = None
                # single-env legacy buffers
                self._episode_start_obs: Optional[np.ndarray] = None
                # multi-env buffers: env_id -> dict(buffers)
                self._multi_buffers: Dict[int, Dict[str, Any]] = {}
                self._is_multi = False

    def _on_training_start(self) -> None:
        n_envs = getattr(self.training_env, 'num_envs', 1)
        self._is_multi = n_envs > 1
        env0 = self.training_env.envs[0]
        inner_env0 = getattr(env0, 'env', env0)
        rand = getattr(inner_env0, 'randomization', None)
        physics_report = getattr(inner_env0, 'physics_report', None)
        env_info = {
            'obs_dim': getattr(inner_env0, 'OBS_DIM', None),
            'action_dim': getattr(inner_env0, 'ACTION_DIM', None),
            'env_class': inner_env0.__class__.__name__,
            'num_envs': n_envs,
        }
        self.logger_obj = RolloutLogger(env_info=env_info, reward_terms=self.reward_terms, randomization=rand, physics_report=physics_report)
        obs = self.training_env.reset()
        if not self._is_multi:
            self._episode_start_obs = obs[0] if isinstance(obs, (list, tuple)) else obs
            self.logger_obj.begin_episode(self._episode_start_obs)
        else:
            # initialize per-env buffers
            first_obs = obs
            for env_id in range(n_envs):
                o = first_obs[env_id]
                self._multi_buffers[env_id] = {
                    'obs': [o.copy()],
                    'actions': [],
                    'rewards': [],
                    'dones': [],
                    'info_store': {},
                    'ep_count': 0,
                }

    def _on_step(self) -> bool:
        if self.logger_obj is None:
            return True
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [])
        rewards = self.locals.get('rewards', [])
        actions = self.locals.get('actions', None)
        new_obs = self.locals.get('new_obs', None)
        if new_obs is None or actions is None:
            return True
        if not self._is_multi:
            act_arr = actions[0] if isinstance(actions, (list, tuple)) else actions
            rew = float(rewards[0]) if isinstance(rewards, (list, tuple)) else float(rewards)
            done = bool(dones[0]) if isinstance(dones, (list, tuple)) else bool(dones)
            info = infos[0] if isinstance(infos, (list, tuple)) else (infos or {})
            self.logger_obj.record_step(new_obs[0] if isinstance(new_obs, (list, tuple)) else new_obs, act_arr, rew, done, info)
            if done:
                self.logger_obj.end_episode()
                self.logger_obj.begin_episode(new_obs[0] if isinstance(new_obs, (list, tuple)) else new_obs)
            return True
        # multi-env path
        n_envs = len(dones)
        for env_id in range(n_envs):
            buf = self._multi_buffers[env_id]
            act = actions[env_id]
            rew = float(rewards[env_id])
            done = bool(dones[env_id])
            info = infos[env_id] if isinstance(infos, (list, tuple)) else infos
            # record
            buf['actions'].append(np.array(act).copy())
            buf['rewards'].append(rew)
            buf['dones'].append(done)
            for k, v in (info or {}).items():
                if isinstance(v, (int, float)):
                    buf['info_store'].setdefault(k, []).append(float(v))
            buf['obs'].append(np.array(new_obs[env_id]).copy())
            if done:
                # flush this env episode using logger_obj's end_episode mechanics
                # Temporarily swap logger buffers
                self.logger_obj._obs = buf['obs']  # type: ignore
                self.logger_obj._actions = buf['actions']  # type: ignore
                self.logger_obj._rewards = buf['rewards']  # type: ignore
                self.logger_obj._dones = buf['dones']  # type: ignore
                self.logger_obj._info_store = buf['info_store']  # type: ignore
                self.logger_obj.end_episode(env_id=env_id)
                # reset per-env buffer for next episode (start with last obs as first)
                new_first = buf['obs'][-1]
                self._multi_buffers[env_id] = {
                    'obs': [new_first.copy()],
                    'actions': [],
                    'rewards': [],
                    'dones': [],
                    'info_store': {},
                    'ep_count': buf['ep_count'] + 1,
                }
        return True

    def _on_training_end(self) -> None:
        # ensure graceful closure: if an episode in progress with steps recorded but not ended
        # we leave it since partial episode may not represent full trajectory
        pass
