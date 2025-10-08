from __future__ import annotations
import argparse
from dataclasses import dataclass
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from gymnasium import spaces

from src.envs.base_env import DummyRoArmM3Env

@dataclass
class TrainConfig:
    total_timesteps: int = 10_000
    seed: int = 0
    policy: str = "MlpPolicy"
    log_interval: int = 10


def make_env(seed: int):
    def _thunk():
        env = DummyRoArmM3Env()
        env.seed(seed)
        return GymAdapter(env)
    return _thunk

class GymAdapter:
    """간단 Gymnasium 호환 어댑터 (VecEnv 래핑 위해)."""
    def __init__(self, env: DummyRoArmM3Env):
        self.env = env
        # 관측: 단순 연속 벡터 (env.OBS_DIM)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(env.OBS_DIM,), dtype=np.float32)
        # 행동: Joint delta (-0.1, 0.1)
        self.action_space = spaces.Box(low=-0.1, high=0.1, shape=(env.ACTION_DIM,), dtype=np.float32)

    def reset(self):
        obs, _ = self.env.reset()
        return obs.astype(np.float32)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        done = terminated or truncated
        return obs.astype(np.float32), reward, done, info

    def render(self, mode="human"):
        return self.env.render(mode)

    def seed(self, seed: int):
        self.env.seed(seed)

    def close(self):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--total_timesteps", type=int, default=10_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--record-rollouts", action="store_true", help="Save rollout episodes (simple logger, single-env only)")
    args = parser.parse_args()

    cfg = TrainConfig(total_timesteps=args.total_timesteps, seed=args.seed)

    vec_env = DummyVecEnv([make_env(cfg.seed)])

    model = PPO(cfg.policy, vec_env, verbose=1)

    if args.record_rollouts:
        # Manual rollout collection (not via SB3 callback to keep simple)
        from src.utils.rollout_logger import RolloutLogger
        import os
        os.makedirs("policies", exist_ok=True)
        env_adapter = vec_env.envs[0]  # DummyVecEnv single
        logger = RolloutLogger(env_info={"obs_dim": env_adapter.env.OBS_DIM, "action_dim": env_adapter.env.ACTION_DIM}, reward_terms=["tracking", "smoothness", "goal_bonus", "action_penalty"])
        total_steps = 0
        obs = env_adapter.reset()
        logger.begin_episode(obs)
        while total_steps < cfg.total_timesteps:
            action, _ = model.predict(obs, deterministic=False)
            new_obs, reward, done, info = env_adapter.step(action)
            logger.record_step(new_obs, action, reward, done, info)
            total_steps += 1
            obs = new_obs
            if done:
                logger.end_episode()
                obs = env_adapter.reset()
                logger.begin_episode(obs)
        model.save("policies/ppo_dummy_roarm")
        print("Saved policy to policies/ppo_dummy_roarm.zip (with rollouts)")
    else:
        model.learn(total_timesteps=cfg.total_timesteps, progress_bar=True)
        model.save("policies/ppo_dummy_roarm")
        print("Saved policy to policies/ppo_dummy_roarm.zip")

if __name__ == "__main__":
    main()
