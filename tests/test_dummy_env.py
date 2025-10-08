from src.envs.base_env import DummyRoArmM3Env
import numpy as np

def test_reset_step_cycle():
    env = DummyRoArmM3Env()
    obs, info = env.reset(seed=42)
    assert obs.shape[0] == env.OBS_DIM
    done = False
    steps = 0
    while not done and steps < 10:
        action = np.zeros(env.ACTION_DIM)
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        steps += 1
    assert steps > 0
