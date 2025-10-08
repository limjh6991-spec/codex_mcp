from src.utils.rollout_logger import RolloutLogger
import numpy as np
import os

def test_rollout_logger_basic(tmp_path):
    root = tmp_path / "rollouts"
    rl = RolloutLogger(root=str(root), env_info={"obs_dim":3, "action_dim":2}, reward_terms=["tracking"])
    first_obs = np.zeros(3)
    rl.begin_episode(first_obs)
    for _ in range(3):
        obs = np.random.randn(3)
        action = np.random.randn(2)
        rl.record_step(obs, action, reward=1.0, done=False, info={"tracking_error":0.1})
    rl.end_episode()
    # verify files
    files = list((root).rglob("*.npz"))
    assert len(files) == 1
    meta = list((root).rglob("meta.json"))
    assert len(meta) == 1
