import numpy as np
import os
from src.utils.rollout_analysis import analyze_rollout_dir, summary_to_table, summary_to_csv

def test_rollout_analysis_basic(tmp_path):
    # create fake rollout directory
    rdir = tmp_path / "rollouts" / "2025-01-01T00-00-00"
    rdir.mkdir(parents=True)
    # meta
    (rdir / "meta.json").write_text('{"reward_terms":["tracking_term"], "reward_weights":{"tracking_term":1.0}}', encoding="utf-8")
    # episode file
    rewards = np.array([1.0, 2.0, 3.0], dtype=float)
    within_tol = np.array([0.0, 0.0, 1.0], dtype=float)
    tracking = np.array([0.1, 0.2, 0.3])
    np.savez_compressed(rdir / "ep_00001.npz", rewards=rewards, within_tol=within_tol, sum_tracking_term=np.array([ -0.6 ]))
    summary = analyze_rollout_dir(str(rdir))
    assert summary.episode_count == 1
    assert summary.success_rate == 1.0
    table = summary_to_table(summary)
    assert "ep_00001.npz" in table
    csv = summary_to_csv(summary)
    assert csv.startswith("file,steps")
