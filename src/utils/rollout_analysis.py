from __future__ import annotations
import os
import json
from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Optional
import numpy as np

@dataclass
class EpisodeStats:
    file: str
    steps: int
    total_reward: float
    avg_reward: float
    success: bool
    tracking: Optional[float] = None
    smoothness: Optional[float] = None
    action_penalty: Optional[float] = None
    goal_bonus: Optional[float] = None

@dataclass
class RolloutSummary:
    directory: str
    episode_count: int
    success_rate: float
    mean_total_reward: float
    episodes: List[EpisodeStats]
    reward_terms: List[str]
    reward_weights: Dict[str, float] | None

SUM_PREFIX = "sum_"

def _latest_subdir(root: str) -> Optional[str]:
    try:
        subs = [d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))]
    except FileNotFoundError:
        return None
    if not subs:
        return None
    return sorted(subs)[-1]

def _load_meta(path: str) -> Dict[str, Any]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}

def analyze_rollout_dir(directory: str) -> RolloutSummary:
    meta = _load_meta(os.path.join(directory, "meta.json"))
    reward_terms = meta.get("reward_terms", [])
    reward_weights = meta.get("reward_weights")
    episodes: List[EpisodeStats] = []
    for fname in sorted(os.listdir(directory)):
        if not fname.endswith(".npz"):
            continue
        fpath = os.path.join(directory, fname)
        try:
            data = np.load(fpath)
        except Exception:
            continue
        rewards = data.get("rewards")
        within = data.get("within_tol")
        steps = int(rewards.shape[0]) if rewards is not None else 0
        total_reward = float(rewards.sum()) if rewards is not None else 0.0
        avg_reward = float(total_reward / steps) if steps > 0 else 0.0
        success = False
        if within is not None:
            # success if any within_tol > 0 (allowing multi-step) or last step >0
            arr = np.array(within)
            success = bool((arr > 0).any())
        # Extract term sums if present
        term_map = {}
        for term in ["tracking_term", "smoothness_term", "action_penalty_term", "goal_bonus_term"]:
            key = f"{SUM_PREFIX}{term}"
            if key in data:
                # Stored as 1-length array
                try:
                    term_map[term] = float(data[key][0])
                except Exception:
                    term_map[term] = float(data[key]) if data[key].shape == () else None
        episodes.append(EpisodeStats(
            file=fname,
            steps=steps,
            total_reward=total_reward,
            avg_reward=avg_reward,
            success=success,
            tracking=term_map.get("tracking_term"),
            smoothness=term_map.get("smoothness_term"),
            action_penalty=term_map.get("action_penalty_term"),
            goal_bonus=term_map.get("goal_bonus_term"),
        ))
    if not episodes:
        return RolloutSummary(directory=directory, episode_count=0, success_rate=0.0, mean_total_reward=0.0, episodes=[], reward_terms=reward_terms, reward_weights=reward_weights)
    success_rate = sum(1 for e in episodes if e.success) / len(episodes)
    mean_total = sum(e.total_reward for e in episodes) / len(episodes)
    return RolloutSummary(
        directory=directory,
        episode_count=len(episodes),
        success_rate=success_rate,
        mean_total_reward=mean_total,
        episodes=episodes,
        reward_terms=reward_terms,
        reward_weights=reward_weights,
    )

def analyze_root(root: str, subdir: str | None = None) -> RolloutSummary:
    target = subdir or _latest_subdir(root)
    if target is None:
        return RolloutSummary(directory="", episode_count=0, success_rate=0.0, mean_total_reward=0.0, episodes=[], reward_terms=[], reward_weights=None)
    full = os.path.join(root, target)
    return analyze_rollout_dir(full)

def summary_to_table(summary: RolloutSummary) -> str:
    if summary.episode_count == 0:
        return "No episodes found."
    headers = ["file", "steps", "total_reward", "avg_reward", "success", "tracking", "smoothness", "action_penalty", "goal_bonus"]
    lines = ["\t".join(headers)]
    for ep in summary.episodes:
        lines.append("\t".join([
            ep.file,
            str(ep.steps),
            f"{ep.total_reward:.3f}",
            f"{ep.avg_reward:.3f}",
            "1" if ep.success else "0",
            f"{ep.tracking:.3f}" if ep.tracking is not None else "",
            f"{ep.smoothness:.3f}" if ep.smoothness is not None else "",
            f"{ep.action_penalty:.3f}" if ep.action_penalty is not None else "",
            f"{ep.goal_bonus:.3f}" if ep.goal_bonus is not None else "",
        ]))
    lines.append("")
    lines.append(f"Episodes: {summary.episode_count}  SuccessRate: {summary.success_rate:.2%}  MeanTotalReward: {summary.mean_total_reward:.3f}")
    return "\n".join(lines)

def summary_to_csv(summary: RolloutSummary) -> str:
    if summary.episode_count == 0:
        return "file,steps,total_reward,avg_reward,success,tracking,smoothness,action_penalty,goal_bonus"\
"\n"
    rows = ["file,steps,total_reward,avg_reward,success,tracking,smoothness,action_penalty,goal_bonus"]
    for ep in summary.episodes:
        rows.append(
            ",".join([
                ep.file,
                str(ep.steps),
                f"{ep.total_reward:.6f}",
                f"{ep.avg_reward:.6f}",
                "1" if ep.success else "0",
                f"{ep.tracking:.6f}" if ep.tracking is not None else "",
                f"{ep.smoothness:.6f}" if ep.smoothness is not None else "",
                f"{ep.action_penalty:.6f}" if ep.action_penalty is not None else "",
                f"{ep.goal_bonus:.6f}" if ep.goal_bonus is not None else "",
            ])
        )
    return "\n".join(rows) + "\n"

def summary_to_markdown(summary: RolloutSummary) -> str:
    if summary.episode_count == 0:
        return "### Rollout Summary\n\n_No episodes found_\n"
    lines = ["### Rollout Summary", "", f"Directory: `{summary.directory}`", "", "| file | steps | total_reward | avg_reward | success | tracking | smoothness | action_penalty | goal_bonus |", "|------|-------|--------------|-----------|---------|----------|-----------|---------------|-----------|"]
    for ep in summary.episodes:
        lines.append(
            f"| {ep.file} | {ep.steps} | {ep.total_reward:.2f} | {ep.avg_reward:.2f} | {'✅' if ep.success else ''} | "
            f"{ep.tracking:.2f if ep.tracking is not None else ''} | {ep.smoothness:.2f if ep.smoothness is not None else ''} | "
            f"{ep.action_penalty:.2f if ep.action_penalty is not None else ''} | {ep.goal_bonus:.2f if ep.goal_bonus is not None else ''} |"
        )
    lines.append("")
    lines.append(f"**Episodes:** {summary.episode_count}  **SuccessRate:** {summary.success_rate:.2%}  **MeanTotalReward:** {summary.mean_total_reward:.2f}")
    return "\n".join(lines)
