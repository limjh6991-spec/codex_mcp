from __future__ import annotations

import argparse
import copy
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import yaml
from gymnasium import Env, spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

from src.envs.base_env import BaseSim2RealEnv, DummyRoArmM3Env
from src.utils.domain_randomizer import DomainRandomizer

ENV_KINDS = ("dummy", "isaac")
DEFAULT_SCHEMA_PATH = "configs/schemas/obs_action_schema.json"
DEFAULT_STAGE_PATH = "assets/roarm_m3/usd/roarm_m3.generated.usd"
DEFAULT_JOINT_CONFIG_PATH = "configs/roarm_joints.yaml"


def resolve_repo_path(path_str: str, description: str) -> Path:
    candidate = Path(path_str).expanduser()
    if not candidate.is_absolute():
        candidate = (PROJECT_ROOT / candidate).resolve()
    if not candidate.exists():
        raise SystemExit(f"{description} not found: {candidate}")
    return candidate


def resolve_optional_path(path_str: Optional[str], description: str) -> Optional[Path]:
    if not path_str:
        return None
    return resolve_repo_path(path_str, description)


def load_domain_randomizer_config(path_str: Optional[str]) -> Optional[dict]:
    if not path_str:
        return None
    resolved = resolve_repo_path(path_str, "Domain randomizer config")
    with resolved.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data or {}


def build_randomizer_factory(config: Optional[dict]) -> Callable[[], Optional[DomainRandomizer]]:
    if config is None:
        return lambda: None

    def _factory() -> DomainRandomizer:
        return DomainRandomizer(config=copy.deepcopy(config))

    return _factory


def load_joint_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "joints" not in data or not data["joints"]:
        raise SystemExit(f"Joint config at {path} must define a non-empty 'joints' list")
    data.setdefault("config_path", str(path))
    return data


def validate_schema_alignment(env_cls: type, joint_config: dict, schema_path: Path) -> None:
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    joint_names = joint_config.get("joints", [])
    action_dim = getattr(env_cls, "ACTION_DIM", None)
    obs_dim = getattr(env_cls, "OBS_DIM", None)
    action_range = getattr(env_cls, "ACTION_RANGE", None)

    if action_dim is None or obs_dim is None or action_range is None:
        raise SystemExit("Environment class must expose ACTION_DIM, OBS_DIM, ACTION_RANGE for schema validation")

    if len(joint_names) != action_dim:
        raise SystemExit(
            f"Joint count mismatch: config lists {len(joint_names)} joints but env expects ACTION_DIM={action_dim}"
        )

    expected_obs_dim = action_dim * 3  # q + dq + goal_delta
    if obs_dim != expected_obs_dim:
        raise SystemExit(
            f"Observation dimension mismatch: env OBS_DIM={obs_dim} but expected {expected_obs_dim} for schema"
        )

    low, high = action_range
    if not np.isclose(abs(low), abs(high), atol=1e-6):
        raise SystemExit(f"Action range must be symmetric, got ({low}, {high})")

    action_scale = float(max(abs(low), abs(high)))
    schema_version = schema.get("schema_version", "?")
    print(
        f"[schema-check] schema_version={schema_version}, joints={len(joint_names)}, "
        f"obs_dim={obs_dim}, action_scale≈{action_scale:.3f}"
    )


@dataclass
class TrainConfig:
    total_timesteps: int = 10_000
    seed: int = 0
    policy: str = "MlpPolicy"
    log_interval: int = 10
    device: str = "auto"


class GymAdapter(Env):
    """간단 Gymnasium 호환 어댑터 (VecEnv 래핑 위해)."""

    metadata = {"render_modes": ["human"], "render.modes": ["human"]}

    def __init__(self, env: BaseSim2RealEnv):
        super().__init__()
        self.env = env
        obs_low = np.full((env.OBS_DIM,), -np.inf, dtype=np.float32)
        obs_high = np.full((env.OBS_DIM,), np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)
        action_low, action_high = getattr(env, "ACTION_RANGE", (-0.1, 0.1))
        low_vec = np.full((env.ACTION_DIM,), action_low, dtype=np.float32)
        high_vec = np.full((env.ACTION_DIM,), action_high, dtype=np.float32)
        self.action_space = spaces.Box(low=low_vec, high=high_vec, dtype=np.float32)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if seed is not None:
            self.seed(seed)
        obs, info = self.env.reset()
        return obs.astype(np.float32), dict(info)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        return (
            obs.astype(np.float32),
            float(reward),
            bool(terminated),
            bool(truncated),
            dict(info),
        )

    def render(self):
        return self.env.render("human")

    def seed(self, seed: int):
        self.env.seed(seed)

    def close(self):
        self.env.close()


def make_env(factory: Callable[[], BaseSim2RealEnv], seed: int):
    def _thunk():
        env = factory()
        adapter = GymAdapter(env)
        adapter.seed(seed)
        return adapter

    return _thunk


def create_env_factory(args) -> Tuple[Callable[[], BaseSim2RealEnv], dict]:
    domain_config = load_domain_randomizer_config(args.domain_randomizer)
    rand_factory = build_randomizer_factory(domain_config)
    metadata: dict = {"env_kind": args.env_kind}

    if args.env_kind == "dummy":
        def factory() -> BaseSim2RealEnv:
            dr = rand_factory()
            return DummyRoArmM3Env(domain_randomizer=dr if dr is not None else args.domain_randomizer)

        return factory, metadata

    try:
        from src.envs.isaac_roarm_env import ISAAC_AVAILABLE, IsaacRoArmM3Env
    except ImportError as exc:  # pragma: no cover - runtime guard
        raise SystemExit(f"Isaac environment import failed: {exc}") from exc

    if not ISAAC_AVAILABLE:
        raise SystemExit("Isaac environment requested but ISAAC_AVAILABLE=False. Activate Isaac Sim venv.")

    stage_path = resolve_repo_path(args.isaac_stage, "Isaac stage USD")
    joint_config_path = resolve_repo_path(args.isaac_joint_config, "Isaac joint config")
    schema_path = resolve_repo_path(args.obs_action_schema, "Obs/Action schema")
    joint_config = load_joint_config(joint_config_path)
    validate_schema_alignment(IsaacRoArmM3Env, joint_config, schema_path)
    robot_usd_path = resolve_optional_path(args.isaac_robot_usd, "Robot USD reference")

    metadata.update(
        {
            "stage_path": str(stage_path),
            "joint_config": str(joint_config_path),
            "schema_path": str(schema_path),
            "robot_usd_path": str(robot_usd_path) if robot_usd_path else None,
        }
    )

    def factory() -> BaseSim2RealEnv:
        dr = rand_factory()
        cfg = dict(joint_config)
        return IsaacRoArmM3Env(
            stage_path=str(stage_path),
            joint_config=cfg,
            domain_randomizer=dr,
            headless=args.isaac_headless,
            articulation_prim=args.isaac_articulation_prim,
            robot_usd_path=str(robot_usd_path) if robot_usd_path else None,
            warmup_steps=args.isaac_warmup_steps,
            attach_retries=args.isaac_attach_retries,
        )

    return factory, metadata


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--total_timesteps", type=int, default=10_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--env-kind", choices=ENV_KINDS, default="dummy", help="Select training backend (dummy or isaac)")
    parser.add_argument("--domain-randomizer", type=str, default=None, help="Domain randomization YAML path")
    parser.add_argument("--obs-action-schema", type=str, default=DEFAULT_SCHEMA_PATH, help="Observation/action schema for validation")
    parser.add_argument("--isaac-stage", type=str, default=DEFAULT_STAGE_PATH, help="Isaac USD stage path")
    parser.add_argument("--isaac-joint-config", type=str, default=DEFAULT_JOINT_CONFIG_PATH, help="Isaac joint config YAML")
    parser.add_argument("--isaac-robot-usd", type=str, default=None, help="Optional robot USD reference to add before reset")
    parser.add_argument("--isaac-articulation-prim", type=str, default=None, help="Root articulation prim path (default uses env)")
    parser.add_argument("--isaac-headless", dest="isaac_headless", action="store_true", help="Run Isaac Sim headless")
    parser.add_argument("--isaac-gui", dest="isaac_headless", action="store_false", help="Run Isaac Sim with GUI")
    parser.add_argument("--isaac-warmup-steps", type=int, default=4, help="Warm-up steps before articulation attach retry")
    parser.add_argument("--isaac-attach-retries", type=int, default=3, help="Attachment retries for Isaac articulation handle")
    parser.set_defaults(isaac_headless=True)
    parser.add_argument("--record-rollouts", action="store_true", help="Save rollout episodes (simple logger, single-env only)")
    parser.add_argument("--record-callback-rollouts", action="store_true", help="Use SB3 callback to save episodes during learning")
    parser.add_argument("--reward-config", type=str, default=None, help="Path to reward_config.yaml")
    parser.add_argument("--tracking-weight", type=float, default=None)
    parser.add_argument("--smoothness-weight", type=float, default=None)
    parser.add_argument("--action-penalty-weight", type=float, default=None)
    parser.add_argument("--goal-tolerance", type=float, default=None)
    parser.add_argument("--goal-bonus", type=float, default=None)
    parser.add_argument(
        "--device",
        type=str,
        choices=("auto", "cpu", "cuda"),
        default="auto",
        help="Computation device for Stable-Baselines3 (default: auto)",
    )
    parser.add_argument(
        "--policy-output",
        type=str,
        default="policies/ppo_roarm",
        help="Base path (without .zip) to save the trained policy",
    )
    parser.add_argument(
        "--load-policy",
        type=str,
        default=None,
        help="Optional existing policy .zip to resume training from",
    )
    args = parser.parse_args()

    cfg = TrainConfig(total_timesteps=args.total_timesteps, seed=args.seed, device=args.device)

    env_factory, env_meta = create_env_factory(args)
    vec_env = DummyVecEnv([make_env(env_factory, cfg.seed)])
    os.makedirs("policies", exist_ok=True)

    if env_meta:
        print(f"[env] configuration: {env_meta}")

    from src.utils.reward_config_loader import load_reward_config

    reward_cfg = load_reward_config(
        args.reward_config,
        override={
            "tracking_weight": args.tracking_weight,
            "smoothness_weight": args.smoothness_weight,
            "action_penalty_weight": args.action_penalty_weight,
            "goal_tolerance": args.goal_tolerance,
            "goal_bonus": args.goal_bonus,
        },
    )

    policy_output_path = Path(args.policy_output)
    if not policy_output_path.is_absolute():
        policy_output_path = (PROJECT_ROOT / policy_output_path).resolve()
    policy_output_path.parent.mkdir(parents=True, exist_ok=True)

    if args.load_policy:
        load_path = resolve_repo_path(args.load_policy, "Existing policy")
        print(f"[policy] loading from {load_path}")
        model = PPO.load(str(load_path), env=vec_env, device=cfg.device)
    else:
        model = PPO(cfg.policy, vec_env, verbose=1, device=cfg.device)

    policy_output = str(policy_output_path)

    if args.record_rollouts and args.record_callback_rollouts:
        raise SystemExit("Choose only one of --record-rollouts or --record-callback-rollouts")

    if args.record_callback_rollouts:
        from src.utils.rollout_callback import RolloutCallback

        callback = RolloutCallback(reward_terms=["tracking", "smoothness", "goal_bonus", "action_penalty"])
        model.learn(total_timesteps=cfg.total_timesteps, progress_bar=True, callback=callback)
        model.save(policy_output)
        print(f"Saved policy to {policy_output}.zip (callback rollouts)")
    elif args.record_rollouts:
        from src.utils.rollout_logger import RolloutLogger

        env_adapter = vec_env.envs[0]
        logger = RolloutLogger(
            env_info={"obs_dim": env_adapter.env.OBS_DIM, "action_dim": env_adapter.env.ACTION_DIM},
            reward_terms=["tracking", "smoothness", "goal_bonus", "action_penalty"],
            randomization=getattr(env_adapter.env, "randomization", None),
            physics_report=getattr(env_adapter.env, "_last_physics_report", None),
        )
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

        model.save(policy_output)
        print(f"Saved policy to {policy_output}.zip (rollouts logged)")
    else:
        model.learn(total_timesteps=cfg.total_timesteps, progress_bar=True)
        model.save(policy_output)
        print(f"Saved policy to {policy_output}.zip")


if __name__ == "__main__":
    main()
