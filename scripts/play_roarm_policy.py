#!/usr/bin/env python3
"""GUI playback runner for SB3 PPO policies in Isaac Sim."""
from __future__ import annotations

import argparse
import contextlib
import gc
import os
import random
import signal
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterable, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RANDOMIZER_PATH = REPO_ROOT / "configs" / "domain_randomization.yaml"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _disable_legacy_extensions() -> None:
    try:
        import carb  # type: ignore
    except ImportError:
        return
    settings = carb.settings.get_settings()
    for key, value in (
        ("/exts/omni.isaac.legacy/enabled", False),
        ("/exts/omni.isaac.sensor_legacy/enabled", False),
    ):
        settings.set(key, value)


def _bootstrap_simulation_app(headless: bool) -> Any:
    _disable_legacy_extensions()
    try:
        from isaacsim import SimulationApp  # type: ignore
    except ImportError:
        from isaacsim.simulation_app import SimulationApp  # type: ignore
    except ImportError:
        from omni.isaac.kit import SimulationApp  # type: ignore

    return SimulationApp({"headless": bool(headless)})


def _load_vecnormalize(env: Any, path: str | None):
    if not path:
        return env
    try:
        from stable_baselines3.common.vec_env import VecNormalize  # type: ignore
    except ImportError:
        raise RuntimeError("VecNormalize 경로가 지정되었지만 stable-baselines3.common.vec_env를 찾을 수 없습니다.")
    resolved = Path(path).expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"VecNormalize 데이터를 찾을 수 없습니다: {resolved}")
    if resolved.is_dir():
        env = VecNormalize.load(str(resolved), env)
    else:
        import pickle

        with resolved.open("rb") as fp:
            payload = pickle.load(fp)
        for attr in ("obs_rms", "ret_rms"):
            if attr in payload:
                setattr(env, attr, payload[attr])
    env.training = False
    env.norm_reward = False
    return env


def _make_fake_args(**overrides: Any) -> SimpleNamespace:
    defaults = dict(
        env_kind="isaac",
        domain_randomizer=None,
        obs_action_schema="configs/schemas/obs_action_schema.json",
        isaac_stage="assets/roarm_m3/usd/roarm_m3.generated.usd",
        isaac_joint_config="configs/roarm_joints.yaml",
        isaac_robot_usd=None,
        isaac_articulation_prim=None,
        isaac_headless=False,
        isaac_warmup_steps=4,
        isaac_attach_retries=3,
    )
    defaults.update(overrides)
    return SimpleNamespace(**defaults)


def _clamp_action(action: Iterable[float], env: Any) -> np.ndarray:
    arr = np.asarray(action, dtype=np.float32)
    space = getattr(env, "action_space", None)
    if space is not None and hasattr(space, "low") and hasattr(space, "high"):
        arr = np.clip(arr, space.low, space.high)
    safe = getattr(env, "safe_action", None)
    if callable(safe):
        arr = np.asarray(safe(arr), dtype=np.float32)
    return arr


GOLDEN_SEEDS: Tuple[int, ...] = (11, 29, 47, 83, 101)
_STOP_FLAGS = {"stop": False, "shutting_down": False}


def _signal_handler(signum, frame):  # type: ignore[override]
    del signum, frame
    if not _STOP_FLAGS["stop"]:
        print("[playback] stop requested")
    _STOP_FLAGS["stop"] = True


def _graceful_close(env: Any, sim_app: Any) -> None:
    if _STOP_FLAGS["shutting_down"]:
        return
    _STOP_FLAGS["shutting_down"] = True
    with contextlib.suppress(Exception):
        env.close()
    with contextlib.suppress(Exception):
        sim_app.close()
    gc.collect()


def _run_episode(env: Any, model: Any, *, deterministic: bool, max_steps: int, slow_ms: int) -> Tuple[float, int]:
    try:
        reset_out = env.reset()
    except TypeError:
        reset_out = env.reset()
    if isinstance(reset_out, tuple):
        obs = reset_out[0]
    else:
        obs = reset_out
    total_reward = 0.0
    steps = 0
    while not _STOP_FLAGS["stop"] and (max_steps <= 0 or steps < max_steps):
        action, _ = model.predict(obs, deterministic=deterministic)
        action = _clamp_action(action, env)
        step_out = env.step(action)
        if len(step_out) == 5:
            obs, reward, terminated, truncated, _info = step_out
            done = bool(terminated or truncated)
        else:
            obs, reward, done, _info = step_out
        total_reward += float(reward)
        steps += 1
        if done:
            break
        if slow_ms > 0:
            time.sleep(slow_ms / 1000.0)
    return total_reward, steps


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Play back PPO policy in Isaac Sim GUI")
    parser.add_argument("--policy", required=True, help="재생할 SB3 정책 .zip 경로")
    parser.add_argument("--vecnorm", help="VecNormalize 통계 경로 (디렉터리 또는 피클)")
    parser.add_argument(
        "--domain-randomizer",
        default=str(DEFAULT_RANDOMIZER_PATH) if DEFAULT_RANDOMIZER_PATH.exists() else None,
        help="도메인 랜덤화 YAML 경로 (기본: configs/domain_randomization.yaml)",
    )
    parser.add_argument("--no-domain-randomizer", action="store_true", help="도메인 랜덤화 비활성화")
    parser.add_argument("--usd", default="assets/roarm_m3/usd/roarm_m3.generated.usd", help="USD 스테이지 경로")
    parser.add_argument("--prim", default="/World/roarm_m3", help="Articulation prim 경로")
    parser.add_argument("--isaac-joint-config", default="configs/roarm_joints.yaml", help="조인트 설정 YAML")
    parser.add_argument("--isaac-robot-usd", help="추가 로봇 참조 USD")
    parser.add_argument("--isaac-articulation-prim", help="명시적 Articulation prim 경로")
    parser.add_argument("--isaac-warmup-steps", type=int, default=4)
    parser.add_argument("--isaac-attach-retries", type=int, default=3)
    parser.add_argument("--episodes", type=int, default=3, help="재생 에피소드 수 (<=0이면 무제한)")
    parser.add_argument("--steps", type=int, default=0, help="에피소드당 최대 스텝")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--stochastic", action="store_true", help="확률 정책으로 재생")
    parser.add_argument("--slow-ms", type=int, default=10, help="프레임당 슬립 ms")
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--use-golden-seeds", action="store_true", help="사전 정의된 시드를 순회")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    simulation_app = _bootstrap_simulation_app(headless=False)

    from src.envs import isaac_roarm_env  # type: ignore  # noqa: WPS433
    isaac_roarm_env._SIM_APP = simulation_app

    from stable_baselines3 import PPO  # type: ignore
    from training.train_ppo import DEFAULT_JOINT_CONFIG_PATH, DEFAULT_SCHEMA_PATH, DEFAULT_STAGE_PATH, create_env_factory

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    stage_path = Path(args.usd or DEFAULT_STAGE_PATH).expanduser().resolve()
    joint_path = Path(args.isaac_joint_config or DEFAULT_JOINT_CONFIG_PATH).expanduser().resolve()
    schema_path = Path(DEFAULT_SCHEMA_PATH).expanduser().resolve()
    robot_usd = Path(args.isaac_robot_usd).expanduser().resolve() if args.isaac_robot_usd else None

    domain_randomizer_path: str | None = None
    if not args.no_domain_randomizer and args.domain_randomizer:
        resolved_randomizer = Path(args.domain_randomizer).expanduser().resolve()
        if not resolved_randomizer.exists():
            raise FileNotFoundError(f"도메인 랜덤화 파일을 찾을 수 없습니다: {resolved_randomizer}")
        domain_randomizer_path = str(resolved_randomizer)

    fake = _make_fake_args(
        isaac_headless=False,
        isaac_stage=str(stage_path),
        isaac_joint_config=str(joint_path),
        isaac_robot_usd=str(robot_usd) if robot_usd else None,
        isaac_articulation_prim=args.isaac_articulation_prim,
        isaac_warmup_steps=max(0, args.isaac_warmup_steps),
        isaac_attach_retries=max(1, args.isaac_attach_retries),
        obs_action_schema=str(schema_path),
        domain_randomizer=domain_randomizer_path,
    )

    env_factory, _meta = create_env_factory(fake)
    env = env_factory()

    env = _load_vecnormalize(env, args.vecnorm)

    policy_path = Path(args.policy).expanduser().resolve()
    if not policy_path.exists():
        raise FileNotFoundError(f"정책 파일을 찾을 수 없습니다: {policy_path}")

    model = PPO.load(str(policy_path), device=args.device)

    episodes_target = args.episodes if args.episodes > 0 else float("inf")
    episode_counter = 0
    seeds = GOLDEN_SEEDS if args.use_golden_seeds else (args.seed,)

    try:
        while episode_counter < episodes_target and not _STOP_FLAGS["stop"]:
            seed_idx = episode_counter % len(seeds)
            seed_value = seeds[seed_idx]
            random.seed(seed_value)
            try:
                reset_out = env.reset(seed=seed_value)
            except TypeError:
                reset_out = env.reset()
            if isinstance(reset_out, tuple):
                obs = reset_out[0]
            else:
                obs = reset_out
            with contextlib.suppress(Exception):
                env.apply_domain_randomization(force=True)
            total_reward = 0.0
            steps = 0
            done = False
            while not done and not _STOP_FLAGS["stop"]:
                action, _ = model.predict(obs, deterministic=not args.stochastic)
                action = _clamp_action(action, env)
                step_out = env.step(action)
                if len(step_out) == 5:
                    obs, reward, terminated, truncated, _info = step_out
                    done = bool(terminated or truncated)
                else:
                    obs, reward, done, _info = step_out
                total_reward += float(reward)
                steps += 1
                if args.steps > 0 and steps >= args.steps:
                    break
                if args.slow_ms > 0:
                    time.sleep(args.slow_ms / 1000.0)
            episode_counter += 1
            print(f"[playback] episode={episode_counter} seed={seed_value} steps={steps} reward={total_reward:.3f}")
    except KeyboardInterrupt:
        _STOP_FLAGS["stop"] = True
    finally:
        _graceful_close(env, simulation_app)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
