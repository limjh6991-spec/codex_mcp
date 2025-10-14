"""IsaacRoArmM3Env - Isaac Sim 기반 실제 학습 환경 스켈레톤.

주의: Isaac Sim Python API는 런처 환경에서만 import 가능할 수 있음.
조건부 import 및 graceful degrade 패턴을 사용.
"""
from __future__ import annotations
from typing import Any, Dict, Tuple, Optional
import logging
import numpy as np
import os

from roarm_app.launch import extensions as roarm_extensions
from roarm_app.cli import _HEADLESS_DISABLE_PATTERNS

from .base_env import BaseSim2RealEnv
from src.utils.isaac_joint_api import JointAPI  # type: ignore

# Isaac 사용 가능성 감지 (신/구 API 모두 탐색)
ISAAC_AVAILABLE = False
try:  # pragma: no cover - CI나 일반 환경에서는 실패 가능
    from isaacsim.simulation_app import SimulationApp  # type: ignore

    if SimulationApp is not None:
        ISAAC_AVAILABLE = True
except Exception:  # noqa: broad-except - bootstrap import may fail
    try:
        from isaacsim import SimulationApp  # type: ignore

        if SimulationApp is not None:
            ISAAC_AVAILABLE = True
    except Exception:  # noqa
        try:
            import importlib

            if importlib.util.find_spec("isaacsim.simulation_app") is not None:
                ISAAC_AVAILABLE = True
            elif importlib.util.find_spec("omni.isaac.kit") is not None:
                ISAAC_AVAILABLE = True
        except Exception:  # noqa
            ISAAC_AVAILABLE = False

_SIM_APP = None  # Lazy SimulationApp (isaacsim.simulation_app)


def _resolve_world_api():  # pragma: no cover - requires Isaac runtime
    try:
        from isaacsim.core.api import World as _World  # type: ignore
        from isaacsim.core.utils.stage import add_reference_to_stage  # type: ignore[attr-defined]
        return _World, add_reference_to_stage
    except Exception:
        from omni.isaac.core import World as _World  # type: ignore
        from omni.isaac.core.utils.stage import add_reference_to_stage  # type: ignore
        return _World, add_reference_to_stage


def _prepare_headless_blocklist(headless: bool) -> set[str]:
    replicator_enabled = os.environ.get("ROARM_ENABLE_REPLICATOR")
    replicator_should_disable = not (
        replicator_enabled and replicator_enabled.lower() in {"1", "true", "yes", "on"}
    )

    blocklist: set[str] = set()
    if headless:
        blocklist = roarm_extensions.collect_extension_blocklist(replicator_should_disable)
        disabled_exts = set(filter(None, os.environ.get("OMNI_KIT_DISABLE_EXTENSIONS", "").split(",")))
        disabled_exts.update(blocklist)
        disabled_exts.update(_HEADLESS_DISABLE_PATTERNS)
        os.environ["OMNI_KIT_DISABLE_EXTENSIONS"] = ",".join(sorted(disabled_exts))
        os.environ.setdefault("CARB_AUDIO_DISABLED", "1")
        os.environ.setdefault("CARB_ENABLE_AUDIO", "0")
    return blocklist

def _ensure_sim_app(headless: bool = False):  # pragma: no cover
    global _SIM_APP
    if _SIM_APP is not None:
        return _SIM_APP
    blocklist = _prepare_headless_blocklist(headless)
    try:
        try:
            from isaacsim import SimulationApp  # type: ignore
        except ImportError:
            import importlib

            sim_mod = importlib.import_module("isaacsim.simulation_app")
            SimulationApp = getattr(sim_mod, "SimulationApp")
        _SIM_APP = SimulationApp({"headless": bool(headless)})
        if headless and _SIM_APP is not None:
            try:
                kit_app = getattr(_SIM_APP, "app", None)
                if kit_app is None:
                    app_getter = getattr(_SIM_APP, "get_app", None)
                    if callable(app_getter):
                        kit_app = app_getter()
                if kit_app is not None:
                    manager_getter = getattr(kit_app, "get_extension_manager", None)
                    if callable(manager_getter):
                        manager = manager_getter()
                        baseline = set(blocklist)
                        baseline.update(_HEADLESS_DISABLE_PATTERNS)
                        expanded = roarm_extensions.expand_prefix_blocklist(manager, baseline)
                        roarm_extensions.sync_extension_blocklist(manager, expanded)
            except Exception:
                pass
        return _SIM_APP
    except Exception:
        # 구 API 경로 (omni.isaac.kit) 초기화는 상위 레이어에서 담당하거나 생략
        return None

class IsaacRoArmM3Env(BaseSim2RealEnv):
    ACTION_DIM = 6  # Placeholder (실제 로봇 관절 수에 맞게 조정)
    # Observation: [q(6), dq(6), goal_delta(6)] = 18 dims (placeholder design)
    OBS_DIM = 18
    ACTION_RANGE = (-0.05, 0.05)

    def __init__(self, stage_path: str, joint_config: Dict[str, Any], domain_randomizer=None, headless: bool = False, articulation_prim: str | None = None, robot_usd_path: Optional[str] = None, warmup_steps: int = 3, attach_retries: int = 3):
        if not ISAAC_AVAILABLE:
            raise RuntimeError("Isaac Sim Python API를 사용할 수 없는 환경입니다. setup_python_env.sh 실행을 확인하세요.")
        super().__init__(domain_randomizer)
        self._stage_path = stage_path
        self._joint_config = joint_config
        # 시뮬레이션 앱 초기화 시도 (신 API 우선)
        _ensure_sim_app(headless=headless)
        # World 동적 import (초기화 이후 시도)
        try:
            World, add_ref = _resolve_world_api()
        except Exception as e:  # noqa
            raise RuntimeError(f"Isaac World import 실패: {type(e).__name__}: {e}")
        self._world = World(stage_units_in_meters=1.0)
        self._add_reference_to_stage = add_ref
        self._joint_api = JointAPI(articulation=None)
        self._articulation_prim = articulation_prim or "/World/roarm"
        self._robot_usd_path = robot_usd_path
        self._warmup_steps = warmup_steps
        self._attach_retries = max(1, attach_retries)
        self.headless = headless
        self._q = np.zeros(self.ACTION_DIM)
        self._dq = np.zeros(self.ACTION_DIM)
        self._goal = np.zeros(self.ACTION_DIM)
        self._limits_loaded = False
        self._limit_lower = np.full(self.ACTION_DIM, -1.0, dtype=float)
        self._limit_upper = np.full(self.ACTION_DIM, 1.0, dtype=float)
        self._stage_loaded = False
        self._joint_alias_map: dict[str, str] = {}
        self._joint_inverse_alias_map: dict[str, str] = {}
        self._actual_joint_names: list[str] = []
        self._logged_alias_hint = False

    # --- 내부 유틸 ---
    def _load_stage_once(self):  # pragma: no cover (Isaac 환경 외부에서는 skip)
        if self._stage_loaded or not ISAAC_AVAILABLE:
            return
        try:
            # Stage file 자체를 World가 로드해야 하는 경우 (버전별 차이 존재) - stage_path는 월드 초기화 참고용
            # 로봇 USD 참조 추가
            if self._robot_usd_path:
                try:
                    self._add_reference_to_stage(self._robot_usd_path, self._articulation_prim)
                except Exception:  # noqa
                    pass
            # World reset을 통해 scene graph 반영
            try:
                self._world.reset()
            except Exception:  # noqa
                pass
            self._stage_loaded = True
        except Exception:  # noqa
            self._stage_loaded = False

    def reset(self, *, seed: int | None = None, options: Dict[str, Any] | None = None):
        if seed is not None:
            self.seed(seed)
        # Stage 로딩 시도 (실제 Isaac 환경에서만 작동)
        self._load_stage_once()
        # Lazy load limits from joint_config if path provided
        if not self._limits_loaded and isinstance(self._joint_config, dict):
            joint_names = self._joint_config.get("joints", [])
            if len(joint_names) == self.ACTION_DIM:
                try:
                    from src.utils.limits import load_joint_limits
                    path = self._joint_config.get("config_path")
                    if path:
                        lower, upper = load_joint_limits(path, joint_names)
                        self._limit_lower[:] = lower
                        self._limit_upper[:] = upper
                        self._limits_loaded = True
                except Exception:  # noqa
                    pass
        # TODO: Stage 로드 및 로봇 초기화 + articulation 핸들 재획득
        # placeholder: articulation prim path attach (실제 handle 획득 전)
        if not self._joint_api.is_attached():
            self._joint_api.attach(self._articulation_prim, world=self._world, tries=self._attach_retries)
        # Warm-up simulation & re-attach if needed (articulation might appear after a few steps)
        if hasattr(self._joint_api, '_articulation') and self._joint_api._articulation is None:
            for _ in range(max(0, self._warmup_steps)):
                try:
                    self._world.step(render=not getattr(self, 'headless', False))
                except Exception:  # noqa
                    break
            if self._joint_api._articulation is None:
                # one more attach attempt after warm-up
                self._joint_api.attach(self._articulation_prim, world=self._world, tries=1)
        # Joint map 검증: joint_config.joints 와 실제 articulation joints 비교 (가능 시)
        try:
            cfg_names = list(self._joint_config.get("joints", [])) if isinstance(self._joint_config, dict) else []
            actual = self._joint_api.list_joints()
            self._actual_joint_names = list(actual)
            self._joint_alias_map = {}
            self._joint_inverse_alias_map = {}
            if cfg_names and actual:
                if len(cfg_names) != len(actual):
                    import json as _json
                    logging.getLogger(__name__).warning(_json.dumps({
                        "event": "joint_count_mismatch",
                        "cfg_count": len(cfg_names),
                        "actual_count": len(actual),
                    }))
                else:
                    alias_map: dict[str, str] = {}
                    inverse_map: dict[str, str] = {}
                    for cfg_name, actual_name in zip(cfg_names, actual):
                        alias_map[cfg_name] = actual_name
                        alias_map.setdefault(actual_name, actual_name)
                        inverse_map[actual_name] = cfg_name
                        inverse_map.setdefault(cfg_name, cfg_name)
                    self._joint_alias_map = alias_map
                    self._joint_inverse_alias_map = inverse_map
                    if set(cfg_names) != set(actual) and not self._logged_alias_hint:
                        import json as _json
                        logging.getLogger(__name__).info(_json.dumps({
                            "event": "joint_alias_applied",
                            "cfg_only": [n for n in cfg_names if n not in actual],
                            "actual_only": [n for n in actual if n not in cfg_names],
                        }))
                        self._logged_alias_hint = True
                if not self._joint_alias_map:
                    self._joint_alias_map = {name: name for name in cfg_names}
                    self._joint_inverse_alias_map = {name: name for name in cfg_names}
            elif actual:
                self._joint_alias_map = {name: name for name in actual}
                self._joint_inverse_alias_map = {name: name for name in actual}
        except Exception:
            pass
        if not self._actual_joint_names and isinstance(self._joint_config, dict):
            self._actual_joint_names = list(self._joint_config.get("joints", []))
        self.apply_domain_randomization(force=True)
        if not np.any(np.abs(self._goal) > 1e-4):
            lower = getattr(self, "_limit_lower", None)
            upper = getattr(self, "_limit_upper", None)
            if isinstance(lower, np.ndarray) and isinstance(upper, np.ndarray):
                span = upper - lower
                self._goal = lower + 0.3 * span
        obs = self._get_obs()
        info = {
            "stage": self._stage_path,
            "articulation_attached": bool(getattr(self._joint_api, '_articulation', None)),
            "goal": self._goal.copy(),
            "joint_names": list(self._actual_joint_names) if self._actual_joint_names else list(cfg_names),
        }
        if hasattr(self, "_last_physics_report"):
            info["physics_report"] = getattr(self, "_last_physics_report")
        self._step_count = 0
        return obs, info

    def apply_domain_randomization(self, *, force: bool = False):  # type: ignore[override]
        # Call base to apply generic (which calls DomainRandomizer.apply if present)
        super().apply_domain_randomization(force=force)
        if not self._domain_randomizer or not ISAAC_AVAILABLE:
            return
        sample = getattr(self, "randomization", None)
        if not isinstance(sample, dict):
            sample = self._domain_randomizer.last_sample() or {}
        try:
            if sample:
                from src.utils.physics_randomizer import apply_physics_randomization

                articulation_handle = getattr(self._joint_api, "_articulation", None)
                self._last_physics_report = apply_physics_randomization(
                    self._world, sample, articulation=articulation_handle
                )
                self._maybe_update_goal_from_randomization(sample)
        except Exception:  # noqa
            self._last_physics_report = {"error": "physics_randomization_failed"}

    def _maybe_update_goal_from_randomization(self, sample: dict) -> None:
        task_cfg = sample.get("task") if isinstance(sample, dict) else None
        if not isinstance(task_cfg, dict):
            return
        joint_goal = task_cfg.get("joint_goal") or task_cfg.get("goal")
        if joint_goal is None:
            return
        joint_names = list(self._joint_config.get("joints", [])) if isinstance(self._joint_config, dict) else []
        alias_map = getattr(self, "_joint_alias_map", {}) or {}
        inverse_map = getattr(self, "_joint_inverse_alias_map", {}) or {}
        actual_order = list(self._actual_joint_names) if getattr(self, "_actual_joint_names", None) else []
        if not actual_order and alias_map:
            actual_order = [alias_map.get(name, name) for name in joint_names]
        new_goal = None
        if isinstance(joint_goal, dict) and joint_names:
            mapped: Dict[str, Any] = {}
            for key, val in joint_goal.items():
                target_key = alias_map.get(key, key)
                mapped[target_key] = val
            order = actual_order or joint_names
            values: list[float] = []
            for actual_name in order:
                source_key = inverse_map.get(actual_name, actual_name)
                val = mapped.get(actual_name, mapped.get(source_key))
                if isinstance(val, (int, float)):
                    values.append(float(val))
                else:
                    values = []
                    break
            if len(values) == self.ACTION_DIM:
                new_goal = np.asarray(values, dtype=float)
        elif isinstance(joint_goal, (list, tuple)) and len(joint_goal) == self.ACTION_DIM:
            arr = np.asarray(joint_goal, dtype=float)
            if actual_order and len(actual_order) == self.ACTION_DIM and joint_names and len(joint_names) == self.ACTION_DIM:
                # Reorder assuming incoming sequence follows joint_names order
                name_to_index = {name: idx for idx, name in enumerate(joint_names)}
                reordered = []
                for actual_name in actual_order:
                    cfg_key = inverse_map.get(actual_name, actual_name)
                    idx = name_to_index.get(cfg_key)
                    if idx is None:
                        reordered = []
                        break
                    reordered.append(arr[idx])
                if len(reordered) == self.ACTION_DIM:
                    arr = np.asarray(reordered, dtype=float)
            new_goal = arr
        elif isinstance(joint_goal, (int, float)):
            new_goal = np.full(self.ACTION_DIM, float(joint_goal), dtype=float)

        if new_goal is not None:
            self._goal = new_goal

    def _get_obs(self):
        # TODO: Replace placeholders with real joint_api.get_state() once articulation available.
        joint_state = self._joint_api.get_state()
        positions = np.array(joint_state.get("positions", self._q), dtype=float)
        velocities = np.array(joint_state.get("velocities", self._dq), dtype=float)
        if positions.shape[0] != self.ACTION_DIM:
            positions = np.resize(positions, self.ACTION_DIM)
        if velocities.shape[0] != self.ACTION_DIM:
            velocities = np.resize(velocities, self.ACTION_DIM)
        goal_delta = self._goal - positions
        obs = np.concatenate([positions, velocities, goal_delta])
        return obs.astype(np.float32)

    def step(self, action: np.ndarray):
        from src.utils.reward import RewardComposer, RewardConfig  # local import to avoid cycles
        from src.utils.limits import apply_delta_with_limits
        raw_action = np.array(action, dtype=float)
        action = np.clip(raw_action, -0.05, 0.05)
        prev_q = self._q.copy()
        # Articulation 존재 시 JointAPI 통해 delta 적용 (내부에서 clamp 처리)
        api_resp = self._joint_api.apply_delta(action, self._limit_lower, self._limit_upper)
        if api_resp.get("applied"):
            # get_state가 실제 pos 반환 가능 시 그 값을 q로 사용, 아니면 기존 delta 누적 fallback
            joint_state = self._joint_api.get_state()
            positions = joint_state.get("positions") or []
            if positions:
                arr = np.asarray(positions, dtype=float)
                if arr.shape[0] == self.ACTION_DIM:
                    self._q = arr
                else:
                    self._q = apply_delta_with_limits(self._q, action, self._limit_lower, self._limit_upper)
            else:
                self._q = apply_delta_with_limits(self._q, action, self._limit_lower, self._limit_upper)
        else:
            # fallback (Isaac 미환경 / articulation 없음)
            self._q = apply_delta_with_limits(self._q, action, self._limit_lower, self._limit_upper)
        self._dq = self._q - prev_q
        # Limits verification (only if articulation path used and state read)
        if api_resp.get("applied") and 'positions' in locals():
            verify = self._joint_api.verify_limits(self._q, self._limit_lower, self._limit_upper, tol=1e-3)
            if verify["violations"] > 0:
                # 간단 로그 (추후 구조화 로깅 연동)
                import logging, json as _json
                logging.getLogger(__name__).warning(_json.dumps({
                    "event": "limit_violation",
                    "count": verify["violations"],
                    "indices": verify["indices"],
                    "max_dev": verify["max_dev"]
                }))
                # episode 누적 카운터 (초기화 없으면 생성)
                if not hasattr(self, "_limit_violation_count"):
                    self._limit_violation_count = 0
                self._limit_violation_count += verify["violations"]
        # Reward (composer cached per instance? simple recreate each call for now)
        if not hasattr(self, "_reward_composer"):
            self._reward_composer = RewardComposer(RewardConfig())
        reward_dict = self._reward_composer.compute(self._q, self._dq, self._goal, action=action)
        terminated = bool(reward_dict.get("within_tol", 0.0) > 0.0)
        truncated = self._step_count >= 499
        self._step_count += 1
        obs = self._get_obs()
        info = {k: v for k, v in reward_dict.items() if k.endswith("_term") or k in ("tracking_error", "smoothness", "within_tol")}
        info["action_clipped"] = bool(np.any(raw_action != action))
        if hasattr(self, "_limit_violation_count") and self._limit_violation_count > 0:
            info["limit_violation_count"] = int(self._limit_violation_count)
        return obs, reward_dict["reward"], terminated, truncated, info

    def render(self, mode: str = "human"):
        # Isaac World 자체가 렌더링 담당 → 여기서는 placeholder dict
        return {"q": self._q.copy()}

    # ---- 추가 Joint I/O 메서드 ----
    def get_joint_state(self) -> Dict[str, Any]:
        return self._joint_api.get_state()

    def apply_action(self, deltas) -> Dict[str, Any]:  # for MCP direct call
        return self._joint_api.apply_delta(deltas, self._limit_lower, self._limit_upper)
