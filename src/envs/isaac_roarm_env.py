"""IsaacRoArmM3Env - Isaac Sim 기반 실제 학습 환경 스켈레톤.

주의: Isaac Sim Python API는 런처 환경에서만 import 가능할 수 있음.
조건부 import 및 graceful degrade 패턴을 사용.
"""
from __future__ import annotations
from typing import Any, Dict, Tuple, Optional
import numpy as np

from .base_env import BaseSim2RealEnv
from src.utils.isaac_joint_api import JointAPI  # type: ignore

ISAAC_AVAILABLE = False
try:  # pragma: no cover - CI나 일반 환경에서는 실패 가능
    import omni.isaac.kit  # type: ignore
    from omni.isaac.core import World  # type: ignore
    ISAAC_AVAILABLE = True
except Exception:  # noqa
    ISAAC_AVAILABLE = False

class IsaacRoArmM3Env(BaseSim2RealEnv):
    ACTION_DIM = 6  # Placeholder (실제 로봇 관절 수에 맞게 조정)
    # Observation: [q(6), dq(6), goal_delta(6)] = 18 dims (placeholder design)
    OBS_DIM = 18

    def __init__(self, stage_path: str, joint_config: Dict[str, Any], domain_randomizer=None, headless: bool = False, articulation_prim: str | None = None, robot_usd_path: Optional[str] = None, warmup_steps: int = 3, attach_retries: int = 3):
        if not ISAAC_AVAILABLE:
            raise RuntimeError("Isaac Sim Python API를 사용할 수 없는 환경입니다. setup_python_env.sh 실행을 확인하세요.")
        super().__init__(domain_randomizer)
        self._stage_path = stage_path
        self._joint_config = joint_config
        self._world = World(stage_units_in_meters=1.0)
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

    # --- 내부 유틸 ---
    def _load_stage_once(self):  # pragma: no cover (Isaac 환경 외부에서는 skip)
        if self._stage_loaded or not ISAAC_AVAILABLE:
            return
        try:
            # Stage file 자체를 World가 로드해야 하는 경우 (버전별 차이 존재) - stage_path는 월드 초기화 참고용
            # 로봇 USD 참조 추가
            if self._robot_usd_path:
                try:
                    from omni.isaac.core.utils.stage import add_reference_to_stage  # type: ignore
                    add_reference_to_stage(self._robot_usd_path, self._articulation_prim)
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
        self.apply_domain_randomization()
        obs = self._get_obs()
        info = {"stage": self._stage_path, "articulation_attached": bool(getattr(self._joint_api, '_articulation', None))}
        if hasattr(self, "_last_physics_report"):
            info["physics_report"] = getattr(self, "_last_physics_report")
        self._step_count = 0
        return obs, info

    def apply_domain_randomization(self):  # type: ignore[override]
        # Call base to apply generic (which calls DomainRandomizer.apply if present)
        super().apply_domain_randomization()
        if self._domain_randomizer and ISAAC_AVAILABLE:
            try:
                sample = self._domain_randomizer.last_sample() or {}
                if sample:
                    from src.utils.physics_randomizer import apply_physics_randomization
                    articulation_handle = getattr(self._joint_api, "_articulation", None)
                    self._last_physics_report = apply_physics_randomization(self._world, sample, articulation=articulation_handle)
            except Exception:  # noqa
                self._last_physics_report = {"error": "physics_randomization_failed"}

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
