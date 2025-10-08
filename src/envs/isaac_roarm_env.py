"""IsaacRoArmM3Env - Isaac Sim 기반 실제 학습 환경 스켈레톤.

주의: Isaac Sim Python API는 런처 환경에서만 import 가능할 수 있음.
조건부 import 및 graceful degrade 패턴을 사용.
"""
from __future__ import annotations
from typing import Any, Dict, Tuple
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

    def __init__(self, stage_path: str, joint_config: Dict[str, Any], domain_randomizer=None, headless: bool = False, articulation_prim: str | None = None):
        if not ISAAC_AVAILABLE:
            raise RuntimeError("Isaac Sim Python API를 사용할 수 없는 환경입니다. setup_python_env.sh 실행을 확인하세요.")
        super().__init__(domain_randomizer)
        self._stage_path = stage_path
        self._joint_config = joint_config
        self._world = World(stage_units_in_meters=1.0)
        # TODO: Stage 로드, 로봇 instancing, articulation 핸들 확보 후 JointAPI 주입
        self._joint_api = JointAPI(articulation=None)
        self._articulation_prim = articulation_prim or "/World/roarm"
        self._q = np.zeros(self.ACTION_DIM)
        self._dq = np.zeros(self.ACTION_DIM)
        self._goal = np.zeros(self.ACTION_DIM)
        # Joint limits (loaded lazily once to avoid file IO in tight loop)
        self._limits_loaded = False
        self._limit_lower = np.full(self.ACTION_DIM, -1.0, dtype=float)
        self._limit_upper = np.full(self.ACTION_DIM, 1.0, dtype=float)

    def reset(self, *, seed: int | None = None, options: Dict[str, Any] | None = None):
        if seed is not None:
            self.seed(seed)
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
            self._joint_api.attach(self._articulation_prim)
        self.apply_domain_randomization()
        obs = self._get_obs()
        info = {"stage": self._stage_path}
        self._step_count = 0
        return obs, info

    def apply_domain_randomization(self):  # type: ignore[override]
        # Call base to apply generic (which calls DomainRandomizer.apply if present)
        super().apply_domain_randomization()
        # Additional physics-level specialization using sampled values
        if self._domain_randomizer and ISAAC_AVAILABLE:
            try:
                sample = self._domain_randomizer.sample()
                from src.utils.physics_randomizer import apply_physics_randomization
                apply_physics_randomization(self._world, sample)
            except Exception:  # noqa
                pass

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
        # TODO: action → articulation drive command (joint_api.apply_delta)
        raw_action = np.array(action, dtype=float)
        action = np.clip(raw_action, -0.05, 0.05)
        prev_q = self._q.copy()
        self._q = apply_delta_with_limits(self._q, action, self._limit_lower, self._limit_upper)
        self._dq = self._q - prev_q  # simplistic discrete velocity placeholder
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
        return obs, reward_dict["reward"], terminated, truncated, info

    def render(self, mode: str = "human"):
        # Isaac World 자체가 렌더링 담당 → 여기서는 placeholder dict
        return {"q": self._q.copy()}

    # ---- 추가 Joint I/O 메서드 ----
    def get_joint_state(self) -> Dict[str, Any]:
        return self._joint_api.get_state()

    def apply_action(self, deltas) -> Dict[str, Any]:  # for MCP direct call
        return self._joint_api.apply_delta(deltas)
