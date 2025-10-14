from __future__ import annotations
import abc
from typing import Any, Dict, Tuple
import numpy as np
from src.utils.domain_randomizer import DomainRandomizer  # type: ignore

class BaseSim2RealEnv(abc.ABC):
    """Sim2Real 공통 인터페이스.

    Isaac Sim / 실제 로봇 모두 이 추상 클래스를 상속.
    관측/행동 스페이스는 Gymnasium Style(여기서는 단순 placeholder)으로 표현.
    """

    metadata: Dict[str, Any] = {}

    def __init__(self, domain_randomizer: Any | None = None):
        self._domain_randomizer = domain_randomizer
        self._step_count = 0

    @abc.abstractmethod
    def reset(self, *, seed: int | None = None, options: Dict[str, Any] | None = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        """환경 초기화.
        Returns: observation, info
        """

    @abc.abstractmethod
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """단일 step.
        Returns: observation, reward, terminated, truncated, info
        """

    @abc.abstractmethod
    def render(self, mode: str = "human") -> Any:
        ...

    def apply_domain_randomization(self, *, force: bool = False):
        if self._domain_randomizer:
            self._domain_randomizer.apply(self, force=force)

    def seed(self, seed: int):
        np.random.seed(seed)

class DummyRoArmM3Env(BaseSim2RealEnv):
    """Isaac Sim 연결 전까지 학습 파이프라인 검증용 Mock.
    관절 3개짜리 sin/cos 관측 + 목표각 추적 task.
    """

    ACTION_DIM = 3
    OBS_DIM = 6
    ACTION_RANGE = (-0.1, 0.1)

    def __init__(self, domain_randomizer=None):
        # domain_randomizer가 경로(str)이면 YAML 로드 시도
        if isinstance(domain_randomizer, str):
            try:
                domain_randomizer = DomainRandomizer.from_yaml(domain_randomizer)
            except Exception:
                domain_randomizer = None
        super().__init__(domain_randomizer)
        self._q = np.zeros(self.ACTION_DIM)
        self._goal = np.array([0.5, -0.3, 0.8])

    def reset(self, *, seed: int | None = None, options=None):
        if seed is not None:
            self.seed(seed)
        self._q = np.random.uniform(low=-1.0, high=1.0, size=self.ACTION_DIM)
        self.apply_domain_randomization(force=True)
        obs = self._get_obs()
        info = {"goal": self._goal.copy()}
        self._step_count = 0
        return obs, info

    def _get_obs(self):
        return np.concatenate([np.sin(self._q), np.cos(self._q)])

    def step(self, action: np.ndarray):
        action = np.clip(action, -0.1, 0.1)
        self._q = np.clip(self._q + action, -2.0, 2.0)
        err = self._goal - self._q
        reward = -float(np.linalg.norm(err))
        terminated = bool(np.linalg.norm(err) < 0.05)
        truncated = self._step_count >= 199
        self._step_count += 1
        obs = self._get_obs()
        info = {"error_norm": float(np.linalg.norm(err))}
        return obs, reward, terminated, truncated, info

    def render(self, mode: str = "human"):
        return {"q": self._q.copy()}
