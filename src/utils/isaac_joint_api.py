"""Isaac Joint API Helper (Placeholder)

실제 Isaac Sim API 연동 시 articulation/joint 접근 로직을 캡슐화.
현재는 스켈레톤: 실제 환경에서 omni.isaac.core.articulations API 사용.
"""
from __future__ import annotations
from typing import List, Dict, Any

try:  # pragma: no cover
    import omni.isaac.kit  # type: ignore
    ISAAC_AVAILABLE = True
except Exception:  # noqa
    ISAAC_AVAILABLE = False

class JointAPI:
    def __init__(self, articulation=None):  # articulation 핸들은 실제 Isaac 로딩 후 주입 예정
        self._articulation = articulation
        self._prim_path: str | None = None

    def attach(self, prim_path: str, articulation=None):
        """Attach articulation handle (placeholder).

        실제 구현 시 prim_path로 Stage에서 prim 검색 후 articulation wrapper 획득.
        """
        self._prim_path = prim_path
        if articulation is not None:
            self._articulation = articulation
        return {"attached": True, "prim_path": prim_path, "has_handle": self._articulation is not None}

    def is_attached(self) -> bool:
        return self._prim_path is not None

    def list_joints(self) -> List[str]:
        if not ISAAC_AVAILABLE or self._articulation is None:
            return []
        # TODO: articulation.get_joints() → name 추출
        return []

    def get_state(self) -> Dict[str, Any]:
        if not ISAAC_AVAILABLE or self._articulation is None:
            return {"positions": [], "velocities": []}
        # TODO: 실제 joint pos/vel 읽기
        return {"positions": [], "velocities": []}

    def apply_delta(self, deltas):
        if not ISAAC_AVAILABLE or self._articulation is None:
            return {"applied": False, "reason": "No articulation or Isaac unavailable"}
        # TODO: 현재 joint pos + delta → drive target set
        return {"applied": True}
