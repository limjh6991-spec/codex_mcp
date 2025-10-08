"""MCP Server Skeleton for Isaac Sim control.

Note: 실제 Isaac Sim API 연동은 Isaac Sim Python 런처 환경에서 실행해야 함.
이 스켈레톤은 VS Code MCP(Runtime)에서 tool 정의/등록 구조를 보여주기 위한 예시.
"""
from __future__ import annotations
import json
import sys
import time
import logging
from functools import wraps
from typing import Any, Dict, Callable

# MCP 서버 구현 시 사용할 수 있는 추상 구조 (실제 MCP SDK 채택 시 수정 필요)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)

def timed(fn: Callable):
    @wraps(fn)
    def wrapper(*args, **kwargs):
        start = time.time()
        try:
            result = fn(*args, **kwargs)
            return result
        finally:
            dur = (time.time() - start) * 1000.0
            logging.info(f"tool={fn.__name__} duration_ms={dur:.2f}")
            sys.stdout.flush()
    return wrapper

class IsaacControlServer:
    def __init__(self):
        self.session_active = False
        # TODO: 실제 Isaac 환경 핸들/Env 인스턴스 참조 (주입 or lazy init)
        self._last_joint_state = {"positions": [], "velocities": []}

    @timed
    def start_sim(self, headless: bool = False) -> Dict[str, Any]:
        # TODO: Isaac Sim 로컬 실행 트리거 (subprocess or existing instance RPC)
        self.session_active = True
        return {"status": "started", "headless": headless}

    @timed
    def stop_sim(self) -> Dict[str, Any]:
        # TODO: 세션 종료 처리
        self.session_active = False
        return {"status": "stopped"}

    @timed
    def list_robots(self) -> Dict[str, Any]:
        # TODO: Isaac Stage에서 로드된 로봇 목록 반환
        return {"robots": ["roarm_m3"], "active": self.session_active}

    @timed
    def capture_observation(self) -> Dict[str, Any]:
        # TODO: 실제 카메라/센서 관측 수집
        return {"image": None, "joints": [0, 0, 0]}

    @timed
    def apply_action(self, joints_delta: list[float]) -> Dict[str, Any]:
        # TODO: Articulation API를 통해 Joint 명령 적용
        return {"applied": joints_delta}

    @timed
    def get_joint_state(self) -> Dict[str, Any]:
        # TODO: IsaacEnv 또는 JointAPI에서 fetch
        return self._last_joint_state

    @timed
    def set_joint_targets(self, targets: list[float]) -> Dict[str, Any]:
        # TODO: drive target 설정 → 결과 joint state 업데이트
        self._last_joint_state = {"positions": targets, "velocities": [0.0] * len(targets)}
        return {"status": "ok", "count": len(targets)}

    def schema(self) -> Dict[str, Any]:
        return {
            "tools": [
                {"name": "start_sim", "params": {"headless": "bool"}},
                {"name": "stop_sim", "params": {}},
                {"name": "list_robots", "params": {}},
                {"name": "capture_observation", "params": {}},
                {"name": "apply_action", "params": {"joints_delta": "List[float]"}},
                {"name": "get_joint_state", "params": {}},
                {"name": "set_joint_targets", "params": {"targets": "List[float]"}},
            ]
        }

if __name__ == "__main__":
    server = IsaacControlServer()
    print(json.dumps(server.schema(), indent=2, ensure_ascii=False))
