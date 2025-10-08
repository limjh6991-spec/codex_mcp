"""MCP Server Skeleton for Isaac Sim control.

Note: 실제 Isaac Sim API 연동은 Isaac Sim Python 런처 환경에서 실행해야 함.
이 스켈레톤은 VS Code MCP(Runtime)에서 tool 정의/등록 구조를 보여주기 위한 예시.
"""
from __future__ import annotations
import json
import sys
import time
import logging
import os
from functools import wraps
from typing import Any, Dict, Callable

# MCP 서버 구현 시 사용할 수 있는 추상 구조 (실제 MCP SDK 채택 시 수정 필요)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)

def _jsonl_logger():
    """Return a function to append structured tool call logs to daily jsonl file.

    File layout: logs/mcp_calls/YYYY-MM-DD.jsonl
    Fields: ts, tool, latency_ms, ok, error_code(optional), input_size, output_size, truncated flags.
    """
    log_dir = os.path.join("logs", "mcp_calls")
    os.makedirs(log_dir, exist_ok=True)
    date_str = time.strftime("%Y-%m-%d")
    path = os.path.join(log_dir, f"{date_str}.jsonl")

    def write(entry: dict):  # best-effort
        try:
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception:  # noqa
            pass
    return write

_append_call_log = _jsonl_logger()

def _truncate_value(val: Any, max_len: int = 500) -> tuple[Any, bool]:
    """Truncate long string/sequence-like for logging to avoid PII/huge payloads.
    Returns (possibly truncated value, truncated_flag)."""
    try:
        if isinstance(val, str) and len(val) > max_len:
            return val[:max_len] + "...<trunc>", True
        if isinstance(val, (list, tuple)) and len(val) > max_len:
            return list(val[:max_len]) + ["<trunc>"] , True
        if isinstance(val, dict) and len(val) > 50:
            # Keep first 50 keys only
            trimmed = dict(list(val.items())[:50])
            trimmed["<trunc>"] = f"{len(val)-50} more keys"
            return trimmed, True
    except Exception:  # noqa
        return val, False
    return val, False

def structured_tool(fn: Callable):
    @wraps(fn)
    def wrapper(*args, **kwargs):
        start = time.time()
        tool_name = fn.__name__
        # Input snapshot (shallow)
        try:
            input_repr = {"args": args[1:] if len(args) > 1 else [], "kwargs": kwargs}
        except Exception:
            input_repr = {"error": "input_capture_failed"}
        input_json = json.dumps(input_repr, default=str) if isinstance(input_repr, (dict, list)) else str(input_repr)
        try:
            result = fn(*args, **kwargs)
        except Exception as e:  # capture exception as error payload
            result = {"ok": False, "error_code": "EXC", "error_message": f"{type(e).__name__}: {e}"}
        dur_ms = (time.time() - start) * 1000.0
        # Determine success
        ok = True
        error_code = None
        if isinstance(result, dict):
            if result.get("ok") is False or "error" in result:
                ok = False
                error_code = result.get("error_code") or result.get("error")
        # Size + truncation logic
        output_obj = result
        output_json = None
        truncated_out = False
        try:
            output_json = json.dumps(output_obj, ensure_ascii=False, default=str)
            if len(output_json) > 4000:
                # attempt truncation of top-level keys
                if isinstance(output_obj, dict):
                    trimmed = {}
                    for k_idx, (k, v) in enumerate(output_obj.items()):
                        if k_idx >= 40:
                            trimmed["<trunc>"] = f"{len(output_obj)-40} more keys"
                            truncated_out = True
                            break
                        tv, trunc_flag = _truncate_value(v)
                        if trunc_flag:
                            truncated_out = True
                        trimmed[k] = tv
                    output_json = json.dumps(trimmed, ensure_ascii=False, default=str)
                else:
                    output_json = output_json[:4000] + "...<trunc>"
                    truncated_out = True
        except Exception:
            output_json = "<serialize_error>"
            truncated_out = True
        entry = {
            "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "tool": tool_name,
            "latency_ms": round(dur_ms, 3),
            "ok": ok,
            "error_code": error_code,
            "input_size": len(input_json),
            "output_size": len(output_json) if output_json else None,
            "truncated_output": truncated_out,
        }
        _append_call_log(entry)
        logging.info(f"tool={tool_name} ok={ok} latency_ms={dur_ms:.2f}")
        return result
    return wrapper

class IsaacControlServer:
    def __init__(self):
        self.session_active = False
        # TODO: 실제 Isaac 환경 핸들/Env 인스턴스 참조 (주입 or lazy init)
        self._last_joint_state = {"positions": [], "velocities": []}
        self._policy = None
        self._policy_path = None
        # Safety state
        self._emergency_stop = False
        self._last_action = None  # last applied delta (list[float])
        self._rate_limit_per_joint = 0.2  # max absolute delta change per joint per call

    @structured_tool
    def start_sim(self, headless: bool = False) -> Dict[str, Any]:
        # TODO: Isaac Sim 로컬 실행 트리거 (subprocess or existing instance RPC)
        self.session_active = True
        return {"status": "started", "headless": headless}

    @structured_tool
    def stop_sim(self) -> Dict[str, Any]:
        # TODO: 세션 종료 처리
        self.session_active = False
        return {"status": "stopped"}

    @structured_tool
    def list_robots(self) -> Dict[str, Any]:
        # TODO: Isaac Stage에서 로드된 로봇 목록 반환
        return {"robots": ["roarm_m3"], "active": self.session_active}

    @structured_tool
    def capture_observation(self) -> Dict[str, Any]:
        # TODO: 실제 카메라/센서 관측 수집
        return {"image": None, "joints": [0, 0, 0]}

    @structured_tool
    def apply_action(self, joints_delta: list[float]) -> Dict[str, Any]:
        # Safety: emergency stop overrides any action
        if self._emergency_stop:
            applied = [0.0 for _ in joints_delta]
            return {"applied": applied, "emergency_stop": True, "rate_limited": False}
        # Rate limiting relative to last action (or zero baseline)
        prev = self._last_action or [0.0] * len(joints_delta)
        rate_limited = False
        applied = []
        for i, v in enumerate(joints_delta):
            baseline = prev[i] if i < len(prev) else 0.0
            delta = v
            # constrain absolute command itself (delta magnitude per joint)
            if abs(delta) > self._rate_limit_per_joint:
                delta = max(-self._rate_limit_per_joint, min(self._rate_limit_per_joint, delta))
                rate_limited = True
            applied.append(delta)
        self._last_action = applied
        return {"applied": applied, "rate_limited": rate_limited, "emergency_stop": False}

    @structured_tool
    def set_emergency_stop(self, enable: bool) -> Dict[str, Any]:
        self._emergency_stop = bool(enable)
        return {"emergency_stop": self._emergency_stop}

    @structured_tool
    def get_safety_status(self) -> Dict[str, Any]:
        return {
            "emergency_stop": self._emergency_stop,
            "rate_limit_per_joint": self._rate_limit_per_joint,
            "last_action": self._last_action,
        }

    @structured_tool
    def get_joint_state(self) -> Dict[str, Any]:
        # TODO: IsaacEnv 또는 JointAPI에서 fetch
        return self._last_joint_state

    @structured_tool
    def set_joint_targets(self, targets: list[float]) -> Dict[str, Any]:
        # TODO: drive target 설정 → 결과 joint state 업데이트
        self._last_joint_state = {"positions": targets, "velocities": [0.0] * len(targets)}
        return {"status": "ok", "count": len(targets)}

    @structured_tool
    def policy_infer(self, observation: list[float] | None = None, policy_path: str | None = None, deterministic: bool = True) -> Dict[str, Any]:
        """Run inference using a saved SB3 policy.

        Parameters
        ----------
        observation : list[float]
            Raw observation vector; if None uses last_joint_state positions padded.
        policy_path : str
            Path to policy zip. If provided and differs from currently loaded, reloads.
        deterministic : bool
            Deterministic inference flag.
        """
        # Structured response baseline
        t_start = time.time()
        def finish(payload: Dict[str, Any]) -> Dict[str, Any]:
            payload.setdefault("latency_ms", (time.time() - t_start) * 1000.0)
            payload.setdefault("policy_path", self._policy_path)
            return payload

        ERROR = {
            "DEP": "stable_baselines3 not available",
            "LOAD_FAIL": "failed to load policy",
            "NO_POLICY": "no policy loaded; provide policy_path",
            "EMPTY": "observation empty",
            "NAN": "observation contains NaN/Inf",
            "OBS_DIM": "observation dimension mismatch",
            "INFER": "inference failed",
        }
        try:
            from stable_baselines3 import PPO  # lazy import
            import numpy as np
        except Exception:
            return finish({"ok": False, "error_code": "DEP", "error_message": ERROR["DEP"]})

        # (Re)load policy if path differs
        if policy_path and policy_path != self._policy_path:
            try:
                load_start = time.time()
                self._policy = PPO.load(policy_path)
                self._policy_path = policy_path
                load_dur = (time.time() - load_start) * 1000.0
            except Exception as e:  # noqa
                return finish({
                    "ok": False,
                    "error_code": "LOAD_FAIL",
                    "error_message": f"{ERROR['LOAD_FAIL']}: {e}",
                })
        if self._policy is None:
            return finish({"ok": False, "error_code": "NO_POLICY", "error_message": ERROR["NO_POLICY"]})

        # Determine expected observation dimension if possible
        expected_dim = None
        try:
            obs_space = getattr(self._policy, "observation_space", None)
            if obs_space is not None:
                shape = getattr(obs_space, "shape", None)
                if shape is not None and len(shape) > 0:
                    import numpy as _np
                    expected_dim = int(_np.prod(shape))
        except Exception:  # noqa
            pass

        # Fallback observation
        if observation is None:
            base = self._last_joint_state.get("positions", [])
            observation = list(base)

        if observation is None or len(observation) == 0:
            return finish({"ok": False, "error_code": "EMPTY", "error_message": ERROR["EMPTY"], "expected_obs_dim": expected_dim})

        try:
            obs_arr = np.array(observation, dtype=float).reshape(1, -1)
        except Exception as e:  # noqa
            return finish({"ok": False, "error_code": "OBS_DIM", "error_message": f"{ERROR['OBS_DIM']}: {e}", "expected_obs_dim": expected_dim})

        # NaN / Inf check
        if not np.isfinite(obs_arr).all():
            return finish({"ok": False, "error_code": "NAN", "error_message": ERROR["NAN"], "expected_obs_dim": expected_dim})

        # Dimension validation
        if expected_dim is not None and int(obs_arr.shape[-1]) != expected_dim:
            return finish({
                "ok": False,
                "error_code": "OBS_DIM",
                "error_message": f"{ERROR['OBS_DIM']}: got {int(obs_arr.shape[-1])}, expected {expected_dim}",
                "expected_obs_dim": expected_dim,
                "received_obs_dim": int(obs_arr.shape[-1]),
            })

        # Inference
        try:
            action, _ = self._policy.predict(obs_arr, deterministic=deterministic)
            import numpy as _npa
            action = _npa.array(action)
            if action.ndim == 1:
                action_list = action.tolist()
            else:
                action_list = action[0].tolist()
        except Exception as e:  # noqa
            return finish({"ok": False, "error_code": "INFER", "error_message": f"{ERROR['INFER']}: {e}"})

        return finish({
            "ok": True,
            "action": action_list,
            "deterministic": deterministic,
            "obs_dim": int(obs_arr.shape[-1]),
            "expected_obs_dim": expected_dim,
        })

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
                {"name": "set_emergency_stop", "params": {"enable": "bool"}},
                {"name": "get_safety_status", "params": {}},
                {"name": "policy_infer", "params": {"observation": "Optional[List[float]]", "policy_path": "Optional[str]", "deterministic": "bool"}},
            ]
        }

if __name__ == "__main__":
    server = IsaacControlServer()
    print(json.dumps(server.schema(), indent=2, ensure_ascii=False))
