#!/usr/bin/env python
"""Isaac Sim Python API 임포트 진단 스크립트 (신/구 API 호환).
우선순위:
 1) isaacsim.simulation_app (신 API)
 2) omni.isaac.kit (구 API, deprecated 경고 가능)
 3) omni.isaac.core (핵심 모듈)
`source scripts/activate_isaacsim_env.sh` 실행 후 사용하세요.
"""
from __future__ import annotations
import sys
import importlib

# If available, initialize SimulationApp early to allow core imports
_sim_app = None
try:
    from isaacsim.simulation_app import SimulationApp  # type: ignore
    try:
        _sim_app = SimulationApp({"headless": True})  # ensure kit is up
    except Exception:
        _sim_app = None
except Exception:
    SimulationApp = None  # type: ignore

results = []

def try_import(mod: str) -> tuple[str, bool, str | None]:
    try:
        importlib.import_module(mod)
        return mod, True, None
    except Exception as e:  # noqa
        return mod, False, str(e)

checks = [
    "isaacsim.simulation_app",
    "omni.isaac.kit",
    "omni.isaac.core",
]

ok = True
for mod in checks:
    name, success, err = try_import(mod)
    results.append((name, success, err))
    # Don't early-return; report all
    if not success and name in ("omni.isaac.core",):
        ok = False

for name, success, err in results:
    print(f"[{'OK' if success else 'FAIL'}] {name}{'' if success else f': {err}'}")

if ok:
    print("[OK] Isaac Sim 기본 임포트 점검 통과")
    try:
        if _sim_app is not None:
            _sim_app.close()
    except Exception:
        pass
    sys.exit(0)
else:
    print("[FAIL] Isaac Sim 핵심 모듈 일부 임포트 실패")
    try:
        if _sim_app is not None:
            _sim_app.close()
    except Exception:
        pass
    sys.exit(1)
