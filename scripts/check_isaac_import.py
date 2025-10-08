#!/usr/bin/env python
"""Isaac Sim Python API 임포트 진단 스크립트.
ISAAC_SIM_PATH 환경 변수 또는 setup_python_env.sh를 통해 PYTHONPATH가 설정되어 있어야 한다.
"""
from __future__ import annotations
import os
import sys
import importlib

REQUIRED = [
    "omni.isaac.kit",
    "omni.isaac.core",
]

missing = []
for mod in REQUIRED:
    try:
        importlib.import_module(mod)
    except Exception as e:  # noqa
        missing.append((mod, str(e)))

if missing:
    print("[FAIL] Isaac Sim 모듈 임포트 실패:")
    for mod, err in missing:
        print(f" - {mod}: {err}")
    print("환경 설정 가이드: source $ISAAC_SIM_PATH/setup_python_env.sh")
    sys.exit(1)
else:
    print("[OK] Isaac Sim 핵심 모듈 임포트 성공")
