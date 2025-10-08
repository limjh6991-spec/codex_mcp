#!/usr/bin/env python
"""Stage / Articulation Inspection Tool.

기능:
  1. USD Stage 로드
  2. Articulation prim 탐색 및 joint 이름/개수 나열
  3. (--update-joints) 옵션으로 configs/roarm_joints.yaml 자동 갱신 (기존 limits 보존 시도)

사용 예:
  python sim/inspect_stage.py --usd /abs/path/roarm.usd
  python sim/inspect_stage.py --usd /abs/path/roarm.usd --update-joints

주의:
  Isaac Python 런처 환경 필요. (source setup_python_env.sh)
"""
from __future__ import annotations
import argparse
import os
import sys
import yaml
from typing import List

try:  # pragma: no cover
    import omni.isaac.kit  # type: ignore
    from omni.isaac.core import World  # type: ignore
    from omni.isaac.core.utils.stage import open_stage  # type: ignore
    from pxr import Usd, UsdGeom  # type: ignore
except Exception as e:  # noqa
    print("[FAIL] Isaac/Usd API import 실패:", e, file=sys.stderr)
    sys.exit(1)

JOINT_FILE = os.path.join("configs", "roarm_joints.yaml")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--usd", required=True, help="Target USD 파일 경로")
    p.add_argument("--update-joints", action="store_true", help="조인트 yaml 자동 갱신")
    return p.parse_args()


def find_joints(stage: Usd.Stage) -> List[str]:
    names: List[str] = []
    for prim in stage.TraverseAll():  # type: ignore
        path_str = str(prim.GetPath())
        # 간단 heuristic: 'joint' substring 또는 drive attribute 존재 검사 (추후 개선)
        if "joint" in path_str.lower():
            names.append(path_str.split("/")[-1])
    # fallback: 중복 제거
    uniq = sorted(list(dict.fromkeys(names)))
    return uniq


def load_existing_limits(path: str):
    if not os.path.isfile(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get("limits", {})


def write_joint_file(joints: List[str], existing_limits: dict):
    out = {
        "joints": joints,
        "limits": {j: existing_limits.get(j, [-3.14, 3.14]) for j in joints},
    }
    with open(JOINT_FILE, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True)
    print(f"[OK] Updated {JOINT_FILE} (joint_count={len(joints)})")


def main():
    args = parse_args()
    usd_path = os.path.abspath(args.usd)
    if not os.path.isfile(usd_path):
        print(f"[ERROR] USD not found: {usd_path}", file=sys.stderr)
        sys.exit(2)

    world = World()
    open_stage(usd_path)
    stage = Usd.Stage.Open(usd_path)
    joints = find_joints(stage)
    print(f"[INFO] Detected joints ({len(joints)}): {joints}")

    if args.update_joints:
        limits = load_existing_limits(JOINT_FILE)
        write_joint_file(joints, limits)

    # Quick step to ensure stage viability
    world.reset()
    for _ in range(2):
        world.step(render=False)
    print("[OK] Stage step sanity check passed")

if __name__ == "__main__":
    main()
