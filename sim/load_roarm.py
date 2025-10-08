#!/usr/bin/env python
"""RoArm M3 Stage Loader Prototype.

Usage:
  python sim/load_roarm.py --usd /absolute/path/to/roarm.usd --headless 0

설명:
- Isaac API import 실패 시 친절한 에러 출력
- USD 경로 존재 여부 검사
- (TODO) Stage에 instancing 후 joint/articulation 확인 로깅
"""
from __future__ import annotations
import argparse
import os
import sys

try:
    import omni.isaac.kit  # type: ignore
    from omni.isaac.core import World  # type: ignore
    from omni.isaac.core.utils.stage import open_stage
except Exception as e:  # noqa
    print("[FAIL] Isaac API import 실패: setup_python_env.sh 실행 여부 확인", e, file=sys.stderr)
    sys.exit(1)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--usd", required=True, help="RoArm M3 USD 파일 경로")
    p.add_argument("--headless", type=int, default=0, help="1=Headless")
    return p.parse_args()


def main():
    args = parse_args()
    usd_path = os.path.abspath(args.usd)
    if not os.path.isfile(usd_path):
        print(f"[ERROR] USD 파일을 찾을 수 없음: {usd_path}", file=sys.stderr)
        sys.exit(2)

    # Kit 초기화 (간단 모드)
    world = World(stage_units_in_meters=1.0)
    open_stage(usd_path)
    print(f"[INFO] Stage opened: {usd_path}")
    # TODO: find articulation, list joints
    world.reset()
    print("[INFO] World reset completed")
    # 한 프레임 실행
    for _ in range(5):
        world.step(render=False)
    print("[OK] Basic stepping done (placeholder)")

if __name__ == "__main__":
    main()
