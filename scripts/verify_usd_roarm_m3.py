#!/usr/bin/env python3
"""Quick USD sanity checker for the RoArm M3 stage.

이 스크립트는 Isaac Sim 런타임(SimulationApp)을 부트스트랩한 뒤 지정한 USD를
열어 아티큘레이션/조인트/링크 요약과 선택적 계층 덤프를 출력합니다.

사용 예시
---------

.. code-block:: bash

    # 기본 요약만 출력 (headless)
    python scripts/verify_usd_roarm_m3.py assets/roarm_m3/roarm_m3_stage.usd

    # 계층 덤프 포함 + 깊이 제한 4, 최대 120 프림
    python scripts/verify_usd_roarm_m3.py --usd assets/roarm_m3/roarm_m3_stage.usd \
        --hierarchy --max-depth 4 --max-prims 120

"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect a USD stage using Isaac Sim's runtime.")
    parser.add_argument("usd", nargs="?", help="Path to the USD stage (positional).")
    parser.add_argument("--usd", dest="usd_flag", metavar="PATH", help="Path to the USD stage (optional flag).")
    parser.add_argument("--hierarchy", action="store_true", help="Print a prim hierarchy dump starting at --root.")
    parser.add_argument("--root", default="/World", help="Prim path to use as the hierarchy root (default: /World).")
    parser.add_argument("--max-depth", type=int, default=3, help="Maximum depth when dumping the hierarchy (default: 3).")
    parser.add_argument(
        "--max-prims",
        type=int,
        default=200,
        help="Maximum number of prims to display in the hierarchy dump (default: 200).",
    )
    parser.add_argument(
        "--warmup-steps",
        type=int,
        default=60,
        help="SimulationApp update steps to wait for stage loading (default: 60).",
    )
    parser.add_argument(
        "--gui",
        dest="headless",
        action="store_false",
        help="Run with the Isaac Sim UI instead of headless mode.",
    )
    parser.set_defaults(headless=True)
    parser.add_argument(
        "--skip-joint-checks",
        dest="validate_joints",
        action="store_false",
        help="Skip validation of joint attachment relationships.",
    )
    parser.set_defaults(validate_joints=True)
    return parser


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)

    usd_raw = args.usd_flag or args.usd
    if not usd_raw:
        parser.error("USD path must be supplied either positionally or via --usd PATH.")

    usd_path = Path(usd_raw).expanduser().resolve()
    if not usd_path.exists():
        parser.error(f"USD file not found: {usd_path}")
    if usd_path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
        print(f"[WARN] Unexpected file suffix for USD stage: {usd_path.suffix}")

    args.usd_path = usd_path
    if args.max_depth < 0:
        parser.error("--max-depth must be >= 0")
    if args.max_prims <= 0:
        parser.error("--max-prims must be > 0")
    if args.warmup_steps < 0:
        parser.error("--warmup-steps must be >= 0")

    return args


def summarize_stage(stage) -> tuple[int, dict[str, list[str]]]:
    # Lazy import after SimulationApp initialisation
    from pxr import UsdPhysics

    world = stage.GetPrimAtPath("/World")
    if not world:
        print("[ERROR] /World prim not found")
        return 2, {"articulations": [], "joints": [], "links": []}

    articulation_roots = []
    joints = []
    links = []

    def walk(prim):
        if UsdPhysics.ArticulationRootAPI(prim):
            articulation_roots.append(prim.GetPath().pathString)

        if prim.GetTypeName() in {
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint",
            "PhysicsSphericalJoint",
            "PhysicsFixedJoint",
            "PhysicsDistanceJoint",
        }:
            joints.append(prim.GetPath().pathString)

        if prim.GetTypeName() in {"Xform", "Scope"}:
            has_child_geom = any(
                c.GetTypeName() in {"Mesh", "Cylinder", "Cone", "Cube", "Sphere"}
                for c in prim.GetChildren()
            )
            try:
                has_collision = bool(UsdPhysics.CollisionAPI(prim))
            except Exception:
                has_collision = False
            if has_child_geom or has_collision:
                links.append(prim.GetPath().pathString)

        for child in prim.GetChildren():
            walk(child)

    walk(world)

    print("=== USD Summary ===")
    print(f"Articulation roots: {len(articulation_roots)}")
    for path in articulation_roots:
        print(f"  - {path}")

    print(f"Joints: {len(joints)}")
    for path in joints:
        print(f"  - {path}")

    print(f"Link-like prims: {len(links)}")
    for path in links[:30]:
        print(f"  - {path}")
    if len(links) > 30:
        print(f"  ... (+{len(links) - 30} more)")

    summary = {
        "articulations": articulation_roots,
        "joints": joints,
        "links": links,
    }

    return 0, summary


def validate_joint_attachments(stage, joint_paths: list[str]) -> int:
    """Ensure every physics joint has exactly one attachment on both sides."""

    issues: list[str] = []

    for path in joint_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim:
            issues.append(f"{path}: prim missing from stage")
            continue

        def _check(rel_name: str) -> None:
            rel = prim.GetRelationship(rel_name)
            if rel is None:
                issues.append(f"{path}: relationship '{rel_name}' missing")
                return

            targets = rel.GetTargets()
            if len(targets) != 1:
                issues.append(
                    f"{path}: expected 1 target for {rel_name}, found {len(targets)}"
                )
                return

            target_path = targets[0].pathString
            if not stage.GetPrimAtPath(target_path):
                issues.append(
                    f"{path}: {rel_name} target '{target_path}' not found in stage"
                )

        _check("physics:body0")
        _check("physics:body1")

    if issues:
        print("=== Joint Attachment Issues ===")
        for msg in issues:
            print(f"  - {msg}")
        return 4

    if joint_paths:
        print(f"Joint attachments: OK ({len(joint_paths)} joints validated)")
    else:
        print("Joint attachments: no joints found to validate")

    return 0


def dump_hierarchy(stage, root_path: str, max_depth: int, max_prims: int) -> int:
    prim = stage.GetPrimAtPath(root_path)
    if not prim:
        print(f"[ERROR] Prim '{root_path}' not found in the stage")
        return 3

    print(f"=== Hierarchy ({root_path}) ===")

    count = 0
    truncated = False

    def recurse(current, depth):
        nonlocal count, truncated
        if count >= max_prims:
            truncated = True
            return

        indent = "  " * depth
        print(f"{indent}- {current.GetPath().pathString}")
        count += 1

        if depth >= max_depth:
            children = list(current.GetChildren())
            if children:
                print(f"{indent}  ... (+{len(children)} children hidden at depth limit {max_depth})")
            return

        for child in current.GetChildren():
            if count >= max_prims:
                truncated = True
                break
            recurse(child, depth + 1)

    recurse(prim, 0)

    if truncated:
        print(f"... (hierarchy truncated at {max_prims} prims)")

    return 0


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)

    from isaacsim import SimulationApp

    sim = SimulationApp({"headless": args.headless})
    exit_code = 0
    try:
        import omni.usd as ou

        ctx = ou.get_context()
        try:
            ctx.open_stage(str(args.usd_path))
        except RuntimeError as exc:
            print(f"[ERROR] Failed to open stage '{args.usd_path}': {exc}")
            return 1

        stage = ctx.get_stage()
        steps = 0
        while stage is None and steps < max(args.warmup_steps, 1):
            sim.update()
            stage = ctx.get_stage()
            steps += 1

        if stage is None:
            print(
                f"[ERROR] Stage failed to load after {args.warmup_steps} SimulationApp updates: {args.usd_path}"
            )
            exit_code = 1
        else:
            exit_code, summary = summarize_stage(stage)
            if args.validate_joints:
                joint_code = validate_joint_attachments(stage, summary["joints"])
                exit_code = max(exit_code, joint_code)
            if args.hierarchy:
                hierarchy_code = dump_hierarchy(stage, args.root, args.max_depth, args.max_prims)
                exit_code = max(exit_code, hierarchy_code)

        sim.update()
        return exit_code
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
