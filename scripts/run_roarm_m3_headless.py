#!/usr/bin/env python3
"""Minimal headless spawner for RoArm M3 USD in Isaac Sim 5.x

- Loads the given USD file via omni.isaac.core.utils.stage.open_stage
- Initializes World, scans for articulation root candidates
- Attaches JointAPI helper and prints joint names and initial state

Usage:
    source ~/isaacsim-venv/bin/activate && \
    python scripts/run_roarm_m3_headless.py --usd assets/roarm_m3/urdf/roarm_m3.clean/roarm_m3.clean.usd --prim /roarm_m3 --steps 120
"""
from __future__ import annotations
import argparse, sys, time, os


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True, help="Path to robot USD")
    ap.add_argument("--prim", default="/roarm_m3", help="Articulation root prim path")
    ap.add_argument("--steps", type=int, default=120)
    ap.add_argument("--headless", action="store_true")
    return ap.parse_args()


def ensure_app(headless: bool=False):
    # Prefer new SimulationApp if available
    try:
        from isaacsim.simulation_app import SimulationApp  # type: ignore
        return SimulationApp({"headless": bool(headless)})
    except Exception:
        # Fallback to old omni.isaac.kit
        try:
            import omni.isaac.kit as kit  # type: ignore
            return kit
        except Exception as e:
            raise RuntimeError(f"Cannot initialize Isaac environment: {e}")


def scan_articulation_candidates(stage, world_prim_path="/World"):
    cands = []
    try:
        from pxr import PhysxSchema  # type: ignore
        for prim in stage.TraverseAll():
            api = PhysxSchema.PhysxArticulationRootAPI.Get(stage, prim.GetPath())
            if api and api.IsApplied():
                cands.append(prim.GetPath().pathString)
    except Exception:
        pass
    if not cands:
        try:
            from pxr import UsdPhysics  # type: ignore
            for prim in stage.TraverseAll():
                try:
                    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                        cands.append(prim.GetPath().pathString)
                except Exception:
                    continue
        except Exception:
            pass
    # If still empty, list direct children under /World to help user pick
    if not cands:
        world_children = []
        w = stage.GetPrimAtPath(world_prim_path)
        if w and w.IsValid():
            world_children = [c.GetPath().pathString for c in w.GetChildren()]
        return cands, world_children
    return cands, []


def resolve_defaultprim_and_world(stage):
    """Return (defaultPrimPath, hasWorld) for diagnostics and auto-fix suggestions."""
    default_prim = stage.GetDefaultPrim()
    default_path = default_prim.GetPath().pathString if default_prim and default_prim.IsValid() else None
    has_world = bool(stage.GetPrimAtPath("/World") and stage.GetPrimAtPath("/World").IsValid())
    return default_path, has_world


def main():
    args = parse_args()

    if not os.path.exists(args.usd):
        print(f"[spawner] ERROR: file not found: {args.usd}")
        return 2

    app = ensure_app(headless=args.headless)
    # Load stage
    from omni.isaac.core import World  # type: ignore
    from omni.isaac.core.utils.stage import open_stage  # type: ignore
    from omni.usd import get_context  # type: ignore

    open_stage(args.usd)
    stage = get_context().get_stage()
    if stage is None:
        print(f"[spawner] ERROR: failed to open stage: {args.usd}")
        return 3

    # Diagnostics on defaultPrim and /World existence
    defprim, has_world = resolve_defaultprim_and_world(stage)
    if defprim:
        print(f"[spawner] defaultPrim: {defprim}")
    else:
        print("[spawner] defaultPrim: None")
    print(f"[spawner] has /World: {has_world}")

    cands, world_children = scan_articulation_candidates(stage)
    print("[spawner] articulation_root_candidates:")
    for c in cands:
        print("  -", c)
    if world_children:
        print("[spawner] world children:")
        for c in world_children:
            print("  -", c)

    world = World(stage_units_in_meters=1.0)

    # Attach helper
    from src.utils.isaac_joint_api import JointAPI  # type: ignore
    api = JointAPI()
    # Auto-adjust prim if provided path doesn't exist but a candidate exists
    attach_prim = args.prim
    prim_obj = stage.GetPrimAtPath(attach_prim)
    if (not prim_obj) or (not prim_obj.IsValid()):
        if cands:
            print(f"[spawner] provided prim {attach_prim} not found; switching to detected {cands[0]}")
            attach_prim = cands[0]
        elif defprim and defprim != "/World":
            # If there is no /World content and defaultPrim looks like the root model, try it
            print(f"[spawner] provided prim {attach_prim} not found; trying defaultPrim {defprim}")
            attach_prim = defprim
    att = api.attach(attach_prim, world=world, tries=3)
    print("[spawner] attach:", att)

    try:
        world.reset()
    except Exception:
        pass

    # Warmup a few frames
    for _ in range(5):
        world.step(render=not args.headless)

    names = api.list_joints()
    print("[spawner] joints:", names)
    # Report DOF limits if available
    try:
        dof_info = api.get_dof_limits()
        if dof_info.get("names"):
            print("[spawner] dof_count:", len(dof_info["names"]))
            print("[spawner] dof_limits.lower:", dof_info.get("lower"))
            print("[spawner] dof_limits.upper:", dof_info.get("upper"))
    except Exception:
        pass

    # Short loop
    steps = max(1, args.steps)
    import math
    for i in range(steps):
        world.step(render=not args.headless)
        if i % 30 == 0:
            js = api.get_state()
            print(f"[{i}] q[:6]={js.get('positions', [])[:6]}")
        # simple motion on first few joints if any
        if i < steps:
            try:
                n = len(names)
                if n > 0:
                    amp = 0.1
                    cmd = [0.0] * n
                    # slow sine deltas
                    for k in range(min(3, n)):
                        cmd[k] = amp * math.sin(2 * math.pi * (i / max(1.0, steps)))
                    api.apply_delta(cmd, verify=False)
            except Exception:
                pass
        time.sleep(0.005)

    print("[spawner] done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
