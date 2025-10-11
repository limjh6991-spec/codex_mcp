#!/usr/bin/env python
"""Quick GUI loop to view RoArm USD in Isaac Sim.

Usage (bundled Python recommended):
    source ~/isaacsim-venv/bin/activate && source scripts/activate_isaacsim_env.sh && \
        python scripts/view_roarm.py --usd /abs/path/roarm.usd --prim /World/roarm --warmup 5 --steps 300
    source ~/isaacsim-venv/bin/activate && source scripts/activate_isaacsim_env.sh && \
        python scripts/view_roarm.py --usd /abs/path/roarm.usd --scan
"""
from __future__ import annotations
import argparse, time

# Try new API first
_sim_app = None
try:
    from isaacsim.simulation_app import SimulationApp  # type: ignore
except Exception:
    SimulationApp = None  # type: ignore

def ensure_app(headless: bool=False):
    global _sim_app
    if _sim_app is not None:
        return _sim_app
    if SimulationApp is not None:
        _sim_app = SimulationApp({"headless": bool(headless)})
        return _sim_app
    # fallback to old API
    try:
        import omni.isaac.kit as kit  # type: ignore
        _sim_app = kit  # minimal placeholder to keep reference
        return _sim_app
    except Exception as e:
        raise RuntimeError(f"Cannot initialize Isaac SimulationApp/kit: {e}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True, help="Robot USD absolute path")
    ap.add_argument("--prim", default="/World/roarm", help="Articulation prim path")
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--steps", type=int, default=300)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--scan", action="store_true", help="Scan USD for articulation root candidates and exit")
    ap.add_argument("--hold", action="store_true", help="Keep running until Ctrl+C (ignore steps)")
    args = ap.parse_args()

    ensure_app(headless=args.headless)

    # world / open existing USD as stage
    from omni.isaac.core import World  # type: ignore
    from omni.isaac.core.utils.stage import open_stage  # type: ignore
    from omni.usd import get_context  # type: ignore
    open_stage(args.usd)
    stage = get_context().get_stage()
    world = World(stage_units_in_meters=1.0)

    # optional: scan and exit
    if args.scan:
        try:
            print("[scan] defaultPrim:", stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else None)
            # prim validity
            p = stage.GetPrimAtPath(args.prim)
            print("[scan] prim_exists:", p.IsValid(), "prim:", args.prim)
            # explicit articulation API presence on target prim
            has_usdphysics_api = False
            has_physxschema_api = False
            try:
                from pxr import UsdPhysics  # type: ignore
                if p and p.IsValid():
                    has_usdphysics_api = p.HasAPI(UsdPhysics.ArticulationRootAPI)
            except Exception:
                pass
            try:
                from pxr import PhysxSchema  # type: ignore
                if p and p.IsValid():
                    api = PhysxSchema.PhysxArticulationRootAPI.Get(stage, p.GetPath())
                    has_physxschema_api = bool(api and api.IsApplied())
            except Exception:
                pass
            print("[scan] prim_has_UsdPhysics_ArticulationRootAPI:", has_usdphysics_api)
            print("[scan] prim_has_PhysxSchema_ArticulationRootAPI:", has_physxschema_api)
            # list immediate children under /World for quick prim discovery
            world_prim = stage.GetPrimAtPath("/World")
            world_children = []
            if world_prim and world_prim.IsValid():
                world_children = [c.GetPath().pathString for c in world_prim.GetChildren()]
            print("[scan] world_children:")
            for c in world_children:
                print(" -", c)
            # articulation root candidates
            cands = []
            try:
                from pxr import PhysxSchema  # type: ignore
                for prim in stage.TraverseAll():
                    api = PhysxSchema.PhysxArticulationRootAPI.Get(stage, prim.GetPath())
                    if api and api.IsApplied():
                        cands.append(prim.GetPath().pathString)
            except Exception:
                pass
            # Fallback: check for older USD Physics schema if available
            if not cands:
                try:
                    from pxr import UsdPhysics  # type: ignore
                    for prim in stage.TraverseAll():
                        # Some versions used specialized ArticulationRoot prim type
                        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                            cands.append(prim.GetPath().pathString)
                except Exception:
                    pass
            print("[scan] articulation_root_candidates:")
            for c in cands:
                print(" -", c)
            if not cands:
                print("[scan] no articulation root API detected. Re-import USD with 'Create PhysX Articulation' enabled or set it in GUI.")
        finally:
            return

    # attach articulation view via helper (before world.reset so views get initialized)
    from src.utils.isaac_joint_api import JointAPI  # type: ignore
    api = JointAPI()
    res = api.attach(args.prim, world=world, tries=3)
    print("attach:", res)

    # Now initialize world to register any newly added views
    try:
        world.reset()
    except Exception:
        pass

    # warmup
    for _ in range(max(0, args.warmup)):
        world.step(render=not args.headless)

    # list joints
    names = api.list_joints()
    print("joints:", names)

    # tick loop
    if args.hold:
        i = 0
        try:
            while True:
                world.step(render=not args.headless)
                if i % 30 == 0:
                    js = api.get_state()
                    print(f"[{i}] positions: {js.get('positions', [])[:6]} ...")
                i += 1
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("interrupted.")
    else:
        steps = max(1, args.steps)
        for i in range(steps):
            world.step(render=not args.headless)
            if i % 30 == 0:
                js = api.get_state()
                print(f"[{i}] positions: {js.get('positions', [])[:6]} ...")
            time.sleep(0.01)

    print("done.")

if __name__ == "__main__":
    main()
