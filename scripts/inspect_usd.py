#!/usr/bin/env python3
"""
Inspect a USD file: print defaultPrim, top-level children, and whether a given prim exists
and has ArticulationRoot APIs. Tries to import pxr; if unavailable, initializes SimulationApp.

Usage:
    source ~/isaacsim-venv/bin/activate && source scripts/activate_isaacsim_env.sh && \
        python scripts/inspect_usd.py --usd assets/roarm_m3/usd/roarm_m3.usd [--prim /World/roarm_m3]
"""
import argparse, os, sys

def ensure_pxr():
    try:
        from pxr import Usd  # noqa: F401
        return True
    except Exception:
        try:
            from isaacsim.simulation_app import SimulationApp  # type: ignore
            SimulationApp({"headless": True, "width": 1, "height": 1})
            from pxr import Usd  # noqa: F401
            return True
        except Exception as e2:
            print(f"[inspect] ERROR: pxr unavailable even after SimulationApp init: {e2}")
            return False

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--prim", default=None)
    args = ap.parse_args()

    if not os.path.exists(args.usd):
        print(f"[inspect] ERROR: file not found: {args.usd}")
        return 2
    if not ensure_pxr():
        return 3

    from pxr import Usd
    try:
        from pxr import UsdPhysics
    except Exception:
        UsdPhysics = None
    try:
        from pxr import PhysxSchema
    except Exception:
        PhysxSchema = None

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print(f"[inspect] ERROR: failed to open stage: {args.usd}")
        return 4

    default_prim = stage.GetDefaultPrim()
    print(f"[inspect] defaultPrim: {default_prim.GetPath() if default_prim else 'None'}")

    # List top-level children under pseudo-root (/)
    root = stage.GetPseudoRoot()
    children = [str(p.GetPath()) for p in root.GetChildren()]
    print("[inspect] top_level:")
    for p in children:
        print(f"  - {p}")

    # List one-level children under defaultPrim (commonly /World)
    if default_prim and default_prim.IsValid():
        world_children = [str(ch.GetPath()) for ch in default_prim.GetChildren()]
        print("[inspect] world_children:")
        for p in world_children:
            print(f"  - {p}")
        if not world_children:
            print("[inspect] world_children: (empty)")

    if args.prim:
        prim = stage.GetPrimAtPath(args.prim)
        print(f"[inspect] prim_exists: {prim.IsValid()} prim: {args.prim}")
        if prim.IsValid():
            has_usdphys = False
            has_physx = False
            if UsdPhysics is not None:
                try:
                    has_usdphys = prim.HasAPI(UsdPhysics.ArticulationRootAPI)
                except Exception:
                    pass
            if PhysxSchema is not None:
                try:
                    has_physx = prim.HasAPI(PhysxSchema.PhysxArticulationRootAPI)
                except Exception:
                    pass
            print(f"[inspect] prim_has_UsdPhysics_ArticulationRootAPI: {has_usdphys}")
            print(f"[inspect] prim_has_PhysxSchema_ArticulationRootAPI: {has_physx}")

            # Deep inspection: count rigid bodies and joints in subtree
            rb_count = 0
            joint_count = 0
            joint_samples = []
            def is_joint(p):
                if UsdPhysics is None:
                    return None
                try:
                    types = [
                        getattr(UsdPhysics, name)
                        for name in (
                            'RevoluteJoint','PrismaticJoint','SphericalJoint','DistanceJoint','FixedJoint'
                        ) if hasattr(UsdPhysics, name)
                    ]
                    for cls in types:
                        if p.IsA(cls):
                            return cls.__name__
                except Exception:
                    return None
                return None

            it = stage.Traverse()
            for p in it:
                if not str(p.GetPath()).startswith(args.prim):
                    continue
                try:
                    if UsdPhysics is not None and p.HasAPI(UsdPhysics.RigidBodyAPI):
                        rb_count += 1
                except Exception:
                    pass
                jt = is_joint(p)
                if jt:
                    joint_count += 1
                    if len(joint_samples) < 8:
                        joint_samples.append(f"{jt}:{p.GetPath()}")
            print(f"[inspect] subtree_rigid_bodies: {rb_count}")
            print(f"[inspect] subtree_joints: {joint_count}")
            if joint_samples:
                print("[inspect] joint_samples:")
                for s in joint_samples:
                    print(f"  - {s}")

    return 0

if __name__ == "__main__":
    sys.exit(main())
