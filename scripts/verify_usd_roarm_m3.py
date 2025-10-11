#!/usr/bin/env python3
# Quick USD sanity checker for RoArm M3 (runs inside Isaac Sim runtime)
# - Bootstraps SimulationApp headless so pxr and omni.usd are available
# - Opens a USD file via omni.usd and summarizes articulation/joints/links

import sys


def summarize_stage(stage):
    # Lazy import after SimulationApp init
    from pxr import UsdPhysics

    world = stage.GetPrimAtPath("/World")
    if not world:
        print("[ERROR] /World prim not found")
        return 2

    articulation_roots = []
    joints = []
    links = []

    def walk(prim):
        # Articulation roots
        if UsdPhysics.ArticulationRootAPI(prim):
            articulation_roots.append(prim.GetPath().pathString)
        # Joints by typename
        if prim.GetTypeName() in (
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint",
            "PhysicsSphericalJoint",
            "PhysicsFixedJoint",
            "PhysicsDistanceJoint",
        ):
            joints.append(prim.GetPath().pathString)
        # Links heuristic: Xform/Scope with any geom children or collision API
        if prim.GetTypeName() in ("Xform", "Scope"):
            has_child_geom = any(
                c.GetTypeName() in ("Mesh", "Cylinder", "Cone", "Cube", "Sphere") for c in prim.GetChildren()
            )
            try:
                has_collision = bool(UsdPhysics.CollisionAPI(prim))
            except Exception:
                has_collision = False
            if has_child_geom or has_collision:
                links.append(prim.GetPath().pathString)
        for c in prim.GetChildren():
            walk(c)

    walk(world)

    print("=== USD Summary ===")
    print(f"Articulation roots: {len(articulation_roots)}")
    for p in articulation_roots:
        print("  -", p)
    print(f"Joints: {len(joints)}")
    for p in joints:
        print("  -", p)
    print(f"Link-like prims: {len(links)}")
    for p in links[:30]:
        print("  -", p)
    if len(links) > 30:
        print(f"  ... (+{len(links)-30} more)")
    return 0


def main():
    if len(sys.argv) < 2:
        print("Usage: verify_usd_roarm_m3.py <path-to-usd>")
        return 1
    usd_path = sys.argv[1]

    # Bootstrap Isaac Sim runtime first
    from isaacsim import SimulationApp

    sim = SimulationApp({"headless": True})
    try:
        import omni.usd as ou

        ctx = ou.get_context()
        ctx.open_stage(usd_path)
        stage = ctx.get_stage()
        if stage is None:
            print(f"[ERROR] Failed to open stage via omni.usd: {usd_path}")
            return 1
        return summarize_stage(stage)
    finally:
        # Ensure clean shutdown
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
