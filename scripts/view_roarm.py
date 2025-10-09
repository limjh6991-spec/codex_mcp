#!/usr/bin/env python
"""Quick GUI loop to view RoArm USD in Isaac Sim.

Usage (bundled Python recommended):
  /path/to/isaac-sim/python.sh scripts/view_roarm.py --usd /abs/path/roarm.usd --prim /World/roarm --warmup 5 --steps 300
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
    args = ap.parse_args()

    ensure_app(headless=args.headless)

    # world
    from omni.isaac.core import World  # type: ignore
    from omni.isaac.core.utils.stage import add_reference_to_stage  # type: ignore
    world = World(stage_units_in_meters=1.0)

    # add reference
    add_reference_to_stage(args.usd, args.prim)

    # warmup
    for _ in range(max(0, args.warmup)):
        world.step(render=not args.headless)

    # attach articulation view via helper
    from src.utils.isaac_joint_api import JointAPI  # type: ignore
    api = JointAPI()
    res = api.attach(args.prim, world=world, tries=3)
    print("attach:", res)

    # list joints
    names = api.list_joints()
    print("joints:", names)

    # tick loop
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
