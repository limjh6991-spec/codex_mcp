#!/usr/bin/env python
"""Remove nested articulation root APIs except on the designated root prim.

Usage:
            source ~/isaacsim-venv/bin/activate && source scripts/activate_isaacsim_env.sh && \
                python scripts/clean_articulation_roots.py --usd assets/roarm_m3/usd/roarm_m3.usd --root /World/roarm_m3

This will open the USD, find any prims with UsdPhysics.ArticulationRootAPI or
PhysxSchema.PhysxArticulationRootAPI applied, and remove it from all prims
other than --root. Saves the stage in-place.
"""
from __future__ import annotations
import argparse

def ensure_app(headless: bool = True):
    try:
        from isaacsim.simulation_app import SimulationApp  # type: ignore
        app = SimulationApp({"headless": bool(headless)})
        return app
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--root", required=True, help="Keep articulation API only on this prim path")
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    app = ensure_app(headless=True)
    from pxr import Usd

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print("[clean] ERROR: Failed to open stage:", args.usd)
        if app:
            app.close()
        return 2

    root_path = args.root
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim or not root_prim.IsValid():
        print("[clean] ERROR: Root prim not found:", root_path)
        if app:
            app.close()
        return 3

    removed = []
    # Pass 1: USD Physics API
    try:
        from pxr import UsdPhysics  # type: ignore
        for prim in stage.TraverseAll():
            if prim.GetPath() == root_prim.GetPath():
                continue
            try:
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    ok = prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                    if ok:
                        removed.append(prim.GetPath().pathString)
            except Exception:
                pass
    except Exception:
        pass

    # Pass 2: PhysxSchema API
    try:
        from pxr import PhysxSchema  # type: ignore
        for prim in stage.TraverseAll():
            if prim.GetPath() == root_prim.GetPath():
                continue
            try:
                api = PhysxSchema.PhysxArticulationRootAPI.Get(stage, prim.GetPath())
                if api and api.IsApplied():
                    # try via RemoveAPI first
                    try:
                        ok = prim.RemoveAPI(PhysxSchema.PhysxArticulationRootAPI)
                        if ok:
                            removed.append(prim.GetPath().pathString)
                            continue
                    except Exception:
                        pass
                    # fallback: explicit schema removal if available
                    try:
                        api.Remove()
                        removed.append(prim.GetPath().pathString)
                    except Exception:
                        pass
            except Exception:
                pass
    except Exception:
        pass

    # Save stage
    stage.GetRootLayer().Save()
    print("[clean] Removed articulation API from:")
    for p in removed:
        print(" -", p)
    print("[clean] Done. Saved:", args.usd)

    if app:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
