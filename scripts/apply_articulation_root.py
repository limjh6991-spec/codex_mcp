#!/usr/bin/env python3
"""
Apply Articulation Root API to a prim in a USD file.

Works with Isaac Sim 5.0 where the URDF import UI no longer exposes
"Create PhysX Articulation" in the same place. This script applies:
 - UsdPhysics.ArticulationRootAPI (if available)
 - PhysxSchema.PhysxArticulationRootAPI (PhysX schema)

Usage (run inside Isaac Sim python):
    source ~/isaacsim-venv/bin/activate && source scripts/activate_isaacsim_env.sh && \
            python scripts/apply_articulation_root.py --usd assets/roarm_m3/usd/roarm_m3.usd --prim /World/roarm_m3
"""

import argparse
import os
import sys

def main():
    parser = argparse.ArgumentParser(description="Apply Articulation Root API to a prim in USD")
    parser.add_argument("--usd", required=True, help="Path to USD file")
    parser.add_argument("--prim", required=True, help="Prim path, e.g., /World/roarm_m3")
    args = parser.parse_args()

    usd_path = args.usd
    prim_path = args.prim

    if not os.path.exists(usd_path):
        print(f"[apply] ERROR: USD file not found: {usd_path}")
        return 2

    # Import pxr modules available in Isaac Sim environment
    try:
        from pxr import Usd, UsdGeom
    except Exception as e:
        # In Isaac Sim 5.0, pxr may only be available after SimulationApp initializes
        print(f"[apply] INFO: pxr import failed initially: {e}. Trying to initialize SimulationApp...")
        try:
            from isaacsim.simulation_app import SimulationApp
            sim_app = SimulationApp({
                "headless": True,
                "renderer": "RayTracedLighting",
                "width": 1,
                "height": 1,
            })
            from pxr import Usd, UsdGeom  # retry after app init
        except Exception as e2:
            print(f"[apply] ERROR: pxr still unavailable after SimulationApp init: {e2}")
            return 3

    # Optional APIs
    UsdPhysics = None
    PhysxSchema = None
    try:
        from pxr import UsdPhysics as _UsdPhysics
        UsdPhysics = _UsdPhysics
    except Exception:
        pass
    try:
        from pxr import PhysxSchema as _PhysxSchema
        PhysxSchema = _PhysxSchema
    except Exception:
        pass

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[apply] ERROR: Failed to open stage: {usd_path}")
        return 4

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[apply] ERROR: Prim not found on stage: {prim_path}")
        return 5

    applied_any = False

    # Apply UsdPhysics articulation root API if available
    if UsdPhysics is not None:
        try:
            if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                UsdPhysics.ArticulationRootAPI.Apply(prim)
                applied_any = True
                print("[apply] Applied UsdPhysics.ArticulationRootAPI")
            else:
                print("[apply] UsdPhysics.ArticulationRootAPI already present")
        except Exception as e:
            print(f"[apply] WARN: Could not apply UsdPhysics.ArticulationRootAPI: {e}")
    else:
        print("[apply] INFO: UsdPhysics module not available; skipping UsdPhysics.ArticulationRootAPI")

    # Apply PhysX schema articulation-related API if available in this version.
    # Note: In Isaac Sim 5.0, PhysxSchema may not expose PhysxArticulationRootAPI; often only UsdPhysics.ArticulationRootAPI is used.
    if PhysxSchema is not None:
        try:
            if hasattr(PhysxSchema, "PhysxArticulationRootAPI"):
                if not prim.HasAPI(PhysxSchema.PhysxArticulationRootAPI):
                    PhysxSchema.PhysxArticulationRootAPI.Apply(prim)
                    applied_any = True
                    print("[apply] Applied PhysxSchema.PhysxArticulationRootAPI")
                else:
                    print("[apply] PhysxSchema.PhysxArticulationRootAPI already present")
            else:
                print("[apply] INFO: PhysxSchema.PhysxArticulationRootAPI not found in this build; skipping.")
        except Exception as e:
            print(f"[apply] WARN: PhysxSchema application skipped due to error: {e}")
    else:
        print("[apply] INFO: PhysxSchema module not available; skipping PhysxSchema")

    # Save changes if any were applied
    try:
        if applied_any:
            stage.GetRootLayer().Save()
            print(f"[apply] Saved USD with articulation root at prim: {prim_path}")
        else:
            print("[apply] No changes needed; articulation root already present or APIs unavailable.")
    except Exception as e:
        print(f"[apply] ERROR: Failed to save stage: {e}")
        return 6

    # Verify
    verified = False
    if UsdPhysics is not None and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        verified = True
    if PhysxSchema is not None and hasattr(PhysxSchema, "PhysxArticulationRootAPI"):
        try:
            if prim.HasAPI(PhysxSchema.PhysxArticulationRootAPI):
                verified = True
        except Exception:
            pass
    print(f"[apply] verify articulation_root_present={verified}")
    return 0 if verified else 1


if __name__ == "__main__":
    sys.exit(main())
