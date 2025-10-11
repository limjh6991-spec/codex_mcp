#!/usr/bin/env python
"""
Convert a URDF into a USD using Isaac Sim's URDF importer.

Usage (pip-installed Isaac Sim environment):
    source ~/isaacsim-venv/bin/activate && source scripts/activate_isaacsim_env.sh && \
        python scripts/convert_urdf_to_usd.py \
            --urdf assets/roarm_m3/urdf/roarm_m3.urdf \
            --out assets/roarm_m3/usd/roarm_m3.usd \
            --prim "/World/roarm"

Notes:
- Must be run after `source scripts/activate_isaacsim_env.sh` (pip 기반 Isaac Sim 환경).
- The script will initialize SimulationApp, import the URDF, and save the stage to the output USD path.
"""
from __future__ import annotations
import argparse
import os
import sys

# Initialize SimulationApp early
SimulationApp = None
try:
    from isaacsim.simulation_app import SimulationApp  # type: ignore
except Exception:
    pass

if SimulationApp is None:
    try:
        import omni.isaac.kit as kit  # type: ignore
    except Exception as e:
        raise RuntimeError(f"Cannot initialize Isaac environment: {e}")
else:
    _app = SimulationApp({"headless": True})

# Try to enable URDF importer extensions if present
try:
    import omni.kit.app as kit_app  # type: ignore
    _omni_app = kit_app.get_app()
    _ext_mgr = _omni_app.get_extension_manager()
    # Common extension ids across versions
    for ext_id in [
        "omni.isaac.urdf",
        "isaacsim.urdf_importer",
        "isaacsim.robots.urdf",
    ]:
        try:
            if _ext_mgr.get_extension_enabled(ext_id) is False:
                _ext_mgr.set_extension_enabled(ext_id, True)
        except Exception:
            pass
except Exception:
    pass

# Imports that require the app
from omni.isaac.core.utils.stage import add_reference_to_stage  # type: ignore
from pxr import Usd, Sdf  # type: ignore
try:
    from pxr import PhysxSchema  # type: ignore
except Exception:
    PhysxSchema = None  # type: ignore

# URDF importer (newer Isaac Sim)
try:
    from omni.isaac.urdf import _urdf
    from omni.isaac.urdf.scripts.convert import convert_urdf
    HAS_NEW_IMPORTER = True
except Exception:
    HAS_NEW_IMPORTER = False

# Legacy importer path
try:
    from omni.isaac.urdf import URDFImporter, URDFImportConfig  # type: ignore
    HAS_LEGACY_IMPORTER = True
except Exception:
    HAS_LEGACY_IMPORTER = False


def import_urdf_to_stage(urdf_path: str, prim_path: str) -> None:
    if HAS_NEW_IMPORTER:
        # Newer convert utility handles full pipeline; it writes a USD to a folder by default.
        convert_urdf(
            input_filename=urdf_path,
            dest_path=os.path.dirname(urdf_path),
            import_in_stage=True,
            merge_fixed_joints=False,
            fix_base=False,
            make_default_prim=False,
            prim_path=prim_path,
        )
    elif HAS_LEGACY_IMPORTER:
        cfg = URDFImportConfig()
        cfg.merge_fixed_joints = False
        cfg.fix_base = False
        cfg.make_default_prim = False
        importer = URDFImporter()
        importer.import_robot(urdf_path, prim_path, cfg)
    else:
        raise RuntimeError("URDF importer not available in this Isaac Sim build.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True, help="Input URDF path")
    ap.add_argument("--out", required=True, help="Output USD path")
    ap.add_argument("--prim", default="/World/roarm", help="Root prim path for articulation")
    args = ap.parse_args()

    urdf_path = os.path.abspath(args.urdf)
    out_path = os.path.abspath(args.out)
    prim_path = args.prim

    # Create stage
    stage = Usd.Stage.CreateNew(out_path)
    if stage is None:
        raise RuntimeError(f"Failed to create USD at {out_path}")

    # Ensure default prim root exists
    root_prim = stage.DefinePrim(Sdf.Path("/World"))
    stage.SetDefaultPrim(root_prim)
    stage.GetRootLayer().Save()

    # Switch to open the stage for import and add reference or import content
    stage = Usd.Stage.Open(out_path)
    # Import URDF into the opened stage
    import_urdf_to_stage(urdf_path, prim_path)

    # Apply PhysX Articulation Root API to the prim if available
    p = stage.GetPrimAtPath(prim_path)
    if not p.IsValid():
        # help user locate probable prims
        world = stage.GetPrimAtPath("/World")
        children = []
        if world.IsValid():
            children = [c.GetPath().pathString for c in world.GetChildren()]
        raise RuntimeError(
            f"Prim not found at '{prim_path}'. Children under /World: {children}. "
            f"Re-run with --prim set to the robot root prim path."
        )
    if PhysxSchema is not None:
        try:
            PhysxSchema.PhysxArticulationRootAPI.Apply(stage, p.GetPath())
            print(f"Applied PhysX Articulation Root to {prim_path}")
        except Exception as e:  # nosec - best effort
            print(f"Warning: failed to apply PhysX Articulation Root to {prim_path}: {e}")

    # Save to disk
    stage.GetRootLayer().Save()
    print(f"Saved USD: {out_path}")


if __name__ == "__main__":
    main()
