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
import importlib
import os
import sys


def _resolve_stage_open():
    try:
        from isaacsim.core.utils.stage import open_stage as _open_stage  # type: ignore[attr-defined]
        return _open_stage
    except Exception:
        try:
            from omni.isaac.core.utils.stage import open_stage as _open_stage  # type: ignore
            return _open_stage
        except Exception:
            return None

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
        "isaacsim.asset.importer.urdf",
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

# URDF importer capability detection
HAS_ISAACSIM_URDF = False
HAS_OMNI_CONVERT = False
HAS_LEGACY_IMPORTER = False
kit_commands = None
convert_urdf = None
URDFImporter = None  # type: ignore[assignment]
URDFImportConfig = None  # type: ignore[assignment]

try:
    kit_commands = importlib.import_module("omni.kit.commands")  # type: ignore
    importlib.import_module("isaacsim.asset.importer.urdf")
    HAS_ISAACSIM_URDF = True
except Exception:
    kit_commands = None

if not HAS_ISAACSIM_URDF:
    try:
        _scripts_convert = importlib.import_module("omni.isaac.urdf.scripts.convert")
        convert_urdf = getattr(_scripts_convert, "convert_urdf")
        HAS_OMNI_CONVERT = callable(convert_urdf)
    except Exception:
        convert_urdf = None
        HAS_OMNI_CONVERT = False
    if not HAS_OMNI_CONVERT:
        try:
            _omni_urdf_mod = importlib.import_module("omni.isaac.urdf")
            URDFImporter = getattr(_omni_urdf_mod, "URDFImporter", None)
            URDFImportConfig = getattr(_omni_urdf_mod, "URDFImportConfig", None)
            HAS_LEGACY_IMPORTER = URDFImporter is not None and URDFImportConfig is not None
        except Exception:
            URDFImporter = None
            URDFImportConfig = None
            HAS_LEGACY_IMPORTER = False


def import_urdf_to_stage(urdf_path: str, prim_path: str) -> None:
    if HAS_ISAACSIM_URDF and kit_commands is not None:
        status, import_config = kit_commands.execute("URDFCreateImportConfig")
        if not status:
            raise RuntimeError("URDFCreateImportConfig failed")
        # Align config with previous behavior (no merge, no default prim, keep base free)
        def _set_if_attr(name: str, value) -> None:
            if hasattr(import_config, name):
                setattr(import_config, name, value)

        _set_if_attr("merge_fixed_joints", False)
        _set_if_attr("fix_base", False)
        _set_if_attr("make_default_prim", False)
        _set_if_attr("import_in_stage", True)
        _set_if_attr("collision_from_visuals", False)
        _set_if_attr("self_collision", False)
        _set_if_attr("parse_mimic", True)
        status, parsed_robot = kit_commands.execute(
            "URDFParseFile",
            urdf_path=urdf_path,
            import_config=import_config,
        )
        if not status:
            raise RuntimeError(f"Failed to parse URDF: {urdf_path}")
        status, imported_path = kit_commands.execute(
            "URDFImportRobot",
            urdf_path=urdf_path,
            urdf_robot=parsed_robot,
            import_config=import_config,
            dest_path="",
            get_articulation_root=True,
        )
        if not status:
            raise RuntimeError(f"Failed to import URDF: {urdf_path}")
        from omni.usd import get_context  # type: ignore

        stage = get_context().get_stage()
        if prim_path and imported_path and stage is not None:
            target_path = prim_path
            if imported_path != target_path:
                from pxr import Sdf  # type: ignore

                parent_path = Sdf.Path(target_path).GetParentPath()
                if parent_path and not stage.GetPrimAtPath(parent_path):
                    stage.DefinePrim(parent_path)
                try:
                    kit_commands.execute("MovePrim", path_from=imported_path, path_to=target_path)
                except Exception:
                    prim = stage.GetPrimAtPath(imported_path)
                    parent = stage.GetPrimAtPath(parent_path) if parent_path else None
                    if prim and parent and prim.GetParent() == parent:
                        prim.SetName(Sdf.Path(target_path).name)
        return

    if HAS_OMNI_CONVERT and callable(convert_urdf):
        convert_urdf(
            input_filename=urdf_path,
            dest_path=os.path.dirname(urdf_path),
            import_in_stage=True,
            merge_fixed_joints=False,
            fix_base=False,
            make_default_prim=False,
            prim_path=prim_path,
        )
        return

    if HAS_LEGACY_IMPORTER and URDFImporter is not None and URDFImportConfig is not None:
        cfg = URDFImportConfig()
        cfg.merge_fixed_joints = False
        cfg.fix_base = False
        cfg.make_default_prim = False
        importer = URDFImporter()
        importer.import_robot(urdf_path, prim_path, cfg)
        return

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

    # Switch to open the stage for import via Kit context when possible
    open_stage_fn = _resolve_stage_open()
    if open_stage_fn is not None:
        try:
            open_stage_fn(out_path)
            from omni.usd import get_context  # type: ignore

            stage = get_context().get_stage()
        except Exception:
            stage = Usd.Stage.Open(out_path)
    else:
        stage = Usd.Stage.Open(out_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD at {out_path}")
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
