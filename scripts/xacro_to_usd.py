#!/usr/bin/env python3
"""
XACRO → URDF → USD converter for Isaac Sim (headless).

- Expands a .xacro to .urdf if xacro is available; otherwise falls back to an existing URDF.
- Uses isaacsim.asset.importer.urdf to import into an open stage and saves to USD.
- Optionally cleans up stray "/<root>/world/..." groups and sets defaultPrim to the model root.
-
Usage:
  source scripts/activate_isaacsim_env.sh
  python scripts/xacro_to_usd.py \
      --xacro assets/roarm_m3/urdf/roarm_m3.xacro \
      --urdf-out assets/roarm_m3/urdf/roarm_m3.generated.urdf \
      --usd-out assets/roarm_m3/urdf/roarm_m3.generated.usd \
      --root-prim /roarm_m3 \
      --clean-stray-world

Notes:
  - Must be run after sourcing scripts/activate_isaacsim_env.sh to ensure pxr/omni paths.
  - If xacro is not installed, this will attempt to use an existing URDF instead.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from typing import Optional


TRACE_PATH = "/tmp/xacro_to_usd.trace"


def trace(message: str) -> None:
    try:
        with open(TRACE_PATH, "a", encoding="utf-8") as f:
            f.write(message + "\n")
    except Exception:
        pass


def which(cmd: str) -> Optional[str]:
    from shutil import which as _which
    return _which(cmd)


def expand_xacro(xacro_path: str, urdf_out: str) -> bool:
    """Expand a XACRO file into URDF.

    Returns True on success, False if xacro isn't available.
    """
    # Try Python xacro first
    try:
        import xacro  # type: ignore
        doc = xacro.process_file(xacro_path)
        xml = doc.toprettyxml(indent="  ")
        os.makedirs(os.path.dirname(urdf_out), exist_ok=True)
        with open(urdf_out, "w", encoding="utf-8") as f:
            f.write(xml)
        return True
    except Exception:
        pass

    # Try xacro CLI
    xacro_bin = which("xacro")
    if xacro_bin:
        os.makedirs(os.path.dirname(urdf_out), exist_ok=True)
        cmd = [xacro_bin, xacro_path, "-o", urdf_out]
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if proc.returncode == 0 and os.path.exists(urdf_out):
            return True
        print(f"[xacro] CLI failed: rc={proc.returncode} stderr=\n{proc.stderr}")

    return False


def ensure_urdf(xacro_path: str, urdf_out: str, urdf_fallbacks: list[str]) -> str:
    """Ensure we have a URDF file, using xacro if possible, else fallback URDFs.

    Returns the path to a URDF file to use.
    """
    if xacro_path and os.path.exists(xacro_path):
        if expand_xacro(xacro_path, urdf_out):
            print(f"[xacro] expanded to: {urdf_out}")
            return urdf_out
        else:
            print("[xacro] not available; falling back to existing URDFs")

    # Fallback URDFs (first existing is used)
    for cand in urdf_fallbacks:
        if cand and os.path.exists(cand):
            if os.path.abspath(cand) != os.path.abspath(urdf_out):
                os.makedirs(os.path.dirname(urdf_out), exist_ok=True)
                shutil.copyfile(cand, urdf_out)
                print(f"[urdf] copied fallback {cand} -> {urdf_out}")
                return urdf_out
            print(f"[urdf] using existing: {urdf_out}")
            return urdf_out

    raise FileNotFoundError("No URDF available. Provide --xacro or a valid --urdf-fallback.")


def import_urdf_into_open_stage(urdf_path: str) -> str:
    """Import URDF into the currently open omni.usd stage using kit commands.

    Returns the articulation root prim path reported by the importer.
    """
    import omni.kit.commands as commands
    from isaacsim.asset.importer.urdf import _urdf

    trace("import_urdf:config:start")
    try:
        cfg_result = commands.execute("URDFCreateImportConfig")
    except Exception as exc:
        trace(f"import_urdf:config:error:{exc}")
        raise
    trace(f"import_urdf:config:result:{type(cfg_result)}")
    cfg = None
    if isinstance(cfg_result, dict):
        cfg = cfg_result.get("import_config")
    elif isinstance(cfg_result, (list, tuple)):
        cfg = next((item for item in cfg_result if isinstance(item, _urdf.ImportConfig)), None)
    elif hasattr(cfg_result, "import_config"):
        cfg = cfg_result.import_config  # type: ignore[attr-defined]
    else:
        cfg = cfg_result if isinstance(cfg_result, _urdf.ImportConfig) else None
    if cfg is None:
        raise RuntimeError(f"URDFCreateImportConfig did not return an import_config: {cfg_result}")

    # Prevent importer from collapsing fixed joints so tcp links remain separate
    # Prevent importer from collapsing fixed joints so tcp links remain separate
    try:
        if hasattr(cfg, "set_merge_fixed_joints"):
            cfg.set_merge_fixed_joints(False)
        else:
            cfg.merge_fixed_joints = False  # type: ignore[attr-defined]
    except AttributeError:
        trace("import_urdf:config:mergeFixedJoints_attr_missing")

    trace("import_urdf:config:created")
    try:
        robot_result = commands.execute(
            "URDFParseFile",
            urdf_path=urdf_path,
            import_config=cfg,
        )
    except Exception as exc:
        trace(f"import_urdf:parse:error:{exc}")
        raise
    trace(f"import_urdf:parse:result:{type(robot_result)}")
    robot = None
    if isinstance(robot_result, dict):
        robot = robot_result.get("urdf_robot")
    elif isinstance(robot_result, (list, tuple)):
        robot = next((item for item in robot_result if isinstance(item, _urdf.UrdfRobot)), None)
    elif hasattr(robot_result, "urdf_robot"):
        robot = robot_result.urdf_robot  # type: ignore[attr-defined]
    elif isinstance(robot_result, _urdf.UrdfRobot):
        robot = robot_result
    if robot is None:
        raise RuntimeError(f"URDFParseFile did not return a urdf_robot: {robot_result}")

    trace("import_urdf:parsed")
    try:
        import_result = commands.execute(
            "URDFImportRobot",
            urdf_path=urdf_path,
            urdf_robot=robot,
            import_config=cfg,
            dest_path="",
            get_articulation_root=True,
        )
    except Exception as exc:
        trace(f"import_urdf:import:error:{exc}")
        raise
    trace(f"import_urdf:import_result:{type(import_result)}")
    prim_path = None
    if isinstance(import_result, dict):
        prim_path = import_result.get("articulation_root") or import_result.get("prim_path")
    elif isinstance(import_result, (list, tuple)):
        prim_path = next((item for item in import_result if isinstance(item, str)), None)
    elif hasattr(import_result, "articulation_root"):
        prim_path = import_result.articulation_root  # type: ignore[attr-defined]
    elif isinstance(import_result, str):
        prim_path = import_result
    if not prim_path or not isinstance(prim_path, str):
        raise RuntimeError(f"URDFImportRobot did not return a prim path string: {prim_path}")
    trace(f"import_urdf:prim_path:{prim_path}")
    return prim_path


def set_default_prim(stage, prim_path: str) -> None:
    from pxr import Sdf

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found at {prim_path}")
    stage.SetDefaultPrim(prim)
    # Ensure root layer has a defaultPrim token written
    root = stage.GetRootLayer()
    # Force a save to persist the default prim
    root.Save()


def move_or_rename_prim(stage, path_from: str, path_to: str) -> bool:
    """Move a prim subtree using omni.kit.commands MovePrim.

    Returns True if moved, False if already at destination.
    """
    if path_from == path_to:
        return False
    import omni.kit.commands as commands
    commands.execute("MovePrim", path_from=path_from, path_to=path_to)
    return True


def delete_prim_if_safe(stage, path: str) -> bool:
    """Delete a prim if it contains no Physics*Joint prims.

    Returns True if deleted, False otherwise.
    """
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return False
    # Check for joints inside
    joint_types = {
        "PhysicsRevoluteJoint",
        "PhysicsPrismaticJoint",
        "PhysicsSphericalJoint",
        "PhysicsFixedJoint",
        "PhysicsDistanceJoint",
    }
    stack = [prim]
    while stack:
        p = stack.pop()
        if p.GetTypeName() in joint_types:
            return False  # don't delete if any joints found
        stack.extend(p.GetChildren())

    import omni.kit.commands as commands
    commands.execute("DeletePrim", path=path)
    return True


def summarize(stage, root_prim: str) -> dict:
    from pxr import UsdPhysics

    default = stage.GetDefaultPrim()
    default_path = default.GetPath().pathString if default and default.IsValid() else None
    stray_world = stage.GetPrimAtPath(f"{root_prim}/world")
    # Count articulation roots and joints for quick sanity
    art_roots = []
    joints = []

    def walk(prim):
        if UsdPhysics.ArticulationRootAPI(prim):
            art_roots.append(prim.GetPath().pathString)
        if prim.GetTypeName() in (
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint",
            "PhysicsSphericalJoint",
            "PhysicsFixedJoint",
            "PhysicsDistanceJoint",
        ):
            joints.append(prim.GetPath().pathString)
        for c in prim.GetChildren():
            walk(c)

    rp = stage.GetPrimAtPath(root_prim)
    if rp and rp.IsValid():
        walk(rp)

    return {
        "defaultPrim": default_path,
        "rootExists": bool(rp and rp.IsValid()),
        "strayWorldExists": bool(stray_world and stray_world.IsValid()),
        "articulationRoots": art_roots,
        "jointCount": len(joints),
    }


def run(args: argparse.Namespace) -> int:
    # 1) Ensure URDF
    trace("run:ensure_urdf:start")
    urdf_path = ensure_urdf(
        xacro_path=args.xacro,
        urdf_out=args.urdf_out,
        urdf_fallbacks=[p for p in [args.urdf_fallback, "assets/roarm_m3/urdf/roarm_m3.clean.urdf", "assets/roarm_m3/urdf/roarm_m3.urdf"] if p],
    )
    trace(f"run:ensure_urdf:done -> {urdf_path}")

    # 2) Start Isaac Sim runtime (headless)
    try:
        from isaacsim.simulation_app import SimulationApp  # type: ignore
    except Exception as e:  # nosec - provide actionable hint
        print(f"[error] Unable to import SimulationApp. Did you source scripts/activate_isaacsim_env.sh?\n{e}")
        return 2

    trace("run:simulation_app:init")
    app = SimulationApp({"headless": True})
    trace("run:simulation_app:ready")
    try:
        # 3) Open a new stage in omni.usd context
        trace("run:stage:new_stage:start")
        import omni.usd as ou
        from pxr import Sdf, Usd

        ctx = ou.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        trace(f"run:stage:new_stage:done stage={bool(stage)}")

        # Define a world prim to keep importer happy; we'll rename/move later as needed
        world = stage.DefinePrim(Sdf.Path("/World"))
        stage.SetDefaultPrim(world)
        trace("run:stage:world_defined")

        # 4) Import URDF onto the open stage
        trace(f"run:import:start path={urdf_path}")
        reported_path = import_urdf_into_open_stage(urdf_path)
        trace(f"run:import:done reported={reported_path}")
        print(f"[import] reported articulation path: {reported_path}")

        # 5) Move to requested root prim path if different
        if args.root_prim:
            if reported_path != args.root_prim:
                trace(f"run:move:start {reported_path}->{args.root_prim}")
                moved = move_or_rename_prim(stage, reported_path, args.root_prim)
                print(f"[move] {reported_path} -> {args.root_prim}: {moved}")
                trace(f"run:move:done moved={moved}")
            root_path = args.root_prim
        else:
            root_path = reported_path
        trace(f"run:root_path={root_path}")

        # 6) Optionally clean stray "/<root>/world" subtree if present and safe
        stray = f"{root_path}/world"
        if args.clean_stray_world:
            trace(f"run:clean:start {stray}")
            deleted = delete_prim_if_safe(stage, stray)
            if deleted:
                print(f"[clean] deleted stray subtree: {stray}")
            else:
                if stage.GetPrimAtPath(stray) and stage.GetPrimAtPath(stray).IsValid():
                    print(f"[clean] retained subtree (contains joints or not found safe): {stray}")
            trace(f"run:clean:done deleted={deleted}")

        # 7) Set defaultPrim to the model root
        trace("run:set_default_prim:start")
        set_default_prim(stage, root_path)
        trace("run:set_default_prim:done")

        # 8) Save to USD path
        trace(f"run:save:start {args.usd_out}")
        os.makedirs(os.path.dirname(args.usd_out), exist_ok=True)
        stage.GetRootLayer().Export(args.usd_out)
        print(f"[save] USD written: {args.usd_out}")
        trace("run:save:done")

        # 9) Summarize
        trace("run:summarize:start")
        summary = summarize(stage, root_path)
        print("=== Summary ===")
        for k, v in summary.items():
            print(f"{k}: {v}")
        trace("run:summarize:done")

        # Basic acceptance checks
        status = 0
        if summary.get("defaultPrim") != root_path:
            print(f"[warn] defaultPrim is {summary.get('defaultPrim')} (expected {root_path})")
        if summary.get("strayWorldExists"):
            print(f"[warn] Stray '{root_path}/world' subtree still exists")
        return status

    finally:
        app.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--xacro", default="assets/roarm_m3/urdf/roarm_m3.xacro", help="Input XACRO file path")
    ap.add_argument("--urdf-out", default="assets/roarm_m3/urdf/roarm_m3.generated.urdf", help="Expanded URDF output path")
    ap.add_argument("--urdf-fallback", default="", help="Optional fallback URDF if xacro unavailable")
    ap.add_argument("--usd-out", default="assets/roarm_m3/urdf/roarm_m3.generated.usd", help="Output USD path")
    ap.add_argument("--root-prim", default="/roarm_m3", help="Desired articulation root prim path")
    ap.add_argument("--clean-stray-world", action="store_true", help="Delete '<root>/world' subtree if it contains no joints")
    args, unknown = ap.parse_known_args()
    if unknown:
        print(f"[args] Ignoring unknown arguments: {unknown}")
    # Prevent SimulationApp from seeing our CLI options
    sys.argv = sys.argv[:1]
    try:
        return run(args)
    except Exception as e:
        print(f"[error] {e}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
