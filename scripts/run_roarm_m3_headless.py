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
import argparse, sys, time, os, math

try:
    import carb  # type: ignore
except ImportError:  # pragma: no cover
    carb = None  # type: ignore


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def _resolve_world_api():
    """Return (WorldCls, open_stage_fn) using the newest Isaac Sim API if available."""
    try:
        from isaacsim.core.api import World as _World  # type: ignore
        from isaacsim.core.utils.stage import open_stage as _open_stage  # type: ignore[attr-defined]
        return _World, _open_stage
    except Exception:
        from omni.isaac.core import World as _World  # type: ignore
        from omni.isaac.core.utils.stage import open_stage as _open_stage  # type: ignore
        return _World, _open_stage


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

    if carb is not None:
        settings = carb.settings.get_settings()
        for ext in (
            "omni.isaac.legacy",
            "omni.isaac.sensor_legacy",
        ):
            settings.set(f"/exts/{ext}/enabled", False)

    app = ensure_app(headless=args.headless)
    # Load stage using preferred API
    World, open_stage = _resolve_world_api()
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
    dof_info = {"names": [], "lower": [], "upper": []}
    try:
        dof_info = api.get_dof_limits()
    except Exception:
        pass
    if dof_info.get("names"):
        print("[spawner] dof_count:", len(dof_info["names"]))
        print("[spawner] dof_limits.lower:", dof_info.get("lower"))
        print("[spawner] dof_limits.upper:", dof_info.get("upper"))

    lower_bounds: list[float] = []
    upper_bounds: list[float] = []
    drive_eps = 1e-3
    default_span = math.tau
    if names:
        raw_lower = list(dof_info.get("lower") or [])
        raw_upper = list(dof_info.get("upper") or [])
        for idx, _ in enumerate(names):
            lo = raw_lower[idx] if idx < len(raw_lower) and raw_lower[idx] is not None else -default_span
            hi = raw_upper[idx] if idx < len(raw_upper) and raw_upper[idx] is not None else default_span
            try:
                lo = float(lo)
            except Exception:
                lo = -default_span
            try:
                hi = float(hi)
            except Exception:
                hi = default_span
            if not math.isfinite(lo):
                lo = -default_span
            if not math.isfinite(hi):
                hi = default_span
            if hi - lo < drive_eps * 10:
                mid = (hi + lo) * 0.5
                lo = mid - default_span * 0.5
                hi = mid + default_span * 0.5
            lower_bounds.append(lo + drive_eps)
            upper_bounds.append(hi - drive_eps)

    def _safe_amplitude(index: int, base_amp: float = 0.1, scale: float = 0.45) -> float:
        if not lower_bounds or index >= len(lower_bounds) or index >= len(upper_bounds):
            return base_amp
        span = upper_bounds[index] - lower_bounds[index]
        if span <= 0:
            return base_amp
        return max(0.0, min(base_amp, span * scale))

    # Short loop
    steps = max(1, args.steps)
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
                    cmd = [0.0] * n
                    # slow sine deltas
                    phase = 2 * math.pi * (i / max(1.0, steps))
                    for k in range(min(3, n)):
                        amp = _safe_amplitude(k)
                        if amp <= 0.0:
                            continue
                        cmd[k] = amp * math.sin(phase)
                    api.apply_delta(
                        cmd,
                        lower=lower_bounds if lower_bounds else None,
                        upper=upper_bounds if upper_bounds else None,
                        verify=False,
                    )
            except Exception:
                pass
        time.sleep(0.005)

    print("[spawner] done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
