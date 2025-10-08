"""Isaac API Adapter Skeleton

Provides a thin abstraction layer so the rest of the codebase can call
stable function names even if Isaac Sim Python APIs shift between versions.

All functions are safe to import/run without Isaac installed; they either
return None / empty structures or raise explicit RuntimeError when a hard
requirement is unmet (callers can decide policy).
"""
from __future__ import annotations
from typing import Any, Optional, List, Dict

try:  # pragma: no cover
    import omni.isaac.kit  # type: ignore
    from omni.isaac.core import World  # type: ignore
    ISAAC_AVAILABLE = True
except Exception:  # noqa
    ISAAC_AVAILABLE = False

__all__ = [
    "ISAAC_AVAILABLE",
    "create_world",
    "find_articulation",
    "get_joint_state",
    "apply_joint_delta",
]

def create_world(**world_kwargs) -> Any:
    """Create a World instance if Isaac available, else return None.
    world_kwargs forwarded to World constructor.
    """
    if not ISAAC_AVAILABLE:
        return None
    try:  # pragma: no cover
        return World(**world_kwargs)
    except Exception:  # noqa
        return None

def find_articulation(world: Any, prim_path: str) -> Any:
    """Best-effort articulation retrieval.
    Returns articulation handle or None.
    """
    if not (ISAAC_AVAILABLE and world):
        return None
    # Heuristic approach (can be specialized per version)
    try:  # pragma: no cover
        scene = getattr(world, "scene", None)
        if scene is None:
            return None
        # Common pattern: scene.get_object(prim_path) or ArticulationView builder
        get_obj = getattr(scene, "get_object", None)
        if callable(get_obj):
            try:
                obj = get_obj(prim_path)
                if obj is not None:
                    return obj
            except Exception:  # noqa
                pass
        # Fallback: iterate object map if available
        get_objects = getattr(scene, "get_objects", None)
        if callable(get_objects):
            try:
                objs = get_objects()
                if isinstance(objs, dict) and prim_path in objs:
                    return objs[prim_path]
            except Exception:  # noqa
                pass
    except Exception:  # noqa
        return None
    return None

def get_joint_state(articulation: Any) -> Dict[str, List[float]]:
    """Return joint positions / velocities lists (maybe empty).
    Robust to missing Isaac, returns empty if not available.
    """
    out = {"positions": [], "velocities": []}
    if articulation is None:
        return out
    try:  # pragma: no cover
        # Try various getter names across versions
        pos_getters = ["get_joint_positions", "get_positions"]
        vel_getters = ["get_joint_velocities", "get_velocities"]
        for g in pos_getters:
            fn = getattr(articulation, g, None)
            if callable(fn):
                try:
                    pos = fn()
                    if pos is not None:
                        out["positions"] = list(pos.tolist() if hasattr(pos, "tolist") else pos)
                        break
                except Exception:  # noqa
                    continue
        for g in vel_getters:
            fn = getattr(articulation, g, None)
            if callable(fn):
                try:
                    vel = fn()
                    if vel is not None:
                        out["velocities"] = list(vel.tolist() if hasattr(vel, "tolist") else vel)
                        break
                except Exception:  # noqa
                    continue
    except Exception:  # noqa
        return out
    return out

def apply_joint_delta(articulation: Any, deltas, lower=None, upper=None) -> Dict[str, Any]:
    """Placeholder delta application.
    Without Isaac, returns a simulated metadata dict.
    With Isaac present, this would set drive targets (future extension).
    """
    import numpy as np
    deltas = np.array(deltas, dtype=float)
    lower = np.array(lower if lower is not None else [-1.0] * deltas.size, dtype=float)
    upper = np.array(upper if upper is not None else [1.0] * deltas.size, dtype=float)
    clipped = np.clip(deltas, lower, upper)
    if articulation is None or not ISAAC_AVAILABLE:
        return {"applied": False, "delta": deltas.tolist(), "clipped": clipped.tolist(), "note": "no_articulation"}
    # TODO: integrate real articulation drive target logic in future
    return {"applied": True, "delta": deltas.tolist(), "clipped": clipped.tolist()}
