"""Physics randomization application utilities for Isaac Sim.

This module maps sampled domain randomization values onto (potential) Isaac Sim
physics parameters. It is designed to run safely in contexts where Isaac APIs
may not be present – all Isaac calls are guarded and failures captured in a
structured report for downstream logging/analysis.

Reporting Contract (returned dict fields):
    applied_physics / applied_robot / applied_env: bool (section present)
    success_keys: list[str] – fully applied leaf keys
    failed_keys: list[str] – keys attempted but raised exception
    skipped_keys: list[str] – keys intentionally ignored (unsupported)
    value_map: {key: value or "error:<msg>"}
    ranges: {section: {key: (min,max) or value}} (simple echo for analytics)
    *_applied_detail: raw per-section applied detail (for backward compatibility)
    errors: list[str] – high‑level errors

Graceful Degrade Philosophy:
    If Isaac world / context objects are missing, we still return a structured
    report with applied_* = False and populate skipped_keys accordingly so that
    training pipelines remain reproducible and analyzable.
"""
from __future__ import annotations
from typing import Dict, Any, List, Optional
import logging

logger = logging.getLogger(__name__)

# Helper: push error string with bounded length (avoid flooding logs)
def _short_error(e: Exception, limit: int = 160) -> str:
    msg = f"{type(e).__name__}:{e}".replace("\n", " ")
    return msg if len(msg) <= limit else msg[: limit - 3] + "..."

# --- Mapping 설계 ---
# 외부 sample 구조 예시:
# sample = {
#   'physics': {'solver_position_iterations': 8, 'solver_velocity_iterations': 1},
#   'robot': {'joint_damping_scale': 1.2, 'link_friction_scale': 0.9},
#   'environment': {'ground_friction': 0.7}
# }

# Isaac 연동 시 예상 적용 경로 (주석 기준):
# world.get_physics_context().set_solver_position_iteration_count(value)
# world.get_physics_context().set_solver_velocity_iteration_count(value)
# articulation / joint drive damping 조정: drive.set_damping(original * scale)
# link collider material friction: prim.GetAttribute('physxMaterial:staticFriction').Set(value)

PHYSICS_KEY_MAPPING = {
    "solver_position_iterations": "solver_position_iteration_count",
    "solver_velocity_iterations": "solver_velocity_iteration_count",
}

def _apply_physics_params(world: Any, cfg: Dict[str, Any], report: Dict[str, Any]):  # pragma: no cover
    ctx = getattr(world, "get_physics_context", lambda: None)()
    if ctx is None:
        report["errors"].append("no_physics_context")
        report["skipped_keys"].extend(list(cfg.keys()))
        return
    detail = {}
    for k, v in cfg.items():
        std_key = PHYSICS_KEY_MAPPING.get(k, k)
        try:
            if std_key == "solver_position_iteration_count":
                set_fn = getattr(ctx, "set_solver_position_iteration_count", None)
                if callable(set_fn):
                    set_fn(int(v))
                    detail[k] = v
                    report["success_keys"].append(k)
                else:
                    report["skipped_keys"].append(k)
            elif std_key == "solver_velocity_iteration_count":
                set_fn = getattr(ctx, "set_solver_velocity_iteration_count", None)
                if callable(set_fn):
                    set_fn(int(v))
                    detail[k] = v
                    report["success_keys"].append(k)
                else:
                    report["skipped_keys"].append(k)
            else:
                # Unknown physics key
                report["skipped_keys"].append(k)
        except Exception as e:  # noqa
            emsg = _short_error(e)
            detail[k] = f"error:{emsg}"
            report["failed_keys"].append(k)
            report["errors"].append(f"{k}:{emsg}")
    report["physics_applied_detail"] = detail

def _locate_articulation(world: Any) -> Optional[Any]:  # pragma: no cover
    """Best-effort articulation retrieval from world.scene.

    This is heuristic: searches for objects whose attribute names suggest an articulation view.
    Returns first match or None.
    """
    scene = getattr(world, "scene", None)
    if scene is None:
        return None
    # Direct accessor pattern
    name_candidates = []
    # Some Isaac scenes allow iteration via get_object / get_objects
    get_objects = getattr(scene, "get_objects", None)
    if callable(get_objects):
        try:
            objs = get_objects()
            # objs could be dict-like
            if isinstance(objs, dict):
                for k, v in objs.items():
                    name_candidates.append((k, v))
            elif isinstance(objs, list):
                for v in objs:
                    name_candidates.append((getattr(v, "name", ""), v))
        except Exception:  # noqa
            pass
    # Fallback: common attribute names
    for attr in dir(scene):
        if "articulation" in attr.lower() or "roarm_view" in attr.lower():
            try:
                candidate = getattr(scene, attr)
                if candidate is not None:
                    return candidate
            except Exception:  # noqa
                continue
    # Choose first candidate heuristically if any
    for name, obj in name_candidates:
        if obj is not None and ("articulation" in (name or "").lower() or "roarm" in (name or "").lower()):
            return obj
    return None


def _apply_robot_params(world: Any, cfg: Dict[str, Any], report: Dict[str, Any], articulation: Any | None = None):  # pragma: no cover
    # Placeholder articulation traversal pattern (future real Isaac API calls)
    # Expected keys (example): joint_damping_scale, link_friction_scale
    detail = {}
    if articulation is None:
        articulation = _locate_articulation(world)

    for k, v in cfg.items():
        try:
            if k == "joint_damping_scale" and articulation is not None:
                scale = float(v)
                # Try multiple getter signatures
                getters = [
                    "get_joint_damping",
                    "get_damping",
                ]
                setters = [
                    "set_joint_damping",
                    "set_damping",
                ]
                damping_vals = None
                for g in getters:
                    fn = getattr(articulation, g, None)
                    if callable(fn):
                        try:
                            damping_vals = fn()
                            break
                        except Exception:  # noqa
                            continue
                if damping_vals is not None:
                    # Convert to list
                    if hasattr(damping_vals, "tolist"):
                        damping_list = damping_vals.tolist()
                    else:
                        damping_list = list(damping_vals)
                    new_vals = [x * scale for x in damping_list]
                    applied = False
                    for s in setters:
                        sfn = getattr(articulation, s, None)
                        if callable(sfn):
                            try:
                                sfn(new_vals)
                                applied = True
                                break
                            except Exception:  # noqa
                                continue
                    if applied:
                        detail[k] = {"scale": scale, "count": len(new_vals)}
                        report["success_keys"].append(k)
                        continue
                # If we reach here, treat as skipped
                detail[k] = {"scale": float(v), "note": "damping_get_or_set_unavailable"}
                report["skipped_keys"].append(k)
            elif k == "link_friction_scale" and articulation is not None:
                scale = float(v)
                updated_links = 0
                # Attempt to iterate links – heuristic method names
                link_getters = [
                    getattr(articulation, "get_links", None),
                    getattr(articulation, "get_link_bodies", None),
                ]
                links = None
                for g in link_getters:
                    if callable(g):
                        try:
                            links = g()
                            if links:
                                break
                        except Exception:  # noqa
                            continue
                if links is not None:
                    for link in links:  # pragma: no cover (needs Isaac or test dummy)
                        try:
                            get_mat = getattr(link, "get_material", None)
                            mat = get_mat() if callable(get_mat) else None
                            if mat is None:
                                continue
                            # Fetch original friction
                            get_sf = getattr(mat, "get_static_friction", None)
                            get_df = getattr(mat, "get_dynamic_friction", None)
                            sf = get_sf() if callable(get_sf) else None
                            df = get_df() if callable(get_df) else None
                            if sf is None or df is None:
                                continue
                            set_sf = getattr(mat, "set_static_friction", None)
                            set_df = getattr(mat, "set_dynamic_friction", None)
                            if callable(set_sf) and callable(set_df):
                                set_sf(sf * scale)
                                set_df(df * scale)
                                updated_links += 1
                        except Exception:  # noqa
                            continue
                if updated_links > 0:
                    detail[k] = {"scale": scale, "links": updated_links}
                    report["success_keys"].append(k)
                else:
                    detail[k] = {"scale": scale, "links": 0, "note": "no_link_materials"}
                    report["skipped_keys"].append(k)
            else:
                # Unknown robot-level key
                detail[k] = v
                report["skipped_keys"].append(k)
        except Exception as e:  # noqa
            emsg = _short_error(e)
            detail[k] = f"error:{emsg}"
            report["failed_keys"].append(k)
            report["errors"].append(f"{k}:{emsg}")
        else:
            # remove from skipped & add to success once real implementation lands
            pass
    report["robot_applied_detail"] = detail

def _apply_env_params(world: Any, cfg: Dict[str, Any], report: Dict[str, Any]):  # pragma: no cover
    # Keys: ground_friction (scale or absolute), ambient_temperature (placeholder)
    detail = {}
    ctx = getattr(world, "get_physics_context", lambda: None)()
    for k, v in cfg.items():
        try:
            if k == "ground_friction" and ctx is not None:
                # Attempt to set global material friction if API exposed.
                set_gf = None
                candidates = [
                    "set_material_static_friction",
                    "set_static_friction",
                ]
                for cname in candidates:
                    fn = getattr(ctx, cname, None)
                    if callable(fn):
                        set_gf = fn
                        break
                if callable(set_gf):
                    try:
                        set_gf(float(v))
                        detail[k] = {"value": float(v)}
                        report["success_keys"].append(k)
                        continue
                    except Exception:  # noqa
                        pass
                detail[k] = {"value": float(v), "note": "no_global_friction_api"}
                report["skipped_keys"].append(k)
            else:
                detail[k] = v
                report["skipped_keys"].append(k)
        except Exception as e:  # noqa
            emsg = _short_error(e)
            detail[k] = f"error:{emsg}"
            report["failed_keys"].append(k)
            report["errors"].append(f"{k}:{emsg}")
    report["env_applied_detail"] = detail


def apply_physics_randomization(world: Any, sample: Dict[str, Any], articulation: Any | None = None):  # world is Isaac World (placeholder)
    physics_cfg = sample.get("physics", {}) or {}
    robot_cfg = sample.get("robot", {}) or {}
    env_cfg = sample.get("environment", {}) or {}

    report: Dict[str, Any] = {
        "applied_physics": bool(physics_cfg),
        "applied_robot": bool(robot_cfg),
        "applied_env": bool(env_cfg),
        "success_keys": [],
        "failed_keys": [],
        "skipped_keys": [],
        "value_map": {},
        "ranges": {},
        "errors": [],
    }

    # Basic value_map & ranges echo for analytics (range detection if tuple-like)
    def _collect_ranges(section: str, cfg: Dict[str, Any]):
        r = {}
        for k, v in cfg.items():
            if isinstance(v, (list, tuple)) and len(v) == 2 and all(isinstance(x, (int, float)) for x in v):
                r[k] = (min(v), max(v))
            else:
                r[k] = v
        if r:
            report["ranges"][section] = r

    _collect_ranges("physics", physics_cfg)
    _collect_ranges("robot", robot_cfg)
    _collect_ranges("environment", env_cfg)

    if physics_cfg:
        logger.info(f"[PhysicsRand] physics params -> {physics_cfg}")
        _apply_physics_params(world, physics_cfg, report)
        report["value_map"].update({k: physics_cfg[k] for k in physics_cfg})
    if robot_cfg:
        logger.info(f"[PhysicsRand] robot params -> {robot_cfg}")
        _apply_robot_params(world, robot_cfg, report, articulation=articulation)
        report["value_map"].update({k: robot_cfg[k] for k in robot_cfg})
    if env_cfg:
        logger.info(f"[PhysicsRand] env params -> {env_cfg}")
        _apply_env_params(world, env_cfg, report)
        report["value_map"].update({k: env_cfg[k] for k in env_cfg})

    # Summary counts for quick glance
    report["success_count"] = len(report["success_keys"])
    report["failed_count"] = len(report["failed_keys"])
    report["skipped_count"] = len(report["skipped_keys"])
    return report
