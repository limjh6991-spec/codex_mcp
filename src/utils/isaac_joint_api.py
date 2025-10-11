"""Isaac Joint API Helper (Placeholder)

실제 Isaac Sim API 연동 시 articulation/joint 접근 로직을 캡슐화.
현재는 스켈레톤: 실제 환경에서 omni.isaac.core.articulations API 사용.
"""
from __future__ import annotations
from typing import List, Dict, Any, Optional, Sequence
import logging
import time
import json

try:  # pragma: no cover
    import omni.isaac.kit  # type: ignore
    ISAAC_AVAILABLE = True
except Exception:  # noqa
    ISAAC_AVAILABLE = False

class JointAPI:
    def __init__(self, articulation=None):  # articulation 핸들은 실제 Isaac 로딩 후 주입 예정
        self._articulation = articulation
        self._prim_path: str | None = None
        self._last_positions: List[float] = []
        self._last_velocities: List[float] = []
        self._log = logging.getLogger(__name__)

    def attach(self, prim_path: str, articulation=None, world=None, tries: int = 1, retry_interval: float = 0.5):
        """Attach articulation handle.

        Parameters
        ----------
        prim_path : str
            Target articulation root prim path (e.g. /World/roarm)
        articulation : optional
            Pre-constructed articulation/view handle (test or dependency injection)
        world : optional
            Isaac World 인스턴스. 제공되면 Isaac 가용 시 ArticulationView 생성 시도.

        Returns
        -------
        dict : {attached: bool, prim_path: str, has_handle: bool, created: bool}
        """
        self._prim_path = prim_path
        created = False
        error_code: str | None = None
        attempt = 0

        def _prim_exists() -> bool:
            if not ISAAC_AVAILABLE or world is None:
                return False
            try:  # pragma: no cover
                stage = getattr(world, "stage", None)
                if stage is None:
                    return False
                get_prim = getattr(stage, "GetPrimAtPath", None)
                if callable(get_prim):
                    prim = get_prim(prim_path)
                    return prim is not None and prim.IsValid()
            except Exception:
                return False
            return False

        if articulation is not None:
            self._articulation = articulation
            created = True
        elif ISAAC_AVAILABLE and world is not None:
            if tries < 1:
                tries = 1
            while attempt < tries and self._articulation is None:
                attempt += 1
                try:  # pragma: no cover
                    if not _prim_exists():
                        error_code = "PRIM_NOT_FOUND"
                    else:
                        # If a different articulation root exists under this path, prefer it
                        attach_path = prim_path
                        try:
                            stage = getattr(world, "stage", None)
                            p = stage.GetPrimAtPath(prim_path) if stage is not None else None
                            # Search for UsdPhysics.ArticulationRootAPI
                            if p is not None and p.IsValid():
                                try:
                                    from pxr import UsdPhysics  # type: ignore
                                    if not p.HasAPI(UsdPhysics.ArticulationRootAPI):
                                        for sub in stage.Traverse():
                                            sp = sub.GetPath().pathString
                                            if not sp.startswith(prim_path):
                                                continue
                                            try:
                                                if sub.HasAPI(UsdPhysics.ArticulationRootAPI):
                                                    attach_path = sp
                                                    break
                                            except Exception:
                                                continue
                                except Exception:
                                    pass
                        except Exception:
                            attach_path = prim_path
                        # Prefer 5.0+ API if available (try multiple import paths)
                        view = None
                        import_error_msgs = []
                        for imp in (
                            "from isaacsim.core.prims import Articulation as _Art",
                            "from isaacsim.core.api.prims import Articulation as _Art",
                            "from isaacsim.core.api.prims.articulation import Articulation as _Art",
                        ):
                            try:
                                _g = {}
                                exec(imp, _g, _g)
                                _Art = _g.get("_Art")
                                if _Art is not None:
                                    view = _Art(prim_path=attach_path, name=f"roarm_art_{attach_path.split('/')[-1]}")
                                    # Some builds need explicit initialize
                                    if hasattr(view, "initialize"):
                                        try:
                                            view.initialize()
                                        except Exception:
                                            pass
                                    break
                            except Exception as e:  # noqa
                                import_error_msgs.append(f"{imp} -> {e}")
                                continue
                        # Fallback to legacy ArticulationView (try exact and glob patterns)
                        if view is None:
                            try:
                                from omni.isaac.core.articulations import ArticulationView  # type: ignore
                                # Try exact path first
                                try:
                                    view = ArticulationView(prim_paths_expr=attach_path, name=f"roarm_view_{attach_path.split('/')[-1]}")
                                except Exception:
                                    view = None
                                # If failed, try glob under the prim
                                if view is None:
                                    try:
                                        pat = f"{attach_path}/*"
                                        view = ArticulationView(prim_paths_expr=pat, name=f"roarm_view_{attach_path.split('/')[-1]}_g")
                                    except Exception:
                                        view = None
                                if view is not None and hasattr(world, "scene") and hasattr(world.scene, "add"):
                                    try:
                                        world.scene.add(view)
                                    except Exception:
                                        pass
                                # Some versions require explicit initialize
                                if view is not None and hasattr(view, "initialize"):
                                    try:
                                        view.initialize()
                                    except Exception:
                                        pass
                            except Exception as e:  # noqa
                                import_error_msgs.append(f"ArticulationView import/create -> {e}")

                        # Last resort: try to create an Articulation from prim directly via isaacsim.core.utils if available
                        if view is None:
                            try:
                                from isaacsim.core.utils.stage import get_current_stage  # type: ignore
                                stage = get_current_stage()
                                prim = None
                                if stage is not None:
                                    prim = stage.GetPrimAtPath(prim_path)
                                if prim is not None and prim.IsValid():
                                    # Some 5.0 builds allow wrapping an existing prim via Articulation handle
                                    for imp in (
                                        "from isaacsim.core.prims import Articulation as _Art",
                                        "from isaacsim.core.api.prims import Articulation as _Art",
                                        "from isaacsim.core.api.prims.articulation import Articulation as _Art",
                                    ):
                                        try:
                                            _g = {}
                                            exec(imp, _g, _g)
                                            _Art = _g.get("_Art")
                                            if _Art is not None:
                                                view = _Art(prim_path=prim_path, name=f"roarm_art_{prim_path.split('/')[-1]}_late")
                                                if hasattr(view, "initialize"):
                                                    try:
                                                        view.initialize()
                                                    except Exception:
                                                        pass
                                                break
                                        except Exception:
                                            continue
                            except Exception as e:  # noqa
                                import_error_msgs.append(f"last_resort_wrap -> {e}")

                        self._articulation = view
                        created = bool(view is not None)
                        error_code = None if self._articulation is not None else (error_code or ("ATTACH_NO_BACKEND:" + ";".join(import_error_msgs[-2:])))
                        # If attached, break; else retry
                        if self._articulation is not None:
                            break
                except Exception as e:  # noqa
                    error_code = "ATTACH_FAIL"
                    self._log.debug(json.dumps({
                        "event": "attach_exception",
                        "attempt": attempt,
                        "prim_path": prim_path,
                        "error": str(e)
                    }))
                if self._articulation is None and attempt < tries:
                    time.sleep(retry_interval)
        else:
            error_code = "ISAAC_UNAVAILABLE"

        # Diagnostics: count dofs if possible
        dof_count = None
        try:
            if self._articulation is not None:
                for attr in ("num_dof", "num_dofs", "dof_count"):
                    if hasattr(self._articulation, attr):
                        val = getattr(self._articulation, attr)
                        dof_count = int(val() if callable(val) else val)
                        break
                if dof_count is None:
                    for meth in ("get_dof_count",):
                        fn = getattr(self._articulation, meth, None)
                        if callable(fn):
                            try:
                                dof_count = int(fn())
                                break
                            except Exception:
                                pass
                if dof_count is None:
                    gdn = getattr(self._articulation, "get_dof_names", None)
                    if callable(gdn):
                        try:
                            dof_count = len(list(gdn()))
                        except Exception:
                            pass
        except Exception:
            pass

        result = {
            "attached": self._articulation is not None,
            "prim_path": prim_path,
            "has_handle": self._articulation is not None,
            "created": created,
            "attempts": attempt if attempt else 1,
            "error_code": error_code,
            "dof_count": dof_count,
        }
        self._log.debug(json.dumps({"event": "attach_result", **result}))
        return result

    def is_attached(self) -> bool:
        return self._prim_path is not None

    def list_joints(self, only_movable: bool = True) -> List[str]:
        """Return list of joint names.

        Isaac 사용 가능 & articulation 확보된 경우 실제 joint name 목록 반환.
        현재는 Isaac 미연동/미탐색 상태이므로 빈 리스트 또는 placeholder.
        """
        if not ISAAC_AVAILABLE or self._articulation is None:
            return []
        # 실제 Isaac articulation view가 제공하는 API 버전에 따라 메서드 이름 상이할 수 있음.
        # 대표 패턴: get_joints() → list[Joint] with .name attribute
        try:  # pragma: no cover - Isaac 환경 필요
            # 1) Newer API often exposes DOF names
            for meth in ("get_joint_names", "get_dof_names"):
                fn = getattr(self._articulation, meth, None)
                if callable(fn):
                    try:
                        names = fn()
                        names_list = list(names)
                        if names_list:
                            # 대부분의 API는 movable DOF만 반환
                            return names_list
                    except Exception:
                        pass
            # 2) Legacy: get_joints() returning objects with name or get_name
            get_joints = getattr(self._articulation, "get_joints", None)
            if callable(get_joints):
                try:
                    objs = get_joints()
                    names = []
                    for j in objs:
                        name = getattr(j, "name", None)
                        if not name:
                            get_name = getattr(j, "get_name", None)
                            if callable(get_name):
                                name = get_name()
                        if name:
                            names.append(str(name))
                    if names:
                        return names
                except Exception:
                    pass
            # 3) Fallback: scan USD stage under prim for physics joints
            try:
                from omni.usd import get_context  # type: ignore
                ctx = get_context()
                stage = ctx.get_stage() if ctx else None
                if stage is not None and self._prim_path:
                    prim_path = str(self._prim_path)
                    try:
                        from pxr import UsdPhysics  # type: ignore
                    except Exception:
                        UsdPhysics = None  # type: ignore
                    joint_names: List[str] = []
                    if UsdPhysics is not None:
                        def is_joint(p):
                            try:
                                for cls_name in ("RevoluteJoint", "PrismaticJoint", "SphericalJoint", "DistanceJoint", "FixedJoint"):
                                    cls = getattr(UsdPhysics, cls_name, None)
                                    if cls and p.IsA(cls):
                                        return True
                            except Exception:
                                return False
                            return False
                        it = stage.Traverse()
                        for p in it:
                            path_str = p.GetPath().pathString
                            if not path_str.startswith(prim_path + "/"):
                                continue
                            if is_joint(p):
                                # movable 필터: Revolute/Prismatic만 포함
                                if only_movable:
                                    try:
                                        if p.IsA(getattr(UsdPhysics, "RevoluteJoint", None)) or p.IsA(getattr(UsdPhysics, "PrismaticJoint", None)):
                                            joint_names.append(path_str.split("/")[-1])
                                    except Exception:
                                        # 타입 확인 실패 시 일단 추가
                                        joint_names.append(path_str.split("/")[-1])
                                else:
                                    joint_names.append(path_str.split("/")[-1])
                    if joint_names:
                        return joint_names
            except Exception:
                pass
        except Exception:
            return []
        return []

    # --- USD Traversal Helpers ---
    def _scan_usd_joints(self) -> List[Dict[str, Any]]:
        """Scan USD under the articulation prim and collect joint infos.

        Returns list of dicts: {name, path, type, lower, upper}
        """
        infos: List[Dict[str, Any]] = []
        try:  # pragma: no cover - Isaac environment required
            from omni.usd import get_context  # type: ignore
            from pxr import UsdPhysics  # type: ignore
            ctx = get_context()
            stage = ctx.get_stage() if ctx else None
            if stage is None or not self._prim_path:
                return infos
            root = str(self._prim_path)
            it = stage.Traverse()
            for p in it:
                path_str = p.GetPath().pathString
                if not path_str.startswith(root + "/"):
                    continue
                jtype = None
                if p.IsA(getattr(UsdPhysics, "RevoluteJoint", None)):
                    jtype = "revolute"
                elif p.IsA(getattr(UsdPhysics, "PrismaticJoint", None)):
                    jtype = "prismatic"
                elif p.IsA(getattr(UsdPhysics, "FixedJoint", None)):
                    jtype = "fixed"
                elif p.IsA(getattr(UsdPhysics, "SphericalJoint", None)):
                    jtype = "spherical"
                elif p.IsA(getattr(UsdPhysics, "DistanceJoint", None)):
                    jtype = "distance"
                else:
                    continue
                name = path_str.split("/")[-1]
                lower = None
                upper = None
                try:
                    # Try strongly-typed accessors if available
                    if jtype == "revolute":
                        api = UsdPhysics.RevoluteJoint.Get(stage, p.GetPath())
                        if api:
                            try:
                                lower = api.GetLowerLimitAttr().Get()
                                upper = api.GetUpperLimitAttr().Get()
                            except Exception:
                                pass
                    elif jtype == "prismatic":
                        api = UsdPhysics.PrismaticJoint.Get(stage, p.GetPath())
                        if api:
                            try:
                                lower = api.GetLowerLimitAttr().Get()
                                upper = api.GetUpperLimitAttr().Get()
                            except Exception:
                                pass
                except Exception:
                    pass
                infos.append({
                    "name": name,
                    "path": path_str,
                    "type": jtype,
                    "lower": lower,
                    "upper": upper,
                })
        except Exception:
            return infos
        return infos

    def get_joint_limits_by_name(self, only_movable: bool = True) -> Dict[str, Any]:
        """Return joint limits from USD, keyed by joint name.

        For movable joints (revolute/prismatic), returns dict name -> {lower, upper}.
        Values may be None if not specified in USD.
        """
        result: Dict[str, Any] = {}
        infos = self._scan_usd_joints()
        for info in infos:
            if only_movable and info.get("type") not in ("revolute", "prismatic"):
                continue
            nm = str(info.get("name"))
            result[nm] = {"lower": info.get("lower"), "upper": info.get("upper")}
        return result

    def get_dof_limits(self, normalize_revolute_to_radians: bool = True) -> Dict[str, Any]:
        """Return lower/upper lists aligned to list_joints(only_movable=True).

        If normalize_revolute_to_radians is True, attempt to detect degree-authored
        revolute limits and convert to radians.
        """
        import math
        names = self.list_joints(only_movable=True)
        lims = self.get_joint_limits_by_name(only_movable=True)
        # also collect joint types for conversion logic
        type_map: Dict[str, str] = {info.get("name"): info.get("type") for info in self._scan_usd_joints()}
        lower: List[Optional[float]] = []
        upper: List[Optional[float]] = []
        converted_any = False
        for nm in names:
            li = lims.get(nm, {})
            lo = li.get("lower")
            hi = li.get("upper")
            jtype = type_map.get(nm)
            if normalize_revolute_to_radians and jtype == "revolute" and lo is not None and hi is not None:
                # Heuristic: if any bound exceeds ~pi*1.2 assume degrees
                try:
                    if abs(float(lo)) > math.pi * 1.2 or abs(float(hi)) > math.pi * 1.2:
                        lo = math.radians(float(lo))
                        hi = math.radians(float(hi))
                        converted_any = True
                except Exception:
                    pass
            lower.append(lo)
            upper.append(hi)
        return {"names": names, "lower": lower, "upper": upper, "types": [type_map.get(n) for n in names], "converted": converted_any}

    def get_state(self) -> Dict[str, Any]:
        """Fetch joint state (positions & velocities).

        Returns
        -------
        dict
            {"positions": List[float], "velocities": List[float]} 최소 키 보장.

        Fallback: Isaac API 미사용 혹은 articulation 없음 → 빈 리스트.
        """
        if not ISAAC_AVAILABLE or self._articulation is None:  # graceful fallback
            return {"positions": self._last_positions, "velocities": self._last_velocities}
        # 실제 Isaac articulation view 사용 시도
        try:  # pragma: no cover - Isaac 환경 필요
            # Prefer joint_*; fallback to dof_*
            get_pos = getattr(self._articulation, "get_joint_positions", None)
            get_vel = getattr(self._articulation, "get_joint_velocities", None)
            if not callable(get_pos):
                get_pos = getattr(self._articulation, "get_dof_positions", None)
            if not callable(get_vel):
                get_vel = getattr(self._articulation, "get_dof_velocities", None)
            if callable(get_pos):
                q = get_pos()
                if hasattr(q, "tolist"):
                    q = q.tolist()
            else:
                q = []
            if callable(get_vel):
                dq = get_vel()
                if hasattr(dq, "tolist"):
                    dq = dq.tolist()
            else:
                dq = []
            # 형식 보정: 리스트 보장
            if not isinstance(q, list):
                try:
                    q = list(q)
                except Exception:
                    q = []
            if not isinstance(dq, list):
                try:
                    dq = list(dq)
                except Exception:
                    dq = []
            self._last_positions = q
            self._last_velocities = dq
            return {"positions": q, "velocities": dq}
        except Exception as e:  # 예외 시 마지막 성공 값 반환 (없으면 빈 리스트)
            self._log.debug(json.dumps({
                "event": "get_state_exception",
                "prim_path": self._prim_path,
                "error": str(e)
            }))
            return {"positions": self._last_positions, "velocities": self._last_velocities}

    def apply_delta(self, deltas: Sequence[float], lower: Sequence[float] | None = None, upper: Sequence[float] | None = None, verify: bool = False):
        """Apply delta joint commands.

        Parameters
        ----------
        deltas : Sequence[float]
            Joint delta targets (same ordering as list_joints()).

        Returns
        -------
        dict
            {"applied": bool, ...메타}

        Notes
        -----
        Isaac 미가용 시 안전하게 no-op.
        실제 구현 시:
          * 현재 joint positions = articulation.get_joint_positions()
          * target = clamp(positions + deltas, lower, upper)
          * articulation.set_joint_position_targets(target)
        """
        if not ISAAC_AVAILABLE or self._articulation is None:
            return {"applied": False, "reason": "No articulation or Isaac unavailable"}
        try:  # pragma: no cover - Isaac 환경 필요
            import numpy as np  # local import to avoid cost when Isaac 미가용
            # 현재 joint positions 조회
            get_pos = getattr(self._articulation, "get_joint_positions", None)
            if not callable(get_pos):
                get_pos = getattr(self._articulation, "get_dof_positions", None)
            cur = None
            if callable(get_pos):
                cur = get_pos()
            if cur is None:
                cur = np.zeros(len(deltas), dtype=float)
            if hasattr(cur, "tolist"):
                cur = cur.tolist()
            cur_arr = np.asarray(cur, dtype=float)
            delta_arr = np.asarray(deltas, dtype=float)
            target = cur_arr + delta_arr
            saturated = False
            if lower is not None and upper is not None:
                lower_arr = np.asarray(lower, dtype=float)
                upper_arr = np.asarray(upper, dtype=float)
                pre = target.copy()
                target = np.minimum(np.maximum(target, lower_arr), upper_arr)
                saturated = bool(np.any(pre != target))
            # 실제 Isaac 호출 (예외 안전)
            set_targets = getattr(self._articulation, "set_joint_position_targets", None)
            if not callable(set_targets):
                set_targets = getattr(self._articulation, "set_dof_position_targets", None)
            if callable(set_targets):
                set_targets(target)
            verified: Dict[str, Any] = {}
            if verify:
                try:
                    # 재확인: set 후 포지션 읽기
                    new_state = self.get_state()
                    verified["verified_positions"] = new_state.get("positions", [])
                except Exception as ve:  # noqa
                    verified["verify_error"] = str(ve)
            result = {
                "applied": True,
                "count": len(delta_arr),
                "saturated": saturated,
                "target": target.tolist(),
                **verified,
            }
            self._log.debug(json.dumps({"event": "apply_delta", **{k: v for k, v in result.items() if k != 'target'}}))
            return result
        except Exception as e:  # pragma: no cover
            err = {"applied": False, "error": str(e)}
            self._log.debug(json.dumps({"event": "apply_delta_exception", **err}))
            return err

    # --- Limits Verification ---
    def verify_limits(self, positions: Sequence[float], lower: Sequence[float], upper: Sequence[float], tol: float = 1e-3) -> Dict[str, Any]:
        """Check if positions exceed (lower, upper) bounds beyond tolerance.

        Returns dict with count and indices of violations plus max deviation.
        """
        import numpy as np
        p = np.asarray(positions, dtype=float)
        lo = np.asarray(lower, dtype=float)
        hi = np.asarray(upper, dtype=float)
        below = p < (lo - tol)
        above = p > (hi + tol)
        viol_mask = below | above
        if not np.any(viol_mask):
            return {"violations": 0, "indices": [], "max_dev": 0.0}
        dev_lo = (lo - p) * below
        dev_hi = (p - hi) * above
        dev = np.maximum(dev_lo, dev_hi)
        max_dev = float(np.max(np.abs(dev)))
        idx = [int(i) for i in np.where(viol_mask)[0]]
        return {"violations": len(idx), "indices": idx, "max_dev": max_dev}
