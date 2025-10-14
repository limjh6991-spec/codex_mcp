#!/usr/bin/env python3
"""Apply articulation metadata and inertial estimates to a USD stage."""

import argparse
from typing import Tuple
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf


PHYSX_ROOT_API = "PhysicsArticulationRootAPI"


def compute_box_inertia(mass: float, size: Tuple[float, float, float]) -> Tuple[float, float, float]:
    sx, sy, sz = size
    ixx = (mass * (sy * sy + sz * sz)) / 12.0
    iyy = (mass * (sx * sx + sz * sz)) / 12.0
    izz = (mass * (sx * sx + sy * sy)) / 12.0
    return ixx, iyy, izz


def approx_mass_from_bbox(size: Tuple[float, float, float], density: float, min_mass: float) -> float:
    sx, sy, sz = size
    volume = max(0.0, sx * sy * sz)
    mass = density * volume
    if mass <= 0.0:
        mass = min_mass
    return max(mass, min_mass)


def _remove_articulation_root_api(prim, stage) -> bool:
    try:
        prim.RemoveAppliedSchema(PHYSX_ROOT_API)
        return True
    except Exception:
        pass

    layer = stage.GetEditTarget().GetLayer()
    prim_spec = layer.GetPrimAtPath(prim.GetPath())
    if prim_spec is None:
        prim_spec = Sdf.CreatePrimInLayer(layer, str(prim.GetPath()))

    current = list(prim.GetAppliedSchemas())
    if PHYSX_ROOT_API in current:
        current.remove(PHYSX_ROOT_API)
        prim_spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(tuple(current)))
        return True
    return False


def normalize_articulation_root(stage: Usd.Stage, root_path: str) -> None:
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"Invalid root prim: {root_path}")

    applied = False
    if UsdPhysics.ArticulationRootAPI.CanApply(root):
        UsdPhysics.ArticulationRootAPI.Apply(root)
        applied = True

    removed = 0
    for prim in stage.Traverse():
        if prim == root:
            continue
        if UsdPhysics.ArticulationRootAPI(prim):
            if _remove_articulation_root_api(prim, stage):
                removed += 1
    if applied:
        print(f"[normalize] Applied ArticulationRootAPI to {root_path}")
    print(f"[normalize] removed nested articulation roots: {removed}")


def patch_stage(args: argparse.Namespace) -> None:
    stage = Usd.Stage.Open(args.in_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {args.in_path}")

    normalize_articulation_root(stage, args.root_prim)

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )

    patched = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue

        mass_api = UsdPhysics.MassAPI(prim) if prim.HasAPI(UsdPhysics.MassAPI) else None
        if mass_api is None and UsdPhysics.MassAPI.CanApply(prim):
            mass_api = UsdPhysics.MassAPI.Apply(prim)
        if mass_api is None:
            continue

        mass_attr = mass_api.GetMassAttr()
        inertia_attr = mass_api.GetDiagonalInertiaAttr()

        cur_mass = mass_attr.Get() if mass_attr.HasAuthoredValue() else None
        cur_inertia = inertia_attr.Get() if inertia_attr.HasAuthoredValue() else None

        need_mass = cur_mass is None or cur_mass <= 0.0
        need_inertia = cur_inertia is None or sum(abs(c) for c in cur_inertia) == 0.0 if cur_inertia else True
        if not need_mass and not need_inertia:
            continue

        bbox = bbox_cache.ComputeWorldBound(prim)
        size_vec = bbox.ComputeAlignedRange().GetSize()
        size = (float(size_vec[0]), float(size_vec[1]), float(size_vec[2]))

        mass = cur_mass if cur_mass is not None else approx_mass_from_bbox(size, args.density, args.min_mass)
        if need_mass:
            mass_attr.Set(mass)

        if need_inertia:
            inertia = compute_box_inertia(mass, size)
            inertia_attr.Set(Gf.Vec3f(*inertia))
        else:
            inertia = cur_inertia  # type: ignore

        com_attr = mass_api.GetCenterOfMassAttr()
        if not com_attr.HasAuthoredValue():
            com_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))

        patched += 1
        print(f"[patch] {prim.GetPath()} mass={mass:.4f} inertia=({inertia[0]:.4f},{inertia[1]:.4f},{inertia[2]:.4f})")

    if args.out_path:
        stage.Export(args.out_path)
        print(f"[done] Exported to {args.out_path} (patched {patched} rigid bodies)")
    else:
        stage.Save()
        print(f"[done] Saved in-place {args.in_path} (patched {patched} rigid bodies)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--in", dest="in_path", required=True)
    parser.add_argument("--out", dest="out_path", default=None)
    parser.add_argument("--root-prim", required=True)
    parser.add_argument("--density", type=float, default=2700.0)
    parser.add_argument("--min-mass", type=float, default=0.05)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    patch_stage(args)


if __name__ == "__main__":
    main()
