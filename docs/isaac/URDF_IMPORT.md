# RoArm-M3 URDF → USD: GUI Import Guide

This doc shows how to import the provided URDF skeleton into Isaac Sim via the GUI, produce a USD, and verify joints headless in our repo tools.

> Status: URDF Importer extension may not be bundled in your current Isaac build. The GUI path below is the fastest way to confirm availability and produce a USD. If the importer is missing, see the Automation section for enabling extensions and a scripted fallback.

## Prerequisites

- ISAAC_SIM_ROOT is set (our VS Code terminal injects it automatically).
- NVIDIA driver working (nvidia-smi OK).
- Our URDF: `assets/roarm_m3/urdf/roarm_m3.urdf` (skeleton; limits/axes to be calibrated).

## GUI Import Steps

1) Start Isaac Sim GUI
- Launch Isaac Sim from your installed location, or use the official launcher.

2) Enable URDF-related Extensions (if disabled)
- Window → Extensions
- Search for and enable:
  - omni.isaac.urdf (URDF Importer 2.0)
  - omni.kit.asset_converter (sometimes required)
  - omni.usd (core)
- Restart Isaac when prompted.

3) Open URDF Importer
- Menu → Isaac Utils → Workflows → URDF Importer
- In the Importer panel:
  - URDF Path: select `assets/roarm_m3/urdf/roarm_m3.urdf`
  - Output Path: pick a writable folder in your workspace, e.g. `assets/roarm_m3/usd/roarm_m3.usd`
  - Options:
    - Fix Inertials: ON (recommended for our placeholder inertias)
    - Create PhysX Articulation: ON
    - Use Decomposition for Collisions: Optional
    - Treat Continuous Joints: ensure base and wrist2 stay continuous
  - Click Import.

4) Save and Test the USD
- After import, Isaac opens a stage. Save it to the path above if not auto-saved.
- Press Play briefly to ensure no sim errors.
- Note the final USD path.

5) Verify Articulation Root and Prim Path (quick scan)
- In VS Code, run the task "Scan RoArm USD" and select your USD.
- It will print:
  - defaultPrim
  - whether your chosen prim exists (prim_exists)
  - articulation_root_candidates (PhysX Articulation Root prims)
- If no articulation root is found, re-import with "Create PhysX Articulation" enabled or set it in the GUI for the robot root prim.

## Headless Verification (from repo)

We ship a tiny viewer that attaches an articulation and prints joints. Run via the bundled python if possible.

- Path: `scripts/view_roarm.py`
- Usage: specify the USD you saved from GUI import.

In VS Code: Run the task "View RoArm (USD)" and provide the USD path and the correct prim path (use the scan results), or run the script with the bundled Python.

If joint names/axes differ, update either the URDF or our mapping in `src/utils/isaac_joint_api.py` accordingly.

## Troubleshooting

- URDF Importer missing in Extensions:
  - Your Isaac build might not include it. Install/enable `omni.isaac.urdf` via Extension Manager or the launcher.
- Errors about mesh/inertial:
  - Our skeleton uses primitives; enable "Fix Inertials" and proceed. Later, replace with accurate inertias/meshes.
- Continuous joints clamped:
  - Ensure the importer recognizes `type="continuous"` for `joint_base` and `joint_wrist2`.
- Axis/sign mismatches:
  - The skeleton guesses common conventions. Validate by commanding small positive motions and compare to hardware. Flip axis in URDF if needed (xyz sign) or update mapping in our code.

## Automation Path (Optional)

We also provide `scripts/convert_urdf_to_usd.py` that tries to enable URDF importer extensions and convert headlessly. If your build lacks those extensions, it will error with a clear message. Once enabled, re-run and then verify with the viewer.

Next steps once a USD exists:
- Integrate with `src/envs/isaac_roarm_env.py` to spawn the articulation and map joints
- Add joint limit tests vs our IPC gateway specs
- Iterate on URDF axes/limits until sim-to-real mapping matches
