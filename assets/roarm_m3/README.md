# RoArm M3 Asset Scaffold

This directory now tracks the verified RoArm M3 articulation assets and metadata used by the Isaac Sim + RL pipeline.

## Files
- `urdf/roarm_m3.generated.urdf`: Waveshare-calibrated URDF with joint limits/dynamics aligned to firmware constants.
- `usd/roarm_m3.generated.usd`: PhysX-ready articulation with matched limits/drive gains for Isaac Sim.
- `joint_spec.json`: Canonical joint ordering + radian limits used by observation/action schemas.
- `external_sources/`: Official vendor firmware, Python demos, STEP + 2D drawings collected on 2025-10-12.

## Integration Plan
1. Validate URDF ↔ USD alignment (`scripts/verify_usd_roarm_m3.py` + pxr-driven limit audit).
2. Regenerate USD snapshots after significant kinematics changes (`usd/snapshots/`).
3. Sync observation/action schema with `joint_spec.json` (bump schema version).
4. Punch through Isaac control loop smoke test (GUI articulation + RL dummy policy).
5. Retrain PPO policy with confirmed DOF + scaling.

## TODO Checklist
- [x] Collect official joint limit/axis data from vendor docs or SDK.
- [x] Replace placeholder URDF and bump `joint_spec.json.version`.
- [ ] Add gripper/end-effector load spec & mimic mapping (optional).
- [ ] Create validation test: asserts len(joint_names)==assumed_dof.
- [ ] Record baseline latency with real articulation loop.

## Notes
The new URDF/USD pair supersedes the placeholder model; invalidate any historical training runs that predate `joint_spec.json` version `1`.
