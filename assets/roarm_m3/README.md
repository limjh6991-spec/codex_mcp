# RoArm M3 Asset Scaffold

This directory contains placeholder assets and specifications for integrating the RoArm M3 desktop robotic arm into Isaac Sim + RL pipeline.

## Files
- `roarm_m3_placeholder.urdf`: Interim URDF (approximate 6-DOF) to enable articulation pipeline prototyping.
- `joint_spec.json`: Structured joint limits + axis metadata (PLACEHOLDER). Update when official specs collected.

## Integration Plan
1. Replace URDF with accurate model or USD converted via Isaac Sim URDF importer.
2. Generate USD: In Isaac Sim GUI: Menu -> Create -> URDF Importer -> select updated URDF.
3. Verify articulation: Use Python console to create `ArticulationView` and print joint count.
4. Update observation/action schema with definitive `joint_names` ordering.
5. Retrain PPO policy with correct DOF.

## TODO Checklist
- [ ] Collect official joint limit/axis data from vendor docs or SDK.
- [ ] Replace placeholder URDF and bump `joint_spec.json.version`.
- [ ] Add gripper/end-effector specification if applicable.
- [ ] Create validation test: asserts len(joint_names)==assumed_dof.
- [ ] Record baseline latency with real articulation loop.

## Notes
Do NOT deploy training results originating from the placeholder kinematics; they are for infrastructure shake-down only.
