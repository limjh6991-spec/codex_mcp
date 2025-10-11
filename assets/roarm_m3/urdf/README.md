RoArm M3 URDF/XACRO assets

Notes for non-ROS environments:
- The file `roarm_m3.xacro` has been adjusted to use relative paths instead of `$(find roarm_description)`.
- Mesh paths now reference `../meshes/roarm_m3/*.stl` so that GUI importers and non-ROS tooling can resolve them.
- Included auxiliary files live under `assets/roarm_m3/urdf/roarm_m3/`.

If you still want ROS-style resolution, restore `$(find roarm_description)` and ensure a proper `ROS_PACKAGE_PATH` with a `roarm_description` package is available.
