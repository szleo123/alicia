# Local Project Changes

This checkout started from `Synria-Robotics/Alicia-D-ROS2` branch `v6.1.0`
at commit `08e632c5bc5a984aa2fc375aa53b9c2aa530f6e9`.

It is now Alicia's local working fork, not a pristine upstream mirror. The
top-level `vendor/` directory keeps mirrored upstream reference pages; this ROS
workspace carries project-specific runtime changes.

Current local change areas:
- Real-robot MoveIt launch support for the Alicia Phase 1 hardware baseline.
- D405 launch defaults and hand-eye TF publication for the wrist camera.
- Custom planning scene support through `alicia_d_moveit/config/world_scene.yaml`.
- Hand-guiding support through `/demonstration`.
- Kinematic and hand-eye calibration config for this physical arm.
- Conservative hardware synchronization and joint-position offset handling.
- D405-oriented cube-sort fallback launch paths.
- D405-oriented `alicia_d_grasp_6d` scripts for Phase 2 grasp generation and execution.
- `alicia_d_teleop` preview nodes for Geomagic Touch live-manual Cartesian teleoperation.

When pulling or comparing upstream changes, treat this tree as a fork and review
local behavior before overwriting package files.
