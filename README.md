# Alicia

Alicia is the main workspace for the current single-arm Alicia-D project.

The current baseline is:
- Ubuntu 22.04 + ROS 2 Humble
- Alicia-D follower arm
- 50 mm gripper
- Intel RealSense D405

The project currently centers on one ROS 2 workspace at [ros2_ws](/home/li/alicia/ros2_ws). Phase 1 is focused on bring-up and validation rather than application logic.

Current references:
- [AGENT.md](/home/li/alicia/AGENT.md) for project rules and operating expectations
- [docs/phase1_bringup.md](/home/li/alicia/docs/phase1_bringup.md) for the current bring-up path and known blockers
- [docs/phase2_grasp6d_runbook.md](/home/li/alicia/docs/phase2_grasp6d_runbook.md) for the Phase 2 D405-based 6D grasping workflow
- [vendor/README.md](/home/li/alicia/vendor/README.md) for mirrored upstream source provenance
- [changelog/CHANGELOG.md](/home/li/alicia/changelog/CHANGELOG.md) for recorded environment and dependency changes

Top-level structure:
- `ros2_ws/` main ROS 2 workspace
- `docs/` project documentation
- `vendor/` mirrored vendor documentation and upstream references
- `notes/` operator and working notes
- `calibration/` calibration records
- `data/` datasets and recordings
- `logs/` experiment and runtime logs
- `changelog/` environment and dependency history

For reproducible builds from this machine, use [ros2_ws/build_system_python.sh](/home/li/alicia/ros2_ws/build_system_python.sh) instead of relying on the active shell Python.
