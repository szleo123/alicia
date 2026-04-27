# Phase 2 Grasp 6D Runbook

## Purpose

This is the operator-facing workflow for the Phase 2 target: live D405-based 6D
grasp generation and supervised execution on the Alicia-D arm.

Use this after the Phase 1 baseline is healthy:
- `real_robot.launch.py` drives the arm and publishes the saved hand-eye TF.
- `d405.launch.py` publishes D405 color, depth, IR, point cloud, aligned depth,
  and camera TF.
- The custom collision scene in `world_scene.yaml` matches the physical table.

## One-Time Setup Checks

Third-party code should live under:

```bash
/home/li/alicia/ros2_ws/src/alicia_d_ros2_upstream/alicia_d_grasp_6d/Models
```

Expected repositories:
- `GraspGen`
- `FoundationStereo`
- `sam2`

Expected Conda environments:
- `GraspGen`
- `foundation_stereo`
- `sam2`

Before robot integration, each external stack should load its model weights and
run its own upstream demo.

## Terminal Setup

For ROS/system-Python terminals:

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
cd /home/li/alicia/ros2_ws/src/alicia_d_ros2_upstream/alicia_d_grasp_6d/scripts/intel_rs_d405
```

For Conda terminals, activate only the environment needed by that process.

## Launch Order

### 1. Real Robot

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
ros2 launch alicia_d_moveit real_robot.launch.py \
  gripper_type:=50mm \
  port:=/dev/ttyACM0 \
  speed_deg_s:=10
```

Keep motion slow for first grasp trials.

### 2. D405

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
ros2 launch alicia_d_moveit d405.launch.py
```

Expected defaults:
- `enable_color:=true`
- `enable_depth:=true`
- `enable_infra1:=true`
- `enable_infra2:=true`
- `pointcloud_enable:=true`
- `align_depth_enable:=true`
- `upside_down:=true`

### 3. ROS Bridge

```bash
python3 d405_ros_bridge.py
```

This forwards ROS image streams to the Conda perception processes and republishes
mask, point-cloud, and grasp data back into ROS.

### 4. MeshCat

```bash
conda activate GraspGen
meshcat-server
```

Open the printed browser URL.

### 5. Grasp Execution Node

```bash
python3 d405_execution.py
```

Execution is supervised. Use the prompt to accept, skip, or quit a proposed
grasp.

### 6. FoundationStereo

```bash
conda activate foundation_stereo
python d405_foundationstereo.py --visualize
```

The D405 close-range default is `--z_near 0.01`. If the point cloud is empty,
check target distance, IR exposure/lighting, and the checkpoint path.

### 7. SAM2

```bash
conda activate sam2
python d405_sam2.py
```

Use left-click for target points, right-click for background points, `r` to
reset, and `q` to quit.

### 8. GraspGen

```bash
conda activate GraspGen
python d405_graspgen.py
```

Start with a large, stable, easy-to-grasp object. Inspect the point cloud, mask,
and grasp poses before accepting execution.

## Validation Checklist

Before accepting a grasp:
- The selected object is segmented cleanly.
- The point cloud is not stale and matches the object on the table.
- MeshCat grasp poses are in the expected frame and location.
- The approach direction is physically safe.
- The collision scene still matches nearby fixtures.
- The operator is ready to stop motion.

After each attempt, record notable failures in `logs/` or `notes/`, and preserve
screenshots or terminal output when they explain a perception or execution issue.

## Common Recovery

If GraspGen keeps using old geometry, restart `d405_ros_bridge.py` so stale point
cloud cache files are cleared.

If the robot pose looks offset from the visualized grasp, re-check:
- hand-eye TF publication from `real_robot.launch.py`
- D405 optical frame conventions
- `kinematic_calibration.yaml`
- current `world_scene.yaml`

If ROS imports fail inside Conda, run only the model-specific process in Conda and
keep `d405_ros_bridge.py` plus `d405_execution.py` in the ROS/system-Python
environment.
