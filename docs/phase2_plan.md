# Phase 2 Plan

## Purpose

Phase 2 should build directly on the completed Phase 1 baseline:
- real Alicia-D arm working through MoveIt
- 50 mm gripper working on hardware
- Intel RealSense D405 running in ROS 2
- eye-in-hand calibration completed and published in the real-robot launch path
- custom planning scene support available through `world_scene.yaml`

The Phase 2 focus is now:
- turning the calibrated wrist-camera system into a practical grasping pipeline

## Recommended Phase 2 Definition

Recommended Phase 2 theme:
- **single-arm D405-based 6D grasping**

This is now the best fit for the project because:
- it directly uses the completed D405 and hand-eye baseline
- the upstream workspace already contains a D405-specific 6D grasp stack
- it is closer to the long-term manipulation direction than color sorting
- it produces a stronger application milestone than another structured pick-and-place demo

## What Phase 2 Can Realistically Achieve

Based on the vendor material and the local upstream workspace, there are three realistic directions:

### 1. D405-based 6D grasp generation and execution

This is now the recommended primary direction.

Available building blocks already exist:
- `alicia_d_grasp_6d`
- D405-specific scripts:
  - `d405_ros_bridge.py`
  - `d405_foundationstereo.py`
  - `d405_sam2.py`
  - `d405_graspgen.py`
  - `d405_execution.py`
- `alicia_d_calibration`
- `alicia_d_moveit`
- `d405.launch.py`
- published hand-eye TF in `real_robot.launch.py`

What this enables:
- bridge the D405 ROS topics into the 6D grasp stack
- estimate depth / point clouds
- segment target objects interactively
- generate 6D grasp candidates
- execute chosen grasps on the real robot

This is heavier than Phase 1 and requires more setup, but it is the strongest next milestone.

### 2. Structured camera-guided sorting

This remains useful, but it is no longer the main Phase 2 target.

Available building blocks already exist:
- `alicia_d_cube_sort`
- D405-compatible detection configuration
- hand-eye calibration and real-robot MoveIt integration

What this enables:
- simpler detection-driven pick-and-place
- repeatable structured object sorting
- fallback task-level validation if the 6D stack stalls

This is now best treated as a fallback / side path rather than the primary milestone.

### 3. Leader-follower teleoperation / imitation

Vendor documentation clearly supports this as a product direction.

However, it is not the best current Phase 2 target because:
- the present project baseline is still single-arm
- it likely requires additional leader-arm hardware
- it shifts the project from perception-manipulation into teleoperation / data collection

This is better treated as a later phase unless the hardware scope changes.

## Recommended Phase 2 Goal

Phase 2 is recommended to mean:

- the Alicia-D arm performs camera-guided 6D grasp generation and grasp execution using the wrist-mounted D405

Success for Phase 2 means:
- the D405, hand-eye calibration, and robot TF chain are used as part of the normal grasp pipeline
- the 6D stack can generate grasp candidates from live camera input
- the robot can execute selected grasps on at least one real object class
- the full stack runs through a documented, reproducible workflow
- the result is validated on the real system rather than only in isolated model demos

Recommended Phase 2 demo:
- **interactive D405-based 6D grasp generation and execution on the real robot**

Operator workflow:
- use [phase2_grasp6d_runbook.md](/home/li/alicia/docs/phase2_grasp6d_runbook.md) as the current launch-order and validation checklist

## Scope Boundaries

Phase 2 should focus on:
- perception-to-action integration
- D405 bridge and live sensor use
- point cloud and mask generation
- grasp proposal generation
- MoveIt-based execution on the real arm
- operator workflow and reproducibility

Phase 2 should not yet depend on:
- leader-arm hardware
- bimanual coordination
- large-scale dataset collection
- training new VLA models
- full production autonomy with zero operator interaction

Those are good later directions, but they should not block the first grasping milestone.

## Proposed Phase 2 Structure

### Phase 2A: D405-based 6D grasping

Primary target:
- get `alicia_d_grasp_6d` running reliably on the real system with the calibrated D405

What to complete:
- clone and verify:
  - `GraspGen`
  - `FoundationStereo`
  - `sam2`
- create and document the required Conda environments
- download and validate required model weights
- verify the D405 ROS bridge works against the current camera topics
- verify FoundationStereo produces usable point clouds
- verify SAM2 can segment a target object interactively
- verify GraspGen produces grasp proposals from live data
- verify `d405_execution.py` can execute selected grasps using the current real-robot launch and hand-eye TF

Definition of done:
- the system can ingest live D405 data, generate grasp candidates, and execute at least one grasp successfully on the real robot
- the launch and environment procedure is documented and repeatable
- the workflow does not require ad hoc code edits on every run

### Phase 2B: Structured fallback manipulation

Secondary target:
- keep `alicia_d_cube_sort` available as a simpler fallback validation path

What to complete:
- preserve D405-aligned detection and sorting launches
- use it when the 6D stack is blocked by external-model setup or environment issues

Definition of done:
- a simpler detection-driven task remains available as a sanity-check manipulation workflow

## Proposed Validation Sequence

### 1. Preserve the Phase 1 baseline

Before Phase 2 work on any given day, confirm:
- `real_robot.launch.py` still drives the real robot correctly
- `d405.launch.py` still publishes the expected camera topics
- the hand-eye TF still looks correct

### 2. Validate external 6D model environments

Goal:
- make sure the required third-party stacks run independently before integrating them with the robot

Validation:
- `GraspGen` demo runs in its Conda environment
- `FoundationStereo` demo runs in its Conda environment
- `sam2` demo runs in its Conda environment
- required checkpoints are present and load correctly

### 3. Validate the D405 bridge and perception data flow

Goal:
- confirm the live D405 stream can feed the 6D pipeline

Validation:
- run the D405 ROS bridge
- confirm expected ROS and bridge topics/files are produced
- confirm point cloud and mask data are flowing

### 4. Validate grasp proposal generation

Goal:
- produce grasp candidates from a real object in the workspace

Validation:
- segment one target object
- run point cloud generation
- run GraspGen and inspect grasp candidates in MeshCat

### 5. Validate supervised grasp execution

Goal:
- execute selected grasps on the real robot under operator supervision

Validation:
- choose a stable test object
- run one grasp attempt at low speed
- inspect approach, contact, closure, lift, and retreat

### 6. Validate repeatability

Goal:
- move from one-off success to a reproducible operator workflow

Validation:
- repeated start-to-finish runs
- consistent environment activation and launch order
- documented recovery steps for common failures

## Main Risks

Phase 2 risks are likely to be:
- Conda environment drift across `GraspGen`, `FoundationStereo`, and `sam2`
- missing or mismatched model weights
- CUDA / PyTorch compatibility issues
- D405 topic or frame mismatches with third-party scripts
- residual hand-eye or joint calibration bias affecting grasp execution
- MoveIt execution tolerance issues near contact-rich trajectories
- object segmentation quality under real lighting
- upstream script assumptions that still reflect Gemini 335 or older calibration flow

## Safety Considerations

Phase 2 increases both runtime complexity and manipulation risk.

Operators and agents should:
- keep execution slow during first grasping trials
- use conservative pre-grasp and retreat motions
- keep the collision scene matched to the real table and fixtures
- validate point cloud, mask, and grasp pose visually before execution
- start with large, stable, easy-to-grasp objects
- preserve logs, terminal output, calibration files, and screenshots for failed runs

## Recommended Immediate Next Steps

The best next sequence is:

1. finish the third-party environment setup for `GraspGen`, `FoundationStereo`, and `sam2`
2. resolve the current installation/runtime problems from the `GraspGen` setup path
3. verify each external stack independently before combining them
4. validate the D405 bridge and data flow into the 6D grasp stack
5. generate first live grasp candidates
6. execute the first supervised real grasp

## Phase 2 Recommendation Summary

Recommended Phase 2:
- **camera-guided single-arm 6D grasping with the wrist-mounted D405**

Recommended baseline deliverable:
- **live D405-based grasp generation and execution on the real robot**

Recommended fallback deliverable:
- **structured D405-based cube sorting if the external 6D stack is temporarily blocked**

Recommended non-goal for now:
- leader-arm imitation / teleoperation as the primary Phase 2 milestone
