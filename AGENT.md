# AGENT.md

## Project Identity

Alicia is the main workspace for a single-arm Alicia-D robotics project. The project combines engineering work and research work in one place. This folder is intended to hold the full project context, including software, ROS workspaces, mirrored vendor documentation, notes, calibration records, experiments, logs, datasets, and recordings.

The current hardware baseline is:
- Alicia-D follower/manipulator arm
- 50 mm gripper
- Intel RealSense D405

The current software baseline is:
- Ubuntu 22.04
- ROS 2 Humble

## Current Scope

Alicia is currently a single-arm project. Future expansion to teleoperation or other control setups is possible, but that is not the primary scope right now.

The main technical center of the project is ROS 2. Other tooling may be added later in support of the ROS 2 workflow.

## Phase 1 Goal

Phase 1 is hardware bring-up and validation.

Success for Phase 1 means:
- the arm is connected and controllable
- the gripper is connected and controllable
- the Intel RealSense D405 is connected and available in ROS 2
- a simple repeatable demo exists, such as joint motion

Phase 1 is not yet focused on advanced learning workflows, full camera calibration pipelines, or large-scale application tasks.

## Workspace Principles

Alicia should remain organized, readable, and structurally clear. A planned top-level structure should exist from the beginning, but it may evolve as the project grows.

The project should contain one main ROS 2 workspace.

Suggested top-level structure:
- `ros2_ws/` for the main ROS 2 workspace
- `docs/` for project documentation
- `vendor/` for mirrored vendor documentation and external references
- `notes/` for working notes and operator notes
- `calibration/` for calibration records and calibration-related outputs
- `data/` for datasets and recordings
- `logs/` for runtime logs and experiment logs
- `changelog/` for environment, dependency, and project-change records

This structure is a starting point, not a rigid final form.

## Documentation Rules

Important vendor documentation should be mirrored locally inside Alicia.

When vendor documentation is copied or adapted locally:
- keep the original source link
- keep the source access or copy date
- preserve provenance clearly even if the local copy is edited later

Local copies may be edited as the project progresses, but agents should avoid losing the distinction between upstream source material and project-specific interpretation.

## Reproducibility Rules

Reproducibility is a project priority.

Agents should prefer:
- scripted setup over one-off manual setup
- documented procedures over undocumented local fixes
- repeatable workflows over ad hoc operator-only steps

When packages, ROS dependencies, or system configuration are changed, the change should be recorded in the project changelog.

## Validation Strategy

Simulation should be used before real hardware when practical.

However, real-hardware validation is the final authority for hardware-facing work. Work should not be treated as fully validated until it has been checked on the real system when appropriate.

## Data and Recordkeeping Rules

Datasets, recordings, raw observations, and logs are first-class project assets.

Agents must preserve raw data and should not silently clean up or overwrite it.

Datasets, recordings, logs, and experiment outputs should be saved using distinct timestamped names. The default timestamp format is:

`YYYYMMDD_HHMMSS`

Older data may later be archived elsewhere, but preservation comes before cleanup.

Calibration records may be updated in place when necessary.

## Safety Rules

Safety rules apply from the beginning of the project.

Agents and operators should:
- prefer simulation before commanding real hardware
- start with low-speed motion for first-run validation
- ensure the workspace is clear before motion
- avoid uncontrolled or unreviewed hardware commands
- treat hardware-facing changes carefully, especially motion-related changes
- preserve logs and records related to hardware behavior when possible

No destructive action should be taken on datasets, logs, or recordings without explicit confirmation.

## Agent Operating Guidance

Agents working inside Alicia should treat this folder as a long-term robotics workspace, not just a temporary code repository.

Agents are allowed to:
- install packages
- change ROS workspace dependencies
- adjust system configuration when needed

But they should:
- record meaningful environment or dependency changes in the changelog
- preserve project readability
- avoid destructive actions on preserved data without explicit confirmation
- favor workflows that support future continuation by both humans and agents
