# Geomagic Touch Teleoperation

## Scope

This workflow is for live manual operation of Alicia-D with a 3D Systems
Geomagic Touch haptic stylus. It is not currently designed around dataset
recording.

The first implementation is intentionally conservative:
- the stylus controls low-speed Cartesian `tool0` motion
- motion requires a held deadman/clutch button
- commands pass through a safety/mode filter before Servo
- orientation control is enabled in the tuning/simulation profile and remains opt-in for the real-robot launch
- gripper output is disabled by default
- real robot motion should only be enabled after simulation validation

The teleop stack is layered:
- `geomagic_cartesian_teleop.py` maps the input device to `/alicia_d_teleop/raw_twist_cmd`
- `teleop_safety_filter.py` enforces hold/jog/approach/grip/retreat modes, deadman, timeout, workspace, speed, acceleration, and smoothing limits
- MoveIt Servo consumes only `/alicia_d_teleop/twist_cmd`, the filtered output
- `trajectory_deadman_gate.py` is an extra real-robot gate between Servo trajectories and the arm controller

Normal operation starts in `jog`, so Geomagic use is simply: hold the deadman
and move. The safety filter still enforces deadman, timeout, speed, smoothing,
and workspace limits. Optional mode commands remain available as software
overrides:

```bash
ros2 topic pub --once /alicia_d_teleop/mode_command std_msgs/msg/String "{data: approach}"
ros2 topic pub --once /alicia_d_teleop/mode_command std_msgs/msg/String "{data: grip}"
ros2 topic pub --once /alicia_d_teleop/mode_command std_msgs/msg/String "{data: retreat}"
ros2 topic pub --once /alicia_d_teleop/mode_command std_msgs/msg/String "{data: hold}"
ros2 topic pub --once /alicia_d_teleop/mode_command std_msgs/msg/String "{data: jog}"
```

## Device Driver

First verify the device with the vendor driver and diagnostics. On Linux this is
normally the 3D Systems Touch Device Driver plus OpenHaptics SDK.

If using the community Geomagic Touch ROS 2 driver, the optional adapter expects:
- topic: `/phantom/state`
- type: `omni_msgs/msg/OmniState`
- fields: `pose`, `locked`, and `close_gripper`

The adapter republishes standard messages:
- `/geomagic_touch/pose` as `geometry_msgs/msg/PoseStamped`
- `/geomagic_touch/buttons` as `sensor_msgs/msg/Joy`

The community driver's `/phantom/state` pose is labeled as meters in the message
file but is published in millimeters in the current source. The adapter scales
position by `pose_position_scale: 0.001` before feeding teleop.

Button mapping:
- button 0: deadman/clutch
- button 1: gripper toggle

## Preview Launch

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
ros2 launch alicia_d_teleop geomagic_preview.launch.py
```

The default command output is:

```bash
/alicia_d_teleop/twist_cmd
```

Inspect it before connecting any robot motion:

```bash
ros2 topic echo /alicia_d_teleop/twist_cmd
ros2 topic echo /alicia_d_teleop/status
```

If another driver already publishes standard `PoseStamped` and `Joy` messages,
leave the OmniState adapter disabled, which is the launch default.

If using the community driver that publishes `/phantom/state`, enable the
OmniState adapter:

```bash
ros2 launch alicia_d_teleop geomagic_preview.launch.py use_omni_adapter:=true
```

## Translation Tuning Launch

Use this before connecting teleop to any robot controller. It starts the
OmniState adapter, Cartesian teleop node, and a local preview integrator that
turns `/alicia_d_teleop/twist_cmd` into a virtual end-effector pose/path:

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
ros2 launch alicia_d_teleop geomagic_tuning.launch.py use_omni_adapter:=true
```

To see that preview in Alicia's simulated MoveIt/RViz scene, launch the combined
demo tuning stack instead:

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
ros2 launch alicia_d_teleop geomagic_demo_tuning.launch.py use_omni_adapter:=true gripper_type:=50mm
```

In another terminal, watch the generated commands and preview motion:

```bash
ros2 topic echo /alicia_d_teleop/raw_twist_cmd
ros2 topic echo /alicia_d_teleop/twist_cmd
ros2 topic echo /alicia_d_teleop/preview_pose
ros2 topic echo /alicia_d_teleop/status
ros2 topic echo /alicia_d_teleop/safety_status
```

The launch starts in `jog`, so no mode command is required for normal testing.

For RViz, use fixed frame `base_link` and add these displays if they are not
already present:
- TF, to see `teleop_preview_tip`
- Pose, topic `/alicia_d_teleop/preview_pose`
- Path, topic `/alicia_d_teleop/preview_path`

Expected first-pass behavior in `velocity` mode:
- with the deadman released, filtered twist should stay zero and the preview tip should stop
- in normal `jog`, holding the deadman and moving the stylus should move the preview tip predictably
- with orientation enabled, holding the deadman and rotating the stylus should rotate Alicia's `tool0` toward the captured leader-orientation change
- if you manually switch to `hold` or `grip`, filtered twist should stay zero even if raw twist changes
- angular twist should remain zero when launched with `orientation_enabled:=false`

Recommended first tuning order:
1. Fix axis signs with `robot_axes_from_device`.
2. Set a comfortable speed cap with `max_linear_speed_m_s`.
3. Adjust responsiveness with `translation_gain`.
4. Smooth jitter with `low_pass_alpha`.
5. Enable and tune orientation-follow with `rotation_gain`, `max_angular_speed_rad_s`, and `robot_angular_axes_from_device`.

Start with small values. A practical initial range for real hardware is
`max_linear_speed_m_s: 0.03` to `0.06`, `translation_gain: 0.5` to `1.5`, and
`low_pass_alpha: 0.15` to `0.35`. The current simulation profile is faster than
that so RViz motion is easier to see.

## Control Behavior

Pressing the deadman button captures the current stylus pose as the clutch
origin. Moving the stylus away from that origin produces a bounded velocity
command in `base_link`. Releasing the deadman publishes a zero twist and resets
the clutch origin.

Relevant parameters live in:

```bash
ros2_ws/src/alicia_d_ros2_upstream/alicia_d_teleop/config/geomagic_teleop.yaml
```

Useful first tuning parameters:
- `max_linear_speed_m_s`
- `max_angular_speed_rad_s`
- `translation_gain`
- `translation_deadband_m`
- `rotation_gain`
- `orientation_deadband_rad`
- `low_pass_alpha`
- `robot_axes_from_device`
- `robot_angular_axes_from_device`
- `orientation_enabled`
- `angular_control_mode`
- `teleop_safety_filter.default_mode`
- `teleop_safety_filter.workspace_min` / `workspace_max`
- `teleop_safety_filter.jog_max_linear_m_s`
- `teleop_safety_filter.jog_max_angular_rad_s`
- `teleop_safety_filter.approach_max_linear_m_s`

Use `robot_axes_from_device` to fix handedness or desk orientation. Examples:
- `["x", "y", "z"]`
- `["y", "-x", "z"]`
- `["x", "-y", "z"]`

Use `robot_angular_axes_from_device` the same way for stylus roll/pitch/yaw.
It defaults to the same mapping as translation in this project, but can be
tuned independently if rotation directions feel wrong after translation is
already correct.

For real hardware, `translation_deadband_m` and `orientation_deadband_rad`
matter a lot. They prevent tiny hand drift, Touch gravity sag, or encoder noise
from becoming a real robot command as soon as the deadman is held.

The normal angular mode is `orientation_follow`. Pressing the deadman captures
the Touch orientation and Alicia `tool0` orientation. While held, Touch
orientation change is applied as a desired local `tool0` orientation change,
and the mapper publishes a bounded base-frame angular velocity to reduce that
orientation error. This keeps Servo's `robot_link_command_frame` in `base_link`,
so translation remains spatial/base-frame while angular motion behaves like
leader-orientation following around the captured tool axes.

## Simulation Path

The tuning launch gives a sim-safe preview of the teleop velocity command, but
it does not move the Alicia model joints. `geomagic_demo_tuning.launch.py`
places that preview in the Alicia MoveIt demo/RViz scene so axis and gain tuning
can happen in the robot frame. The next validation step after the preview feels
correct is to connect `/alicia_d_teleop/twist_cmd` to a Cartesian servo
controller in simulation.

After installing `ros-humble-moveit-servo`, launch the fake-hardware Servo demo:

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
ros2 launch alicia_d_teleop geomagic_servo_demo.launch.py use_omni_adapter:=true gripper_type:=50mm
```

Watch Servo status and generated controller commands:

```bash
ros2 topic echo /alicia_d_teleop/servo_status
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /Alicia_controller/joint_trajectory
```

The launch starts and unpauses Servo automatically a few seconds after startup.
If running Servo manually, call:

```bash
ros2 service call /moveit_servo/start_servo std_srvs/srv/Trigger "{}"
ros2 service call /moveit_servo/unpause_servo std_srvs/srv/Trigger "{}"
```

The Servo config is intentionally slow and simulation-only:

```bash
ros2_ws/src/alicia_d_ros2_upstream/alicia_d_teleop/config/alicia_servo.yaml
```

The Servo demo also uses a bent fake-hardware start pose to avoid the all-zero
singularity. If Servo reports status `1` or `6`, it is decelerating near a
singularity; status `2` is a singularity halt.

```bash
ros2_ws/src/alicia_d_ros2_upstream/alicia_d_moveit/config/servo_initial_positions.yaml
```

Do not reuse this launch against the real robot. Add a separate real-robot
Servo launch only after simulation behavior is predictable.

## Real Robot Servo Profile

The real-robot profile is intentionally separate and slower than the simulation
profile. Start the normal real robot stack first:

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
ros2 launch alicia_d_moveit real_robot.launch.py gripper_type:=50mm
```

For teleop tests, prefer disabling the hand-guiding UI so it cannot accidentally
publish `/demonstration=true`:

```bash
ros2 launch alicia_d_moveit real_robot.launch.py gripper_type:=50mm demonstration_ui:=false
```

In another terminal, start the Touch driver:

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
ros2 launch omni_common omni_state.launch.py
```

Then start the real teleop/Servo bridge in paused dry-run mode:

```bash
source /opt/ros/humble/setup.bash
source /home/li/alicia/ros2_ws/install/setup.bash
ros2 launch alicia_d_teleop geomagic_servo_real.launch.py use_omni_adapter:=true gripper_type:=50mm
```

Real-robot angular teleop is disabled by default. After translation is stable,
enable it explicitly while staying in dry-run first:

```bash
ros2 launch alicia_d_teleop geomagic_servo_real.launch.py use_omni_adapter:=true gripper_type:=50mm orientation_enabled:=true
```

This forces `/demonstration=false` once on startup. Servo auto-starts and
auto-unpauses by default, then publishes only to
`/alicia_d_teleop/servo_raw_joint_trajectory`; a separate trajectory gate
forwards only while the Touch deadman is held. In dry-run mode, the gate can
only publish to `/alicia_d_teleop/servo_dry_run_joint_trajectory`, not the real
controller. Inspect inputs and dry-run output before live hardware output:

Before live Servo, move Alicia away from the near-zero singular posture. In
RViz, use the `Alicia` planning group and the named target `teleop_ready`, then
Plan and Execute. The zero/home posture is a poor Cartesian Servo starting
point.

```bash
ros2 topic echo /alicia_d_teleop/twist_cmd
ros2 topic echo /alicia_d_teleop/safety_status
ros2 topic echo /alicia_d_teleop/status
ros2 topic echo /alicia_d_teleop/servo_status
ros2 topic echo /alicia_d_teleop/trajectory_gate_status
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /alicia_d_teleop/servo_raw_joint_trajectory
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /alicia_d_teleop/servo_dry_run_joint_trajectory
```

Manual Servo start is no longer required in the normal launch. To opt out of
auto-start for debugging, launch with `start_servo:=false`, then call:

```bash
ros2 service call /moveit_servo/start_servo std_srvs/srv/Trigger "{}"
ros2 service call /moveit_servo/unpause_servo std_srvs/srv/Trigger "{}"
```

The real bridge starts in `jog`; for a slower near-branch test, you can
optionally switch to `approach`. The trajectory gate starts armed by default,
but still forwards only while the Touch deadman is held. To force the gate
closed for debugging:

```bash
ros2 service call /trajectory_deadman_gate/set_armed std_srvs/srv/SetBool "{data: false}"
```

For live hardware output, restart the bridge with `dry_run:=false`. The same
deadman gate remains active, so trajectories reach the real controller only
while the Touch deadman is actively held.

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: false}"
ros2 launch alicia_d_teleop geomagic_servo_real.launch.py use_omni_adapter:=true gripper_type:=50mm dry_run:=false
```

The real profile lives at:

```bash
ros2_ws/src/alicia_d_ros2_upstream/alicia_d_teleop/config/alicia_servo_real.yaml
```

## Real Robot Safety

Before real robot teleop:
- confirm simulation motion is smooth and axis mapping is correct
- set low speed limits
- keep the deadman behavior enabled
- keep orientation disabled until translation feels reliable
- verify `world_scene.yaml` matches the physical workspace
- keep the operator ready to stop motion

Only remap the twist output to a live controller after the above checks pass.
