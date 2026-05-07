# Changelog

## 20260506_real_teleop_deadband

Real teleop neutral guard:
- added translation and orientation deadbands to the Geomagic mapper
- set larger real-launch deadbands so holding the Touch deadman alone should command zero
- kept simulation/tuning deadbands smaller so axis tuning remains responsive

Current interpretation:
- real teleop should no longer turn small leader drift or Touch sag into immediate arm motion

## 20260506_real_teleop_single_deadman

Real teleop workflow simplification:
- changed the real Geomagic Servo launch to start and unpause MoveIt Servo automatically by default
- changed the trajectory gate to start armed by default in the real launch
- kept the Touch deadman as the required live-motion permission for forwarding trajectories
- kept `start_servo:=false` and `gate_armed_on_start:=false` as debug overrides

Current interpretation:
- real teleop now has one normal operator hold-to-move control instead of separate Servo start, unpause, gate arm, and deadman steps

## 20260506_geomagic_orientation_follow

Geomagic orientation-follow:
- added an orientation-follow angular controller to the Geomagic mapper
- kept translation commands in `base_link` while using TF `base_link -> tool0` only to compute angular orientation error
- added `angular_control_mode: orientation_follow` with `axis_delta` still available as a simpler angular-jog fallback
- raised the simulation orientation gain for clearer angular response while keeping real-robot angular gain lower
- changed orientation-follow to apply leader orientation deltas around the captured `tool0` local axes instead of around base-frame axes
- increased simulation angular speed, acceleration, and smoothing responsiveness in the teleop safety layer

Current interpretation:
- Alicia can now keep spatial translation intuitive in the world/base frame while making `tool0` orientation follow the leader's captured orientation change

## 20260506_geomagic_velocity_only

Geomagic teleop simplification:
- removed the follower-style Cartesian mode and its target preview topic from the input mapper
- removed the corresponding launch arguments and Servo-demo overrides
- kept one normal Geomagic behavior: clutched velocity jog through the safety filter into MoveIt Servo

Current interpretation:
- Alicia teleop tuning is now centered on one motion model: deadman-held velocity commands in `base_link`

## 20260506_geomagic_angular_teleop

Geomagic angular teleop:
- enabled stylus-orientation mapping in the normal tuning profile
- added launch overrides for `orientation_enabled`, `rotation_gain`, and `max_angular_speed_rad_s`
- added `robot_angular_axes_from_device` so angular direction can be tuned separately from the verified translation axes
- kept real-robot angular teleop opt-in with lower default angular gain and speed caps
- fixed boolean parameter parsing so launch values like `orientation_enabled:=false` are interpreted correctly

Current interpretation:
- translation axes are now verified, and the next tuning loop can include slow roll/pitch/yaw commands through the same deadman and safety filter

## 20260506_teleop_safety_layers

Layered branch-cutting teleoperation profile:
- added `teleop_safety_filter.py` as a supervisory Cartesian command filter between input mapping and MoveIt Servo
- split raw input `/alicia_d_teleop/raw_twist_cmd` from filtered Servo input `/alicia_d_teleop/twist_cmd`
- added explicit `hold`, `jog`, `approach`, `grip`, and `retreat` modes through `/alicia_d_teleop/mode_command`
- added safety-filter enforcement for deadman, command timeout, workspace limits, per-mode speed caps, acceleration limiting, and smoothing
- wired preview, tuning, simulation Servo, and real Servo launches through the safety-filter layer
- kept the real-robot trajectory gate so Servo trajectory output cannot directly reach `/Alicia_controller/joint_trajectory`

Current interpretation:
- Alicia teleop now defaults to normal constrained Cartesian `tool0` jog with deadman, while raw joint teleop remains avoided
- real hardware remains dry-run/gated until deliberately armed

## 20260427_123000

Geomagic Touch translation tuning preview:
- added `twist_preview_integrator.py` to integrate teleop `TwistStamped` output into a virtual end-effector pose, path, and TF frame
- added `geomagic_tuning.launch.py` for the driver-adapter, Cartesian teleop, and preview integrator stack
- added `geomagic_demo_tuning.launch.py` to show the teleop preview in the Alicia MoveIt demo/RViz scene
- added `alicia_servo.yaml` and `geomagic_servo_demo.launch.py` for conservative fake-hardware MoveIt Servo testing after installing `ros-humble-moveit-servo`
- updated the Servo demo launch to start and unpause Servo automatically after launch
- added a Servo-specific fake-hardware initial pose to avoid starting Cartesian Servo from the all-zero singularity
- threaded `initial_positions_file` through the Alicia MoveIt demo launch so controller-manager joint states match the Servo demo start pose
- adjusted the simulation-only Servo start pose and singularity thresholds to reduce repeated deceleration warnings during Geomagic translation tests
- increased the simulation teleop profile after first successful motion so RViz movement is visible enough for axis/gain tuning
- added a separate slower real-robot Servo profile and launch that defaults to paused/manual arming
- made real Servo launch default to dry-run output and force `/demonstration=false` before hardware arming
- inserted a deadman/armed trajectory gate so real Servo output cannot reach `/Alicia_controller/joint_trajectory` directly
- added a MoveIt `teleop_ready` named arm state as a non-zero starting posture for real Cartesian Servo tests
- added preview topics `/alicia_d_teleop/preview_pose` and `/alicia_d_teleop/preview_path`
- documented the first tuning loop for translation axes, speed cap, gain, and smoothing before enabling orientation or any robot controller

Current interpretation:
- the immediate teleop milestone is predictable manual translation in a sim-safe preview
- angular/orientation control stays disabled until the hand motion feels natural
- MoveIt Servo remains the likely bridge from preview to simulated robot motion, but it is not installed in this workspace yet

## 20260427_112300

Geomagic Touch teleoperation preview:
- added `alicia_d_teleop` as a conservative live-manual teleoperation package
- added `geomagic_cartesian_teleop.py` to convert clutched stylus pose input into bounded Cartesian `TwistStamped` commands
- added `geomagic_omni_state_adapter.py` as an optional adapter for Geomagic Touch ROS 2 drivers that publish `omni_msgs/msg/OmniState`
- added `geomagic_preview.launch.py` with preview output on `/alicia_d_teleop/twist_cmd` rather than a live robot controller
- added `config/geomagic_teleop.yaml` for deadman, gripper, velocity, filtering, and axis-mapping parameters
- documented the live-manual teleop workflow in `docs/geomagic_touch_teleop.md`

Current interpretation:
- the first teleop milestone is preview/simulation validation, not real-hardware motion
- MoveIt Servo remains the preferred live-controller target, but it is not installed in this workspace yet
- real robot teleoperation should stay gated behind deadman, conservative limits, and simulation validation

## 20260427_104154

Phase 2 cleanup and record organization:
- made the installed Alicia calibration package config the hand-eye runtime source of truth
- moved the duplicate workspace-root hand-eye result into `calibration/hand_eye/`
- moved loose kinematic calibration samples/results from `ros2_ws/` into `calibration/kinematic/20260417/`
- removed the accidental duplicate `ros2_ws/src/alicia_d_calibration` config path
- updated calibration scripts so future sample/result outputs go into timestamped `calibration/kinematic/` folders
- updated hand-eye calibration output so it writes to the real package config and preserves a timestamped copy under `calibration/hand_eye/`
- restored `d405.launch.py` aligned-depth default to match the documented D405 baseline
- restored conservative MoveIt default velocity/acceleration scaling for real-arm Phase 2 work
- added `LOCAL_CHANGES.md` to clarify that `alicia_d_ros2_upstream` is now a local project fork
- added `docs/phase2_grasp6d_runbook.md` as the operator workflow for the D405 6D grasping stack

Current interpretation:
- Phase 2 direction remains D405-based 6D grasping
- project records are now separated from active ROS runtime config
- future calibration runs should no longer clutter the ROS workspace root

## 20260414_111747

Phase 2 direction change:
- changed the documented Phase 2 priority from D405-based cube sorting first to D405-based `grasp_6d` first
- updated the project Phase 2 plan so `alicia_d_grasp_6d` is now the primary milestone and `alicia_d_cube_sort` is treated as a fallback validation path

Current interpretation:
- the next major project objective is no longer structured cube sorting
- the immediate technical blocker is now third-party environment and model setup for `GraspGen`, `FoundationStereo`, and `sam2`

## 20260414_101916

Phase 2A launch alignment for D405-guided cube sorting:
- replaced the old `alicia_d_cube_sort/launch/cube_detection.launch.py` logic that rebuilt hand-eye TF inline with a simpler launch that can reuse `alicia_d_calibration/publish_hand_eye_tf.py`
- set the cube-detection launch defaults to the current D405 topics and depth-enabled localization path
- added `alicia_d_cube_sort/launch/phase2a_cube_sort.launch.py` as a D405-oriented wrapper for cube detection plus sorting, intended to run alongside the existing real-robot and camera launches
- added `alicia_d_calibration` as an execution dependency of `alicia_d_cube_sort`
- rebuilt `alicia_d_cube_sort`

Current interpretation:
- Phase 2A now has a clean launch entry point that matches the current Phase 1 baseline instead of the older duplicated calibration flow
- the next real validation step is to run D405-based cube detection and then supervised sorting against the real workspace

## 20260414_094819

Real-robot perception integration and calibration publication:
- removed the earlier autogenerated workspace-boundary overlay from the real-robot launch path
- simplified the planning-scene helper so it now loads only user-defined collision boxes from `alicia_d_moveit/config/world_scene.yaml`
- added a small `demonstration_toggle_ui.py` side window and bridged `/demonstration` into the ros2_control hardware path for hand-guiding while `real_robot.launch.py` is running
- added a dedicated `alicia_d_moveit/launch/d405.launch.py` wrapper for the upside-down Intel RealSense D405 with aligned depth, point cloud, and 180 degree image rotation enabled by default
- updated the hand-eye calibration scripts for OpenCV ArUco API compatibility across older and newer OpenCV builds
- hardened `verify_calibration.launch.py` so it skips empty/invalid YAML candidates and reports which calibration file it actually loads
- added `publish_hand_eye_tf.py` to publish the saved hand-eye calibration as a static TF, including optional optical-frame correction using the camera's internal TF
- integrated hand-eye TF publication into `alicia_d_moveit/launch/real_robot.launch.py` with launch arguments for enabling, file selection, optical frame, and optical correction
- rebuilt `alicia_d_calibration` and `alicia_d_moveit`

Current interpretation:
- the real-robot MoveIt stack now supports a custom planning scene, hand-guiding toggle UI, wrist-mounted D405 bring-up, completed hand-eye calibration, and automatic publication of the calibrated camera TF in one launch path
- Phase 1 bring-up is now operationally complete, with remaining work centered on calibration refinement and application-level perception/manipulation rather than basic connectivity

## 20260410_124935

Custom world-scene objects:
- extended the planning-scene helper so it can load user-defined collision boxes from a YAML file
- added `world_scene_file` to the real-robot launch
- added `config/world_scene.yaml` as the editable template for custom boxes
- kept `workspace_boundaries` as an optional overlay for conservative floor/wall limits
- rebuilt `alicia_d_moveit`

Current interpretation:
- the planning scene no longer has to be a single hardcoded boundary box model
- custom fixtures like tables, posts, bins, and keep-out boxes can now be added by editing the scene file directly

## 20260410_124200

Real-robot execution recovery:
- confirmed the earlier motion problems traced back to a failed servo rather than the planner stack
- confirmed OMPL planning and execution now succeed on the repaired arm
- fixed `publish_workspace_boundaries.py` so it exits cleanly without shutting down the global ROS context from inside its callback
- restored the full conservative workspace boundary set: floor, ceiling, front/rear walls, left/right walls
- rebuilt `alicia_d_moveit`

Current interpretation:
- the real arm is now reaching commanded poses through MoveIt at conservative speed
- remaining shutdown crashes in `move_group`/`rviz2` still look like upstream MoveIt/Humble teardown issues rather than motion-path failures

## 20260403_112130

MoveIt preview and workspace safety improvements:
- added a one-shot planning-scene helper that publishes conservative floor, ceiling, and side wall collision boundaries for the real-robot launch
- added real-robot launch arguments for enabling or tuning workspace boundary extents in `base_link`
- updated RViz MotionPlanning defaults so planned trajectories are easier to see:
  - enabled planned-path coloring
  - enabled path trail and loop animation
  - enlarged interactive markers
  - enabled workspace box visualization
  - adjusted workspace visualization extents and robot transparency
- rebuilt `alicia_d_moveit`

Current interpretation:
- if RViz still shows no meaningful planned-path preview after these display changes, the problem is more likely deeper state/calibration inconsistency than a simple visualization setting
- the new planning-scene walls are conservative generic safety bounds and should be tuned later to match the real table and surrounding hardware

## 20260403_110922

Startup jump mitigation:
- updated `AliciaDHardwareInterface` to remain passive after synchronization until the controller actually changes the commanded target
- added command-change detection so identical hold commands are not resent continuously while idle
- rebuilt `alicia_d_driver` and `alicia_d_moveit`

Current interpretation:
- the small startup twitch was likely caused by the hardware interface sending an immediate hold-position command on startup rather than by planning itself
- the real robot should now stay still on launch until a controller target actually changes

## 20260403_110345

Pilz planning enablement:
- added explicit acceleration limits to `alicia_d_moveit/config/joint_limits.yaml`
- rebuilt `alicia_d_moveit`

Current interpretation:
- the Pilz planning failures were due to missing acceleration limits, not because Pilz itself was unavailable
- OMPL planning was already functioning; Pilz now has the required dynamics metadata to plan as well

## 20260403_110000

MoveIt execution and planning-pipeline tuning:
- enabled `pilz_industrial_motion_planner` alongside `ompl` in the Alicia MoveIt config builder
- added real-robot launch arguments for `planning_pipeline` and `allowed_start_tolerance`
- set real-robot launch support for `trajectory_execution.allowed_start_tolerance`
- included Pilz cartesian limits in the generated MoveIt configuration and RViz parameters
- rebuilt `alicia_d_moveit`

Current interpretation:
- the recurring execution aborts are still primarily start-state validation problems, not proof that OMPL is failing to find paths
- Pilz is now available as a useful deterministic motion option, but better planning alone does not remove execution-start mismatch

## 20260403_105139

Hardware interface synchronization fix:
- updated `AliciaDHardwareInterface` to wait for a real measured joint state before enabling hardware writes
- removed the unsafe startup behavior where command buffers could begin from all-zero state on activation
- increased hardware joint-state polling from `20 Hz` to `100 Hz` to reduce start-state drift between planning and execution
- rebuilt `alicia_d_driver` and `alicia_d_moveit`

Current interpretation:
- the intermittent `Invalid Trajectory: start point deviates from current robot state more than 0.01` failures are more likely state-synchronization/control issues than immediate evidence of bad mechanical calibration
- calibration may still be worth checking later, but startup synchronization needed fixing first

## 20260403_102526

Runtime linking fix:
- updated `alicia_d_driver` CMake install/runtime path handling so installed binaries and plugin libraries can find sibling driver libraries reliably
- rebuilt `alicia_d_driver`
- verified `libalicia_d_hardware_interface.so` now resolves `libalicia_d_data_parser_control_lib.so` and `libserial_communicator_lib.so`
- verified `alicia_d_driver_node` now carries a runtime search path to the installed driver library directory

Phase 1 implication:
- the previous `libalicia_d_data_parser_control_lib.so: cannot open shared object file` failure should be resolved after rebuilding and relaunching from the updated workspace

## 20260403_101140

Real hardware validation:
- verified the Alicia-D driver connects successfully on `/dev/ttyACM0`
- verified firmware `6.1.0`, detected `50mm` gripper type, and self-check status `OK`
- verified `/joint_states` is published from the real arm
- verified small real-arm joint motion commands succeed
- verified real gripper actuation succeeds

Observed behavior:
- direct manual publishing to `/joint_commands` works but looks somewhat jumpy on the real arm
- joint-state updates show small quantized changes consistent with hardware servo resolution

Current interpretation:
- the real arm and gripper are controllable, satisfying the core Phase 1 bring-up requirement
- the next preferred validation path is `alicia_d_moveit real_robot.launch.py` at conservative speed for smoother real-hardware motion

Operational notes:
- fresh terminals must source `/home/li/alicia/ros2_ws/install/setup.bash` or workspace libraries may not be found at launch time
- if the current shell session lacks effective `dialout` access, the real-robot MoveIt launch falls back to demo mode and execution aborts because the planned start state does not match the fake hardware state

## 20260403_095551

Simulation validation:
- verified `ros2 launch alicia_d_moveit demo.launch.py gripper_type:=50mm` starts successfully
- verified RViz loads, controllers activate, and MoveIt reports the system is ready to plan
- verified planning and execution succeed in simulation
- verified dragging the end-effector in RViz causes the simulated arm to move as expected

Observed warnings and shutdown behavior:
- saw KDL warnings about root-link inertia on `base_link`
- saw a warning that `gripper_center` has visual geometry but no collision geometry
- saw an octomap warning about no 3D sensor plugin being configured
- saw a controller-manager realtime scheduling warning
- saw `rviz2` exit with `-11` and `move_group` hang/crash during shutdown after `Ctrl-C`

Current interpretation:
- simulation bring-up is successful
- the shutdown crash is worth noting but is not blocking Phase 1 simulation validation
- the next meaningful milestone is real arm/gripper validation, then D405 runtime validation
## 20260403_095244

Dependency and build update:
- installed the required Phase 1 ROS and system packages that are available from the local Humble apt repository
- verified `ros-humble-realsense2-camera` is installed and discoverable
- verified the full Alicia workspace builds successfully with `./build_system_python.sh`

Repository/package findings:
- confirmed `ros-humble-warehouse-ros-mongo` cannot be located in the current apt repository on this machine
- confirmed `ros-humble-warehouse-ros` and `ros-humble-moveit-ros-warehouse` are available instead
- confirmed the upstream Alicia package metadata still lists `warehouse_ros_mongo` as an `exec_depend`

Phase 1 implication:
- `warehouse_ros_mongo` is not blocking the current build
- it should be treated as optional unless MoveIt warehouse database support is specifically needed later

## 20260401_173355

Environment and workspace initialization:
- created the Alicia Phase 1 top-level structure
- created the main ROS 2 workspace at `ros2_ws/`
- cloned `Synria-Robotics/Alicia-D-ROS2` branch `v6.1.0` into `ros2_ws/src/alicia_d_ros2_upstream`
- mirrored the provided SparklingRobo and GitHub reference pages into `vendor/`

Validation and findings:
- verified the host baseline is `Ubuntu 22.04.5 LTS`
- verified `/opt/ros/humble` is installed
- verified `ros2` is available after sourcing Humble
- verified `realsense2_camera` is not currently installed
- verified the workspace package set is discoverable by `colcon list`
- identified missing ROS/system dependencies via `rosdep install --from-paths src --ignore-src -r -y -s`
- identified a local Conda Python conflict because the default `python3` resolves to `/home/li/miniconda3/bin/python3`
- verified `alicia_d_descriptions` builds successfully when forcing `/usr/bin/python3`

Blocked actions:
- attempted package installation for ROS and RealSense dependencies, but `sudo` requires an interactive password in this session
- full workspace build is still blocked on missing dependencies such as `hardware_interface`
