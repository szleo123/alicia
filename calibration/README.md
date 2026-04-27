# Calibration Records

Runtime calibration files live in the ROS packages that install them. Preserve raw
records, copied results, and fitting outputs here so the ROS workspace root stays
clean.

Active runtime files:
- Hand-eye calibration: `ros2_ws/src/alicia_d_ros2_upstream/alicia_d_calibration/config/hand_eye_calibration_result.yaml`
- Kinematic calibration: `ros2_ws/src/alicia_d_ros2_upstream/alicia_d_moveit/config/kinematic_calibration.yaml`

Preserved records:
- `hand_eye/` stores copied hand-eye calibration results.
- `kinematic/` stores pivot, plane, joint-offset, TCP, and bundle-fit records.

Calibration scripts should write new records under timestamped directories using
`YYYYMMDD_HHMMSS` names or folders.
