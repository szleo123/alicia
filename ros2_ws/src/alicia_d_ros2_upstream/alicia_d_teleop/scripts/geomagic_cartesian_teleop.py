#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def clamp_unit(value: float) -> float:
    return max(0.0, min(1.0, value))


def clamp_vector(values: List[float], max_norm: float) -> List[float]:
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= max_norm or norm <= 1e-12:
        return values
    scale = max_norm / norm
    return [value * scale for value in values]


def apply_deadband(value: float, deadband: float) -> float:
    deadband = max(0.0, deadband)
    magnitude = abs(value)
    if magnitude <= deadband:
        return 0.0
    return math.copysign(magnitude - deadband, value)


def apply_vector_deadband(values: List[float], deadband: float) -> List[float]:
    return [apply_deadband(value, deadband) for value in values]


def normalize_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def inverse_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = normalize_quaternion(q)
    return (-x, -y, -z, w)


def multiply_quaternion(
    lhs: Tuple[float, float, float, float],
    rhs: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = lhs
    x2, y2, z2, w2 = rhs
    return normalize_quaternion((
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ))


def quaternion_to_rotvec(q: Tuple[float, float, float, float]) -> List[float]:
    x, y, z, w = normalize_quaternion(q)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half <= 1e-9:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(sin_half, w)
    return [angle * x / sin_half, angle * y / sin_half, angle * z / sin_half]


def rotvec_to_quaternion(rotvec: List[float]) -> Tuple[float, float, float, float]:
    angle = math.sqrt(sum(value * value for value in rotvec))
    if angle <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    half_angle = 0.5 * angle
    scale = math.sin(half_angle) / angle
    return normalize_quaternion((
        rotvec[0] * scale,
        rotvec[1] * scale,
        rotvec[2] * scale,
        math.cos(half_angle),
    ))


def parse_axis_token(token: str) -> Tuple[int, float]:
    token = token.strip().lower()
    sign = -1.0 if token.startswith("-") else 1.0
    axis = token[1:] if token.startswith("-") else token
    axis_map = {"x": 0, "y": 1, "z": 2}
    if axis not in axis_map:
        raise ValueError(f"Invalid axis token '{token}'. Use x, y, z, -x, -y, or -z.")
    return axis_map[axis], sign


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class GeomagicCartesianTeleop(Node):
    """Convert clutched stylus pose motion into bounded Cartesian twist commands."""

    def __init__(self):
        super().__init__("geomagic_cartesian_teleop")

        self.declare_parameter("input_pose_topic", "/geomagic_touch/pose")
        self.declare_parameter("input_buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("output_twist_topic", "/alicia_d_teleop/twist_cmd")
        self.declare_parameter("status_topic", "/alicia_d_teleop/status")
        self.declare_parameter("gripper_command_topic", "/alicia_d_teleop/gripper_command")
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("input_timeout_s", 0.25)
        self.declare_parameter("deadman_button_index", 0)
        self.declare_parameter("gripper_button_index", 1)
        self.declare_parameter("gripper_enabled", False)
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_closed_position", 0.03)
        self.declare_parameter("gripper_start_open", True)
        self.declare_parameter("ee_frame_name", "tool0")
        self.declare_parameter("translation_gain", 1.0)
        self.declare_parameter("translation_deadband_m", 0.0)
        self.declare_parameter("rotation_gain", 1.0)
        self.declare_parameter("orientation_deadband_rad", 0.0)
        self.declare_parameter("max_linear_speed_m_s", 0.05)
        self.declare_parameter("max_angular_speed_rad_s", 0.35)
        self.declare_parameter("low_pass_alpha", 0.25)
        self.declare_parameter("orientation_enabled", False)
        self.declare_parameter("angular_control_mode", "orientation_follow")
        self.declare_parameter("robot_axes_from_device", ["x", "y", "z"])
        self.declare_parameter("robot_angular_axes_from_device", ["x", "y", "z"])
        self.declare_parameter("zero_on_release", True)

        self.input_pose_topic = self.get_parameter("input_pose_topic").value
        self.input_buttons_topic = self.get_parameter("input_buttons_topic").value
        self.output_twist_topic = self.get_parameter("output_twist_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.gripper_command_topic = self.get_parameter("gripper_command_topic").value
        self.command_frame = self.get_parameter("command_frame").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.deadman_button_index = int(self.get_parameter("deadman_button_index").value)
        self.gripper_button_index = int(self.get_parameter("gripper_button_index").value)
        self.gripper_enabled = as_bool(self.get_parameter("gripper_enabled").value)
        self.gripper_open_position = float(self.get_parameter("gripper_open_position").value)
        self.gripper_closed_position = float(self.get_parameter("gripper_closed_position").value)
        self.gripper_open = as_bool(self.get_parameter("gripper_start_open").value)
        self.ee_frame_name = str(self.get_parameter("ee_frame_name").value)
        self.translation_gain = float(self.get_parameter("translation_gain").value)
        self.translation_deadband = float(self.get_parameter("translation_deadband_m").value)
        self.rotation_gain = float(self.get_parameter("rotation_gain").value)
        self.orientation_deadband = float(self.get_parameter("orientation_deadband_rad").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed_m_s").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed_rad_s").value)
        self.low_pass_alpha = clamp_unit(float(self.get_parameter("low_pass_alpha").value))
        self.orientation_enabled = as_bool(self.get_parameter("orientation_enabled").value)
        self.angular_control_mode = str(self.get_parameter("angular_control_mode").value)
        self.zero_on_release = as_bool(self.get_parameter("zero_on_release").value)

        axis_tokens = list(self.get_parameter("robot_axes_from_device").value)
        if len(axis_tokens) != 3:
            raise ValueError("robot_axes_from_device must have exactly three axis tokens.")
        self.axis_map = [parse_axis_token(token) for token in axis_tokens]
        angular_axis_tokens = list(self.get_parameter("robot_angular_axes_from_device").value)
        if len(angular_axis_tokens) != 3:
            raise ValueError("robot_angular_axes_from_device must have exactly three axis tokens.")
        self.angular_axis_map = [parse_axis_token(token) for token in angular_axis_tokens]

        self.latest_pose: Optional[PoseStamped] = None
        self.latest_pose_time = self.get_clock().now()
        self.buttons: List[int] = []
        self.previous_buttons: List[int] = []
        self.clutch_anchor: Optional[PoseStamped] = None
        self.robot_anchor_orientation: Optional[Tuple[float, float, float, float]] = None
        self.filtered_linear = [0.0, 0.0, 0.0]
        self.filtered_angular = [0.0, 0.0, 0.0]
        self.last_deadman = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, self.output_twist_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, self.gripper_command_topic, 10)
        self.create_subscription(PoseStamped, self.input_pose_topic, self.pose_callback, 10)
        self.create_subscription(Joy, self.input_buttons_topic, self.buttons_callback, 10)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"Geomagic teleop ready: pose={self.input_pose_topic} "
            f"buttons={self.input_buttons_topic} output={self.output_twist_topic} "
            f"frame={self.command_frame} mode=velocity angular={self.angular_control_mode}"
        )
        gripper_state = "enabled" if self.gripper_enabled else "disabled"
        self.get_logger().info(
            f"Motion requires deadman button {self.deadman_button_index}. "
            f"Gripper output is {gripper_state}."
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.latest_pose_time = self.get_clock().now()

    def buttons_callback(self, msg: Joy) -> None:
        self.previous_buttons = self.buttons
        self.buttons = [1 if value else 0 for value in msg.buttons]
        if self.button_pressed_edge(self.gripper_button_index):
            self.toggle_gripper()

    def button_active(self, index: int) -> bool:
        return 0 <= index < len(self.buttons) and bool(self.buttons[index])

    def button_pressed_edge(self, index: int) -> bool:
        current = 0 <= index < len(self.buttons) and bool(self.buttons[index])
        previous = 0 <= index < len(self.previous_buttons) and bool(self.previous_buttons[index])
        return current and not previous

    def timer_callback(self) -> None:
        deadman = self.button_active(self.deadman_button_index)
        fresh = self.latest_pose is not None and (
            self.get_clock().now() - self.latest_pose_time
        ) < Duration(seconds=self.input_timeout_s)

        if deadman and fresh:
            if not self.last_deadman or self.clutch_anchor is None:
                self.clutch_anchor = self.latest_pose
                self.robot_anchor_orientation = None
                if self.orientation_enabled and self.angular_control_mode == "orientation_follow":
                    self.robot_anchor_orientation = self.get_robot_tool_orientation()
                self.filtered_linear = [0.0, 0.0, 0.0]
                self.filtered_angular = [0.0, 0.0, 0.0]
                self.publish_status("clutched")
            self.publish_motion_command()
        else:
            self.clutch_anchor = None
            self.robot_anchor_orientation = None
            self.filtered_linear = [0.0, 0.0, 0.0]
            self.filtered_angular = [0.0, 0.0, 0.0]
            if self.zero_on_release:
                self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.publish_status("idle" if fresh else "waiting_for_input")

        self.last_deadman = deadman

    def get_robot_tool_orientation(self) -> Optional[Tuple[float, float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.command_frame,
                self.ee_frame_name,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.command_frame}->{self.ee_frame_name}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        rotation = transform.transform.rotation
        return normalize_quaternion((rotation.x, rotation.y, rotation.z, rotation.w))

    def publish_motion_command(self) -> None:
        if self.latest_pose is None or self.clutch_anchor is None:
            return

        current = self.latest_pose.pose
        anchor = self.clutch_anchor.pose
        device_delta = [
            current.position.x - anchor.position.x,
            current.position.y - anchor.position.y,
            current.position.z - anchor.position.z,
        ]
        mapped_delta = [
            sign * device_delta[index]
            for index, sign in self.axis_map
        ]
        mapped_delta = apply_vector_deadband(mapped_delta, self.translation_deadband)
        linear = [self.translation_gain * value for value in mapped_delta]
        linear = clamp_vector(linear, self.max_linear_speed)

        angular = [0.0, 0.0, 0.0]
        if self.orientation_enabled:
            angular = self.compute_angular_command(current, anchor)

        alpha = self.low_pass_alpha
        self.filtered_linear = [
            alpha * target + (1.0 - alpha) * previous
            for target, previous in zip(linear, self.filtered_linear)
        ]
        self.filtered_angular = [
            alpha * target + (1.0 - alpha) * previous
            for target, previous in zip(angular, self.filtered_angular)
        ]

        self.publish_twist(self.filtered_linear, self.filtered_angular)
        self.publish_status("moving")

    def compute_angular_command(self, current, anchor) -> List[float]:
        q_current = (
            current.orientation.x,
            current.orientation.y,
            current.orientation.z,
            current.orientation.w,
        )
        q_anchor = (
            anchor.orientation.x,
            anchor.orientation.y,
            anchor.orientation.z,
            anchor.orientation.w,
        )
        q_delta = multiply_quaternion(inverse_quaternion(q_anchor), q_current)
        device_rotvec = quaternion_to_rotvec(q_delta)
        mapped_rotvec = [
            sign * device_rotvec[index]
            for index, sign in self.angular_axis_map
        ]
        mapped_rotvec = apply_vector_deadband(mapped_rotvec, self.orientation_deadband)

        if self.angular_control_mode != "orientation_follow":
            angular = [self.rotation_gain * value for value in mapped_rotvec]
            return clamp_vector(angular, self.max_angular_speed)

        if self.robot_anchor_orientation is None:
            self.robot_anchor_orientation = self.get_robot_tool_orientation()
        robot_current_orientation = self.get_robot_tool_orientation()
        if self.robot_anchor_orientation is None or robot_current_orientation is None:
            return [0.0, 0.0, 0.0]

        q_delta_robot = rotvec_to_quaternion(mapped_rotvec)
        q_desired = multiply_quaternion(self.robot_anchor_orientation, q_delta_robot)
        q_error = multiply_quaternion(q_desired, inverse_quaternion(robot_current_orientation))
        error_rotvec = quaternion_to_rotvec(q_error)
        angular = [self.rotation_gain * value for value in error_rotvec]
        return clamp_vector(angular, self.max_angular_speed)

    def publish_twist(self, linear: List[float], angular: List[float]) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x = linear[0]
        msg.twist.linear.y = linear[1]
        msg.twist.linear.z = linear[2]
        msg.twist.angular.x = angular[0]
        msg.twist.angular.y = angular[1]
        msg.twist.angular.z = angular[2]
        self.twist_pub.publish(msg)

    def toggle_gripper(self) -> None:
        self.gripper_open = not self.gripper_open
        if not self.gripper_enabled:
            self.publish_status("gripper_toggle_ignored")
            return

        target = self.gripper_open_position if self.gripper_open else self.gripper_closed_position
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["Gripper"]
        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start.sec = 1
        msg.points = [point]
        self.gripper_pub.publish(msg)
        self.publish_status("gripper_open" if self.gripper_open else "gripper_closed")

    def publish_status(self, state: str) -> None:
        msg = String()
        msg.data = (
            f"state={state}; deadman={int(self.button_active(self.deadman_button_index))}; "
            f"mode=velocity; angular={self.angular_control_mode}; "
            f"gripper_open={int(self.gripper_open)}; "
            f"output={self.output_twist_topic}"
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = GeomagicCartesianTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
