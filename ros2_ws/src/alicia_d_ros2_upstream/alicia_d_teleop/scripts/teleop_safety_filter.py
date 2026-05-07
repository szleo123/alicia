#!/usr/bin/env python3

import math
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


VALID_MODES = ("hold", "jog", "approach", "grip", "retreat")


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def clamp_vector(values: List[float], max_norm: float) -> List[float]:
    norm = math.sqrt(sum(value * value for value in values))
    if max_norm <= 0.0:
        return [0.0 for _ in values]
    if norm <= max_norm or norm <= 1e-12:
        return values
    scale = max_norm / norm
    return [value * scale for value in values]


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class TeleopSafetyFilter(Node):
    """Apply mode, deadman, workspace, timeout, and smoothing limits to teleop twists."""

    def __init__(self):
        super().__init__("teleop_safety_filter")

        self.declare_parameter("input_twist_topic", "/alicia_d_teleop/raw_twist_cmd")
        self.declare_parameter("output_twist_topic", "/alicia_d_teleop/twist_cmd")
        self.declare_parameter("input_buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("mode_command_topic", "/alicia_d_teleop/mode_command")
        self.declare_parameter("status_topic", "/alicia_d_teleop/safety_status")
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("ee_frame_name", "tool0")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("input_timeout_s", 0.25)
        self.declare_parameter("button_timeout_s", 0.25)
        self.declare_parameter("deadman_button_index", 0)
        self.declare_parameter("default_mode", "jog")
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("require_tool_tf", False)
        self.declare_parameter("low_pass_alpha", 0.25)
        self.declare_parameter("max_linear_accel_m_s2", 0.08)
        self.declare_parameter("max_angular_accel_rad_s2", 0.30)
        self.declare_parameter("workspace_min", [-0.35, -0.45, 0.02])
        self.declare_parameter("workspace_max", [0.55, 0.45, 0.65])
        self.declare_parameter("retreat_twist_linear", [-0.03, 0.0, 0.03])
        self.declare_parameter("retreat_twist_angular", [0.0, 0.0, 0.0])
        self.declare_parameter("jog_max_linear_m_s", 0.05)
        self.declare_parameter("jog_max_angular_rad_s", 0.20)
        self.declare_parameter("approach_max_linear_m_s", 0.015)
        self.declare_parameter("approach_max_angular_rad_s", 0.08)
        self.declare_parameter("grip_max_linear_m_s", 0.0)
        self.declare_parameter("grip_max_angular_rad_s", 0.0)
        self.declare_parameter("retreat_max_linear_m_s", 0.03)
        self.declare_parameter("retreat_max_angular_rad_s", 0.08)

        self.input_twist_topic = self.get_parameter("input_twist_topic").value
        self.output_twist_topic = self.get_parameter("output_twist_topic").value
        self.input_buttons_topic = self.get_parameter("input_buttons_topic").value
        self.mode_command_topic = self.get_parameter("mode_command_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.command_frame = self.get_parameter("command_frame").value
        self.ee_frame_name = self.get_parameter("ee_frame_name").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.button_timeout_s = float(self.get_parameter("button_timeout_s").value)
        self.deadman_button_index = int(self.get_parameter("deadman_button_index").value)
        self.mode = self.normalize_mode(str(self.get_parameter("default_mode").value))
        self.require_deadman = as_bool(self.get_parameter("require_deadman").value)
        self.require_tool_tf = as_bool(self.get_parameter("require_tool_tf").value)
        self.low_pass_alpha = max(0.0, min(1.0, float(self.get_parameter("low_pass_alpha").value)))
        self.max_linear_accel = max(0.0, float(self.get_parameter("max_linear_accel_m_s2").value))
        self.max_angular_accel = max(0.0, float(self.get_parameter("max_angular_accel_rad_s2").value))
        self.workspace_min = self.read_vector_param("workspace_min")
        self.workspace_max = self.read_vector_param("workspace_max")
        self.retreat_linear = self.read_vector_param("retreat_twist_linear")
        self.retreat_angular = self.read_vector_param("retreat_twist_angular")

        self.mode_limits: Dict[str, tuple[float, float]] = {
            "hold": (0.0, 0.0),
            "jog": (
                float(self.get_parameter("jog_max_linear_m_s").value),
                float(self.get_parameter("jog_max_angular_rad_s").value),
            ),
            "approach": (
                float(self.get_parameter("approach_max_linear_m_s").value),
                float(self.get_parameter("approach_max_angular_rad_s").value),
            ),
            "grip": (
                float(self.get_parameter("grip_max_linear_m_s").value),
                float(self.get_parameter("grip_max_angular_rad_s").value),
            ),
            "retreat": (
                float(self.get_parameter("retreat_max_linear_m_s").value),
                float(self.get_parameter("retreat_max_angular_rad_s").value),
            ),
        }

        self.latest_twist: Optional[TwistStamped] = None
        self.latest_twist_time = self.get_clock().now()
        self.buttons: List[int] = []
        self.latest_button_time = self.get_clock().now()
        self.filtered_linear = [0.0, 0.0, 0.0]
        self.filtered_angular = [0.0, 0.0, 0.0]
        self.last_publish_time = self.get_clock().now()
        self.last_block_reason = "startup_hold"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, self.output_twist_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(TwistStamped, self.input_twist_topic, self.twist_callback, 10)
        self.create_subscription(Joy, self.input_buttons_topic, self.buttons_callback, 10)
        self.create_subscription(String, self.mode_command_topic, self.mode_callback, 10)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(period, self.timer_callback)

        self.get_logger().warn(
            "Teleop safety filter ready: raw=%s filtered=%s mode=%s workspace=%s..%s"
            % (
                self.input_twist_topic,
                self.output_twist_topic,
                self.mode,
                self.workspace_min,
                self.workspace_max,
            )
        )

    def read_vector_param(self, name: str) -> List[float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must have exactly three values")
        return [float(value) for value in values]

    def normalize_mode(self, mode: str) -> str:
        mode = mode.strip().lower()
        if mode == "stop":
            mode = "hold"
        if mode not in VALID_MODES:
            self.get_logger().warn(f"Invalid teleop mode '{mode}', using hold.")
            return "hold"
        return mode

    def twist_callback(self, msg: TwistStamped) -> None:
        self.latest_twist = msg
        self.latest_twist_time = self.get_clock().now()

    def buttons_callback(self, msg: Joy) -> None:
        self.buttons = [1 if value else 0 for value in msg.buttons]
        self.latest_button_time = self.get_clock().now()

    def mode_callback(self, msg: String) -> None:
        new_mode = self.normalize_mode(msg.data)
        if new_mode != self.mode:
            self.get_logger().warn(f"Teleop mode changed: {self.mode} -> {new_mode}")
            self.mode = new_mode
            self.filtered_linear = [0.0, 0.0, 0.0]
            self.filtered_angular = [0.0, 0.0, 0.0]

    def deadman_active(self) -> bool:
        if not self.require_deadman:
            return True
        fresh = (self.get_clock().now() - self.latest_button_time) < Duration(seconds=self.button_timeout_s)
        if not fresh:
            return False
        return 0 <= self.deadman_button_index < len(self.buttons) and bool(self.buttons[self.deadman_button_index])

    def twist_fresh(self) -> bool:
        return self.latest_twist is not None and (
            self.get_clock().now() - self.latest_twist_time
        ) < Duration(seconds=self.input_timeout_s)

    def get_tool_position(self) -> Optional[List[float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.command_frame,
                self.ee_frame_name,
                Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.command_frame}->{self.ee_frame_name}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        translation = transform.transform.translation
        return [translation.x, translation.y, translation.z]

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_publish_time).nanoseconds * 1e-9, 1.0 / max(self.publish_rate_hz, 1.0))
        self.last_publish_time = now

        linear, angular, reason = self.compute_target()
        linear = self.apply_accel_limit(self.filtered_linear, linear, self.max_linear_accel, dt)
        angular = self.apply_accel_limit(self.filtered_angular, angular, self.max_angular_accel, dt)

        alpha = self.low_pass_alpha
        self.filtered_linear = [
            alpha * target + (1.0 - alpha) * previous
            for target, previous in zip(linear, self.filtered_linear)
        ]
        self.filtered_angular = [
            alpha * target + (1.0 - alpha) * previous
            for target, previous in zip(angular, self.filtered_angular)
        ]
        self.last_block_reason = reason
        self.publish_twist(self.filtered_linear, self.filtered_angular)
        self.publish_status()

    def compute_target(self) -> tuple[List[float], List[float], str]:
        if self.mode == "hold":
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "hold_mode"

        if not self.deadman_active():
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "deadman_released"

        if self.mode == "grip":
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "grip_mode_motion_frozen"

        if self.mode == "retreat":
            linear = list(self.retreat_linear)
            angular = list(self.retreat_angular)
        else:
            if not self.twist_fresh() or self.latest_twist is None:
                return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "input_timeout"
            twist = self.latest_twist.twist
            linear = [twist.linear.x, twist.linear.y, twist.linear.z]
            angular = [twist.angular.x, twist.angular.y, twist.angular.z]

        max_linear, max_angular = self.mode_limits[self.mode]
        linear = clamp_vector(linear, max_linear)
        angular = clamp_vector(angular, max_angular)

        tool_position = self.get_tool_position()
        if tool_position is None:
            if self.require_tool_tf:
                return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "missing_tool_tf"
            return linear, angular, "tf_unavailable_workspace_not_checked"

        linear = self.apply_workspace_limits(tool_position, linear)
        if all(abs(value) <= 1e-9 for value in linear) and max_linear > 0.0:
            return linear, angular, "workspace_boundary"

        return linear, angular, "allowed"

    def apply_workspace_limits(self, position: List[float], linear: List[float]) -> List[float]:
        limited = list(linear)
        for index in range(3):
            if position[index] <= self.workspace_min[index] and limited[index] < 0.0:
                limited[index] = 0.0
            if position[index] >= self.workspace_max[index] and limited[index] > 0.0:
                limited[index] = 0.0
        return limited

    def apply_accel_limit(self, previous: List[float], target: List[float], accel_limit: float, dt: float) -> List[float]:
        if accel_limit <= 0.0:
            return target
        max_delta = accel_limit * dt
        return [
            previous_value + clamp(target_value - previous_value, max_delta)
            for previous_value, target_value in zip(previous, target)
        ]

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

    def publish_status(self) -> None:
        msg = String()
        msg.data = (
            f"mode={self.mode}; deadman={int(self.deadman_active())}; "
            f"raw_fresh={int(self.twist_fresh())}; reason={self.last_block_reason}; "
            f"output={self.output_twist_topic}"
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopSafetyFilter()
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
