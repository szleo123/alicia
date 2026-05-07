#!/usr/bin/env python3

import math
from typing import List, Optional

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory


class TrajectoryDeadmanGate(Node):
    """Forward Servo trajectories only when armed and deadman-held."""

    def __init__(self):
        super().__init__("trajectory_deadman_gate")

        self.declare_parameter("input_trajectory_topic", "/alicia_d_teleop/servo_raw_joint_trajectory")
        self.declare_parameter("output_trajectory_topic", "/Alicia_controller/joint_trajectory")
        self.declare_parameter("input_twist_topic", "/alicia_d_teleop/twist_cmd")
        self.declare_parameter("input_buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("status_topic", "/alicia_d_teleop/trajectory_gate_status")
        self.declare_parameter("deadman_button_index", 0)
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("require_nonzero_twist", True)
        self.declare_parameter("button_timeout_s", 0.25)
        self.declare_parameter("twist_timeout_s", 0.25)
        self.declare_parameter("min_linear_speed_m_s", 1.0e-5)
        self.declare_parameter("min_angular_speed_rad_s", 1.0e-4)
        self.declare_parameter("armed_on_start", False)

        self.input_trajectory_topic = self.get_parameter("input_trajectory_topic").value
        self.output_trajectory_topic = self.get_parameter("output_trajectory_topic").value
        self.input_twist_topic = self.get_parameter("input_twist_topic").value
        self.input_buttons_topic = self.get_parameter("input_buttons_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.deadman_button_index = int(self.get_parameter("deadman_button_index").value)
        self.require_deadman = bool(self.get_parameter("require_deadman").value)
        self.require_nonzero_twist = bool(self.get_parameter("require_nonzero_twist").value)
        self.button_timeout_s = float(self.get_parameter("button_timeout_s").value)
        self.twist_timeout_s = float(self.get_parameter("twist_timeout_s").value)
        self.min_linear_speed = float(self.get_parameter("min_linear_speed_m_s").value)
        self.min_angular_speed = float(self.get_parameter("min_angular_speed_rad_s").value)
        self.armed = bool(self.get_parameter("armed_on_start").value)

        self.buttons: List[int] = []
        self.latest_button_time = self.get_clock().now()
        self.latest_twist: Optional[TwistStamped] = None
        self.latest_twist_time = self.get_clock().now()
        self.forwarded_count = 0
        self.blocked_count = 0

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.output_trajectory_topic,
            10,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(JointTrajectory, self.input_trajectory_topic, self.trajectory_callback, 10)
        self.create_subscription(TwistStamped, self.input_twist_topic, self.twist_callback, 10)
        self.create_subscription(Joy, self.input_buttons_topic, self.buttons_callback, 10)
        self.create_service(SetBool, "~/set_armed", self.set_armed_callback)
        self.create_timer(0.5, self.publish_status)

        self.get_logger().warn(
            "Trajectory gate active: input=%s output=%s twist=%s armed=%s require_deadman=%s require_nonzero_twist=%s"
            % (
                self.input_trajectory_topic,
                self.output_trajectory_topic,
                self.input_twist_topic,
                self.armed,
                self.require_deadman,
                self.require_nonzero_twist,
            )
        )

    def buttons_callback(self, msg: Joy) -> None:
        self.buttons = [1 if value else 0 for value in msg.buttons]
        self.latest_button_time = self.get_clock().now()

    def twist_callback(self, msg: TwistStamped) -> None:
        self.latest_twist = msg
        self.latest_twist_time = self.get_clock().now()

    def deadman_active(self) -> bool:
        if not self.require_deadman:
            return True
        fresh = (self.get_clock().now() - self.latest_button_time) < Duration(seconds=self.button_timeout_s)
        if not fresh:
            return False
        return 0 <= self.deadman_button_index < len(self.buttons) and bool(self.buttons[self.deadman_button_index])

    def command_active(self) -> bool:
        if not self.require_nonzero_twist:
            return True
        if self.latest_twist is None:
            return False
        fresh = (self.get_clock().now() - self.latest_twist_time) < Duration(seconds=self.twist_timeout_s)
        if not fresh:
            return False
        linear = self.latest_twist.twist.linear
        angular = self.latest_twist.twist.angular
        linear_norm = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        angular_norm = math.sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z)
        return linear_norm > self.min_linear_speed or angular_norm > self.min_angular_speed

    def trajectory_callback(self, msg: JointTrajectory) -> None:
        if self.armed and self.deadman_active() and self.command_active():
            self.trajectory_pub.publish(msg)
            self.forwarded_count += 1
        else:
            self.blocked_count += 1

    def set_armed_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.armed = bool(request.data)
        state = "armed" if self.armed else "disarmed"
        response.success = True
        response.message = f"trajectory gate {state}"
        self.publish_status()
        return response

    def publish_status(self) -> None:
        msg = String()
        msg.data = (
            f"armed={int(self.armed)}; deadman={int(self.deadman_active())}; "
            f"command={int(self.command_active())}; "
            f"input={self.input_trajectory_topic}; output={self.output_trajectory_topic}; "
            f"forwarded={self.forwarded_count}; blocked={self.blocked_count}"
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = TrajectoryDeadmanGate()
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
