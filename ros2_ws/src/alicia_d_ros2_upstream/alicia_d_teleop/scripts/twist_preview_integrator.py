#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from tf2_ros import TransformBroadcaster


def normalize_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


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


def rotvec_to_quaternion(rotvec: List[float]) -> Tuple[float, float, float, float]:
    angle = math.sqrt(sum(value * value for value in rotvec))
    if angle <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    scale = math.sin(angle / 2.0) / angle
    return normalize_quaternion((
        rotvec[0] * scale,
        rotvec[1] * scale,
        rotvec[2] * scale,
        math.cos(angle / 2.0),
    ))


class TwistPreviewIntegrator(Node):
    """Integrate teleop TwistStamped commands into a preview pose and TF."""

    def __init__(self):
        super().__init__("twist_preview_integrator")

        self.declare_parameter("input_twist_topic", "/alicia_d_teleop/twist_cmd")
        self.declare_parameter("output_pose_topic", "/alicia_d_teleop/preview_pose")
        self.declare_parameter("output_path_topic", "/alicia_d_teleop/preview_path")
        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "teleop_preview_tip")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("input_timeout_s", 0.25)
        self.declare_parameter("path_max_poses", 400)
        self.declare_parameter("initial_position", [0.25, 0.0, 0.25])

        self.input_twist_topic = self.get_parameter("input_twist_topic").value
        self.output_pose_topic = self.get_parameter("output_pose_topic").value
        self.output_path_topic = self.get_parameter("output_path_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.path_max_poses = int(self.get_parameter("path_max_poses").value)
        initial_position = list(self.get_parameter("initial_position").value)
        if len(initial_position) != 3:
            raise ValueError("initial_position must contain three values")

        self.position = [float(value) for value in initial_position]
        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.latest_twist: Optional[TwistStamped] = None
        self.latest_twist_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.path = Path()
        self.path.header.frame_id = self.parent_frame

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.path_pub = self.create_publisher(Path, self.output_path_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf_enabled else None
        self.create_subscription(TwistStamped, self.input_twist_topic, self.twist_callback, 10)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f"Teleop preview integrating {self.input_twist_topic} into "
            f"{self.output_pose_topic} and TF {self.parent_frame}->{self.child_frame}"
        )

    def twist_callback(self, msg: TwistStamped) -> None:
        self.latest_twist = msg
        self.latest_twist_time = self.get_clock().now()

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        fresh = self.latest_twist is not None and (
            now - self.latest_twist_time
        ) < Duration(seconds=self.input_timeout_s)

        if fresh and self.latest_twist is not None:
            twist = self.latest_twist.twist
            self.position[0] += twist.linear.x * dt
            self.position[1] += twist.linear.y * dt
            self.position[2] += twist.linear.z * dt

            dq = rotvec_to_quaternion([
                twist.angular.x * dt,
                twist.angular.y * dt,
                twist.angular.z * dt,
            ])
            self.orientation = multiply_quaternion(dq, self.orientation)

        pose = self.make_pose(now)
        self.pose_pub.publish(pose)
        self.publish_path(pose)
        self.publish_tf(pose)

    def make_pose(self, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = self.parent_frame
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1]
        pose.pose.position.z = self.position[2]
        pose.pose.orientation.x = self.orientation[0]
        pose.pose.orientation.y = self.orientation[1]
        pose.pose.orientation.z = self.orientation[2]
        pose.pose.orientation.w = self.orientation[3]
        return pose

    def publish_path(self, pose: PoseStamped) -> None:
        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.path_max_poses:
            self.path.poses = self.path.poses[-self.path_max_poses:]
        self.path_pub.publish(self.path)

    def publish_tf(self, pose: PoseStamped) -> None:
        if self.tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = TwistPreviewIntegrator()
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
