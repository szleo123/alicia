#!/usr/bin/env python3

import time
from typing import Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformException, TransformListener


class JointTrajectoryProbeMeasure(Node):
    def __init__(self):
        super().__init__("joint_trajectory_probe_measure")

        self.declare_parameter("trajectory_topic", "/Alicia_controller/joint_trajectory")
        self.declare_parameter("trajectory_action", "/Alicia_controller/follow_joint_trajectory")
        self.declare_parameter("command_mode", "action")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("joint_name", "Joint3")
        self.declare_parameter("joint_delta_rad", 0.02)
        self.declare_parameter("duration_s", 2.0)
        self.declare_parameter("settle_s", 0.5)
        self.declare_parameter("restore_start", False)
        self.declare_parameter("print_all_joints", True)

        self.trajectory_topic = self.get_parameter("trajectory_topic").value
        self.trajectory_action = self.get_parameter("trajectory_action").value
        self.command_mode = self.get_parameter("command_mode").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tool_frame = self.get_parameter("tool_frame").value
        self.joint_name = self.get_parameter("joint_name").value
        self.joint_delta_rad = float(self.get_parameter("joint_delta_rad").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.settle_s = float(self.get_parameter("settle_s").value)
        self.restore_start = bool(self.get_parameter("restore_start").value)
        self.print_all_joints = bool(self.get_parameter("print_all_joints").value)

        self.joints = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        if self.joint_name not in self.joints:
            raise ValueError(f"joint_name must be one of {self.joints}")

        self.latest_joint_state: Optional[JointState] = None
        self.pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, self.trajectory_action)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def wait_for_joint_state(self) -> Dict[str, float]:
        deadline = self.get_clock().now() + Duration(seconds=3.0)
        while rclpy.ok() and self.get_clock().now() < deadline:
            if self.latest_joint_state is not None:
                positions = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
                if all(joint in positions for joint in self.joints):
                    return positions
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError("Timed out waiting for /joint_states")

    def lookup_position(self):
        deadline = self.get_clock().now() + Duration(seconds=3.0)
        while rclpy.ok() and self.get_clock().now() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.tool_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
                t = transform.transform.translation
                return [t.x, t.y, t.z]
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"Timed out waiting for TF {self.base_frame}->{self.tool_frame}")

    def make_duration(self, duration_s: float) -> DurationMsg:
        sec = int(duration_s)
        return DurationMsg(
            sec=sec,
            nanosec=int((duration_s - sec) * 1e9),
        )

    def build_trajectory(
        self,
        start_positions: Dict[str, float],
        target_positions: Dict[str, float],
        duration_s: float,
    ) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = self.joints

        start_point = JointTrajectoryPoint()
        start_point.positions = [start_positions[joint] for joint in self.joints]
        start_point.velocities = [0.0] * len(self.joints)
        start_point.time_from_start = self.make_duration(0.1)

        target_point = JointTrajectoryPoint()
        target_point.positions = [target_positions[joint] for joint in self.joints]
        target_point.velocities = [0.0] * len(self.joints)
        target_point.time_from_start = self.make_duration(duration_s)

        msg.points.append(start_point)
        msg.points.append(target_point)
        return msg

    def send_trajectory_topic(
        self,
        start_positions: Dict[str, float],
        target_positions: Dict[str, float],
        duration_s: float,
    ):
        msg = self.build_trajectory(start_positions, target_positions, duration_s)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def send_trajectory_action(
        self,
        start_positions: Dict[str, float],
        target_positions: Dict[str, float],
        duration_s: float,
    ):
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f"Timed out waiting for action {self.trajectory_action}")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = self.build_trajectory(start_positions, target_positions, duration_s)
        goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=3.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Trajectory goal was rejected by {self.trajectory_action}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_s + self.settle_s + 3.0)
        result = result_future.result()
        if result is None:
            raise RuntimeError(f"Timed out waiting for result from {self.trajectory_action}")
        self.get_logger().warn(
            "Action result: status=%d error_code=%d error_string=%s"
            % (result.status, result.result.error_code, result.result.error_string)
        )

    def send_positions(
        self,
        start_positions: Dict[str, float],
        target_positions: Dict[str, float],
        duration_s: float,
    ):
        if self.command_mode == "action":
            self.send_trajectory_action(start_positions, target_positions, duration_s)
        elif self.command_mode == "topic":
            self.send_trajectory_topic(start_positions, target_positions, duration_s)
        else:
            raise ValueError("command_mode must be 'action' or 'topic'")

    def run_probe(self):
        start_joints = self.wait_for_joint_state()
        start_tool = self.lookup_position()
        target_joints = dict(start_joints)
        target_joints[self.joint_name] += self.joint_delta_rad

        self.get_logger().warn(
            "%s start=%.6f target=%.6f delta=%.6f"
            % (
                self.joint_name,
                start_joints[self.joint_name],
                target_joints[self.joint_name],
                self.joint_delta_rad,
            )
        )
        self.get_logger().warn(
            "Start %s->%s position: x=%.6f y=%.6f z=%.6f"
            % (self.base_frame, self.tool_frame, start_tool[0], start_tool[1], start_tool[2])
        )

        self.get_logger().warn("Sending trajectory with command_mode=%s" % self.command_mode)
        self.send_positions(start_joints, target_joints, self.duration_s)
        if self.command_mode == "topic":
            end_time = time.monotonic() + self.duration_s + self.settle_s
            while rclpy.ok() and time.monotonic() < end_time:
                rclpy.spin_once(self, timeout_sec=0.05)

        stop_joints = self.wait_for_joint_state()
        stop_tool = self.lookup_position()
        self.get_logger().warn(
            "%s actual_stop=%.6f actual_delta=%.6f"
            % (
                self.joint_name,
                stop_joints[self.joint_name],
                stop_joints[self.joint_name] - start_joints[self.joint_name],
            )
        )
        if self.print_all_joints:
            for joint in self.joints:
                self.get_logger().warn(
                    "%s start=%.6f target=%.6f stop=%.6f actual_delta=%.6f target_error=%.6f"
                    % (
                        joint,
                        start_joints[joint],
                        target_joints[joint],
                        stop_joints[joint],
                        stop_joints[joint] - start_joints[joint],
                        stop_joints[joint] - target_joints[joint],
                    )
                )
        self.get_logger().warn(
            "Stop  %s->%s position: x=%.6f y=%.6f z=%.6f"
            % (self.base_frame, self.tool_frame, stop_tool[0], stop_tool[1], stop_tool[2])
        )
        self.get_logger().warn(
            "Delta %s->%s position: dx=%.6f dy=%.6f dz=%.6f"
            % (
                self.base_frame,
                self.tool_frame,
                stop_tool[0] - start_tool[0],
                stop_tool[1] - start_tool[1],
                stop_tool[2] - start_tool[2],
            )
        )

        if self.restore_start:
            self.get_logger().warn("Restoring start joint positions.")
            self.send_positions(stop_joints, start_joints, self.duration_s)


def main():
    rclpy.init()
    node = JointTrajectoryProbeMeasure()
    try:
        node.run_probe()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
