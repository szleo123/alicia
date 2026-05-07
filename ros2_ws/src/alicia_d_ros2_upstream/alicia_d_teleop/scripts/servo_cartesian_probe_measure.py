#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int8
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import Buffer, TransformException, TransformListener


class ServoCartesianProbeMeasure(Node):
    STATUS_TEXT = {
        0: "NO_WARNING",
        1: "DECELERATE_FOR_APPROACHING_SINGULARITY",
        2: "HALT_FOR_SINGULARITY",
        3: "DECELERATE_FOR_COLLISION",
        4: "HALT_FOR_COLLISION",
        5: "JOINT_BOUND",
        6: "DECELERATE_FOR_LEAVING_SINGULARITY",
    }

    def __init__(self):
        super().__init__("servo_cartesian_probe_measure")

        self.declare_parameter("command_topic", "/alicia_d_teleop/servo_probe_twist_cmd")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("linear_xyz", [-0.005, 0.0, 0.0])
        self.declare_parameter("angular_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("duration_s", 2.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("wait_for_subscriber_s", 3.0)
        self.declare_parameter("servo_status_topic", "/alicia_d_teleop/servo_probe_status")
        self.declare_parameter("trajectory_topic", "/Alicia_controller/joint_trajectory")

        self.command_topic = self.get_parameter("command_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tool_frame = self.get_parameter("tool_frame").value
        self.command_frame = self.get_parameter("command_frame").value
        self.linear_xyz = [float(v) for v in self.get_parameter("linear_xyz").value]
        self.angular_xyz = [float(v) for v in self.get_parameter("angular_xyz").value]
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.wait_for_subscriber_s = float(self.get_parameter("wait_for_subscriber_s").value)
        self.servo_status_topic = self.get_parameter("servo_status_topic").value
        self.trajectory_topic = self.get_parameter("trajectory_topic").value

        self.pub = self.create_publisher(TwistStamped, self.command_topic, 10)
        self.trajectory_count = 0
        self.max_trajectory_position_delta = 0.0
        self.last_commanded_positions = None
        self.last_commanded_joint_names = []
        self.commanded_delta_min = {}
        self.commanded_delta_max = {}
        self.commanded_delta_sum = {}
        self.record_trajectories = False
        self.last_status = None
        self.start_joint_positions = None
        self.create_subscription(JointTrajectory, self.trajectory_topic, self.trajectory_callback, 10)
        self.create_subscription(Int8, self.servo_status_topic, self.status_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def trajectory_callback(self, msg):
        if not self.record_trajectories:
            return

        self.trajectory_count += 1
        if self.start_joint_positions is None or not msg.points:
            return

        start_by_name = self.start_joint_positions
        final_point = msg.points[-1]
        for point in msg.points:
            if point.positions:
                self.last_commanded_joint_names = list(msg.joint_names)
                self.last_commanded_positions = list(point.positions)
            for index, joint_name in enumerate(msg.joint_names):
                if index >= len(point.positions) or joint_name not in start_by_name:
                    continue
                signed_delta = point.positions[index] - start_by_name[joint_name]
                self.commanded_delta_min[joint_name] = min(
                    self.commanded_delta_min.get(joint_name, signed_delta),
                    signed_delta,
                )
                self.commanded_delta_max[joint_name] = max(
                    self.commanded_delta_max.get(joint_name, signed_delta),
                    signed_delta,
                )
                self.max_trajectory_position_delta = max(self.max_trajectory_position_delta, abs(signed_delta))
        for index, joint_name in enumerate(msg.joint_names):
            if index >= len(final_point.positions) or joint_name not in start_by_name:
                continue
            signed_delta = final_point.positions[index] - start_by_name[joint_name]
            self.commanded_delta_sum[joint_name] = self.commanded_delta_sum.get(joint_name, 0.0) + signed_delta

    def status_callback(self, msg):
        self.last_status = int(msg.data)

    def latest_joint_positions(self):
        joint_state_topic = "/joint_states"
        joint_state = None

        from sensor_msgs.msg import JointState

        def joint_state_callback(msg):
            nonlocal joint_state
            joint_state = msg

        subscription = self.create_subscription(JointState, joint_state_topic, joint_state_callback, 10)
        deadline = self.get_clock().now() + Duration(seconds=3.0)
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if joint_state is not None:
                self.destroy_subscription(subscription)
                return dict(zip(joint_state.name, joint_state.position))

        self.destroy_subscription(subscription)
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

    def publish_command(self):
        subscriber_deadline = time.monotonic() + self.wait_for_subscriber_s
        while rclpy.ok() and self.pub.get_subscription_count() == 0 and time.monotonic() < subscriber_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.pub.get_subscription_count() == 0:
            self.get_logger().warn(
                "No subscribers on %s. Start servo_real_probe.launch.py or use the active Servo input topic."
                % self.command_topic
            )
        else:
            self.get_logger().warn(
                "Publishing probe commands on %s to %d subscriber(s)."
                % (self.command_topic, self.pub.get_subscription_count())
            )

        self.trajectory_count = 0
        self.max_trajectory_position_delta = 0.0
        self.last_commanded_positions = None
        self.last_commanded_joint_names = []
        self.commanded_delta_min = {}
        self.commanded_delta_max = {}
        self.commanded_delta_sum = {}
        self.record_trajectories = False
        self.start_joint_positions = self.latest_joint_positions()
        start = self.lookup_position()
        self.get_logger().warn(
            "Start %s->%s position: x=%.6f y=%.6f z=%.6f"
            % (self.base_frame, self.tool_frame, start[0], start[1], start[2])
        )

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        end_time = time.monotonic() + self.duration_s
        self.record_trajectories = True
        while rclpy.ok() and time.monotonic() < end_time:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.command_frame
            msg.twist.linear.x = self.linear_xyz[0]
            msg.twist.linear.y = self.linear_xyz[1]
            msg.twist.linear.z = self.linear_xyz[2]
            msg.twist.angular.x = self.angular_xyz[0]
            msg.twist.angular.y = self.angular_xyz[1]
            msg.twist.angular.z = self.angular_xyz[2]
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        time.sleep(0.2)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.record_trajectories = False
        stop_joint_positions = self.latest_joint_positions()
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        stop = self.lookup_position()
        delta = [stop[i] - start[i] for i in range(3)]
        self.get_logger().warn(
            "Stop  %s->%s position: x=%.6f y=%.6f z=%.6f"
            % (self.base_frame, self.tool_frame, stop[0], stop[1], stop[2])
        )
        self.get_logger().warn(
            "Delta %s->%s position: dx=%.6f dy=%.6f dz=%.6f"
            % (self.base_frame, self.tool_frame, delta[0], delta[1], delta[2])
        )
        self.get_logger().warn(
            "Servo output observed: trajectories=%d max_position_delta=%.6f last_status=%s"
            % (
                self.trajectory_count,
                self.max_trajectory_position_delta,
                (
                    "none"
                    if self.last_status is None
                    else "%d %s" % (self.last_status, self.STATUS_TEXT.get(self.last_status, "UNKNOWN"))
                ),
            )
        )
        if self.last_commanded_positions is not None:
            for index, joint_name in enumerate(self.last_commanded_joint_names):
                if (
                    index >= len(self.last_commanded_positions)
                    or joint_name not in self.start_joint_positions
                    or joint_name not in stop_joint_positions
                ):
                    continue
                commanded_delta = self.last_commanded_positions[index] - self.start_joint_positions[joint_name]
                actual_delta = stop_joint_positions[joint_name] - self.start_joint_positions[joint_name]
                self.get_logger().warn(
                    "%s servo_last_delta=%.6f servo_min_delta=%.6f servo_max_delta=%.6f actual_delta=%.6f cmd_minus_actual=%.6f"
                    % (
                        joint_name,
                        commanded_delta,
                        self.commanded_delta_min.get(joint_name, 0.0),
                        self.commanded_delta_max.get(joint_name, 0.0),
                        actual_delta,
                        commanded_delta - actual_delta,
                    )
                )
                if self.trajectory_count:
                    self.get_logger().warn(
                        "%s servo_sum_delta=%.6f servo_avg_step_delta=%.6f"
                        % (
                            joint_name,
                            self.commanded_delta_sum.get(joint_name, 0.0),
                            self.commanded_delta_sum.get(joint_name, 0.0) / self.trajectory_count,
                        )
                    )


def main():
    rclpy.init()
    node = ServoCartesianProbeMeasure()
    try:
        node.publish_command()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
