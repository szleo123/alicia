#!/usr/bin/env python3

import argparse
import math
import os
import select
import sys
import termios
import tty
from datetime import datetime

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import yaml


def matrix_from_transform(transform):
    t = transform.transform.translation
    q = transform.transform.rotation

    T = np.eye(4)
    T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def solve_tcp_pivot(samples):
    """
    Solve the classic pivot calibration problem.

    For each sample i:
      R_i * t_tcp + p_i = p_pivot

    Unknowns:
      t_tcp   : TCP offset in the mount frame (gripper_center)
      p_pivot : fixed pivot point in the parent/world frame
    """
    A_rows = []
    b_rows = []

    for sample in samples:
        R_i = sample[:3, :3]
        p_i = sample[:3, 3]
        A_rows.append(np.hstack([R_i, -np.eye(3)]))
        b_rows.append(-p_i.reshape(3, 1))

    A = np.vstack(A_rows)
    b = np.vstack(b_rows)

    x, residuals, rank, singular_values = np.linalg.lstsq(A, b, rcond=None)
    tcp_offset = x[:3, 0]
    pivot_point = x[3:, 0]

    per_sample_errors = []
    for sample in samples:
        R_i = sample[:3, :3]
        p_i = sample[:3, 3]
        estimated_pivot = R_i @ tcp_offset + p_i
        per_sample_errors.append(np.linalg.norm(estimated_pivot - pivot_point))

    errors = np.array(per_sample_errors)
    return {
        "tcp_offset_xyz": tcp_offset,
        "pivot_point_xyz": pivot_point,
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "max_error": float(np.max(errors)),
        "mean_error": float(np.mean(errors)),
        "rank": int(rank),
        "singular_values": singular_values.tolist(),
        "sample_errors": errors.tolist(),
    }


class TcpPivotCalibrator(Node):
    def __init__(self, args):
        super().__init__("tcp_pivot_calibrator")
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples = []
        self.last_auto_capture = None
        self.joint_state = None
        self.subscription = self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    def current_arm_positions(self):
        if self.joint_state is None:
            raise RuntimeError("No joint state available yet.")
        mapping = dict(zip(self.joint_state.name, self.joint_state.position))
        joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        return {name: float(mapping[name]) for name in joint_names}

    def lookup_mount_transform(self):
        transform = self.tf_buffer.lookup_transform(
            self.args.parent_frame,
            self.args.mount_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        return matrix_from_transform(transform)

    def wait_for_mount_transform(self, timeout_sec=10.0):
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        last_error = None
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                self.lookup_mount_transform()
                self.get_logger().info(
                    f"TF is ready: {self.args.parent_frame} <- {self.args.mount_frame}"
                )
                return True
            except Exception as exc:  # noqa: BLE001
                last_error = exc

        self.get_logger().error(
            "Required TF is not available. Make sure the robot stack is running "
            "and robot_state_publisher is publishing the chain that contains "
            f"'{self.args.mount_frame}'. Last error: {last_error}"
        )
        self.get_logger().error(
            "Expected workflow: source the workspace, launch "
            "`ros2 launch alicia_d_moveit real_robot.launch.py`, then run this tool."
        )
        return False

    def capture_sample(self):
        T = self.lookup_mount_transform()
        joint_positions = self.current_arm_positions()
        self.samples.append({"transform": T, "joint_positions": joint_positions})

        position = T[:3, 3]
        euler = R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
        self.get_logger().info(
            f"Captured sample #{len(self.samples)}: "
            f"pos=[{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}], "
            f"rpy_deg=[{euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}]"
        )
        return T

    def maybe_auto_capture(self):
        T = self.lookup_mount_transform()
        joint_positions = self.current_arm_positions()
        if self.last_auto_capture is None:
            self.last_auto_capture = T
            self.samples.append({"transform": T, "joint_positions": joint_positions})
            self.get_logger().info("Auto-captured sample #1")
            return

        delta_rot = np.linalg.norm(
            R.from_matrix(self.last_auto_capture[:3, :3].T @ T[:3, :3]).as_rotvec()
        )
        delta_pos = np.linalg.norm(T[:3, 3] - self.last_auto_capture[:3, 3])

        if (
            math.degrees(delta_rot) >= self.args.auto_min_rotation_deg
            or delta_pos >= self.args.auto_min_translation_m
        ):
            self.samples.append({"transform": T, "joint_positions": joint_positions})
            self.last_auto_capture = T
            self.get_logger().info(
                f"Auto-captured sample #{len(self.samples)} "
                f"(d_rot={math.degrees(delta_rot):.1f} deg, d_pos={delta_pos:.4f} m)"
            )

    def write_results(self, result):
        if self.args.samples_output:
            payload = {
                "metadata": {
                    "created_at": datetime.now().isoformat(timespec="seconds"),
                    "parent_frame": self.args.parent_frame,
                    "mount_frame": self.args.mount_frame,
                    "sample_count": len(self.samples),
                },
                "samples": [
                    {
                        "transform": sample["transform"].tolist(),
                        "joint_positions": sample["joint_positions"],
                    }
                    for sample in self.samples
                ],
                "result": {
                    "tcp_offset_xyz": result["tcp_offset_xyz"].tolist(),
                    "pivot_point_xyz": result["pivot_point_xyz"].tolist(),
                    "rmse": result["rmse"],
                    "max_error": result["max_error"],
                    "mean_error": result["mean_error"],
                },
            }
            output_dir = os.path.dirname(self.args.samples_output)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            with open(self.args.samples_output, "w", encoding="utf-8") as stream:
                yaml.safe_dump(payload, stream, sort_keys=False)
            self.get_logger().info(f"Wrote sample log to {self.args.samples_output}")

        if not self.args.update_calibration:
            return

        calibration_path = self.args.calibration_file
        if not os.path.exists(calibration_path):
            raise FileNotFoundError(f"Calibration file not found: {calibration_path}")

        with open(calibration_path, "r", encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream) or {}

        tcp_correction = calibration.setdefault("tcp_correction", {})
        tcp_correction["xyz"] = [float(v) for v in result["tcp_offset_xyz"]]
        tcp_correction.setdefault("rpy_deg", [0.0, 0.0, 0.0])

        with open(calibration_path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(calibration, stream, sort_keys=False)

        self.get_logger().info(f"Updated TCP correction in {calibration_path}")


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.original = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)


def parse_args():
    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    parser = argparse.ArgumentParser(
        description="UR-style pivot calibration for Alicia TCP."
    )
    parser.add_argument(
        "--parent-frame",
        default="base_link",
        help="Reference frame in which the pivot point is fixed. Default: base_link",
    )
    parser.add_argument(
        "--mount-frame",
        default="gripper_center",
        help="Mount frame to calibrate the TCP from. Default: gripper_center",
    )
    parser.add_argument(
        "--calibration-file",
        default=os.path.expanduser(
            "~/alicia/ros2_ws/src/alicia_d_ros2_upstream/alicia_d_moveit/config/kinematic_calibration.yaml"
        ),
        help="Path to kinematic_calibration.yaml",
    )
    parser.add_argument(
        "--samples-output",
        default=os.path.expanduser(f"~/alicia/calibration/kinematic/{run_stamp}/tcp_pivot_samples.yaml"),
        help="Where to save captured samples and fit statistics.",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=6,
        help="Minimum number of samples required before solving.",
    )
    parser.add_argument(
        "--update-calibration",
        action="store_true",
        help="Write the solved tcp_correction.xyz back into kinematic_calibration.yaml",
    )
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Auto-capture when pose changes enough instead of using Enter.",
    )
    parser.add_argument(
        "--auto-min-rotation-deg",
        type=float,
        default=8.0,
        help="Auto-capture rotation threshold in degrees.",
    )
    parser.add_argument(
        "--auto-min-translation-m",
        type=float,
        default=0.01,
        help="Auto-capture translation threshold in meters.",
    )
    return parser.parse_args()


def print_instructions(args):
    print()
    print("TCP pivot calibration")
    print("---------------------")
    print(f"Parent frame : {args.parent_frame}")
    print(f"Mount frame  : {args.mount_frame}")
    print()
    print("Procedure:")
    print("1. Put the real tool tip on one fixed physical point.")
    print("2. Keep the tip on that point while changing wrist/arm orientation.")
    print("3. Capture many poses spanning different orientations.")
    print()
    if args.auto:
        print("Auto mode:")
        print("  The script auto-captures when the mount pose changes enough.")
        print("  Press 's' to solve, 'q' to quit.")
    else:
        print("Manual mode:")
        print("  Press Enter to capture a sample.")
        print("  Press 's' then Enter to solve.")
        print("  Press 'q' then Enter to quit.")
    print()


def main():
    args = parse_args()
    print_instructions(args)

    rclpy.init()
    node = TcpPivotCalibrator(args)

    try:
        if not node.wait_for_mount_transform():
            return

        if args.auto:
            with RawTerminal():
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)
                    try:
                        node.maybe_auto_capture()
                    except Exception as exc:  # noqa: BLE001
                        node.get_logger().warn(f"Waiting for TF: {exc}")

                    if select.select([sys.stdin], [], [], 0.0)[0]:
                        key = sys.stdin.read(1)
                        if key.lower() == "q":
                            break
                        if key.lower() == "s":
                            break
        else:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    response = input("[Enter]=capture, s=solve, q=quit: ").strip().lower()
                except EOFError:
                    response = "q"

                if response == "q":
                    break
                if response == "s":
                    break

                try:
                    node.capture_sample()
                except Exception as exc:  # noqa: BLE001
                    node.get_logger().warn(f"Failed to capture sample: {exc}")

        if len(node.samples) < args.min_samples:
            node.get_logger().warn(
                f"Only {len(node.samples)} samples captured. Need at least {args.min_samples} to solve."
            )
            return

        result = solve_tcp_pivot([sample["transform"] for sample in node.samples])
        node.get_logger().info("Solved TCP pivot calibration")
        node.get_logger().info(
            f"TCP offset xyz in {args.mount_frame}: "
            f"[{result['tcp_offset_xyz'][0]:.6f}, "
            f"{result['tcp_offset_xyz'][1]:.6f}, "
            f"{result['tcp_offset_xyz'][2]:.6f}]"
        )
        node.get_logger().info(
            f"Estimated pivot point in {args.parent_frame}: "
            f"[{result['pivot_point_xyz'][0]:.6f}, "
            f"{result['pivot_point_xyz'][1]:.6f}, "
            f"{result['pivot_point_xyz'][2]:.6f}]"
        )
        node.get_logger().info(
            f"Residuals: rmse={result['rmse']:.6f} m, "
            f"mean={result['mean_error']:.6f} m, "
            f"max={result['max_error']:.6f} m"
        )

        node.write_results(result)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
