#!/usr/bin/env python3

import argparse
import math
import os
import re
import select
import sys
import termios
import tty
import xml.etree.ElementTree as ET
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import yaml


DEFAULT_PARAM_NAMES = [
    "Joint1.z",
    "Joint2.x",
    "Joint2.z",
    "Joint3.x",
    "Joint3.y",
    "Joint4.y",
    "Joint5.z",
    "Joint6.y",
    "Joint6.z",
]


def parse_xyz(text, length):
    if text is None:
        return np.zeros(length, dtype=float)
    tokens = re.findall(r"\$\{[^}]+\}|[^\s]+", text)
    values = []
    for token in tokens:
        try:
            values.append(float(token))
            continue
        except ValueError:
            pass

        match = re.search(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", token)
        if match is None:
            raise ValueError(f"Could not parse numeric token `{token}` from `{text}`")
        values.append(float(match.group(0)))
    if len(values) != length:
        raise ValueError(f"Expected {length} values, got {values}")
    return np.array(values, dtype=float)


def transform_from_xyz_rpy(xyz, rpy):
    T = np.eye(4)
    T[:3, 3] = xyz
    T[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    return T


def transform_from_axis_angle(axis, angle):
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(4)
    axis = axis / norm
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(axis * angle).as_matrix()
    return T


def to_builtin_types(value):
    if isinstance(value, dict):
        return {str(k): to_builtin_types(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_builtin_types(v) for v in value]
    if isinstance(value, np.ndarray):
        return [to_builtin_types(v) for v in value.tolist()]
    if isinstance(value, np.generic):
        return value.item()
    return value


def load_calibration(calibration_file):
    with open(calibration_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    tcp = data.get("tcp_correction", {})
    tcp_xyz = np.array([float(v) for v in tcp.get("xyz", [0.0, 0.0, 0.0])], dtype=float)

    geometric = data.setdefault("joint_origin_xyz_corrections_m", {})
    normalized_geometric = {}
    for name in ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]:
        raw = geometric.get(name, [0.0, 0.0, 0.0])
        if not isinstance(raw, list) or len(raw) != 3:
            raise ValueError(f"joint_origin_xyz_corrections_m.{name} must be [x, y, z]")
        normalized_geometric[name] = np.array([float(v) for v in raw], dtype=float)

    return tcp_xyz, normalized_geometric, data


class KinematicChain:
    def __init__(self, urdf_path, base_link, tip_link, tcp_xyz):
        self.urdf_path = urdf_path
        self.base_link = base_link
        self.tip_link = tip_link
        self.tcp_xyz = np.array(tcp_xyz, dtype=float)
        self.path = self._load_path()

    def _load_path(self):
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        joints = []
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            origin = joint.find("origin")
            axis = joint.find("axis")
            joints.append(
                {
                    "name": joint.attrib["name"],
                    "type": joint.attrib["type"],
                    "parent": parent.attrib["link"],
                    "child": child.attrib["link"],
                    "origin_xyz": parse_xyz(origin.attrib.get("xyz", "0 0 0") if origin is not None else None, 3),
                    "origin_rpy": parse_xyz(origin.attrib.get("rpy", "0 0 0") if origin is not None else None, 3),
                    "axis": parse_xyz(axis.attrib.get("xyz", "0 0 1") if axis is not None else None, 3),
                }
            )

        by_parent = {}
        for joint in joints:
            by_parent.setdefault(joint["parent"], []).append(joint)

        path = []
        if not self._dfs(self.base_link, self.tip_link, by_parent, path):
            raise RuntimeError(f"Could not find chain {self.base_link} -> {self.tip_link} in {self.urdf_path}")
        return path

    def _dfs(self, current, tip, by_parent, path):
        if current == tip:
            return True
        for joint in by_parent.get(current, []):
            path.append(joint)
            if self._dfs(joint["child"], tip, by_parent, path):
                return True
            path.pop()
        return False

    def fk(self, joint_positions, origin_xyz_corrections):
        T = np.eye(4)
        for joint in self.path:
            xyz = joint["origin_xyz"] + origin_xyz_corrections.get(joint["name"], np.zeros(3, dtype=float))
            T = T @ transform_from_xyz_rpy(xyz, joint["origin_rpy"])
            if joint["type"] == "revolute":
                q = joint_positions.get(joint["name"], 0.0)
                T = T @ transform_from_axis_angle(joint["axis"], q)
            elif joint["type"] == "prismatic":
                q = joint_positions.get(joint["name"], 0.0)
                delta = np.eye(4)
                delta[:3, 3] = joint["axis"] * q
                T = T @ delta

        if np.linalg.norm(self.tcp_xyz) > 0:
            tcp = np.eye(4)
            tcp[:3, 3] = self.tcp_xyz
            T = T @ tcp
        return T


def parse_param_name(name):
    joint_name, axis_name = name.split(".", 1)
    axis_index = {"x": 0, "y": 1, "z": 2}[axis_name]
    return joint_name, axis_index


def build_correction_map(param_names, x, base_corrections):
    corrections = {
        joint_name: np.array(value, dtype=float).copy()
        for joint_name, value in base_corrections.items()
    }
    for i, param_name in enumerate(param_names):
        joint_name, axis_index = parse_param_name(param_name)
        corrections[joint_name][axis_index] = x[i]
    return corrections


def plane_normal_from_roll_pitch(roll, pitch):
    normal = R.from_euler("xy", [roll, pitch]).apply([0.0, 0.0, 1.0])
    return normal / np.linalg.norm(normal)


def solve_geometric_plane_corrections(chain, samples, param_names, base_corrections, prior_std_m, plane_tilt_prior_deg):
    if len(samples) < 12:
        raise ValueError("Need at least 12 samples for reduced geometric plane calibration.")

    def unpack(x):
        corrections = build_correction_map(param_names, x[:len(param_names)], base_corrections)
        plane_roll = x[len(param_names)]
        plane_pitch = x[len(param_names) + 1]
        plane_offset = x[len(param_names) + 2]
        return corrections, plane_roll, plane_pitch, plane_offset

    def residual_vector(x):
        corrections, plane_roll, plane_pitch, plane_offset = unpack(x)
        normal = plane_normal_from_roll_pitch(plane_roll, plane_pitch)
        residuals = []
        for sample in samples:
            T = chain.fk(sample["joint_positions"], corrections)
            pos = T[:3, 3]
            residuals.append(float(np.dot(normal, pos) + plane_offset))
        if prior_std_m > 0:
            for i, param_name in enumerate(param_names):
                joint_name, axis_index = parse_param_name(param_name)
                residuals.append((x[i] - base_corrections[joint_name][axis_index]) / prior_std_m)
        if plane_tilt_prior_deg > 0:
            prior_rad = math.radians(plane_tilt_prior_deg)
            residuals.append(plane_roll / prior_rad)
            residuals.append(plane_pitch / prior_rad)
        return np.array(residuals, dtype=float)

    seed_positions = []
    for sample in samples:
        T0 = chain.fk(sample["joint_positions"], base_corrections)
        seed_positions.append(T0[:3, 3])
    seed_positions = np.array(seed_positions, dtype=float)
    z_seed = float(np.mean(seed_positions[:, 2]))
    x0 = np.concatenate(
        [
            np.array(
                [base_corrections[parse_param_name(name)[0]][parse_param_name(name)[1]] for name in param_names],
                dtype=float,
            ),
            np.array([0.0, 0.0, -z_seed], dtype=float),
        ]
    )

    result = least_squares(
        residual_vector,
        x0,
        method="trf",
        loss="soft_l1",
        f_scale=0.001,
        x_scale="jac",
        verbose=0,
    )

    corrections, plane_roll, plane_pitch, plane_offset = unpack(result.x)
    normal = plane_normal_from_roll_pitch(plane_roll, plane_pitch)
    sample_signed_distances = []
    fitted_positions = []
    for sample in samples:
        T = chain.fk(sample["joint_positions"], corrections)
        pos = T[:3, 3]
        fitted_positions.append(pos.tolist())
        sample_signed_distances.append(float(np.dot(normal, pos) + plane_offset))

    distances = np.abs(np.array(sample_signed_distances, dtype=float))

    solved_params = {}
    for i, name in enumerate(param_names):
        solved_params[name] = float(result.x[i])

    return {
        "param_names": param_names,
        "solved_params_m": solved_params,
        "joint_origin_xyz_corrections_m": {name: corrections[name].tolist() for name in corrections},
        "plane_normal": normal.tolist(),
        "plane_roll_deg": math.degrees(plane_roll),
        "plane_pitch_deg": math.degrees(plane_pitch),
        "plane_offset_m": float(plane_offset),
        "rmse": float(np.sqrt(np.mean(distances ** 2))),
        "mean_error": float(np.mean(distances)),
        "max_error": float(np.max(distances)),
        "cost": float(result.cost),
        "success": bool(result.success),
        "message": result.message,
        "sample_signed_distances": sample_signed_distances,
        "fitted_positions": fitted_positions,
    }


class GeometricPlaneCalibrator(Node):
    def __init__(self, args):
        super().__init__("geometric_plane_calibrator")
        self.args = args
        self.joint_state = None
        self.samples = []
        self.last_auto_state = None
        self.subscription = self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    def wait_for_joint_state(self, timeout_sec=10.0):
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and (self.get_clock().now().nanoseconds / 1e9) < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state is not None:
                self.get_logger().info("Joint state stream is ready.")
                return True
        self.get_logger().error("No /joint_states received. Launch the real robot stack first.")
        return False

    def current_arm_positions(self):
        if self.joint_state is None:
            raise RuntimeError("No joint state available yet.")
        mapping = dict(zip(self.joint_state.name, self.joint_state.position))
        return {name: float(mapping[name]) for name in self.args.joint_names}

    def capture_sample(self):
        joint_positions = self.current_arm_positions()
        self.samples.append({"joint_positions": joint_positions})
        self.get_logger().info(
            "Captured sample #%d: %s" % (
                len(self.samples),
                ", ".join(f"{name}={joint_positions[name]:.4f}" for name in self.args.joint_names),
            )
        )

    def maybe_auto_capture(self):
        joint_positions = self.current_arm_positions()
        current = np.array([joint_positions[name] for name in self.args.joint_names], dtype=float)
        if self.last_auto_state is None:
            self.last_auto_state = current
            self.samples.append({"joint_positions": joint_positions})
            self.get_logger().info("Auto-captured sample #1")
            return
        delta = np.max(np.abs(current - self.last_auto_state))
        if delta >= self.args.auto_min_joint_change_rad:
            self.last_auto_state = current
            self.samples.append({"joint_positions": joint_positions})
            self.get_logger().info(
                f"Auto-captured sample #{len(self.samples)} (max joint delta={delta:.4f} rad)"
            )

    def write_results(self, result):
        if self.args.samples_output:
            payload = {
                "metadata": {
                    "created_at": datetime.now().isoformat(timespec="seconds"),
                    "sample_count": len(self.samples),
                    "joint_names": self.args.joint_names,
                    "fit_params": self.args.fit_params,
                },
                "samples": self.samples,
                "result": to_builtin_types(result),
            }
            output_dir = os.path.dirname(self.args.samples_output)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            with open(self.args.samples_output, "w", encoding="utf-8") as stream:
                yaml.safe_dump(payload, stream, sort_keys=False)
            self.get_logger().info(f"Wrote sample log to {self.args.samples_output}")

        if not self.args.update_calibration:
            return

        with open(self.args.calibration_file, "r", encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream) or {}

        corrections = calibration.setdefault("joint_origin_xyz_corrections_m", {})
        for joint_name, xyz in result["joint_origin_xyz_corrections_m"].items():
            corrections[joint_name] = [float(v) for v in xyz]

        with open(self.args.calibration_file, "w", encoding="utf-8") as stream:
            yaml.safe_dump(calibration, stream, sort_keys=False)

        self.get_logger().info(f"Updated joint origin corrections in {self.args.calibration_file}")


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
        description="Fit reduced geometric corrections from plane-contact samples."
    )
    parser.add_argument(
        "--urdf-file",
        default=os.path.expanduser(
            "~/alicia/ros2_ws/src/alicia_d_ros2_upstream/alicia_d_descriptions/urdf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.urdf"
        ),
        help="Nominal Alicia URDF path.",
    )
    parser.add_argument(
        "--calibration-file",
        default=os.path.expanduser(
            "~/alicia/ros2_ws/src/alicia_d_ros2_upstream/alicia_d_moveit/config/kinematic_calibration.yaml"
        ),
        help="Path to kinematic_calibration.yaml",
    )
    parser.add_argument(
        "--base-link",
        default="base_link",
        help="Base link for the kinematic chain.",
    )
    parser.add_argument(
        "--tip-link",
        default="gripper_center",
        help="Nominal mount link before tcp_correction is applied.",
    )
    parser.add_argument(
        "--fit-params",
        default=",".join(DEFAULT_PARAM_NAMES),
        help="Comma-separated reduced parameter set, e.g. Joint2.x,Joint2.z,Joint3.x",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=20,
        help="Minimum number of samples required before solving.",
    )
    parser.add_argument(
        "--prior-std-mm",
        type=float,
        default=8.0,
        help="Regularization strength for geometric corrections, in millimeters.",
    )
    parser.add_argument(
        "--plane-tilt-prior-deg",
        type=float,
        default=10.0,
        help="Regularization on plane roll/pitch away from the nominal base frame z-axis.",
    )
    parser.add_argument(
        "--samples-output",
        default=os.path.expanduser(f"~/alicia/calibration/kinematic/{run_stamp}/geometric_plane_samples.yaml"),
        help="Where to save captured samples and fit statistics.",
    )
    parser.add_argument(
        "--update-calibration",
        action="store_true",
        help="Write the solved joint_origin_xyz_corrections_m back into kinematic_calibration.yaml",
    )
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Auto-capture when arm joints change enough instead of using Enter.",
    )
    parser.add_argument(
        "--auto-min-joint-change-deg",
        type=float,
        default=6.0,
        help="Auto-capture threshold based on max absolute joint change.",
    )
    args = parser.parse_args()
    args.auto_min_joint_change_rad = math.radians(args.auto_min_joint_change_deg)
    args.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
    args.fit_params = [item.strip() for item in args.fit_params.split(",") if item.strip()]
    return args


def print_instructions(args):
    print()
    print("Reduced geometric plane calibration")
    print("----------------------------------")
    print(f"URDF file    : {args.urdf_file}")
    print(f"Base link    : {args.base_link}")
    print(f"Tip link     : {args.tip_link}")
    print(f"Calibration  : {args.calibration_file}")
    print(f"Fit params   : {', '.join(args.fit_params)}")
    print()
    print("Procedure:")
    print("1. Touch the calibrated TCP to many different points on the same physical plane.")
    print("2. Vary XY location and posture as much as safely possible.")
    print("3. Capture joint-state samples at each plane contact.")
    print("4. Solve for reduced geometric corrections that make the TCP lie on one plane.")
    print()
    print("This is a height-consistency calibration, not full 6D kinematic identification.")
    print()
    if args.auto:
        print("Auto mode: press 's' to solve, 'q' to quit.")
    else:
        print("Manual mode: press Enter to capture, 's' to solve, 'q' to quit.")
    print()


def main():
    args = parse_args()
    print_instructions(args)

    tcp_xyz, base_corrections, _ = load_calibration(args.calibration_file)
    chain = KinematicChain(args.urdf_file, args.base_link, args.tip_link, tcp_xyz)

    rclpy.init()
    node = GeometricPlaneCalibrator(args)

    try:
        if not node.wait_for_joint_state():
            return

        if args.auto:
            with RawTerminal():
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)
                    try:
                        node.maybe_auto_capture()
                    except Exception as exc:  # noqa: BLE001
                        node.get_logger().warn(f"Auto-capture failed: {exc}")
                    if select.select([sys.stdin], [], [], 0.0)[0]:
                        key = sys.stdin.read(1)
                        if key.lower() in ("s", "q"):
                            break
        else:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    response = input("[Enter]=capture, s=solve, q=quit: ").strip().lower()
                except EOFError:
                    response = "q"
                if response == "q":
                    return
                if response == "s":
                    break
                try:
                    node.capture_sample()
                except Exception as exc:  # noqa: BLE001
                    node.get_logger().warn(f"Failed to capture sample: {exc}")

        if len(node.samples) < args.min_samples:
            node.get_logger().warn(
                f"Only {len(node.samples)} samples captured. Need at least {args.min_samples}."
            )
            return

        result = solve_geometric_plane_corrections(
            chain,
            node.samples,
            args.fit_params,
            base_corrections=base_corrections,
            prior_std_m=args.prior_std_mm / 1000.0,
            plane_tilt_prior_deg=args.plane_tilt_prior_deg,
        )
        node.get_logger().info("Solved reduced geometric plane calibration")
        for name in args.fit_params:
            value = result["solved_params_m"][name]
            node.get_logger().info(f"{name}: {value:+.6f} m ({value * 1000.0:+.2f} mm)")
        node.get_logger().info(
            f"Plane tilt: roll={result['plane_roll_deg']:+.3f} deg, pitch={result['plane_pitch_deg']:+.3f} deg"
        )
        node.get_logger().info(
            f"Residuals: rmse={result['rmse']:.6f} m, mean={result['mean_error']:.6f} m, max={result['max_error']:.6f} m"
        )
        node.get_logger().info(f"Solver: success={result['success']} message={result['message']}")

        if result["rmse"] > 0.01:
            node.get_logger().warn(
                "Residual is still above 10 mm. That usually means the reduced parameter set is not enough "
                "or the contact samples on the plane are not repeatable."
            )

        node.write_results(result)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
