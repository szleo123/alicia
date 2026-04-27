#!/usr/bin/env python3

import argparse
import math
import os
import re
import xml.etree.ElementTree as ET
from datetime import datetime

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
import yaml


DEFAULT_PARAM_NAMES = [
    "tcp.x",
    "tcp.y",
    "tcp.z",
    "joint_offset.Joint1",
    "joint_offset.Joint2",
    "joint_offset.Joint3",
    "joint_offset.Joint4",
    "joint_offset.Joint5",
    "joint_offset.Joint6",
    "joint_xyz.Joint1.z",
    "joint_xyz.Joint2.x",
    "joint_xyz.Joint2.z",
    "joint_xyz.Joint3.x",
    "joint_xyz.Joint3.y",
    "joint_xyz.Joint4.y",
    "joint_xyz.Joint5.z",
    "joint_xyz.Joint6.y",
    "joint_xyz.Joint6.z",
    "joint_rpy.Joint1.y",
    "joint_rpy.Joint2.x",
    "joint_rpy.Joint3.z",
    "joint_rpy.Joint4.x",
    "joint_rpy.Joint5.x",
    "joint_rpy.Joint6.x",
]

JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]


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


class KinematicChain:
    def __init__(self, urdf_path, base_link, tip_link):
        self.urdf_path = urdf_path
        self.base_link = base_link
        self.tip_link = tip_link
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

    def fk(self, joint_positions, params):
        T = np.eye(4)
        for joint in self.path:
            xyz = joint["origin_xyz"] + params["joint_xyz"].get(joint["name"], np.zeros(3, dtype=float))
            rpy = joint["origin_rpy"] + params["joint_rpy"].get(joint["name"], np.zeros(3, dtype=float))
            T = T @ transform_from_xyz_rpy(xyz, rpy)
            if joint["type"] == "revolute":
                q = joint_positions.get(joint["name"], 0.0) + params["joint_offsets"].get(joint["name"], 0.0)
                T = T @ transform_from_axis_angle(joint["axis"], q)
            elif joint["type"] == "prismatic":
                q = joint_positions.get(joint["name"], 0.0)
                delta = np.eye(4)
                delta[:3, 3] = joint["axis"] * q
                T = T @ delta

        tcp = np.eye(4)
        tcp[:3, 3] = params["tcp_xyz"]
        T = T @ tcp
        return T


def load_calibration(calibration_file):
    with open(calibration_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    tcp = np.array([float(v) for v in data.get("tcp_correction", {}).get("xyz", [0.0, 0.0, 0.0])], dtype=float)
    joint_offsets = {name: float(data.get("joint_position_offsets_rad", {}).get(name, 0.0)) for name in JOINT_NAMES}
    joint_xyz = {
        name: np.array(data.get("joint_origin_xyz_corrections_m", {}).get(name, [0.0, 0.0, 0.0]), dtype=float)
        for name in JOINT_NAMES
    }
    joint_rpy = {
        name: np.radians(np.array(data.get("joint_origin_rpy_corrections_deg", {}).get(name, [0.0, 0.0, 0.0]), dtype=float))
        for name in JOINT_NAMES
    }
    return {"tcp_xyz": tcp, "joint_offsets": joint_offsets, "joint_xyz": joint_xyz, "joint_rpy": joint_rpy}, data


def parse_param_name(name):
    if name.startswith("tcp."):
        return ("tcp", {"x": 0, "y": 1, "z": 2}[name.split(".")[1]])
    if name.startswith("joint_offset."):
        return ("joint_offset", name.split(".", 1)[1])
    if name.startswith("joint_xyz."):
        _, joint_name, axis_name = name.split(".")
        return ("joint_xyz", joint_name, {"x": 0, "y": 1, "z": 2}[axis_name])
    if name.startswith("joint_rpy."):
        _, joint_name, axis_name = name.split(".")
        return ("joint_rpy", joint_name, {"x": 0, "y": 1, "z": 2}[axis_name])
    raise ValueError(f"Unsupported parameter name: {name}")


def pack_params(base_params, param_names):
    values = []
    for name in param_names:
        kind = parse_param_name(name)
        if kind[0] == "tcp":
            values.append(base_params["tcp_xyz"][kind[1]])
        elif kind[0] == "joint_offset":
            values.append(base_params["joint_offsets"][kind[1]])
        elif kind[0] == "joint_xyz":
            values.append(base_params["joint_xyz"][kind[1]][kind[2]])
        elif kind[0] == "joint_rpy":
            values.append(base_params["joint_rpy"][kind[1]][kind[2]])
    return np.array(values, dtype=float)


def unpack_params(x, base_params, param_names):
    params = {
        "tcp_xyz": np.array(base_params["tcp_xyz"], dtype=float).copy(),
        "joint_offsets": dict(base_params["joint_offsets"]),
        "joint_xyz": {k: np.array(v, dtype=float).copy() for k, v in base_params["joint_xyz"].items()},
        "joint_rpy": {k: np.array(v, dtype=float).copy() for k, v in base_params["joint_rpy"].items()},
    }
    for i, name in enumerate(param_names):
        kind = parse_param_name(name)
        if kind[0] == "tcp":
            params["tcp_xyz"][kind[1]] = x[i]
        elif kind[0] == "joint_offset":
            params["joint_offsets"][kind[1]] = x[i]
        elif kind[0] == "joint_xyz":
            params["joint_xyz"][kind[1]][kind[2]] = x[i]
        elif kind[0] == "joint_rpy":
            params["joint_rpy"][kind[1]][kind[2]] = x[i]
    return params


def plane_normal_from_roll_pitch(roll, pitch):
    normal = R.from_euler("xy", [roll, pitch]).apply([0.0, 0.0, 1.0])
    return normal / np.linalg.norm(normal)


def load_point_samples(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    samples = []
    for sample in data.get("samples", []):
        if isinstance(sample, dict) and "joint_positions" in sample:
            samples.append({"joint_positions": {k: float(v) for k, v in sample["joint_positions"].items()}})
    if not samples:
        raise ValueError(
            f"No joint-position point samples found in {path}. Recollect with the updated tcp pivot tool."
        )
    return samples


def load_plane_samples(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    samples = []
    for sample in data.get("samples", []):
        if isinstance(sample, dict) and "joint_positions" in sample:
            samples.append({"joint_positions": {k: float(v) for k, v in sample["joint_positions"].items()}})
    if not samples:
        raise ValueError(f"No joint-position plane samples found in {path}")
    return samples


def solve_bundle(chain, point_samples, plane_samples, base_params, param_names, priors):
    if len(point_samples) < 6:
        raise ValueError("Need at least 6 point-contact samples.")
    if len(plane_samples) < 10:
        raise ValueError("Need at least 10 plane-contact samples.")

    x0 = np.concatenate([
        pack_params(base_params, param_names),
        np.zeros(3, dtype=float),
        np.zeros(2, dtype=float),
        np.zeros(1, dtype=float),
    ])

    nominal = unpack_params(x0[:len(param_names)], base_params, param_names)
    point_positions = np.array([chain.fk(s["joint_positions"], nominal)[:3, 3] for s in point_samples], dtype=float)
    plane_positions = np.array([chain.fk(s["joint_positions"], nominal)[:3, 3] for s in plane_samples], dtype=float)
    x0[len(param_names):len(param_names)+3] = point_positions.mean(axis=0)
    x0[len(param_names)+5] = -float(np.mean(plane_positions[:, 2]))

    base_pack = pack_params(base_params, param_names)

    def residual_vector(x):
        params = unpack_params(x[:len(param_names)], base_params, param_names)
        pivot = x[len(param_names):len(param_names)+3]
        plane_roll = x[len(param_names)+3]
        plane_pitch = x[len(param_names)+4]
        plane_offset = x[len(param_names)+5]
        normal = plane_normal_from_roll_pitch(plane_roll, plane_pitch)

        residuals = []
        for sample in point_samples:
            pos = chain.fk(sample["joint_positions"], params)[:3, 3]
            residuals.extend((pos - pivot).tolist())
        for sample in plane_samples:
            pos = chain.fk(sample["joint_positions"], params)[:3, 3]
            residuals.append(float(np.dot(normal, pos) + plane_offset))

        for i, name in enumerate(param_names):
            std = priors.get(name)
            if std and std > 0:
                residuals.append((x[i] - base_pack[i]) / std)
        if priors.get("plane_roll_pitch_rad", 0) > 0:
            std = priors["plane_roll_pitch_rad"]
            residuals.append(plane_roll / std)
            residuals.append(plane_pitch / std)

        return np.array(residuals, dtype=float)

    result = least_squares(
        residual_vector,
        x0,
        method="trf",
        loss="soft_l1",
        f_scale=0.002,
        x_scale="jac",
        verbose=0,
    )

    params = unpack_params(result.x[:len(param_names)], base_params, param_names)
    pivot = result.x[len(param_names):len(param_names)+3]
    plane_roll = result.x[len(param_names)+3]
    plane_pitch = result.x[len(param_names)+4]
    plane_offset = result.x[len(param_names)+5]
    normal = plane_normal_from_roll_pitch(plane_roll, plane_pitch)

    point_errors = []
    for sample in point_samples:
        pos = chain.fk(sample["joint_positions"], params)[:3, 3]
        point_errors.append(float(np.linalg.norm(pos - pivot)))
    plane_errors_signed = []
    for sample in plane_samples:
        pos = chain.fk(sample["joint_positions"], params)[:3, 3]
        plane_errors_signed.append(float(np.dot(normal, pos) + plane_offset))
    plane_errors = np.abs(np.array(plane_errors_signed, dtype=float))

    summary = {
        "success": bool(result.success),
        "message": result.message,
        "point_rmse": float(np.sqrt(np.mean(np.square(point_errors)))),
        "point_mean_error": float(np.mean(point_errors)),
        "point_max_error": float(np.max(point_errors)),
        "plane_rmse": float(np.sqrt(np.mean(np.square(plane_errors)))),
        "plane_mean_error": float(np.mean(plane_errors)),
        "plane_max_error": float(np.max(plane_errors)),
        "pivot_point_xyz": pivot.tolist(),
        "plane_roll_deg": math.degrees(plane_roll),
        "plane_pitch_deg": math.degrees(plane_pitch),
        "plane_offset_m": float(plane_offset),
        "plane_normal": normal.tolist(),
        "fit_params": {},
    }

    for name in param_names:
        kind = parse_param_name(name)
        if kind[0] == "joint_rpy":
            summary["fit_params"][name] = math.degrees(params["joint_rpy"][kind[1]][kind[2]])
        elif kind[0] == "joint_xyz":
            summary["fit_params"][name] = float(params["joint_xyz"][kind[1]][kind[2]])
        elif kind[0] == "joint_offset":
            summary["fit_params"][name] = float(params["joint_offsets"][kind[1]])
        elif kind[0] == "tcp":
            summary["fit_params"][name] = float(params["tcp_xyz"][kind[1]])

    summary["updated_calibration"] = {
        "tcp_correction": {"xyz": params["tcp_xyz"].tolist()},
        "joint_position_offsets_rad": params["joint_offsets"],
        "joint_origin_xyz_corrections_m": {k: v.tolist() for k, v in params["joint_xyz"].items()},
        "joint_origin_rpy_corrections_deg": {k: np.degrees(v).tolist() for k, v in params["joint_rpy"].items()},
    }
    return summary


def write_results(calibration_file, result, output_file=None, update_calibration=False):
    if output_file:
        output_dir = os.path.dirname(output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        with open(output_file, "w", encoding="utf-8") as stream:
            yaml.safe_dump(to_builtin_types(result), stream, sort_keys=False)

    if not update_calibration:
        return

    with open(calibration_file, "r", encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream) or {}

    calibration.setdefault("tcp_correction", {})["xyz"] = [
        float(v) for v in result["updated_calibration"]["tcp_correction"]["xyz"]
    ]
    calibration["tcp_correction"].setdefault("rpy_deg", [0.0, 0.0, 0.0])
    calibration["joint_position_offsets_rad"] = {
        k: float(v) for k, v in result["updated_calibration"]["joint_position_offsets_rad"].items()
    }
    calibration["joint_origin_xyz_corrections_m"] = {
        k: [float(x) for x in v] for k, v in result["updated_calibration"]["joint_origin_xyz_corrections_m"].items()
    }
    calibration["joint_origin_rpy_corrections_deg"] = {
        k: [float(x) for x in v] for k, v in result["updated_calibration"]["joint_origin_rpy_corrections_deg"].items()
    }

    with open(calibration_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(calibration, stream, sort_keys=False)


def build_priors(param_names, args):
    priors = {}
    for name in param_names:
        if name.startswith("tcp."):
            priors[name] = args.tcp_prior_mm / 1000.0
        elif name.startswith("joint_offset."):
            priors[name] = math.radians(args.joint_offset_prior_deg)
        elif name.startswith("joint_xyz."):
            priors[name] = args.joint_xyz_prior_mm / 1000.0
        elif name.startswith("joint_rpy."):
            priors[name] = math.radians(args.joint_rpy_prior_deg)
    priors["plane_roll_pitch_rad"] = math.radians(args.plane_tilt_prior_deg)
    return priors


def parse_args():
    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    parser = argparse.ArgumentParser(
        description="One-stop mixed-constraint kinematic calibration using point and plane contact datasets."
    )
    parser.add_argument("--urdf-file", default=os.path.expanduser("~/alicia/ros2_ws/src/alicia_d_ros2_upstream/alicia_d_descriptions/urdf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.urdf"))
    parser.add_argument("--calibration-file", default=os.path.expanduser("~/alicia/ros2_ws/src/alicia_d_ros2_upstream/alicia_d_moveit/config/kinematic_calibration.yaml"))
    parser.add_argument("--point-samples", default=os.path.expanduser("~/alicia/calibration/kinematic/20260417/tcp_pivot_samples.yaml"))
    parser.add_argument("--plane-samples", default=os.path.expanduser("~/alicia/calibration/kinematic/20260417/geometric_plane_samples.yaml"))
    parser.add_argument("--base-link", default="base_link")
    parser.add_argument("--tip-link", default="gripper_center")
    parser.add_argument("--fit-params", default=",".join(DEFAULT_PARAM_NAMES))
    parser.add_argument("--tcp-prior-mm", type=float, default=20.0)
    parser.add_argument("--joint-offset-prior-deg", type=float, default=3.0)
    parser.add_argument("--joint-xyz-prior-mm", type=float, default=8.0)
    parser.add_argument("--joint-rpy-prior-deg", type=float, default=3.0)
    parser.add_argument("--plane-tilt-prior-deg", type=float, default=10.0)
    parser.add_argument("--output-file", default=os.path.expanduser(f"~/alicia/calibration/kinematic/{run_stamp}/kinematic_contact_bundle_result.yaml"))
    parser.add_argument("--update-calibration", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    param_names = [item.strip() for item in args.fit_params.split(",") if item.strip()]

    print()
    print("One-stop contact-bundle calibration")
    print("-----------------------------------")
    print(f"URDF file      : {args.urdf_file}")
    print(f"Calibration    : {args.calibration_file}")
    print(f"Point samples  : {args.point_samples}")
    print(f"Plane samples  : {args.plane_samples}")
    print(f"Fit params     : {', '.join(param_names)}")
    print()

    base_params, _ = load_calibration(args.calibration_file)
    point_samples = load_point_samples(args.point_samples)
    plane_samples = load_plane_samples(args.plane_samples)
    chain = KinematicChain(args.urdf_file, args.base_link, args.tip_link)
    priors = build_priors(param_names, args)

    result = solve_bundle(chain, point_samples, plane_samples, base_params, param_names, priors)

    print("Solved mixed-constraint calibration")
    print(f"Point residuals : rmse={result['point_rmse']:.6f} m, mean={result['point_mean_error']:.6f} m, max={result['point_max_error']:.6f} m")
    print(f"Plane residuals : rmse={result['plane_rmse']:.6f} m, mean={result['plane_mean_error']:.6f} m, max={result['plane_max_error']:.6f} m")
    print(f"Plane tilt      : roll={result['plane_roll_deg']:+.3f} deg, pitch={result['plane_pitch_deg']:+.3f} deg")
    print(f"Solver          : success={result['success']} message={result['message']}")
    print()
    print("Fitted parameters:")
    for name in param_names:
        value = result["fit_params"][name]
        if name.startswith(("tcp.", "joint_xyz.")):
            print(f"  {name}: {value:+.6f} m ({value * 1000.0:+.2f} mm)")
        elif name.startswith("joint_offset."):
            print(f"  {name}: {value:+.6f} rad ({math.degrees(value):+.3f} deg)")
        elif name.startswith("joint_rpy."):
            print(f"  {name}: {value:+.6f} deg")

    write_results(args.calibration_file, result, output_file=args.output_file, update_calibration=args.update_calibration)
    print()
    print(f"Wrote detailed result to {args.output_file}")
    if args.update_calibration:
        print(f"Updated calibration file: {args.calibration_file}")


if __name__ == "__main__":
    main()
