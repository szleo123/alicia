#!/usr/bin/env python3
"""Publish saved hand-eye calibration as a static TF."""

import os
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster


def _parse_bool(value) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _resolve_calibration_path(calibration_file: str) -> str:
    if os.path.isabs(calibration_file):
        return calibration_file

    candidates = []

    try:
        share_dir = Path(get_package_share_directory("alicia_d_calibration")).resolve()
        candidates.append(str((share_dir / "config" / calibration_file).resolve()))
        for parent in list(share_dir.parents)[:8]:
            candidates.append(str((parent / "alicia_d_calibration" / "config" / calibration_file).resolve()))
            candidates.append(str((parent / "src" / "alicia_d_calibration" / "config" / calibration_file).resolve()))
    except Exception:
        pass

    for path in candidates:
        if path and os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as stream:
                    data = yaml.safe_load(stream)
                if isinstance(data, dict) and "hand_eye_calibration" in data:
                    return path
            except Exception:
                pass

    return calibration_file


def _load_matrix_from_yaml(hand_eye: dict) -> np.ndarray:
    transform = hand_eye["transform"]
    matrix_data = transform.get("matrix_4x4")
    if matrix_data:
        return np.array(matrix_data, dtype=float)

    translation = transform["translation"]
    quaternion = transform["rotation"]["quaternion"]
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat([
        quaternion["x"],
        quaternion["y"],
        quaternion["z"],
        quaternion["w"],
    ]).as_matrix()
    matrix[:3, 3] = [translation["x"], translation["y"], translation["z"]]
    return matrix


class HandEyeTFPublisher(Node):
    def __init__(self):
        super().__init__("hand_eye_tf_publisher")

        self.declare_parameter("calibration_file", "hand_eye_calibration_result.yaml")
        self.declare_parameter("camera_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("apply_optical_correction", True)
        self.declare_parameter("lookup_timeout_sec", 1.0)

        calibration_file = self.get_parameter("calibration_file").value
        self.camera_optical_frame = str(self.get_parameter("camera_optical_frame").value)
        self.apply_optical_correction = _parse_bool(
            self.get_parameter("apply_optical_correction").value
        )
        self.lookup_timeout_sec = float(self.get_parameter("lookup_timeout_sec").value)

        self.calibration_file = _resolve_calibration_path(str(calibration_file))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self._try_publish)
        self.published = False

        self.hand_eye = self._load_calibration()
        self.parent_frame = self.hand_eye.get("frame_id", "gripper_center")
        self.child_frame = self.hand_eye.get("child_frame_id", "camera_link")
        self.raw_matrix = _load_matrix_from_yaml(self.hand_eye)

        self.get_logger().info(f"Using hand-eye calibration file: {self.calibration_file}")

    def _load_calibration(self):
        if not os.path.exists(self.calibration_file):
            raise FileNotFoundError(f"Calibration file not found: {self.calibration_file}")

        with open(self.calibration_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)

        if not isinstance(data, dict) or "hand_eye_calibration" not in data:
            raise ValueError(
                f"Calibration file is empty or missing 'hand_eye_calibration': {self.calibration_file}"
            )

        return data["hand_eye_calibration"]

    def _lookup_link_to_optical(self):
        transform = self.tf_buffer.lookup_transform(
            self.child_frame,
            self.camera_optical_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = np.eye(4)
        matrix[:3, :3] = R.from_quat([
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        ]).as_matrix()
        matrix[:3, 3] = [translation.x, translation.y, translation.z]
        return matrix

    def _try_publish(self):
        if self.published:
            return

        final_matrix = self.raw_matrix
        if self.apply_optical_correction and self.child_frame != self.camera_optical_frame:
            try:
                link_to_optical = self._lookup_link_to_optical()
                final_matrix = self.raw_matrix @ np.linalg.inv(link_to_optical)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().info(
                    f"Waiting for TF {self.child_frame} -> {self.camera_optical_frame} "
                    f"before publishing hand-eye calibration ({exc})"
                )
                return

        translation = final_matrix[:3, 3]
        quaternion = R.from_matrix(final_matrix[:3, :3]).as_quat()

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        msg.transform.rotation.x = float(quaternion[0])
        msg.transform.rotation.y = float(quaternion[1])
        msg.transform.rotation.z = float(quaternion[2])
        msg.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(msg)
        self.published = True
        self.timer.cancel()
        self.get_logger().info(
            f"Published static TF {self.parent_frame} -> {self.child_frame} from hand-eye calibration."
        )


def main():
    rclpy.init()
    node = HandEyeTFPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
