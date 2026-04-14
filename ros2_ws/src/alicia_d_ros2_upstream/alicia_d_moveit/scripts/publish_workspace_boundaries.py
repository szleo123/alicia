#!/usr/bin/env python3

import os

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
import yaml


def make_box(object_id, frame_id, center_xyz, size_xyz):
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = frame_id
    obj.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size_xyz)

    pose = Pose()
    pose.position.x = center_xyz[0]
    pose.position.y = center_xyz[1]
    pose.position.z = center_xyz[2]
    pose.orientation.w = 1.0

    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    return obj


class WorkspaceBoundaryPublisher(Node):
    def __init__(self):
        super().__init__("world_scene_publisher")
        self.done = False

        self.declare_parameter("scene_file", "")

        self.scene_file = self.get_parameter("scene_file").value

        if not self.scene_file:
            self.get_logger().info("No world scene file configured. Skipping custom collision objects.")
            self.done = True
            return

        self.apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.timer = self.create_timer(0.5, self._try_apply_scene)

    def _load_custom_objects(self):
        if not self.scene_file:
            return []

        if not os.path.exists(self.scene_file):
            raise ValueError(f"Scene file does not exist: {self.scene_file}")

        with open(self.scene_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        raw_objects = config.get("objects", [])
        if raw_objects is None:
            return []
        if not isinstance(raw_objects, list):
            raise ValueError("Scene file must contain an 'objects' list.")

        objects = []
        for index, spec in enumerate(raw_objects):
            if not isinstance(spec, dict):
                raise ValueError(f"Scene object #{index} must be a mapping.")

            object_type = spec.get("type", "box")
            if object_type != "box":
                raise ValueError(
                    f"Scene object '{spec.get('id', index)}' has unsupported type '{object_type}'. Only 'box' is supported."
                )

            object_id = spec.get("id", f"scene_box_{index}")
            frame_id = spec.get("frame_id", "base_link")
            position = spec.get("position")
            size = spec.get("size")
            orientation = spec.get("orientation", [0.0, 0.0, 0.0, 1.0])

            if not isinstance(position, list) or len(position) != 3:
                raise ValueError(f"Scene object '{object_id}' must define position as [x, y, z].")
            if not isinstance(size, list) or len(size) != 3:
                raise ValueError(f"Scene object '{object_id}' must define size as [x, y, z].")
            if not isinstance(orientation, list) or len(orientation) != 4:
                raise ValueError(f"Scene object '{object_id}' must define orientation as [x, y, z, w].")
            if any(float(dimension) <= 0.0 for dimension in size):
                raise ValueError(f"Scene object '{object_id}' must have strictly positive size values.")

            obj = make_box(
                object_id,
                frame_id,
                tuple(float(value) for value in position),
                tuple(float(value) for value in size),
            )
            obj.primitive_poses[0].orientation.x = float(orientation[0])
            obj.primitive_poses[0].orientation.y = float(orientation[1])
            obj.primitive_poses[0].orientation.z = float(orientation[2])
            obj.primitive_poses[0].orientation.w = float(orientation[3])
            objects.append(obj)

        return objects

    def _build_scene(self):
        objects = self._load_custom_objects()

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        return scene

    def _try_apply_scene(self):
        if not self.apply_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info("Waiting for /apply_planning_scene service...")
            return

        try:
            scene = self._build_scene()
        except ValueError as exc:
            self.get_logger().error(f"Invalid workspace boundary parameters: {exc}")
            self.done = True
            return

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_client.call_async(request)
        future.add_done_callback(self._handle_result)
        self.timer.cancel()

    def _handle_result(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to apply workspace boundaries: {exc}")
            self.done = True
            return

        if response.success:
            self.get_logger().info(f"Applied custom world scene from {self.scene_file}")
        else:
            self.get_logger().error("MoveIt rejected the world scene planning scene update.")

        self.done = True


def main():
    rclpy.init()
    node = WorkspaceBoundaryPublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
