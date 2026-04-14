"""Convenience launch wrapper for Intel RealSense D405 with sensible defaults."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ])
            ),
            launch_arguments={
                "enable_color": "true",
                "enable_depth": "true",
                "enable_infra1": "true",
                "enable_infra2": "true",
                "pointcloud.enable": "true",
                "align_depth.enable": "true",
                "rotation_filter.enable": "true",
                "rotation_filter.rotation": "180.0",
                "publish_tf": "true",
                "log_level": "info",
            }.items(),
        )
    ])
