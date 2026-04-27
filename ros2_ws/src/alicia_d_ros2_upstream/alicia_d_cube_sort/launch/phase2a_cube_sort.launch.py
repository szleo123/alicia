#!/usr/bin/env python3
"""Phase 2A convenience launch for D405-guided cube detection and sorting."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for the current Phase 2A task stack."""
    pkg_share = get_package_share_directory('alicia_d_cube_sort')
    config_file = os.path.join(pkg_share, 'config', 'cube_sorting.yaml')
    launch_dir = os.path.join(pkg_share, 'launch')

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start the sorting workflow. False is safer for first-run validation.'
    )

    show_image_arg = DeclareLaunchArgument(
        'show_image',
        default_value='true',
        description='Show the detection visualization window.'
    )

    publish_hand_eye_tf_arg = DeclareLaunchArgument(
        'publish_hand_eye_tf',
        default_value='false',
        description='Publish saved hand-eye TF inside this launch. Keep false if real_robot.launch.py already publishes it.'
    )

    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='hand_eye_calibration_result.yaml',
        description='Hand-eye calibration YAML filename or absolute path.'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to cube sorting configuration file.'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame.'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description='Camera optical frame used by the detector.'
    )

    camera_optical_frame_arg = DeclareLaunchArgument(
        'camera_optical_frame',
        default_value='camera_color_optical_frame',
        description='Camera optical frame used during calibration.'
    )

    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/camera/color/image_rect_raw',
        description='D405 color image topic.'
    )

    color_info_topic_arg = DeclareLaunchArgument(
        'color_info_topic',
        default_value='/camera/camera/color/camera_info',
        description='D405 color camera info topic.'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/camera/depth/image_rect_raw',
        description='D405 depth image topic.'
    )

    depth_info_topic_arg = DeclareLaunchArgument(
        'depth_info_topic',
        default_value='/camera/camera/depth/camera_info',
        description='D405 depth camera info topic.'
    )

    detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cube_detection.launch.py')),
        launch_arguments={
            'calibration_file': LaunchConfiguration('calibration_file'),
            'publish_hand_eye_tf': LaunchConfiguration('publish_hand_eye_tf'),
            'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
            'depth_mode': 'true',
            'show_image': LaunchConfiguration('show_image'),
            'color_topic': LaunchConfiguration('color_topic'),
            'color_info_topic': LaunchConfiguration('color_info_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_info_topic': LaunchConfiguration('depth_info_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'config_file': LaunchConfiguration('config_file'),
        }.items(),
    )

    sorting_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cube_sorting.launch.py')),
        launch_arguments={
            'auto_start': LaunchConfiguration('auto_start'),
            'base_frame': LaunchConfiguration('base_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'config_file': LaunchConfiguration('config_file'),
        }.items(),
    )

    return LaunchDescription([
        LogInfo(msg='========================================'),
        LogInfo(msg='Alicia Phase 2A: D405 Cube Sorting'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Assumes the robot is already running via real_robot.launch.py'),
        LogInfo(msg='Assumes the D405 is already running via d405.launch.py or an equivalent RealSense launch'),
        LogInfo(msg='========================================'),
        auto_start_arg,
        show_image_arg,
        publish_hand_eye_tf_arg,
        calibration_file_arg,
        config_file_arg,
        base_frame_arg,
        camera_frame_arg,
        camera_optical_frame_arg,
        color_topic_arg,
        color_info_topic_arg,
        depth_topic_arg,
        depth_info_topic_arg,
        detection_launch,
        sorting_launch,
    ])
