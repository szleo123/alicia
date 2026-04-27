#!/usr/bin/env python3
"""Launch cube detection for the current Alicia-D wrist-camera stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for cube detection."""
    try:
        pkg_share = get_package_share_directory('alicia_d_cube_sort')
        config_file = os.path.join(pkg_share, 'config', 'cube_sorting.yaml')
    except Exception:
        config_file = ''

    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='hand_eye_calibration_result.yaml',
        description='Hand-eye calibration YAML filename or absolute path.'
    )

    publish_hand_eye_tf_arg = DeclareLaunchArgument(
        'publish_hand_eye_tf',
        default_value='false',
        description='Publish saved hand-eye TF for detection-only workflows. Keep false if real_robot.launch.py already publishes it.'
    )

    camera_optical_frame_arg = DeclareLaunchArgument(
        'camera_optical_frame',
        default_value='camera_color_optical_frame',
        description='Camera optical frame used during calibration and detection.'
    )

    apply_optical_correction_arg = DeclareLaunchArgument(
        'apply_optical_correction',
        default_value='true',
        description='Convert saved optical-frame calibration result to child_frame_id before publishing.'
    )

    depth_mode_arg = DeclareLaunchArgument(
        'depth_mode',
        default_value='true',
        description='Use aligned depth for 3D cube localization.'
    )

    show_image_arg = DeclareLaunchArgument(
        'show_image',
        default_value='true',
        description='Show OpenCV visualization window.'
    )

    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/camera/color/image_rect_raw',
        description='Color image topic.'
    )

    color_info_topic_arg = DeclareLaunchArgument(
        'color_info_topic',
        default_value='/camera/camera/color/camera_info',
        description='Color camera info topic.'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/camera/depth/image_rect_raw',
        description='Depth image topic.'
    )

    depth_info_topic_arg = DeclareLaunchArgument(
        'depth_info_topic',
        default_value='/camera/camera/depth/camera_info',
        description='Depth camera info topic.'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame.'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description='Camera frame used by the detector before TF into the robot base frame.'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to cube sorting configuration file.'
    )

    hand_eye_tf_node = Node(
        package='alicia_d_calibration',
        executable='publish_hand_eye_tf.py',
        name='cube_detection_hand_eye_tf',
        output='screen',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
            'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
            'apply_optical_correction': LaunchConfiguration('apply_optical_correction'),
        }],
        condition=IfCondition(LaunchConfiguration('publish_hand_eye_tf')),
    )

    cube_detector_node = Node(
        package='alicia_d_cube_sort',
        executable='cube_detection.py',
        name='cube_detector',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'depth_mode': LaunchConfiguration('depth_mode'),
            'show_image': LaunchConfiguration('show_image'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'color_topic': LaunchConfiguration('color_topic'),
            'color_info_topic': LaunchConfiguration('color_info_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_info_topic': LaunchConfiguration('depth_info_topic'),
            'calibration_file': LaunchConfiguration('calibration_file'),
        }]
    )

    return LaunchDescription([
        calibration_file_arg,
        publish_hand_eye_tf_arg,
        camera_optical_frame_arg,
        apply_optical_correction_arg,
        depth_mode_arg,
        show_image_arg,
        color_topic_arg,
        color_info_topic_arg,
        depth_topic_arg,
        depth_info_topic_arg,
        base_frame_arg,
        camera_frame_arg,
        config_file_arg,
        hand_eye_tf_node,
        cube_detector_node,
    ])
