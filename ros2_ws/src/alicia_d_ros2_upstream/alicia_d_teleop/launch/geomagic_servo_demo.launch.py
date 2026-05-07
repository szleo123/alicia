"""Launch Alicia-D fake hardware, Geomagic teleop, and MoveIt Servo."""

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    gripper_type = LaunchConfiguration("gripper_type").perform(context)

    alicia_moveit_share = get_package_share_directory("alicia_d_moveit")
    moveit_launch_dir = os.path.join(alicia_moveit_share, "launch")
    if moveit_launch_dir not in sys.path:
        sys.path.append(moveit_launch_dir)

    from moveit_config_builder import get_versioned_moveit_config

    moveit_config = get_versioned_moveit_config(
        gripper_type=gripper_type,
        use_fake_hardware=True,
        initial_positions_file=os.path.join(
            alicia_moveit_share,
            "config",
            "servo_initial_positions.yaml",
        ),
    )

    alicia_teleop_share = get_package_share_directory("alicia_d_teleop")
    servo_config_file = os.path.join(alicia_teleop_share, "config", "alicia_servo.yaml")
    with open(servo_config_file, "r", encoding="utf-8") as config:
        servo_yaml = yaml.safe_load(config)
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="moveit_servo",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return [servo_node]


def generate_launch_description():
    use_omni_adapter = LaunchConfiguration("use_omni_adapter")
    output_twist_topic = LaunchConfiguration("output_twist_topic")
    initial_mode = LaunchConfiguration("initial_mode")
    gripper_type = LaunchConfiguration("gripper_type")

    moveit_demo_launch = PathJoinSubstitution([
        FindPackageShare("alicia_d_moveit"),
        "launch",
        "demo.launch.py",
    ])
    servo_initial_positions_file = PathJoinSubstitution([
        FindPackageShare("alicia_d_moveit"),
        "config",
        "servo_initial_positions.yaml",
    ])
    tuning_launch = PathJoinSubstitution([
        FindPackageShare("alicia_d_teleop"),
        "launch",
        "geomagic_tuning.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_omni_adapter",
            default_value="true",
            description="Adapt /phantom/state from the Geomagic Touch ROS 2 driver.",
        ),
        DeclareLaunchArgument(
            "output_twist_topic",
            default_value="/alicia_d_teleop/twist_cmd",
            description="Teleop twist command topic sent to MoveIt Servo.",
        ),
        DeclareLaunchArgument(
            "initial_mode",
            default_value="jog",
            description="Safety-filter mode at startup: hold, jog, approach, grip, or retreat.",
        ),
        DeclareLaunchArgument(
            "gripper_type",
            default_value="50mm",
            description="Alicia-D demo gripper type: 50mm or 100mm.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_demo_launch),
            launch_arguments={
                "gripper_type": gripper_type,
                "initial_positions_file": servo_initial_positions_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tuning_launch),
            launch_arguments={
                "use_omni_adapter": use_omni_adapter,
                "output_twist_topic": output_twist_topic,
                "initial_mode": initial_mode,
                "require_tool_tf": "true",
            }.items(),
        ),
        OpaqueFunction(function=launch_setup),
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/moveit_servo/start_servo",
                        "std_srvs/srv/Trigger",
                        "{}",
                    ],
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/moveit_servo/unpause_servo",
                        "std_srvs/srv/Trigger",
                        "{}",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
