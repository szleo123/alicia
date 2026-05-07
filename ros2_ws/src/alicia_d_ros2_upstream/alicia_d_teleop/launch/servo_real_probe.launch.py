"""Launch MoveIt Servo alone for real-robot Cartesian command isolation tests."""

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    gripper_type = LaunchConfiguration("gripper_type").perform(context)
    port = LaunchConfiguration("port").perform(context)
    speed_deg_s = float(LaunchConfiguration("speed_deg_s").perform(context))
    planning_pipeline = LaunchConfiguration("planning_pipeline").perform(context)
    command_topic = LaunchConfiguration("command_topic").perform(context)
    command_frame = LaunchConfiguration("command_frame").perform(context)
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() == "true"
    live_output_topic = LaunchConfiguration("live_output_topic").perform(context)
    check_collisions = LaunchConfiguration("check_collisions").perform(context).lower() == "true"

    alicia_moveit_share = get_package_share_directory("alicia_d_moveit")
    moveit_launch_dir = os.path.join(alicia_moveit_share, "launch")
    if moveit_launch_dir not in sys.path:
        sys.path.append(moveit_launch_dir)

    from moveit_config_builder import get_versioned_moveit_config

    moveit_config = get_versioned_moveit_config(
        gripper_type=gripper_type,
        port=port,
        use_fake_hardware=False,
        speed_deg_s=speed_deg_s,
        default_planning_pipeline=planning_pipeline,
    )

    alicia_teleop_share = get_package_share_directory("alicia_d_teleop")
    servo_config_file = os.path.join(alicia_teleop_share, "config", "alicia_servo_real.yaml")
    with open(servo_config_file, "r", encoding="utf-8") as config:
        servo_yaml = yaml.safe_load(config)

    servo_yaml["cartesian_command_in_topic"] = command_topic
    servo_yaml["robot_link_command_frame"] = command_frame
    servo_yaml["status_topic"] = "/alicia_d_teleop/servo_probe_status"
    servo_yaml["command_out_topic"] = (
        "/alicia_d_teleop/servo_probe_joint_trajectory"
        if dry_run
        else live_output_topic
    )
    servo_yaml["check_collisions"] = check_collisions

    return [
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="moveit_servo",
            output="screen",
            parameters=[
                {"moveit_servo": servo_yaml},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )
    ]


def generate_launch_description():
    start_servo = LaunchConfiguration("start_servo")

    return LaunchDescription([
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Publish Servo output to a probe topic instead of the real arm controller.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="/alicia_d_teleop/servo_probe_twist_cmd",
            description="TwistStamped topic to send direct probe commands into MoveIt Servo.",
        ),
        DeclareLaunchArgument(
            "command_frame",
            default_value="base_link",
            description="Frame for direct probe TwistStamped commands: base_link or tool0.",
        ),
        DeclareLaunchArgument(
            "live_output_topic",
            default_value="/Alicia_controller/joint_trajectory",
            description="Live controller topic used only when dry_run is false.",
        ),
        DeclareLaunchArgument(
            "check_collisions",
            default_value="true",
            description="Enable Servo collision scaling during probe runs.",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="true",
            description="Automatically start and unpause MoveIt Servo after launch.",
        ),
        DeclareLaunchArgument(
            "gripper_type",
            default_value="50mm",
            description="Alicia-D real gripper type: 50mm or 100mm.",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="",
            description="Serial port for matching the already-running real robot config.",
        ),
        DeclareLaunchArgument(
            "speed_deg_s",
            default_value="20",
            description="Default hardware speed for matching the already-running real robot config.",
        ),
        DeclareLaunchArgument(
            "planning_pipeline",
            default_value="ompl",
            description="Planning pipeline for matching the already-running real robot config.",
        ),
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "topic",
                        "pub",
                        "--once",
                        "/demonstration",
                        "std_msgs/msg/Bool",
                        "{data: false}",
                    ],
                    output="screen",
                ),
            ],
        ),
        OpaqueFunction(function=launch_setup),
        TimerAction(
            period=5.0,
            condition=IfCondition(start_servo),
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
            condition=IfCondition(start_servo),
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
        TimerAction(
            period=7.0,
            condition=IfCondition(start_servo),
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/moveit_servo/change_control_dimensions",
                        "moveit_msgs/srv/ChangeControlDimensions",
                        (
                            "{control_x_translation: true, control_y_translation: true, "
                            "control_z_translation: true, control_x_rotation: true, "
                            "control_y_rotation: true, control_z_rotation: true}"
                        ),
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
