"""Launch Geomagic teleop and MoveIt Servo against an already-running real robot stack."""

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    gripper_type = LaunchConfiguration("gripper_type").perform(context)
    port = LaunchConfiguration("port").perform(context)
    speed_deg_s = float(LaunchConfiguration("speed_deg_s").perform(context))
    planning_pipeline = LaunchConfiguration("planning_pipeline").perform(context)

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
    servo_yaml["command_out_topic"] = "/alicia_d_teleop/servo_raw_joint_trajectory"
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


def trajectory_gate_setup(context, *args, **kwargs):
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() == "true"
    gate_armed_on_start = LaunchConfiguration("gate_armed_on_start").perform(context).lower() == "true"
    gate_output_topic = LaunchConfiguration("gate_output_topic").perform(context)
    output_topic = (
        "/alicia_d_teleop/servo_dry_run_joint_trajectory"
        if dry_run
        else gate_output_topic
    )

    return [
        Node(
            package="alicia_d_teleop",
            executable="trajectory_deadman_gate.py",
            name="trajectory_deadman_gate",
            output="screen",
            parameters=[{
                "input_trajectory_topic": "/alicia_d_teleop/servo_raw_joint_trajectory",
                "output_trajectory_topic": output_topic,
                "input_twist_topic": "/alicia_d_teleop/twist_cmd",
                "input_buttons_topic": "/geomagic_touch/buttons",
                "deadman_button_index": 0,
                "require_deadman": True,
                "require_nonzero_twist": True,
                "armed_on_start": gate_armed_on_start,
            }],
        )
    ]


def generate_launch_description():
    use_omni_adapter = LaunchConfiguration("use_omni_adapter")
    output_twist_topic = LaunchConfiguration("output_twist_topic")
    start_servo = LaunchConfiguration("start_servo")
    dry_run = LaunchConfiguration("dry_run")
    gripper_type = LaunchConfiguration("gripper_type")
    gate_output_topic = LaunchConfiguration("gate_output_topic")
    orientation_enabled = LaunchConfiguration("orientation_enabled")

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
            "start_servo",
            default_value="true",
            description="Automatically start and unpause MoveIt Servo after launch.",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Publish Servo output to a dry-run topic instead of the real arm controller.",
        ),
        DeclareLaunchArgument(
            "gate_output_topic",
            default_value="/Alicia_controller/joint_trajectory",
            description="Live output topic for the trajectory gate when dry_run is false.",
        ),
        DeclareLaunchArgument(
            "gate_armed_on_start",
            default_value="true",
            description="Arm trajectory gate on startup; deadman is still required to forward commands.",
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
        DeclareLaunchArgument(
            "orientation_enabled",
            default_value="false",
            description="Enable Touch stylus orientation as real-robot angular teleop.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tuning_launch),
            launch_arguments={
                "use_omni_adapter": use_omni_adapter,
                "output_twist_topic": output_twist_topic,
                "initial_mode": "jog",
                "require_tool_tf": "true",
                "translation_gain": "1.0",
                "translation_deadband_m": "0.015",
                "rotation_gain": "0.4",
                "orientation_deadband_rad": "0.10",
                "max_linear_speed_m_s": "0.006",
                "max_angular_speed_rad_s": "0.03",
                "low_pass_alpha": "0.15",
                "orientation_enabled": orientation_enabled,
                "angular_control_mode": "orientation_follow",
            }.items(),
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
        OpaqueFunction(function=trajectory_gate_setup),
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
