"""Preview Geomagic Touch teleoperation without commanding the robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("alicia_d_teleop"),
        "config",
        "geomagic_teleop.yaml",
    ])

    use_omni_adapter = LaunchConfiguration("use_omni_adapter")
    raw_twist_topic = LaunchConfiguration("raw_twist_topic")
    output_twist_topic = LaunchConfiguration("output_twist_topic")
    initial_mode = LaunchConfiguration("initial_mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_omni_adapter",
            default_value="false",
            description="Start the optional omni_msgs/OmniState adapter after the Geomagic ROS 2 driver is installed.",
        ),
        DeclareLaunchArgument(
            "raw_twist_topic",
            default_value="/alicia_d_teleop/raw_twist_cmd",
            description="Unsupervised teleop twist from the input mapper.",
        ),
        DeclareLaunchArgument(
            "output_twist_topic",
            default_value="/alicia_d_teleop/twist_cmd",
            description="Filtered twist output topic. Remap to MoveIt Servo only after simulation validation.",
        ),
        DeclareLaunchArgument(
            "initial_mode",
            default_value="jog",
            description="Safety-filter mode at startup: hold, jog, approach, grip, or retreat.",
        ),
        Node(
            package="alicia_d_teleop",
            executable="geomagic_omni_state_adapter.py",
            name="geomagic_omni_state_adapter",
            output="screen",
            condition=IfCondition(use_omni_adapter),
            parameters=[config_file],
        ),
        Node(
            package="alicia_d_teleop",
            executable="geomagic_cartesian_teleop.py",
            name="geomagic_cartesian_teleop",
            output="screen",
            parameters=[
                config_file,
                {"output_twist_topic": raw_twist_topic},
            ],
        ),
        Node(
            package="alicia_d_teleop",
            executable="teleop_safety_filter.py",
            name="teleop_safety_filter",
            output="screen",
            parameters=[
                config_file,
                {
                    "input_twist_topic": raw_twist_topic,
                    "output_twist_topic": output_twist_topic,
                    "default_mode": initial_mode,
                    "require_tool_tf": False,
                },
            ],
        ),
    ])
