"""Launch Alicia-D MoveIt demo plus Geomagic Touch teleop preview tuning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
            description="Teleop twist command topic.",
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
    ])
