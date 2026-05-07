"""Launch Geomagic teleop plus a preview pose integrator for safe tuning."""

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
    require_tool_tf = LaunchConfiguration("require_tool_tf")
    translation_gain = LaunchConfiguration("translation_gain")
    translation_deadband_m = LaunchConfiguration("translation_deadband_m")
    rotation_gain = LaunchConfiguration("rotation_gain")
    orientation_deadband_rad = LaunchConfiguration("orientation_deadband_rad")
    max_linear_speed_m_s = LaunchConfiguration("max_linear_speed_m_s")
    max_angular_speed_rad_s = LaunchConfiguration("max_angular_speed_rad_s")
    low_pass_alpha = LaunchConfiguration("low_pass_alpha")
    orientation_enabled = LaunchConfiguration("orientation_enabled")
    angular_control_mode = LaunchConfiguration("angular_control_mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_omni_adapter",
            default_value="true",
            description="Adapt /phantom/state from the Geomagic Touch ROS 2 driver.",
        ),
        DeclareLaunchArgument(
            "raw_twist_topic",
            default_value="/alicia_d_teleop/raw_twist_cmd",
            description="Unsupervised teleop twist from the input mapper.",
        ),
        DeclareLaunchArgument(
            "output_twist_topic",
            default_value="/alicia_d_teleop/twist_cmd",
            description="Filtered teleop twist command topic.",
        ),
        DeclareLaunchArgument(
            "initial_mode",
            default_value="jog",
            description="Safety-filter mode at startup: hold, jog, approach, grip, or retreat.",
        ),
        DeclareLaunchArgument(
            "require_tool_tf",
            default_value="false",
            description="Fail closed unless tool TF is available for workspace checks.",
        ),
        DeclareLaunchArgument(
            "translation_gain",
            default_value="1.5",
            description="Optional override for Touch-to-robot translation scale.",
        ),
        DeclareLaunchArgument(
            "translation_deadband_m",
            default_value="0.004",
            description="Ignore small Touch displacement after clutching.",
        ),
        DeclareLaunchArgument(
            "rotation_gain",
            default_value="4.0",
            description="Optional override for Touch orientation-to-tool angular scale.",
        ),
        DeclareLaunchArgument(
            "orientation_deadband_rad",
            default_value="0.05",
            description="Ignore small Touch orientation changes after clutching.",
        ),
        DeclareLaunchArgument(
            "max_linear_speed_m_s",
            default_value="0.18",
            description="Optional override for maximum Cartesian linear speed.",
        ),
        DeclareLaunchArgument(
            "max_angular_speed_rad_s",
            default_value="0.60",
            description="Optional override for maximum Cartesian angular speed.",
        ),
        DeclareLaunchArgument(
            "low_pass_alpha",
            default_value="0.70",
            description="Optional override for command smoothing alpha.",
        ),
        DeclareLaunchArgument(
            "orientation_enabled",
            default_value="false",
            description="Map Touch stylus orientation delta into angular twist.",
        ),
        DeclareLaunchArgument(
            "angular_control_mode",
            default_value="orientation_follow",
            description="Angular mode: orientation_follow or axis_delta.",
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
                {
                    "output_twist_topic": raw_twist_topic,
                    "translation_gain": translation_gain,
                    "translation_deadband_m": translation_deadband_m,
                    "rotation_gain": rotation_gain,
                    "orientation_deadband_rad": orientation_deadband_rad,
                    "max_linear_speed_m_s": max_linear_speed_m_s,
                    "max_angular_speed_rad_s": max_angular_speed_rad_s,
                    "low_pass_alpha": low_pass_alpha,
                    "orientation_enabled": orientation_enabled,
                    "angular_control_mode": angular_control_mode,
                },
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
                    "require_tool_tf": require_tool_tf,
                },
            ],
        ),
        Node(
            package="alicia_d_teleop",
            executable="twist_preview_integrator.py",
            name="twist_preview_integrator",
            output="screen",
            parameters=[
                config_file,
                {"input_twist_topic": output_twist_topic},
            ],
        ),
    ])
