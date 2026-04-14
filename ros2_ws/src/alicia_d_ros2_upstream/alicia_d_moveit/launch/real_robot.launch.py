"""Launch file for controlling real Alicia-D robot with MoveIt."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
import sys
import os
import subprocess
sys.path.append(os.path.dirname(__file__))
from moveit_config_builder import get_versioned_moveit_config


def launch_setup(context, *args, **kwargs):
    """Setup real robot launch with versioned config."""
    # Get launch configuration values
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    port = LaunchConfiguration('port').perform(context)
    speed_deg_s = float(LaunchConfiguration('speed_deg_s').perform(context))
    planning_pipeline = LaunchConfiguration('planning_pipeline').perform(context)
    allowed_start_tolerance = float(LaunchConfiguration('allowed_start_tolerance').perform(context))
    world_scene_file = LaunchConfiguration('world_scene_file').perform(context)
    demonstration_ui = LaunchConfiguration('demonstration_ui').perform(context).lower() == 'true'
    publish_hand_eye_calibration = LaunchConfiguration('publish_hand_eye_calibration').perform(context).lower() == 'true'
    hand_eye_calibration_file = LaunchConfiguration('hand_eye_calibration_file').perform(context)
    hand_eye_camera_optical_frame = LaunchConfiguration('hand_eye_camera_optical_frame').perform(context)
    apply_hand_eye_optical_correction = LaunchConfiguration('apply_hand_eye_optical_correction').perform(context).lower() == 'true'
    
    # Validate gripper type
    if gripper_type not in ["50mm", "100mm"]:
        print(f'\033[1;33m[WARN] Invalid gripper_type: {gripper_type}, using default: 50mm\033[0m')
        gripper_type = "50mm"
    
    print(f'\033[1;32m[INFO] Serial port: {port if port else "(auto-detect)"}\033[0m')
    print(f'\033[1;32m[INFO] Gripper type: {gripper_type}\033[0m')
    print(f'\033[1;32m[INFO] Speed: {speed_deg_s} deg/s\033[0m')
    print(f'\033[1;32m[INFO] Planning pipeline: {planning_pipeline}\033[0m')
    print(f'\033[1;32m[INFO] Allowed start tolerance: {allowed_start_tolerance} rad\033[0m')
    print(f'\033[1;32m[INFO] World scene file: {world_scene_file if world_scene_file else "(none)"}\033[0m')
    print(f'\033[1;32m[INFO] Demonstration UI: {"enabled" if demonstration_ui else "disabled"}\033[0m')
    print(f'\033[1;32m[INFO] Hand-eye TF publish: {"enabled" if publish_hand_eye_calibration else "disabled"}\033[0m')
    if publish_hand_eye_calibration:
        print(f'\033[1;32m[INFO] Hand-eye calibration file: {hand_eye_calibration_file}\033[0m')
        print(f'\033[1;32m[INFO] Hand-eye optical frame: {hand_eye_camera_optical_frame}\033[0m')
        print(f'\033[1;32m[INFO] Hand-eye optical correction: {"enabled" if apply_hand_eye_optical_correction else "disabled"}\033[0m')
    print(f'\033[1;33m[INFO] Real robot mode: Hardware connection required\033[0m')
    
    # Get versioned MoveIt config with specified gripper type, port, and speed
    moveit_config = get_versioned_moveit_config(
        gripper_type,
        port,
        use_fake_hardware=False,
        speed_deg_s=speed_deg_s,
        default_planning_pipeline=planning_pipeline,
    )
    
    # Update robot description with hardware interface parameters
    robot_description = moveit_config.robot_description
    
    # Controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("alicia_d_moveit"),
                "config",
                "ros2_controllers.yaml"
            ]),
        ],
        output="screen",
    )
    
    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    
    # Spawner for Alicia_controller (arm)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["Alicia_controller", "-c", "/controller_manager"],
        output="screen",
    )
    
    # Spawner for Gripper_controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["Gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )
    
    # Generate move_group launch (with fake_execution=false for real robot)
    move_group_params = {
        "allow_trajectory_execution": True,
        "fake_execution": False,
        "capabilities": "",
        "disable_capabilities": "",
        "monitor_dynamics": False,
        "trajectory_execution": {
            "allowed_start_tolerance": allowed_start_tolerance,
        },
    }
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_params,
        ],
    )
    
    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("alicia_d_moveit"),
        "config",
        "moveit.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.pilz_cartesian_limits,
            moveit_config.joint_limits,
        ],
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    workspace_boundaries_node = Node(
        package="alicia_d_moveit",
        executable="publish_workspace_boundaries.py",
        name="workspace_boundaries",
        output="screen",
        parameters=[{
            "scene_file": world_scene_file,
        }],
    )

    demonstration_ui_node = Node(
        package="alicia_d_moveit",
        executable="demonstration_toggle_ui.py",
        name="demonstration_toggle_ui",
        output="screen",
    ) if demonstration_ui else None

    hand_eye_tf_node = Node(
        package="alicia_d_calibration",
        executable="publish_hand_eye_tf.py",
        name="hand_eye_tf_publisher",
        output="screen",
        parameters=[{
            "calibration_file": hand_eye_calibration_file,
            "camera_optical_frame": hand_eye_camera_optical_frame,
            "apply_optical_correction": apply_hand_eye_optical_correction,
        }],
    ) if publish_hand_eye_calibration else None
    
    # Delay arm and gripper controller spawners until joint_state_broadcaster is loaded
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )
    
    # Delay move_group until controllers are loaded
    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[move_group_node],
        )
    )
    
    nodes_to_start = [
        robot_state_publisher,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_gripper_controller_spawner,
        delay_move_group,
        rviz_node,
        workspace_boundaries_node,
    ]

    if demonstration_ui_node is not None:
        nodes_to_start.append(demonstration_ui_node)

    if hand_eye_tf_node is not None:
        nodes_to_start.append(hand_eye_tf_node)
    
    return nodes_to_start


def generate_launch_description():
    """Generate launch description for real robot control."""
    return LaunchDescription([

        DeclareLaunchArgument(
            'gripper_type',
            default_value='50mm',
            description='Gripper type: "50mm" or "100mm"'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='',
            description='Serial port for robot connection (e.g., /dev/ttyACM0). Leave empty for auto-detection.'
        ),
        DeclareLaunchArgument(
            'speed_deg_s',
            default_value='20',
            description='Default speed in degrees per second for joint movements.'
        ),
        DeclareLaunchArgument(
            'planning_pipeline',
            default_value='ompl',
            description='Default planning pipeline. Supported values: "ompl", "pilz_industrial_motion_planner".'
        ),
        DeclareLaunchArgument(
            'allowed_start_tolerance',
            default_value='0.03',
            description='Maximum allowed deviation in radians between planned start state and current robot state.'
        ),
        DeclareLaunchArgument(
            'world_scene_file',
            default_value=PathJoinSubstitution([
                FindPackageShare("alicia_d_moveit"),
                "config",
                "world_scene.yaml"
            ]),
            description='YAML file describing custom collision boxes to add to the planning scene.'
        ),
        DeclareLaunchArgument(
            'demonstration_ui',
            default_value='true',
            description='Launch a small toggle window for /demonstration hand-guiding mode.'
        ),
        DeclareLaunchArgument(
            'publish_hand_eye_calibration',
            default_value='true',
            description='Publish saved hand-eye calibration TF while running the real robot stack.'
        ),
        DeclareLaunchArgument(
            'hand_eye_calibration_file',
            default_value='hand_eye_calibration_result.yaml',
            description='Calibration YAML filename or absolute path. Resolved through the alicia_d_calibration package.'
        ),
        DeclareLaunchArgument(
            'hand_eye_camera_optical_frame',
            default_value='camera_color_optical_frame',
            description='Camera optical frame used by ArUco/PnP during calibration.'
        ),
        DeclareLaunchArgument(
            'apply_hand_eye_optical_correction',
            default_value='true',
            description='Convert the saved optical-frame calibration result to child_frame_id before publishing.'
        ),
        OpaqueFunction(function=launch_setup)
    ])
