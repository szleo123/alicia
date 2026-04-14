from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    """Setup launch with parameters."""
    # Get launch configuration values
    port = LaunchConfiguration('port').perform(context)
    default_speed_deg_s = LaunchConfiguration('default_speed_deg_s').perform(context)
    debug_mode = LaunchConfiguration('debug_mode').perform(context).lower() == 'true'
    
    alicia_driver_node = Node(
        package='alicia_d_driver',
        executable='alicia_d_driver_node',
        name='alicia_d_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': port,
            'default_speed_deg_s': float(default_speed_deg_s) if default_speed_deg_s else 20.0,
            'debug_mode': debug_mode,
        }]
    )

    return [alicia_driver_node]

def generate_launch_description():
    """Launch the Alicia D driver node."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='',
        description='Serial port for robot connection (e.g., /dev/ACM0 or /dev/ttyCH341USB0). Leave empty for auto-detection.'
    )
    
    default_speed_deg_s_arg = DeclareLaunchArgument(
        'default_speed_deg_s',
        default_value='20.0',
        description='Default speed for joint movements in degrees per second (4.39-439.45). Default: 20.0 deg/s.'
    )

    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable verbose serial/debug logging for troubleshooting.'
    )

    return LaunchDescription([
        port_arg,
        default_speed_deg_s_arg,
        debug_mode_arg,
        OpaqueFunction(function=launch_setup)
    ])
