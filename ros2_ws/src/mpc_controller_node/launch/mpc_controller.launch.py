"""Launch the MPC controller node with its parameter file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mpc_controller_node')
    default_params = os.path.join(pkg_share, 'config', 'mpc_params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path to the MPC parameter YAML.'),
        Node(
            package='mpc_controller_node',
            executable='mpc_controller_node',
            name='mpc_controller_node',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('odometry', '/px4/odometry'),
                ('target_bearing', '/target/bearing'),
                ('target_point', '/target/point'),
                ('mpc/control_command', '/mpc/control_command'),
                ('gimbal_feedback', '/gimbal/angle_state'),
            ],
        ),
    ])
