"""Launch the PX4 offboard bridge.

Remaps the bridge's generic topic names onto the PX4 /fmu/{in,out} topics (as
exposed by the uXRCE-DDS agent) and the shared MPC / gimbal topics.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('px4_offboard_bridge')
    default_params = os.path.join(pkg_share, 'config', 'bridge_params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path to the bridge parameter YAML.'),
        Node(
            package='px4_offboard_bridge',
            executable='px4_offboard_bridge',
            name='px4_offboard_bridge',
            output='screen',
            parameters=[params_file],
            remappings=[
                # From the MPC node.
                ('mpc/control_command', '/mpc/control_command'),
                # PX4 uXRCE-DDS topics.
                ('fmu/in/offboard_control_mode', '/fmu/in/offboard_control_mode'),
                ('fmu/in/trajectory_setpoint', '/fmu/in/trajectory_setpoint'),
                ('fmu/in/vehicle_command', '/fmu/in/vehicle_command'),
                ('fmu/out/vehicle_odometry', '/fmu/out/vehicle_odometry'),
                ('fmu/out/vehicle_status', '/fmu/out/vehicle_status'),
                # To the MPC node / gz gimbal.
                ('odometry', '/px4/odometry'),
                ('gimbal_cmd', '/gimbal/angle_cmd'),
            ],
        ),
    ])
