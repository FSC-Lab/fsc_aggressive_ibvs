"""Launch the ground-truth target-bearing node.

Wires drone odometry (from px4_offboard_bridge), the target ground-truth pose
and gimbal state (bridged from gz), and publishes TargetBearing + target point
for mpc_controller_node.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('fsc_perception')
    default_params = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path to the perception parameter YAML.'),
        Node(
            package='fsc_perception',
            executable='target_bearing_node',
            name='target_bearing_node',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('odometry', '/px4/odometry'),
                ('target_pose', '/target/pose_gt'),
                ('gimbal_state', '/gimbal/angle_state'),
                ('target_bearing', '/target/bearing'),
                ('target_point', '/target/point'),
            ],
        ),
    ])
