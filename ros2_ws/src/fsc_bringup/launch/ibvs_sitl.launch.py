"""Top-level bringup for the IBVS NMPC on PX4 SITL + gz Harmonic.

Brings up the ROS2 side of the stack and (optionally) the uXRCE-DDS agent and
the ros_gz_bridge:

  * MicroXRCEAgent      (optional; launch arg `dds_agent`)
  * ros_gz_bridge       (optional; launch arg `gz_bridge`) — gimbal + target GT
  * px4_offboard_bridge — ControlCommand <-> PX4 offboard, frames, arm SM
  * target_bearing_node — ground-truth bearing/range
  * mpc_controller_node — the NMPC

PX4 SITL and the gz server themselves are launched separately (they live in the
PX4-Autopilot tree); see fsc_bringup/README.md. Start this once PX4 SITL + gz
are already running.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mpc_share = get_package_share_directory('mpc_controller_node')
    bridge_share = get_package_share_directory('px4_offboard_bridge')
    perc_share = get_package_share_directory('fsc_perception')

    dds_agent = LaunchConfiguration('dds_agent')
    gz_bridge = LaunchConfiguration('gz_bridge')
    dds_port = LaunchConfiguration('dds_port')

    gz_bridge_config = os.path.join(perc_share, 'gz', 'ros_gz_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('dds_agent', default_value='true',
                              description='Start the MicroXRCEAgent (UDP).'),
        DeclareLaunchArgument('dds_port', default_value='8888',
                              description='MicroXRCEAgent UDP port.'),
        DeclareLaunchArgument('gz_bridge', default_value='true',
                              description='Start the ros_gz_bridge.'),

        # uXRCE-DDS agent (PX4 <-> ROS2). Requires MicroXRCEAgent on PATH.
        ExecuteProcess(
            condition=IfCondition(dds_agent),
            cmd=['MicroXRCEAgent', 'udp4', '-p', dds_port],
            output='screen',
        ),

        # gz <-> ROS bridge for the gimbal + ground-truth target.
        ExecuteProcess(
            condition=IfCondition(gz_bridge),
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '--ros-args', '-p', ['config_file:=', gz_bridge_config]],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bridge_share, 'launch',
                             'px4_offboard_bridge.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perc_share, 'launch',
                             'target_bearing.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mpc_share, 'launch',
                             'mpc_controller.launch.py'))),
    ])
