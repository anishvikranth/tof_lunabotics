"""MAVROS with only the plugins the ToF crater nodes need, and timesync off."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('crater_detector')
    mavros_share = get_package_share_directory('mavros')
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:115200'),
        Node(package='mavros', executable='mavros_node', output='screen',
             parameters=[
                 {'fcu_url': LaunchConfiguration('fcu_url'), 'gcs_url': '',
                  'tgt_system': 1, 'tgt_component': 1, 'fcu_protocol': 'v2.0'},
                 os.path.join(share, 'config', 'mavros_crater_pluginlists.yaml'),
                 os.path.join(mavros_share, 'launch', 'apm_config.yaml'),
                 os.path.join(share, 'config', 'mavros_crater_overrides.yaml'),
             ]),
    ])
