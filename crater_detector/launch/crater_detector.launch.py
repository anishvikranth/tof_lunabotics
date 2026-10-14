import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('crater_detector'),
                          'config', 'crater_detector.yaml')
    return LaunchDescription([
        Node(package='crater_detector', executable='crater_detector_node',
             name='crater_detector', parameters=[params], output='screen'),
    ])
