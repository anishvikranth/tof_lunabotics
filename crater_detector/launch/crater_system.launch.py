"""Starts crater_detector, crater_mapper and the base_link -> tof static transform."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf(context):
    a = {k: LaunchConfiguration(k).perform(context)
         for k in ('x', 'y', 'z', 'pitch_deg', 'base_frame', 'sensor_frame')}
    return [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tof_static_tf', output='screen',
        condition=IfCondition(LaunchConfiguration('publish_static_tf')),
        arguments=['--x', a['x'], '--y', a['y'], '--z', a['z'],
                   '--roll', '0', '--pitch', str(math.radians(float(a['pitch_deg']))), '--yaw', '0',
                   '--frame-id', a['base_frame'], '--child-frame-id', a['sensor_frame']])]


def generate_launch_description():
    share = get_package_share_directory('crater_detector')
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.30', description='sensor x from base_link (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='sensor y from base_link (m)'),
        DeclareLaunchArgument('z', default_value='0.45', description='sensor z from base_link (m)'),
        DeclareLaunchArgument('pitch_deg', default_value='20.0',
                              description='downward tilt (positive = down)'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('sensor_frame', default_value='tof'),
        DeclareLaunchArgument('map_topic', default_value='/map',
                              description='occupancy grid whose size/origin the crater map follows '
                                          '(rover: /rtabmap/grid_prob_map)'),
        DeclareLaunchArgument('publish_static_tf', default_value='true',
                              description='false if your URDF already has base_link -> tof'),
        OpaqueFunction(function=static_tf),
        Node(package='crater_detector', executable='crater_detector_node',
             name='crater_detector', output='screen',
             parameters=[os.path.join(share, 'config', 'crater_detector.yaml')]),
        Node(package='crater_detector', executable='crater_mapper_node',
             name='crater_mapper', output='screen',
             parameters=[os.path.join(share, 'config', 'crater_mapper.yaml'),
                         {'sensor_frame': LaunchConfiguration('sensor_frame'),
                          'map_topic': LaunchConfiguration('map_topic')}]),
    ])
