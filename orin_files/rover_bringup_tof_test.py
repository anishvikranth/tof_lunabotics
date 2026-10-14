import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to cac_rover.urdf in cac_rover_description package
    rover_desc_pkg = get_package_share_directory('cac_rover_description')
    urdf_path = os.path.join(rover_desc_pkg, 'urdf', 'cac_rover.urdf')
    ekf_config_path = os.path.join(rover_desc_pkg, 'config', 'ekf.yaml')

    # Read the URDF contents into a string parameter
    with open(urdf_path, 'r') as file:
        robot_description_content = file.read()

    # 1. Include zed_aruco_localization launch file
    zed_aruco_pkg = get_package_share_directory('zed_aruco_localization')
    zed_aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_aruco_pkg, 'launch', 'zed_aruco_loc.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'rviz': 'false',
            'publish_tf': 'false',  
            'publish_map_tf': 'false'
        }.items()
    )

    # 2. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    zed_tf_node = Node(
        package='cac_rover_description',
        executable='zed_tf_node',
        name='zed_tf_node'
    )

    # 3. ToF crater detection + crater map (crater_detector package, crater_ws).
    #    The tof frame comes from the URDF (zed_camera_link -> tof), so no static TF here.
    crater_pkg = get_package_share_directory('crater_detector')
    crater_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(crater_pkg, 'launch', 'crater_system.launch.py')
        ),
        launch_arguments={
            'publish_static_tf': 'false',
            'sensor_frame': 'tof',
            'map_topic': '/rtabmap/grid_prob_map',
        }.items()
    )

    # 4. Optional MAVROS (ToF data comes from the Pixhawk). Off by default:
    #    enable with start_mavros:=true only if MAVROS is not started elsewhere.
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
        ),
        launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items(),
        condition=IfCondition(LaunchConfiguration('start_mavros'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_mavros', default_value='false'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:115200'),
        zed_aruco_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        ekf_node,
        zed_tf_node,
        mavros_launch,
        crater_launch
    ])
