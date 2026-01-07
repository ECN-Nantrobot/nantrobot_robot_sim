"""
@file two_robot_sim.launch.py
@brief Complete two-robot simulation launch file for ROS2.

This launch file sets up a complete simulation environment with Gazebo, RViz,
and spawns two robots (MainRobot and Opponent) with all necessary components.
It provides a generic dual-robot simulation without project-specific functionality.

Usage example:
    ros2 launch nantrobot_robot_sim two_robot_sim.launch.py

Launch arguments:
    - button_config: Button configuration YAML file for RViz panel (optional)

Components launched:
    - Complete simulation environment (Gazebo + RViz + bridges)
    - Two robots: MainRobot and Opponent
    - Static TF transforms for both robots
    - All necessary bridges and communication nodes

@note This is a generic version without project-specific strategy configurations

@version 1.0
@date 04/01/2026

@author Alexis MORICE
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'nantrobot_robot_sim'
    package_share = get_package_share_directory(package_name)

    # Launch arguments
    button_config = LaunchConfiguration('button_config')
    
    # Default button config
    default_button_config = os.path.join(
        get_package_share_directory('nantrobot_rviz_panel'),
        'config',
        'default_button.yaml'
    )
    
    button_config_arg = DeclareLaunchArgument(
        'button_config',
        default_value=default_button_config,
        description='Button configuration YAML file for RViz panel'
    )

    # Start simulation (Gazebo + RViz)
    start_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_share, 'launch', 'start_sim.launch.py')
        ]),
        launch_arguments={
            'button_config': button_config
        }.items()
    )

    # Static transform: world -> MainRobot/odom
    static_tf_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_robot1',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'MainRobot/odom']
    )

    # Static transform: world -> Opponent/odom
    static_tf_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_robot2',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'Opponent/odom']
    )

    # Spawn first robot (MainRobot)
    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_share, 'launch', 'spawn_robot.launch.py')
        ]),
        launch_arguments={
            'robot_namespace': 'MainRobot',
            'teleport_x': '0.3',
            'teleport_y': '1.8'
        }.items()
    )

    # Spawn second robot (Opponent)
    spawn_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_share, 'launch', 'spawn_robot.launch.py')
        ]),
        launch_arguments={
            'robot_namespace': 'Opponent',
            'teleport_x': '2.7',
            'teleport_y': '1.8'
        }.items()
    )

    return LaunchDescription([
        button_config_arg,
        start_sim,
        static_tf_robot1,
        static_tf_robot2,
        spawn_robot1,
        spawn_robot2,
    ])