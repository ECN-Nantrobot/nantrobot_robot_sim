"""
@file start_sim.launch.py
@brief Simulation startup launch file for ROS2.

This launch file sets up a complete simulation environment including Gazebo,
RViz with custom button panel, and necessary bridges for communication.
It provides configurable world loading and button panel configuration.

Usage example:
    ros2 launch nantrobot_robot_sim start_sim.launch.py world:=/path/to/world.world button_config:=/path/to/buttons.yaml

Launch arguments:
    - world: World file to load in Gazebo (default: empty.world from package)
    - button_config: Button configuration YAML file for RViz panel (default: default_button.yaml)

Components launched:
    - Gazebo simulator with specified world
    - ROS-Gazebo bridges for clock and TF
    - RViz2 with custom button panel
    - Table mesh publisher for visualization

@note The button panel is from the nantrobot_rviz_panel package

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

    strategy_directory = os.path.join(get_package_share_directory(package_name), 'strategies')

    # Button panel config - default value
    default_button_config = os.path.join(
        get_package_share_directory('nantrobot_rviz_panel'),
        'config',
        'default_button.yaml'
    )

    # Button config launch argument
    button_config = LaunchConfiguration('button_config')
    
    button_config_arg = DeclareLaunchArgument(
        'button_config',
        default_value=default_button_config,
        description='Button configuration file path'
    )

    # World configuration
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # TF bridge
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    rviz_config = os.path.join(get_package_share_directory(
        package_name), 'config', 'two_robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True,
            'strategy_directory': strategy_directory,
            'button_panel.config_file': button_config
        }],
        output='screen',
        additional_env={'QT_X11_NO_MITSHM': '1',
                        'QT_QUICK_BACKEND': 'software'}
    )

    table_publisher = Node(
        package='nantrobot_robot_sim',
        executable='table_mesh_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        button_config_arg,
        gazebo,
        clock_bridge,
        tf_bridge,
        rviz,
        table_publisher
    ])