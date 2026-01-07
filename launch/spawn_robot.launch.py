"""
@file spawn_robot.launch.py
@brief Robot spawning launch file for ROS2 simulation.

This launch file spawns a robot in Gazebo simulation with essential components
including robot state publisher, twist mux, bridges.
It provides a generic robot spawning solution without project-specific functionality.

Usage example:
    ros2 launch nantrobot_robot_sim spawn_robot.launch.py robot_namespace:=robot1

Launch arguments:
    - robot_namespace: Namespace for the robot (default: robot1)

Components launched:
    - Robot State Publisher with URDF/xacro processing
    - Twist multiplexer for velocity command handling
    - Twist stamper for velocity command timestamping
    - Gazebo entity spawner
    - ROS-Gazebo bridges for sensors and control
    - Image bridge for camera data
    - Odometry TF broadcaster
    - Teleport node for robot positioning

@version 1.0
@date 04/01/2026

@author Alexis MORICE
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'nantrobot_robot_sim'

    # Launch arguments
    robot_namespace = LaunchConfiguration('robot_namespace')
    teleport_x = LaunchConfiguration('teleport_x')
    teleport_y = LaunchConfiguration('teleport_y')

    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Namespace for the robot'
    )

    teleport_x_arg = DeclareLaunchArgument(
        'teleport_x',
        default_value='0.3',
        description='X position for robot teleport'
    )

    teleport_y_arg = DeclareLaunchArgument(
        'teleport_y',
        default_value='1.8',
        description='Y position for robot teleport'
    )

    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 
            'launch', 
            'rsp.launch.py'
        )]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'false',
            'use_namespace': robot_namespace
        }.items()
    )

    # Twist mux
    twist_mux_params = os.path.join(get_package_share_directory(
        package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'diff_cont/cmd_vel_unstamped')],
        namespace=robot_namespace
    )

    # Twist stamper
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('cmd_vel_in', 'diff_cont/cmd_vel_unstamped'),
            ('cmd_vel_out', 'diff_cont/cmd_vel')
        ],
        namespace=robot_namespace
    )

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_namespace,
            '-z', '0.1'
        ],
        output='screen',
        namespace=robot_namespace
    )

    # Gazebo bridge
    bridge_params = os.path.join(get_package_share_directory(
        package_name), 'config', 'gz_bridge.yaml')
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        parameters=[{'expand_gz_topic_names': True}],
        namespace=robot_namespace,
    )

    # Image bridge
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[f'{robot_namespace}/camera/image_raw'],
        output='screen'
    )

    # Odom TF broadcaster
    odom_tf_broadcaster = Node(
        package='nantrobot_robot_sim',
        executable='odom_tf_broadcaster',
        parameters=[{'use_sim_time': True}],
        output='screen',
        namespace=robot_namespace
    )

    # Teleport node
    teleport_node = Node(
        package='nantrobot_robot_sim',
        executable='teleport_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        namespace=robot_namespace
    )

    # Initial teleport command
    teleport_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            [robot_namespace, '/set_robot_position'],
            'geometry_msgs/msg/PoseStamped',
            ['{header: {frame_id: "odom"}, pose: {position: {x: ',
             teleport_x,
             ', y: ',
             teleport_y, 
             ', z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}']
        ],
        shell=False
    )

    # Delay the teleport to ensure everything is ready
    delayed_teleport = TimerAction(
        period=6.0,
        actions=[teleport_command]
    )

    return LaunchDescription([
        robot_namespace_arg,
        teleport_x_arg,
        teleport_y_arg,
        rsp,
        twist_stamper,
        twist_mux,
        spawn_entity,
        gz_bridge,
        ros_gz_image_bridge,
        odom_tf_broadcaster,
        teleport_node,
        delayed_teleport,
    ])