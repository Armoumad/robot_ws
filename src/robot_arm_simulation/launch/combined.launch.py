#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package path
    pkg_share = get_package_share_directory('robot_arm_simulation')
    
    # Paths to key files
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'arm_config.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    controllers_file = os.path.join(pkg_share, 'config', 'arm_controllers.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Process URDF with xacro
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch',
                         'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false'
        }.items()
    )
    
    # Spawn robot in Gazebo (with delay)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'robot_arm',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0'
                ],
                output='screen'
            )
        ]
    )
    
    # Spawn controllers (with delay)
    spawn_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    spawn_arm_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller'],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # RViz (with delay)
    rviz_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        rviz_node
    ])
