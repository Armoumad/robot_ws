#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('robot_arm_simulation')

    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Process URDF
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # RSP
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot_arm'],
        output='screen'
    )

    # Spawners (déclenchés après l'insertion du robot)
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen'
    )

    spawner_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen'
    )

    spawner_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen'
    )

    # Ordonner: après spawn_robot -> JSB -> arm -> gripper
    after_spawn_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawner_jsb]
        )
    )
    after_jsb_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_jsb,
            on_exit=[spawner_arm]
        )
    )
    after_arm_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_arm,
            on_exit=[spawner_gripper]
        )
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        after_spawn_jsb,
        after_jsb_arm,
        after_arm_gripper,
    ])