#!/usr/bin/env python3
"""
Full warehouse simulation launch with all nodes integrated.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    tb3_warehouse_dir = get_package_share_directory('tb3_warehouse')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Paths
    world_file = os.path.join(tb3_warehouse_dir, 'worlds', 'warehouse_static_human.world')
    robot_sdf = os.path.join(tb3_gazebo_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    robot_urdf = os.path.join(tb3_gazebo_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')

    # Read URDF content
    with open(robot_urdf, 'r') as f:
        robot_description_content = f.read()

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-4.0')
    y_pose = LaunchConfiguration('y_pose', default='-7.0')

    # =========================================================================
    # GAZEBO SIMULATION
    # =========================================================================

    # Gazebo server with world
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )

    # Spawn robot after Gazebo is ready (5 second delay)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                arguments=[
                    '-entity', 'turtlebot3_waffle_pi',
                    '-file', robot_sdf,
                    '-x', '-4.0',
                    '-y', '-7.0',
                    '-z', '0.01',
                    '-timeout', '30'
                ],
                output='screen'
            )
        ]
    )

    # =========================================================================
    # APPLICATION NODES (8 second delay for robot spawn)
    # =========================================================================

    app_nodes = TimerAction(
        period=10.0,
        actions=[
            # Vision node
            Node(
                package='tb3_warehouse',
                executable='vision_node.py',
                name='vision_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'image_topic': '/camera/image_raw',
                    'detection_confidence': 0.5,
                    'publish_debug_image': True
                }],
                remappings=[('/gesture_cmd', '/gesture_cmd_raw')]
            ),

            # Behavior tree node
            Node(
                package='tb3_warehouse',
                executable='behavior_tree.py',
                name='behavior_tree_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'tick_rate': 10.0,
                    'target_lost_timeout': 3.0,
                    'search_angular_vel': 0.5,
                }]
            ),

            # Control node
            Node(
                package='tb3_warehouse',
                executable='control_node.py',
                name='control_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'angular_kp': 1.2,
                    'angular_ki': 0.01,
                    'angular_kd': 0.3,
                    'linear_kp': 0.5,
                    'linear_ki': 0.0,
                    'linear_kd': 0.1,
                    'max_linear_vel': 0.22,
                    'max_angular_vel': 1.5,
                    'target_distance': 0.4,
                }],
                remappings=[('/cmd_vel', '/cmd_vel_raw')]
            ),

            # Safety node
            Node(
                package='tb3_warehouse',
                executable='safety_node.py',
                name='safety_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'stop_distance': 0.5,
                    'slow_distance': 1.0,
                    'side_distance': 0.35,
                    'front_angle': 60.0,
                    'enable_avoidance': True,
                }],
                remappings=[
                    ('/cmd_vel', '/cmd_vel_raw'),
                    ('/cmd_vel_safe', '/cmd_vel'),
                ]
            ),
        ]
    )

    return LaunchDescription([
        # Environment variable
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-4.0'),
        DeclareLaunchArgument('y_pose', default_value='-7.0'),

        # Launch sequence
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        app_nodes,
    ])
