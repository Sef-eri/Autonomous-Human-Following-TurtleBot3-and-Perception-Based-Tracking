#!/usr/bin/env python3
"""
Color-Based Following Launch File

Launches the complete color tracking and following system:
- Gazebo with warehouse and green target
- TurtleBot3 Waffle Pi
- Color Tracker Node (HSV-based detection)
- Follower Node (proximity-based following)
- Safety Node (obstacle avoidance)

Usage:
  ros2 launch tb3_warehouse color_follow.launch.py

To control the target (move it around):
  ros2 topic pub /target/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    tb3_warehouse_dir = get_package_share_directory('tb3_warehouse')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Paths
    world_file = os.path.join(tb3_warehouse_dir, 'worlds', 'warehouse_color_target.world')
    robot_sdf = os.path.join(tb3_gazebo_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    robot_urdf = os.path.join(tb3_gazebo_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')

    # Read URDF content
    with open(robot_urdf, 'r') as f:
        robot_description_content = f.read()

    # =========================================================================
    # GAZEBO SIMULATION
    # =========================================================================

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

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

    # Spawn robot after Gazebo is ready
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
    # COLOR TRACKING SYSTEM (starts after robot spawns)
    # =========================================================================

    tracking_nodes = TimerAction(
        period=10.0,
        actions=[
            # Color Tracker Node
            Node(
                package='tb3_warehouse',
                executable='color_tracker_node.py',
                name='color_tracker_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'image_topic': '/camera/image_raw',
                    # HSV range for neon green
                    'hue_min': 35,
                    'hue_max': 85,
                    'sat_min': 100,
                    'sat_max': 255,
                    'val_min': 100,
                    'val_max': 255,
                    # Detection parameters
                    'min_blob_area': 500,
                    'max_blob_area': 100000,
                    # Proximity thresholds (adjust based on testing)
                    'close_area': 15000,
                    'far_area': 1000,
                    'publish_debug': True,
                }]
            ),

            # Follower Node (uses proximity-based control)
            Node(
                package='tb3_warehouse',
                executable='follower_node.py',
                name='follower_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    # Velocity limits
                    'max_linear_vel': 0.20,
                    'max_angular_vel': 1.2,
                    'min_linear_vel': 0.05,
                    # PID gains for angular control
                    'angular_kp': 1.5,
                    'angular_ki': 0.01,
                    'angular_kd': 0.2,
                    # Following behavior
                    'stop_distance': 0.2,    # Stop when distance < 0.2
                    'slow_distance': 0.5,    # Slow down when distance < 0.5
                    'deadzone_x': 0.05,
                    'lost_target_timeout': 3.0,
                    'search_angular_vel': 0.4,
                    'auto_start': True,      # Start following immediately
                }],
                remappings=[
                    ('/cmd_vel', '/cmd_vel_raw'),  # Output to safety node
                ]
            ),

            # Safety Node (obstacle avoidance)
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
                    ('/cmd_vel', '/cmd_vel_raw'),      # Input from follower
                    ('/cmd_vel_safe', '/cmd_vel'),     # Output to robot
                ]
            ),
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        DeclareLaunchArgument('use_sim_time', default_value='true'),

        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        tracking_nodes,
    ])
