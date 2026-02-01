#!/usr/bin/env python3
"""Simplified launch - uses turtlebot3_gazebo with custom world."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    tb3_warehouse_dir = get_package_share_directory('tb3_warehouse')
    world_file = os.path.join(tb3_warehouse_dir, 'worlds', 'warehouse.world')

    return LaunchDescription([
        # Set TB3 model
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Gazebo world file'
        ),

        # Launch Gazebo with warehouse world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_file
            ],
            output='screen'
        ),

        # Spawn TB3 robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_waffle_pi',
                '-topic', 'robot_description',
                '-x', '-4.0',
                '-y', '-7.0',
                '-z', '0.01'
            ],
            output='screen'
        ),
    ])
