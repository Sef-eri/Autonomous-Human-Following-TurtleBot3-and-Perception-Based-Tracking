#!/usr/bin/env python3
"""Launch behavior tree node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tick_rate', default_value='10.0'),
        DeclareLaunchArgument('target_lost_timeout', default_value='3.0'),
        DeclareLaunchArgument('search_angular_vel', default_value='0.5'),

        Node(
            package='tb3_warehouse',
            executable='behavior_tree.py',
            name='behavior_tree_node',
            output='screen',
            parameters=[{
                'tick_rate': LaunchConfiguration('tick_rate'),
                'target_lost_timeout': LaunchConfiguration('target_lost_timeout'),
                'search_angular_vel': LaunchConfiguration('search_angular_vel'),
            }]
        ),
    ])
