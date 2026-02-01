#!/usr/bin/env python3
"""Launch safety node for obstacle avoidance."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stop_distance', default_value='0.5'),
        DeclareLaunchArgument('slow_distance', default_value='1.0'),
        DeclareLaunchArgument('side_distance', default_value='0.35'),
        DeclareLaunchArgument('front_angle', default_value='60.0'),
        DeclareLaunchArgument('enable_avoidance', default_value='true'),

        Node(
            package='tb3_warehouse',
            executable='safety_node.py',
            name='safety_node',
            output='screen',
            parameters=[{
                'stop_distance': LaunchConfiguration('stop_distance'),
                'slow_distance': LaunchConfiguration('slow_distance'),
                'side_distance': LaunchConfiguration('side_distance'),
                'front_angle': LaunchConfiguration('front_angle'),
                'enable_avoidance': LaunchConfiguration('enable_avoidance'),
            }],
            remappings=[
                ('/cmd_vel_safe', '/cmd_vel_final'),
            ]
        ),
    ])
