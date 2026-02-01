#!/usr/bin/env python3
"""Launch control node for person following."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # PID parameters
        DeclareLaunchArgument('angular_kp', default_value='1.2'),
        DeclareLaunchArgument('angular_ki', default_value='0.01'),
        DeclareLaunchArgument('angular_kd', default_value='0.3'),
        DeclareLaunchArgument('linear_kp', default_value='0.5'),
        DeclareLaunchArgument('linear_ki', default_value='0.0'),
        DeclareLaunchArgument('linear_kd', default_value='0.1'),

        # Velocity limits
        DeclareLaunchArgument('max_linear_vel', default_value='0.22'),
        DeclareLaunchArgument('max_angular_vel', default_value='1.5'),

        # Following behavior
        DeclareLaunchArgument('target_distance', default_value='0.4'),

        Node(
            package='tb3_warehouse',
            executable='control_node.py',
            name='control_node',
            output='screen',
            parameters=[{
                'angular_kp': LaunchConfiguration('angular_kp'),
                'angular_ki': LaunchConfiguration('angular_ki'),
                'angular_kd': LaunchConfiguration('angular_kd'),
                'linear_kp': LaunchConfiguration('linear_kp'),
                'linear_ki': LaunchConfiguration('linear_ki'),
                'linear_kd': LaunchConfiguration('linear_kd'),
                'max_linear_vel': LaunchConfiguration('max_linear_vel'),
                'max_angular_vel': LaunchConfiguration('max_angular_vel'),
                'target_distance': LaunchConfiguration('target_distance'),
            }]
        ),
    ])
