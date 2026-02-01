#!/usr/bin/env python3
"""Launch vision node for person detection and gesture recognition."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Camera image topic'
        ),
        DeclareLaunchArgument(
            'detection_confidence',
            default_value='0.5',
            description='MediaPipe detection confidence threshold'
        ),

        Node(
            package='tb3_warehouse',
            executable='vision_node.py',
            name='vision_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'detection_confidence': LaunchConfiguration('detection_confidence'),
                'publish_debug_image': True
            }]
        ),
    ])
