#!/usr/bin/env python3
"""
Delivery Manager Launch File
Launches the delivery manager which controls robot navigation and human_follower lifecycle
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        # Delivery Manager Node
        # This node handles:
        # - Receiving delivery targets from server
        # - Navigating to delivery location
        # - Starting/stopping human_follower automatically
        # - Returning to base
        Node(
            package='human_follower',
            executable='delivery_manager',
            name='delivery_manager',
            output='screen',
            parameters=[
                {
                    'cmd_vel_topic': '/cmd_vel',
                    'scan_topic': '/scan',
                    'delivery_target_topic': '/target_location',
                    'delivery_server_url': 'http://localhost:8080',
                    'max_linear': 0.8,
                    'max_angular': 1.5,
                    'k_lin': 2.0,
                    'k_ang': 2.0,
                    'arrival_distance': 0.5,
                    'log_file': '~/delivery_logs/manager.log',
                }
            ]
        ),
    ])
