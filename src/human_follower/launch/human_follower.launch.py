from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('human_follower')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'params.yaml'),
            description='YAML parameters file'
        ),

        Node(
            package='human_follower',
            executable='dr_spaam_detector',
            name='dr_spaam_detector',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='human_follower',
            executable='yolo_person_tracker',
            name='yolo_person_tracker',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='human_follower',
            executable='human_follower',
            name='human_follower',
            output='screen',
            parameters=[params_file],
        ),
    ])
