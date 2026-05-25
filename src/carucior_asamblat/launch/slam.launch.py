import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("carucior_asamblat")
    default_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("slam_params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use simulation (Gazebo) clock"
        ),
        DeclareLaunchArgument(
            "slam_params_file", default_value=default_params,
            description="slam_toolbox parameter file"
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
