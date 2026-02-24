from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("carucior_asamblat")
    default_model = PathJoinSubstitution(
        [pkg_share, "urdf", "carucior_asamblat.urdf"]
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Absolute path to the URDF file",
    )

    robot_description = ParameterValue(
        Command(["cat ", LaunchConfiguration("model")]), value_type=str
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    map_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_base_link",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "base_link",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            model_arg,
            joint_state_publisher_gui,
            robot_state_publisher,
            map_to_base,
            rviz2,
        ]
    )
