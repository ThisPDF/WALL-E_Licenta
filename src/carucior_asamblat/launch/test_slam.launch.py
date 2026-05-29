"""
test_slam.launch.py — launch izolat pentru a testa SLAM + fizica robotului
fara nav2 / explore_lite / human_follower in cale.

Porneste:
  - Gazebo + URDF + bridge + cmd_vel_inverter (via gazebo.launch.py)
  - scan_qos_relay (filtru self + QoS conversie)
  - slam_toolbox in mod mapping
  - RViz

Tu controlezi robotul manual cu teleop:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_gz

Te uiti la harta /map in RViz pe masura ce conduci. Daca harta arata
curat → SLAM/fizica OK, problema e in nav2/explore. Daca arata explodata
sau drifteaza → tot mai sunt issues in SLAM/filtru/fizica.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler, TimerAction
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    car_pkg = get_package_share_directory("carucior_asamblat")

    rviz_config = os.path.join(car_pkg, "config", "carucior.rviz")
    slam_params = os.path.join(car_pkg, "config", "slam_toolbox_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declared = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    # Gazebo + URDF + bridge + cmd_vel_inverter
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("carucior_asamblat"), "launch", "gazebo.launch.py"])
        ),
    )

    # cmd_vel_inverter (acelasi ca in bringup, dar cu ambele inversiuni False)
    cmd_vel_inverter = Node(
        package="human_follower",
        executable="cmd_vel_inverter",
        name="cmd_vel_inverter",
        output="screen",
        parameters=[{
            "use_sim_time":     use_sim_time,
            "input_topic":      "/cmd_vel",
            "output_topic":     "/cmd_vel_gz",
            "invert_linear_x":  False,
            "invert_angular_z": False,
        }],
    )

    # scan_qos_relay (filtru self + QoS conversie)
    scan_relay = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="human_follower",
                executable="scan_qos_relay",
                name="scan_qos_relay",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ],
    )

    # slam_toolbox (mapping fresh)
    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_params, {
            "use_lifecycle_manager": False,
            "use_sim_time": use_sim_time,
        }],
    )
    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[test_slam] slam configured → activating"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )
    slam = TimerAction(
        period=5.0,
        actions=[slam_node, slam_configure, slam_activate],
    )

    # RViz
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ],
    )

    return LaunchDescription(declared + [
        gazebo,
        cmd_vel_inverter,
        scan_relay,
        slam,
        rviz,
        LogInfo(msg="[test_slam] Toate componentele pornite. Controleaza robotul cu:"),
        LogInfo(msg="  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_gz"),
    ])
