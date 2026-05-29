"""
first_scan.launch.py

Stack-ul complet nav2 + pre-rotate + explore_lite.

Ordine de lansare:
  1. nav2_bringup/navigation_launch.py — toata stiva (controller, planner,
     bt_navigator, behavior, smoother, velocity_smoother, collision_monitor,
     route_server, opennav_docking, waypoint_follower, lifecycle_manager).
  2. dupa 8s: pre_explore_rotate — roteste 360° ca slam_toolbox sa
     adauge noduri in graf si harta sa se extinda in jurul robotului.
  3. dupa exit pre_rotate (~14s mai tarziu): explore_lite — frontier
     exploration normala.

NOTA: save_and_exit watcher-ul porneste din bringup.launch.py, NU de aici.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("first_scan")
    nav2_params    = os.path.join(pkg_share, "config", "nav2_params.yaml")
    explore_params = os.path.join(pkg_share, "config", "explore_params.yaml")

    use_sim_time    = LaunchConfiguration("use_sim_time")
    explore_arg     = LaunchConfiguration("explore")
    pre_rotate_arg  = LaunchConfiguration("pre_rotate")

    declared = [
        DeclareLaunchArgument("use_sim_time",  default_value="true"),
        DeclareLaunchArgument("map_save_path", default_value=""),
        DeclareLaunchArgument("explore",       default_value="true",
            description="Daca true, porneste explore_lite. False = doar nav2 stack (pentru test manual cu RViz 2D Goal Pose)."),
        DeclareLaunchArgument("pre_rotate",    default_value="false",
            description="Daca true, robotul face 360° initial inainte de explore. LIDAR are deja FOV 360° deci nu adauga info noua, doar introduce wobble in SLAM."),
    ]

    # ── nav2 stack COMPLET (oficial) ──────────────────────────────────────
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file":  nav2_params,
            "autostart":    "true",
            "use_composition": "False",  # individual nodes, cu remap cmd_vel→cmd_vel_nav
        }.items(),
    )

    # ── Pre-rotate 360° (extinde harta SLAM inainte de explore_lite) ──────
    pre_rotate_node = Node(
        package="first_scan",
        executable="pre_explore_rotate",
        name="pre_explore_rotate",
        output="screen",
        parameters=[{
            "use_sim_time":    use_sim_time,
            "cmd_topic":       "/cmd_vel_nav",
            # 0.25 rad/s (anterior 0.5) → SLAM are timp dublu intre scanuri
            # sa faca scan-matching peste wobble-ul cart-ului (base_footprint
            # face cerc de 0.42m raza in jurul centrului fizic de rotatie).
            # Rezultat: harta mai stabila, fara "jumps".
            "angular_speed":   0.25,
            "target_radians":  7.0,
            "startup_delay":   2.0,
        }],
    )

    pre_rotate = TimerAction(
        period=10.0,    # asteapta nav2 sa fie complet active (~8s) + marja
        actions=[
            LogInfo(msg="[first_scan] Pornesc pre_explore_rotate (360°)"),
            pre_rotate_node,
        ],
        condition=IfCondition(pre_rotate_arg),
    )

    # ── explore_lite — doua instante (aceeasi configurare, dar fiecare
    # va fi triggered de un mecanism diferit). Acelasi Node obj nu poate
    # fi adaugat de doua ori intr-un launch description.
    explore_node_after = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[explore_params, {"use_sim_time": use_sim_time}],
    )
    explore_node_direct = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[explore_params, {"use_sim_time": use_sim_time}],
    )

    # Cand pre_rotate e ON: explore_lite porneste dupa ce pre_rotate exits
    explore_after_pre_rotate = RegisterEventHandler(
        OnProcessExit(
            target_action=pre_rotate_node,
            on_exit=[
                LogInfo(msg="[first_scan] pre_rotate done → pornesc explore_lite"),
                explore_node_after,
            ],
        ),
        condition=IfCondition(explore_arg),
    )

    # Cand pre_rotate e OFF si explore e ON: explore_lite porneste dupa un
    # delay fix (dupa ce nav2 stack-ul e complet activ).
    explore_direct = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="[first_scan] pre_rotate=false → pornesc explore_lite direct"),
            explore_node_direct,
        ],
        condition=IfCondition(PythonExpression([
            "'", explore_arg, "' == 'true' and '", pre_rotate_arg, "' == 'false'"
        ])),
    )

    return LaunchDescription(declared + [
        nav2_navigation,
        pre_rotate,
        explore_after_pre_rotate,
        explore_direct,
    ])
