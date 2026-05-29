"""
mapping.launch.py — launch dedicat DOAR pentru construirea hartii.

Porneste:
  - Gazebo + URDF + bridge + cmd_vel_inverter
  - scan_qos_relay (filtru self + QoS conversie)
  - slam_toolbox in mod mapping (fresh, fara incarcare harta veche)
  - nav2 stack complet (controller, planner, bt_navigator, etc.)
  - pre_explore_rotate (rotatie 360 initiala)
  - explore_lite (explorare automata)
  - save_and_exit watcher (salveaza harta cand robotul a stat 45s)
  - RViz

NU porneste: human_follower, YOLO, dr_spaam.

Cand este harta gata (watcher-ul iese):
  - Harta este salvata la ~/maps/carucior_market.posegraph
  - Launch-ul continua dar nimic nu mai face nimic. Poti opri cu Ctrl+C.
  - Pentru a folosi harta cu human_follower: opreste acest launch si porneste
    bringup-ul normal (va detecta harta existenta si va sari direct la urmarire).

Sterge harta veche inainte: rm -rf ~/maps/carucior_market*
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


MAP_DIR  = os.path.expanduser("~/maps")
MAP_NAME = "carucior_market"
MAP_PATH = os.path.join(MAP_DIR, MAP_NAME)


def generate_launch_description():
    car_pkg = get_package_share_directory("carucior_asamblat")

    rviz_config = os.path.join(car_pkg, "config", "carucior.rviz")
    slam_params = os.path.join(car_pkg, "config", "slam_toolbox_params.yaml")

    use_sim_time    = LaunchConfiguration("use_sim_time")
    explore_arg     = LaunchConfiguration("explore")
    pre_rotate_arg  = LaunchConfiguration("pre_rotate")

    declared = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("explore",      default_value="true",
            description="False = nav2 stack rulează DAR explore_lite NU. Util pentru testarea cu RViz 2D Goal Pose."),
        DeclareLaunchArgument("pre_rotate",   default_value="false",
            description="Default OFF. LIDAR-ul are FOV 360° deci pre_rotate doar introduce wobble in SLAM fara info noua."),
    ]

    os.makedirs(MAP_DIR, exist_ok=True)

    # ── Gazebo + URDF + bridge ────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("carucior_asamblat"), "launch", "gazebo.launch.py"])
        ),
    )

    # ── cmd_vel_inverter (passthrough, fara inversari) ────────────────────
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

    # ── scan_qos_relay (filtru self + QoS) ────────────────────────────────
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

    # ── slam_toolbox (mapping fresh, no map_file_name) ────────────────────
    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_params, {
            "use_lifecycle_manager": False,
            "use_sim_time": use_sim_time,
            "mode": "mapping",
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
                LogInfo(msg="[mapping] slam configured → activating"),
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

    # ── first_scan stack (nav2 + pre_rotate + explore_lite) ───────────────
    first_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("first_scan"), "launch", "first_scan.launch.py"])
        ),
        launch_arguments={
            "use_sim_time":  use_sim_time,
            "map_save_path": MAP_PATH,
            "explore":       explore_arg,
            "pre_rotate":    pre_rotate_arg,
        }.items(),
    )
    first_scan_delayed = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="[mapping] Pornesc nav2 + explore_lite"),
            first_scan_launch,
        ],
    )

    # ── save_and_exit watcher (salveaza harta cand robotul s-a oprit) ─────
    sentinel_node = Node(
        package="first_scan",
        executable="save_and_exit",
        name="first_scan_sentinel",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_save_path": MAP_PATH,
            # PRIMARA: salvam cand explore_lite n-a mai gasit frontiere
            # active timp de 25s consecutiv. Asta = harta cu adevarat completa.
            "no_frontiers_seconds": 25.0,
            # Minim explore pana putem declara done (sa nu salvam in primele
            # secunde cand explore_lite inca nu si-a publicat primele frontiere)
            "min_explore_seconds":  60.0,
            # FALLBACK: robotul blocat fizic indelungat — salvam totusi
            "done_idle_seconds":    180.0,
            # Safety net global
            "max_explore_seconds":  600.0,
        }],
    )
    sentinel_delayed = TimerAction(
        period=14.0,
        actions=[sentinel_node],
    )

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz = TimerAction(
        period=4.0,
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
        first_scan_delayed,
        sentinel_delayed,
        LogInfo(msg=f"[mapping] Harta va fi salvata la: {MAP_PATH}.posegraph"),
        LogInfo(msg="[mapping] Cand watcher-ul detecteaza ca robotul s-a oprit 45s → salveaza si iese"),
    ])
