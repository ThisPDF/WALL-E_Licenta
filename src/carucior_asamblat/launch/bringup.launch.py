"""
Bringup smart:

1. Daca harta SLAM exista (~/maps/carucior_market.posegraph):
   → o incarca in slam_toolbox (mode mapping cu map_file_name)
   → urmarirea (dr_spaam + yolo + human_follower) porneste IMEDIAT

2. Daca harta NU exista:
   → slam_toolbox in mapping mode fresh
   → map_explorer porneste si conduce robotul prin magazin
   → la final salveaza harta si iese
   → bringup detecteaza iesirea explorer-ului si lanseaza urmarirea

Plus:
- nav2_costmap_2d standalone: global costmap (peste harta) + local costmap (rolling)
- nav2_lifecycle_manager pentru a activa costmaps

Stergere harta pentru re-mapare: rm -rf ~/maps/carucior_market*
Force remapping:                  pass `force_remap:=true`
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler, TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
MAP_FILE = MAP_PATH + ".posegraph"


def generate_launch_description():
    car_pkg = get_package_share_directory("carucior_asamblat")
    hf_pkg  = get_package_share_directory("human_follower")

    rviz_config         = os.path.join(car_pkg, "config", "carucior.rviz")
    slam_params         = os.path.join(car_pkg, "config", "slam_toolbox_params.yaml")
    hf_params           = os.path.join(hf_pkg,  "config", "params.yaml")
    global_costmap_yaml = os.path.join(car_pkg, "config", "global_costmap.yaml")
    local_costmap_yaml  = os.path.join(car_pkg, "config", "local_costmap.yaml")

    use_sim_time  = LaunchConfiguration("use_sim_time")
    rviz_arg      = LaunchConfiguration("rviz")
    slam_arg      = LaunchConfiguration("slam")
    follower_arg  = LaunchConfiguration("follower")
    force_remap   = LaunchConfiguration("force_remap")

    declared = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz",         default_value="true"),
        DeclareLaunchArgument("slam",         default_value="true"),
        DeclareLaunchArgument("follower",     default_value="true"),
        DeclareLaunchArgument("force_remap",  default_value="false",
            description="Forteaza re-mapping chiar daca exista harta salvata"),
    ]

    # ── Detectie harta existenta ──────────────────────────────────────────
    os.makedirs(MAP_DIR, exist_ok=True)
    map_exists = os.path.exists(MAP_FILE)
    # `force_remap:=true` din linia de comanda → ignora harta existenta
    # Detectie statica la generare-time (suficient pentru un launch normal)

    # ── Gazebo + URDF + bridge ────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("carucior_asamblat"), "launch", "gazebo.launch.py"])
        ),
    )

    # ── Scan QoS relay (necesar pentru SLAM) ──────────────────────────────
    scan_relay = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="human_follower",
                executable="scan_qos_relay",
                name="scan_qos_relay",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(slam_arg),
            ),
        ],
    )

    # ── SLAM (LifecycleNode + configure + activate) ───────────────────────
    # Daca exista harta → pass map_file_name pentru a o incarca
    slam_extra_params = {
        "use_lifecycle_manager": False,
        "use_sim_time": use_sim_time,
    }
    if map_exists:
        slam_extra_params["map_file_name"] = MAP_PATH
        print(f"[bringup] HARTA EXISTENTA gasita: {MAP_FILE} → o incarc")
    else:
        print(f"[bringup] Harta NU exista la {MAP_FILE} → mapping initial")

    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_params, slam_extra_params],
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
                LogInfo(msg="[slam_toolbox] configured → activating"),
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
        condition=IfCondition(slam_arg),
    )

    # ── nav2 costmaps (global + local) ────────────────────────────────────
    # Sunt LifecycleNodes, gestionate de nav2_lifecycle_manager.
    global_costmap_node = LifecycleNode(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="global_costmap",
        namespace="",
        output="screen",
        parameters=[global_costmap_yaml, {"use_sim_time": use_sim_time}],
    )
    local_costmap_node = LifecycleNode(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="local_costmap",
        namespace="",
        output="screen",
        parameters=[local_costmap_yaml, {"use_sim_time": use_sim_time}],
    )
    costmap_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="costmap_lifecycle_manager",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["global_costmap", "local_costmap"],
            "bond_timeout": 0.0,        # nu blocheaza la timeout
            "attempt_respawn_reconnection": False,
        }],
    )
    # IMPORTANT: pornim costmap-urile DUPA ce SLAM are timp sa publice /map.
    # SLAM activeaza la ~5s, publica primul /map la 5-8s. La t=12s avem
    # marja sa fim siguri ca /map exista cand global_costmap face configure.
    costmaps = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="[bringup] Pornesc global_costmap + local_costmap (nav2)"),
            global_costmap_node, local_costmap_node, costmap_lifecycle,
        ],
    )

    # ── DR-SPAAM / YOLO / human_follower ──────────────────────────────────
    dr_spaam_node = Node(
        package="human_follower",
        executable="dr_spaam_detector",
        name="dr_spaam_detector",
        output="screen",
        parameters=[hf_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(follower_arg),
    )
    yolo_node = Node(
        package="human_follower",
        executable="yolo_person_tracker",
        name="yolo_person_tracker",
        output="screen",
        parameters=[hf_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(follower_arg),
    )
    follower_node = Node(
        package="human_follower",
        executable="human_follower",
        name="human_follower",
        output="screen",
        parameters=[hf_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(follower_arg),
    )

    # ── Map Explorer (doar daca harta NU exista) ──────────────────────────
    explorer_node = Node(
        package="human_follower",
        executable="map_explorer",
        name="map_explorer",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_save_path": MAP_PATH,
            "explore_duration": 180.0,    # 3 min, ajustabil
        }],
    )

    if map_exists:
        # HARTA EXISTA → urmarire imediat (dupa stabilizare SLAM + costmaps)
        follower_group = TimerAction(
            period=14.0,
            actions=[
                LogInfo(msg="[bringup] Harta incarcata → pornesc urmarirea"),
                dr_spaam_node, yolo_node, follower_node,
            ],
        )
        explorer_group = []  # nimic
    else:
        # MAPPING → explorer porneste, la iesirea lui → followers
        # 15s = SLAM activ (5s) + /map publicat + costmaps activate (12s) + marja
        explorer_with_delay = TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg="[bringup] Pornesc map_explorer (mapping autonom)"),
                explorer_node,
            ],
        )
        # Cand explorer-ul moare (a salvat harta) → start followers
        on_explorer_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=explorer_node,
                on_exit=[
                    LogInfo(msg="[bringup] map_explorer a iesit → pornesc urmarirea"),
                    dr_spaam_node, yolo_node, follower_node,
                ],
            ),
        )
        explorer_group = [explorer_with_delay, on_explorer_exit]
        follower_group = None  # urmarirea porneste prin event handler

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
                condition=IfCondition(rviz_arg),
            ),
        ],
    )

    actions = declared + [gazebo, scan_relay, slam, rviz, costmaps]
    if follower_group is not None:
        actions.append(follower_group)
    actions.extend(explorer_group)

    return LaunchDescription(actions)
