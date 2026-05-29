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

    # ── cmd_vel_inverter — inverseaza linear.x intre publishers ROS
    # (nav2, follower, delivery_manager — toti publica /cmd_vel) si gazebo
    # (subscribe /cmd_vel_gz dupa remap). Compensseaza diferenta de
    # conventie introdusa de rotatia base_footprint→base_link in URDF.
    cmd_vel_inverter = Node(
        package="human_follower",
        executable="cmd_vel_inverter",
        name="cmd_vel_inverter",
        output="screen",
        parameters=[{
            "use_sim_time":       use_sim_time,
            "input_topic":        "/cmd_vel",
            "output_topic":       "/cmd_vel_gz",
            # Doar linear.x are nevoie de inversare (URDF-ul are
            # base_footprint→base_link cu yaw +π/2 + wheel axes originale,
            # ceea ce inverseaza sensul "forward" intre cmd_vel ROS si
            # plugin-ul diff_drive Gazebo).
            #
            # angular.z NU trebuie inversat: daca il inversezi, plugin-ul
            # comanda fizic robotul sa se invarta in directia opusa fata
            # de ce raporteaza in odom → RViz (citeste TF din odom)
            # arata o rotatie, dar fizic robotul face alta → SLAM primeste
            # input inconsistent si harta se corupe (frontiere zero pentru
            # explore_lite). Cu invert_angular_z=False, plugin-ul comanda
            # si raporteaza acelasi sens → consistenta TF↔fizic.
            # TEST diagnostic: dezactivam complet inverter-ul.
            # Daca robotul merge in directia gresita (fata <-> spate sau
            # stanga <-> dreapta), re-pun True pe componenta gresita.
            # Cu URDF-ul cu rotatia base_footprint +π/2 si wheel axes ca in
            # URDF, plugin-ul diff_drive ar trebui sa comande fizic corect
            # fara inversare (sensul "forward" in cmd_vel coincide cu
            # base_footprint +X, iar rotile rotesc cart-ul in +X la viteza
            # pozitiva de joint).
            "invert_linear_x":    False,
            "invert_angular_z":   False,
        }],
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

    # ── nav2 costmaps standalone (DOAR cand harta e incarcata, NU la mapping) ─
    # In modul mapping, costmap-urile vin din nav2 navigation stack (planner/
    # controller_server), deci nu le pornim separat ca sa nu fie duplicate.
    # In modul localization (harta exista), pornim costmap-urile standalone
    # pentru vizualizare in rviz.
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
            "bond_timeout": 0.0,
            "attempt_respawn_reconnection": False,
        }],
    )
    standalone_costmaps = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="[bringup] Pornesc costmaps standalone (mod localization)"),
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

    # ── first_scan (mapping autonom: nav2 + explore_lite + save) ──────────
    # Folosim launch-ul din pachetul first_scan, care porneste nav2 stack
    # complet + explore_lite + watcher de salvare. Cand watcher-ul iese,
    # bringup-ul porneste urmarirea.
    # save_and_exit este nodul care iese prima oara (cu rclpy.shutdown);
    # lansam un proces dedicat ca sa-l detectam in OnProcessExit.
    first_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("first_scan"), "launch", "first_scan.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_save_path": MAP_PATH,
        }.items(),
    )

    # Detectie "first_scan terminat" prin rularea SEPARATA a save_and_exit
    # (un duplicat al watcher-ului) cu un timeout extins — atunci cand iese,
    # bringup-ul stie ca mapping-ul s-a terminat. Asa evitam dependinta pe
    # un nod din interiorul launch-ului inclus.
    sentinel_node = Node(
        package="first_scan",
        executable="save_and_exit",
        name="first_scan_sentinel",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_save_path": MAP_PATH,
            "done_idle_seconds":   45.0,    # un pic mai relaxat decat watcher-ul intern
            "max_explore_seconds": 480.0,
            "min_explore_seconds": 70.0,
        }],
    )

    if map_exists:
        # HARTA EXISTA → urmarire imediat + costmaps standalone pt vizualizare
        follower_group = TimerAction(
            period=14.0,
            actions=[
                LogInfo(msg="[bringup] Harta incarcata → pornesc urmarirea"),
                dr_spaam_node, yolo_node, follower_node,
            ],
        )
        explorer_group = [standalone_costmaps]
    else:
        # MAPPING → first_scan stack porneste (nav2 + explore_lite + watcher)
        first_scan_delayed = TimerAction(
            period=12.0,
            actions=[
                LogInfo(msg="[bringup] Pornesc first_scan (nav2 + explore_lite)"),
                first_scan_launch,
            ],
        )
        sentinel_delayed = TimerAction(
            period=14.0,
            actions=[sentinel_node],
        )
        # Cand sentinel-ul moare (a salvat harta) → start followers
        on_sentinel_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=sentinel_node,
                on_exit=[
                    LogInfo(msg="[bringup] first_scan complet → pornesc urmarirea"),
                    dr_spaam_node, yolo_node, follower_node,
                ],
            ),
        )
        explorer_group = [first_scan_delayed, sentinel_delayed, on_sentinel_exit]
        follower_group = None

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

    actions = declared + [gazebo, cmd_vel_inverter, scan_relay, slam, rviz]
    if follower_group is not None:
        actions.append(follower_group)
    actions.extend(explorer_group)

    return LaunchDescription(actions)
