#!/usr/bin/env python3
"""
delivery.launch.py — porneste TOT stack-ul de delivery deodata.

Componente:
  1. Gazebo + URDF + bridge        (carucior_asamblat/gazebo.launch.py)
  2. cmd_vel_inverter              (/cmd_vel -> /cmd_vel_gz, ce asculta Gazebo)
  3. scan_qos_relay                (republica /scan cu QoS pe care-l vrea SLAM)
  4. slam_toolbox LOCALIZARE       (incarca ~/maps/carucior_market, publica /map + TF)
  5. map_uploader                  (/map -> POST /api/robot/map la server)
  6. rviz                          (vizualizare, optional: rviz:=false)
  7. delivery_manager              (poll /tmp target, navigheaza, posteaza status la server)

Fluxul: server up -> app trimite tinta -> delivery_manager navigheaza spre ea
(control direct pe /cmd_vel) -> posteaza pozitia live + harta la server -> app afiseaza.

NB: delivery_manager-ul actual navigheaza DIRECT (P-controller propriu), nu prin
Nav2. SLAM ruleaza doar ca sa furnizeze /map (pentru harta din app) + TF pentru rviz.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler, TimerAction
)
from launch.conditions import IfCondition
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
    fs_pkg  = get_package_share_directory("first_scan")
    hf_pkg  = get_package_share_directory("human_follower")

    rviz_config = os.path.join(car_pkg, "config", "carucior.rviz")
    slam_params = os.path.join(car_pkg, "config", "slam_toolbox_params.yaml")
    nav2_params = os.path.join(fs_pkg,  "config", "nav2_params.yaml")
    hf_params   = os.path.join(hf_pkg,  "config", "params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_arg     = LaunchConfiguration("rviz")
    server_url   = LaunchConfiguration("server_url")

    declared = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz",        default_value="true"),
        DeclareLaunchArgument("server_url",  default_value="http://localhost:8080"),
    ]

    if not os.path.exists(MAP_FILE):
        print(f"[delivery] ATENTIE: harta {MAP_FILE} NU exista. "
              f"Ruleaza intai bringup ca sa o creezi, sau muta-o aici.")

    # 1. Gazebo + URDF + bridge
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("carucior_asamblat"), "launch", "gazebo.launch.py"])
        ),
    )

    # 2. cmd_vel_inverter — puntea catre Gazebo (asculta /cmd_vel_gz).
    #    Nav2 (navigation_launch.py) remapeaza iesirea controllerului la
    #    /cmd_vel_nav, deci inverter-ul citeste /cmd_vel_nav -> /cmd_vel_gz.
    #    Fara el robotul nu se misca.
    cmd_vel_inverter = Node(
        package="human_follower",
        executable="cmd_vel_inverter",
        name="cmd_vel_inverter",
        output="screen",
        parameters=[{
            "use_sim_time":     use_sim_time,
            "input_topic":      "/cmd_vel_nav",
            "output_topic":     "/cmd_vel_gz",
            "invert_linear_x":  False,
            "invert_angular_z": False,
        }],
    )

    # 3. scan_qos_relay (necesar pentru SLAM)
    scan_relay = TimerAction(
        period=4.0,
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

    # 4. slam_toolbox in LOCALIZARE pe harta salvata
    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_params, {
            "use_lifecycle_manager": False,
            "use_sim_time": use_sim_time,
            "mode": "localization",
            "map_file_name": MAP_PATH,
        }],
    )
    slam_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    slam_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_node,
        start_state="configuring", goal_state="inactive",
        entities=[
            LogInfo(msg="[slam_toolbox] configured -> activating"),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )),
        ],
    ))
    slam = TimerAction(period=5.0, actions=[slam_node, slam_configure, slam_activate])

    # 4b. Nav2 — stiva completa de navigare (planner + controller + bt_navigator).
    #     Planeaza in frame `map` (furnizat de slam localizare). Controllerul scoate
    #     viteza pe /cmd_vel_nav (remap din navigation_launch) -> cmd_vel_inverter.
    #     Pornit dupa slam (~5s) ca sa aiba deja /map + TF map->odom.
    nav2 = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[delivery] Pornesc Nav2 (navigation_launch)"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
                ),
                launch_arguments={
                    "use_sim_time":    use_sim_time,
                    "params_file":     nav2_params,
                    "autostart":       "true",
                    "use_composition": "False",
                }.items(),
            ),
        ],
    )

    # 4c. Senzorii follower-ului (ca in bringup). human_follower e pornit/oprit
    #     de delivery_manager ca subproces; aici rulam DOAR senzorii lui ca sa
    #     aiba input cand intra in FOLLOWING (follower-ul cere require_yolo):
    #       scan_map_filter -> /scan_legs_filtered (mascat cu harta)
    #       dr_spaam        -> /human_detections   (picioare LIDAR)
    #       yolo            -> /yolo_person_target  (persoana camera)
    #       uwb             -> /uwb_person_pose     (ground truth actor, sim)
    follower_sensors = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="[delivery] Pornesc senzorii follower-ului (dr_spaam+yolo+uwb)"),
            Node(package="human_follower", executable="scan_map_filter",
                 name="scan_map_filter", output="screen",
                 parameters=[hf_params, {"use_sim_time": use_sim_time}]),
            Node(package="human_follower", executable="uwb_pose_publisher",
                 name="uwb_pose_publisher", output="screen",
                 parameters=[hf_params, {"use_sim_time": use_sim_time}]),
            Node(package="human_follower", executable="dr_spaam_detector",
                 name="dr_spaam_detector", output="screen",
                 parameters=[hf_params, {"use_sim_time": use_sim_time}]),
            Node(package="human_follower", executable="yolo_person_tracker",
                 name="yolo_person_tracker", output="screen",
                 parameters=[hf_params, {"use_sim_time": use_sim_time}]),
        ],
    )

    # 5. map_uploader — /map -> server (dupa ce SLAM e activ)
    map_uploader = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="human_follower",
                executable="map_uploader",
                name="map_uploader",
                output="screen",
                parameters=[{
                    "use_sim_time":   use_sim_time,
                    "map_topic":      "/map",
                    "server_url":     server_url,
                    "world_offset_x": 0.0,
                    "world_offset_y": -11.2,
                }],
            ),
        ],
    )

    # 6. rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz_arg),
    )

    # 7. delivery_manager — creierul livrarii (poll /tmp, navigheaza, status la server)
    delivery_manager = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="human_follower",
                executable="delivery_manager",
                name="delivery_manager",
                output="screen",
                parameters=[{
                    "use_sim_time":        use_sim_time,
                    "cmd_vel_topic":       "/cmd_vel",
                    "odom_topic":          "/odom",
                    "scan_topic":          "/scan",
                    "delivery_server_url": server_url,
                    "arrival_distance":    0.6,
                    "max_linear":          0.5,
                    "max_angular":         1.2,
                    "k_lin":               1.5,
                    "k_ang":               2.5,
                }],
            ),
        ],
    )

    return LaunchDescription(declared + [
        gazebo,
        cmd_vel_inverter,
        scan_relay,
        slam,
        nav2,
        follower_sensors,
        map_uploader,
        rviz,
        delivery_manager,
    ])
