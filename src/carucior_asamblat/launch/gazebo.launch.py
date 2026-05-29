import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _make_robot_description(context, *args, **kwargs):
    pkg = "carucior_asamblat"
    pkg_share = get_package_share_directory(pkg)
    urdf_path = os.path.join(pkg_share, "urdf", "carucior_asamblat.urdf")

    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf = f.read()

    # package:// -> file:// pentru mesh-uri (incarcare robusta in Gazebo+RViz)
    urdf = urdf.replace(f"package://{pkg}/", f"file://{pkg_share}/")

    # Robot state publisher FARA frame_prefix (un singur robot)
    # → publica TF: base_link, lidar, camera, roti, etc. (nume simple)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(urdf, value_type=str),
        }],
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # IMPORTANT: toate static_transform_publisher au use_sim_time=true,
    # altfel TF-urile au timestamp wall-time iar lookup-urile de SLAM
    # (care folosesc sim time din /clock) esueaza si harta nu se publica.
    sim_time = {"use_sim_time": True}

    # NOTA: TF base_footprint->base_link e DEFINIT IN URDF (joint fix cu
    # rotatie +pi/2 + translatie de centrare). Nu mai pornim static
    # transform publisher pentru asta (era fix pana cand URDF avea root pe
    # base_link; acum URDF are root pe base_footprint).

    # Alias TF pentru frame-ul publicat de Gazebo pe LaserScan
    # Gazebo Sim foloseste formatul: <model>/<canonical_link>/<sensor_name>
    # Canonical link e ROOT-UL URDF-ului. DUPA fix-ul nostru, root-ul e
    # base_footprint (nu base_link), deci frame_id devine:
    #   "carucior_asamblat/base_footprint/lidar_sensor"
    # NU "carucior_asamblat/base_link/lidar_sensor" cum era inainte!
    static_tf_lidar_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_lidar_alias",
        output="screen",
        parameters=[sim_time],
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "lidar",
            "--child-frame-id", "carucior_asamblat/base_footprint/lidar_sensor",
        ],
    )

    # Acelasi pentru camera depth/rgb (canonical link = base_footprint)
    static_tf_depthcam_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_depthcam_alias",
        output="screen",
        parameters=[sim_time],
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "camera",
            "--child-frame-id", "carucior_asamblat/base_footprint/depth_camera",
        ],
    )

    static_tf_rgbcam_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_rgbcam_alias",
        output="screen",
        parameters=[sim_time],
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "camera",
            "--child-frame-id", "carucior_asamblat/base_footprint/rgb_camera",
        ],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_carucior_asamblat",
        output="screen",
        arguments=[
            "-world", "carucior_world",
            "-name", "carucior_asamblat",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "-11.2",
            # base_footprint este la nivelul solului (z=0 in URDF), iar
            # roata bottom e exact la z=0 datorita joint base_footprint→
            # base_link (z=-0.24). Asadar spawnam la z=0 (anterior era
            # -0.25, compensand pentru ca rootul era base_link mai sus).
            "-z", "0.0",
            "-R", "0",
            "-P", "0",
            # Yaw=0 → robotul priveste spre world +X. SLAM initializeaza
            # map frame la aceeasi orientare cu base_footprint, deci
            # map +X = world +X si map +Y = world +Y. Astfel harta din
            # RViz coincide cu vederea de sus din Gazebo — 2D Goal Pose
            # click-urile in RViz merg unde te astepti vizual.
            #
            # Anterior era yaw=1.57 (robot facing +Y), iar map era rotita
            # 90° fata de world → click-urile pareau "in directii gresite".
            "-Y", "0.0",
        ],
    )

    # Bridge Gazebo <-> ROS2 (] = ROS->GZ, [ = GZ->ROS, @ = bidirectional)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            # --- Gazebo -> ROS ---
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            "/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",

            "/model/carucior_asamblat/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",

            # TF de la DiffDrive (odom -> base_link)
            "/model/carucior_asamblat/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",

            # --- ROS -> Gazebo ---
            # ATENTIE: argumentul "TOPIC@ROS]GZ" foloseste TOPIC ca nume
            # AL TOPICULUI atat pe ROS cat si pe Gazebo. Plugin-ul din URDF
            # asculta pe Gazebo /cmd_vel (vezi <topic>cmd_vel</topic>), deci
            # numele in bridge trebuie sa fie /cmd_vel. Apoi remap ROS side
            # ca bridge sa subscriba la ROS /cmd_vel_gz (output-ul inverter-ului).
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        remappings=[
            ("/lidar", "/scan"),
            ("/model/carucior_asamblat/odometry", "/odom"),
            ("/model/carucior_asamblat/tf", "/tf"),
            ("/rgb_camera", "/camera/rgb/image_raw"),
            ("/rgb_camera/camera_info", "/camera/rgb/camera_info"),
            ("/depth_camera", "/camera/depth/image_raw"),
            ("/depth_camera/camera_info", "/camera/depth/camera_info"),
            # remap ROS side: bridge "vede" /cmd_vel ca fiind /cmd_vel_gz
            # → subscribe pe ROS /cmd_vel_gz (output-ul inverter-ului),
            # publish pe Gazebo /cmd_vel (intrarea plugin-ului diff_drive)
            ("/cmd_vel", "/cmd_vel_gz"),
        ],
    )

    return [
        rsp,
        jsp,
        static_tf_lidar_alias,
        static_tf_depthcam_alias,
        static_tf_rgbcam_alias,
        bridge,
        TimerAction(period=2.0, actions=[spawn]),
    ]


def generate_launch_description():
    pkg_share_path = get_package_share_directory("carucior_asamblat")
    world_path = os.path.join(pkg_share_path, "worlds", "world.sdf")
    pkg_share = FindPackageShare("carucior_asamblat")
    battery_plugin_lib_path = os.path.join(get_package_prefix("battery_system_plugin"), "lib")

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            TextSubstitution(text=":"),
            pkg_share,
        ],
    )

    gz_system_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
            TextSubstitution(text=":"),
            TextSubstitution(text=battery_plugin_lib_path),
        ],
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": f"{world_path}"}.items(),
    )

    return LaunchDescription([
        gz_resource_path,
        gz_system_plugin_path,
        gz_launch,
        OpaqueFunction(function=_make_robot_description),
    ])
