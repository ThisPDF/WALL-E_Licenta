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

    # base_footprint -> base_link (proiectie pe sol; conventie ROS Nav/SLAM)
    # DiffDrive publica odom -> base_footprint, deci base_footprint e parinte.
    # base_link e in chassis, la z=+0.24 fata de sol (≈ raza roata 0.12 + offset).
    static_tf_basefootprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_footprint_to_base_link",
        output="screen",
        parameters=[sim_time],
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.24",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
        ],
    )

    # Alias TF pentru frame-ul publicat de Gazebo pe LaserScan
    # Gazebo Sim foloseste formatul: <model>/<canonical_link>/<sensor_name>
    # → "carucior_asamblat/base_link/lidar_sensor"
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
            "--child-frame-id", "carucior_asamblat/base_link/lidar_sensor",
        ],
    )

    # Acelasi pentru camera depth/rgb
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
            "--child-frame-id", "carucior_asamblat/base_link/depth_camera",
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
            "--child-frame-id", "carucior_asamblat/base_link/rgb_camera",
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
            "-z", "-0.25",
            "-R", "0",
            "-P", "0",
            "-Y", "1.57",
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
        ],
    )

    return [
        rsp,
        jsp,
        static_tf_basefootprint,
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
