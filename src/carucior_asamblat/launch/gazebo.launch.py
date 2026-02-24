import os

from ament_index_python.packages import get_package_share_directory

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

    # Convert package://carucior_asamblat/... to absolute file:// paths (mesh loading always works)
    urdf = urdf.replace(f"package://{pkg}/", f"file://{pkg_share}/")

    # ---- TF from URDF (now prefixed for multi-robot consistency) ----
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(urdf, value_type=str),
            # IMPORTANT: make TF names consistent with Gazebo model name
            "frame_prefix": "carucior_asamblat/",
        }],
    )

    # Optional: TF visualization helper (not needed once you publish real joint_states)
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # IMPORTANT: base_footprint should be TF-only (not a physical root link in Gazebo)
    # Now also prefixed.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_basefootprint_to_baselink",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "carucior_asamblat/base_footprint",
            "--child-frame-id", "carucior_asamblat/base_link",
        ],
    )

    # Alias TF: Gazebo/bridge uses carucior_asamblat/base_link/lidar_sensor in LaserScan header,
    # while URDF publishes carucior_asamblat/lidar. This links them (no physical offset).
    static_tf_lidar_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_lidar_alias",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "carucior_asamblat/lidar",
            "--child-frame-id", "carucior_asamblat/base_link/lidar_sensor",
        ],
    )
    static_tf_depthcam_alias = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_depthcam_alias",
    output="screen",
    arguments=[
        "--x", "0", "--y", "0", "--z", "0",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", "carucior_asamblat/camera",
        "--child-frame-id", "carucior_asamblat/base_link/depth_camera",
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

    # Bridge Gazebo <-> ROS2
    # Note: ] means ROS -> Gazebo, [ means Gazebo -> ROS
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=[
            # --- Gazebo -> ROS ---
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            # Camera (FIX): use /depth_camera (has publisher), not /depth_camera/image (no publisher)
            "/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",


            # Prefer the model odometry topic
            "/model/carucior_asamblat/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",

            # --- ROS -> Gazebo ---
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        remappings=[
            ("/lidar", "/scan"),
            ("/model/carucior_asamblat/odometry", "/odom"),
            ("/rgb_camera", "/camera/rgb/image_raw"),
            ("/rgb_camera/camera_info", "/camera/rgb/camera_info"),
            ("/depth_camera", "/camera/depth/image_raw"),
            ("/depth_camera/camera_info", "/camera/depth/camera_info"),


        ],
    )

    return [
        rsp,
        jsp,
        static_tf,
        static_tf_lidar_alias,
        static_tf_depthcam_alias,
        bridge,
        TimerAction(period=2.0, actions=[spawn]),
    ]


def generate_launch_description():
    pkg_share_path = get_package_share_directory("carucior_asamblat")
    world_path = os.path.join(pkg_share_path, "worlds", "world.sdf")
    pkg_share = FindPackageShare("carucior_asamblat")

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH"),
            TextSubstitution(text=":"),
            pkg_share,  # <-- IMPORTANT: share/carucior_asamblat
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
        gz_launch,
        OpaqueFunction(function=_make_robot_description),
    ])
