#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker

# NumPy 2.x compatibility (dr_spaam încă folosește np.int)
if not hasattr(np, "int"):
    np.int = int  # type: ignore

# Optional dependency (comes from the DR-SPAAM project)
try:
    from dr_spaam.detector import Detector  # type: ignore
except Exception:  # pragma: no cover
    Detector = None


def detections_to_pose_array(dets_xy: np.ndarray) -> PoseArray:
    msg = PoseArray()
    for x, y in dets_xy:
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = 0.0
        p.orientation.w = 1.0
        msg.poses.append(p)
    return msg


def _point(x: float, y: float):
    from geometry_msgs.msg import Point
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = 0.0
    return p


def detections_to_rviz_marker(dets_xy: np.ndarray) -> Marker:
    """
    Draw each detection as a circle using LINE_LIST.
    """
    m = Marker()
    m.action = Marker.ADD
    m.ns = "dr_spaam_ros2"
    m.id = 0
    m.type = Marker.LINE_LIST

    m.pose.orientation.w = 1.0
    m.scale.x = 0.03  # line width
    m.color.r = 1.0
    m.color.a = 1.0

    r = 0.4
    ang = np.linspace(0, 2.0 * np.pi, 20, endpoint=False)
    circle = np.stack((r * np.cos(ang), r * np.sin(ang)), axis=1)

    # LINE_LIST expects pairs of points
    for (cx, cy) in dets_xy:
        pts = circle + np.array([cx, cy])
        for i in range(len(pts)):
            p1 = pts[i]
            p2 = pts[(i + 1) % len(pts)]
            m.points.append(_point(p1[0], p1[1]))
            m.points.append(_point(p2[0], p2[1]))
    return m


class DrSpaamDetectorNode(Node):
    """
    ROS2 wrapper around DR-SPAAM detector.
    Publishes PoseArray (x,y in laser frame) and an RViz Marker.
    """

    def __init__(self):
        super().__init__("dr_spaam_detector")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("detections_topic", "/human_detections")
        self.declare_parameter("rviz_topic", "/human_detections_marker")

        self.declare_parameter("weight_file", "")
        self.declare_parameter("conf_thresh", 0.3)
        self.declare_parameter("stride", 1)
        self.declare_parameter("detector_model", "dr_spaam")
        self.declare_parameter("panoramic_scan", False)
        self.declare_parameter("use_gpu", True)

        # --- Self filtering to ignore robot/cart detections ---
        # Radius (m) around an OFFSET center (x_offset, 0). If a detection is inside this
        # offset-circle it will be ignored. Set x_offset negative to bias the filter behind the LiDAR.
        self.declare_parameter("self_filter_radius", 0.80)     # meters
        self.declare_parameter("self_filter_x_offset", -0.35)  # meters; negative = behind sensor (assuming +X forward)

        # Optional angular sector ignore (disabled if min>max)
        self.declare_parameter("ignore_angle_min_deg", 999.0)
        self.declare_parameter("ignore_angle_max_deg", -999.0)

        self.self_filter_radius = float(self.get_parameter("self_filter_radius").value)
        self.self_filter_x_offset = float(self.get_parameter("self_filter_x_offset").value)
        self.ignore_angle_min = math.radians(float(self.get_parameter("ignore_angle_min_deg").value))
        self.ignore_angle_max = math.radians(float(self.get_parameter("ignore_angle_max_deg").value))
        # ------------------------------------------------------

        if Detector is None:
            self.get_logger().error(
                "Python module 'dr_spaam' not found. Install the DR-SPAAM package "
                "(and its deps) in this ROS2 environment."
            )
            raise RuntimeError("Missing dependency: dr_spaam")

        weight_file = self.get_parameter("weight_file").get_parameter_value().string_value
        if not weight_file:
            self.get_logger().warn("Parameter 'weight_file' is empty. Detector may fail to initialize.")

        self.conf_thresh = float(self.get_parameter("conf_thresh").value)
        self.stride = int(self.get_parameter("stride").value)
        self.detector_model = str(self.get_parameter("detector_model").value)
        self.panoramic_scan = bool(self.get_parameter("panoramic_scan").value)
        self.use_gpu = bool(self.get_parameter("use_gpu").value)

        self.detector = Detector(
            weight_file,
            model=self.detector_model,
            gpu=self.use_gpu,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        scan_topic = self.get_parameter("scan_topic").value
        det_topic = self.get_parameter("detections_topic").value
        rviz_topic = self.get_parameter("rviz_topic").value

        self.dets_pub = self.create_publisher(PoseArray, det_topic, 10)
        self.rviz_pub = self.create_publisher(Marker, rviz_topic, 10)

        self.sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, qos)

        self._laser_fov_set = False
        self.get_logger().info(f"Listening to {scan_topic}. Publishing detections to {det_topic}.")

        angle_enabled = self.ignore_angle_min <= self.ignore_angle_max
        self.get_logger().info(
            f"Self-filter (offset circle): radius<{self.self_filter_radius:.2f}m "
            f"around center (x={self.self_filter_x_offset:.2f}, y=0.00). "
            f"Angle filter enabled={angle_enabled}"
        )

    def _apply_self_filters(self, dets_xy: np.ndarray) -> np.ndarray:
        if dets_xy.size == 0:
            return dets_xy

        x = dets_xy[:, 0]
        y = dets_xy[:, 1]

        # Offset-circle self filter: ignore detections close to (x_offset, 0)
        dx = x - self.self_filter_x_offset
        dy = y
        r = np.hypot(dx, dy)
        keep = r >= self.self_filter_radius

        # Optional: filter by angle sector (disabled unless min<=max)
        if self.ignore_angle_min <= self.ignore_angle_max:
            a = np.arctan2(y, x)
            keep = keep & ~((a >= self.ignore_angle_min) & (a <= self.ignore_angle_max))

        return dets_xy[keep]

    def on_scan(self, msg: LaserScan):
        # Set FOV on first message (DR-SPAAM expects degrees)
        if (not self._laser_fov_set) and hasattr(self.detector, "set_laser_fov"):
            try:
                fov_deg = math.degrees(msg.angle_increment * len(msg.ranges))
                self.detector.set_laser_fov(fov_deg)
                self._laser_fov_set = True
            except Exception as e:
                self.get_logger().warn(f"Could not set laser FOV: {e}")

        scan = np.array(msg.ranges, dtype=np.float32)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        try:
            dets_xy, dets_conf, _ = self.detector(scan)
        except Exception as e:
            self.get_logger().error(f"Detector inference failed: {e}")
            return

        dets_conf = np.array(dets_conf).reshape(-1)
        conf_mask = dets_conf >= self.conf_thresh
        dets_xy = np.array(dets_xy)[conf_mask]
        # Inverseaza axa X (fata <-> spate)
        dets_xy[:, 0] = -dets_xy[:, 0]

        # Ignore cart/robot detections
        dets_xy = self._apply_self_filters(dets_xy)

        det_msg = detections_to_pose_array(dets_xy)
        det_msg.header = msg.header
        self.dets_pub.publish(det_msg)

        marker = detections_to_rviz_marker(dets_xy)
        marker.header = msg.header
        self.rviz_pub.publish(marker)


def main():
    rclpy.init()
    try:
        node = DrSpaamDetectorNode()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()