#!/usr/bin/env python3
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from tf2_ros import Buffer, TransformListener, TransformException

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

        # X-flip pe detectii. Necesar doar daca LIDAR-ul fizic e montat rotit
        # ~π fata de base_footprint (vechiul /scan al caruciorului). Pentru
        # un LIDAR cu yaw=0 in base_footprint (ex: /scan_legs) → flip_x_axis=False.
        self.declare_parameter("flip_x_axis", False)

        # Optional angular sector ignore (disabled if min>max)
        self.declare_parameter("ignore_angle_min_deg", 999.0)
        self.declare_parameter("ignore_angle_max_deg", -999.0)

        # --- Map-based detection filter ----------------------------------
        # Proiecteaza fiecare detectie (x,y) in frame `map` si o arunca daca
        # cade pe celula OCCUPIED, UNKNOWN sau OUT-OF-MAP. Necesar pentru a
        # evita false-positives DR-SPAAM pe colturi de raft / pereti.
        self.declare_parameter("map_filter_enabled",  False)
        self.declare_parameter("map_topic",           "/map")
        self.declare_parameter("map_frame",           "map")
        self.declare_parameter("map_occupied_threshold", 50)
        # Inflatie celule OCCUPIED+UNKNOWN, in celule. ~2 = 10cm la 0.05m/celula.
        self.declare_parameter("map_inflation_cells", 2)
        self.declare_parameter("map_filter_unknown",  True)
        self.declare_parameter("map_filter_out_of_map", True)
        self.declare_parameter("map_tf_lookup_timeout", 0.05)

        self.self_filter_radius = float(self.get_parameter("self_filter_radius").value)
        self.self_filter_x_offset = float(self.get_parameter("self_filter_x_offset").value)
        self.ignore_angle_min = math.radians(float(self.get_parameter("ignore_angle_min_deg").value))
        self.ignore_angle_max = math.radians(float(self.get_parameter("ignore_angle_max_deg").value))
        self.flip_x_axis = bool(self.get_parameter("flip_x_axis").value)

        self.map_filter_enabled = bool(self.get_parameter("map_filter_enabled").value)
        self.map_frame          = str(self.get_parameter("map_frame").value)
        self.map_occ_thresh     = int(self.get_parameter("map_occupied_threshold").value)
        self.map_infl_cells     = int(self.get_parameter("map_inflation_cells").value)
        self.map_filt_unknown   = bool(self.get_parameter("map_filter_unknown").value)
        self.map_filt_oob       = bool(self.get_parameter("map_filter_out_of_map").value)
        self.map_tf_timeout_s   = float(self.get_parameter("map_tf_lookup_timeout").value)
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

        # Map subscription + TF, doar daca map_filter e activat
        self._map_info = None
        self._reject_mask: Optional[np.ndarray] = None
        self._tf_buf = None
        self._tf_listener = None
        if self.map_filter_enabled:
            map_topic = str(self.get_parameter("map_topic").value)
            map_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.sub_map = self.create_subscription(
                OccupancyGrid, map_topic, self._on_map, map_qos
            )
            self._tf_buf = Buffer()
            self._tf_listener = TransformListener(self._tf_buf, self)
            self.get_logger().info(
                f"Map-filter ENABLED: map={map_topic} frame={self.map_frame} "
                f"infl={self.map_infl_cells} occ_thr={self.map_occ_thresh} "
                f"unknown={self.map_filt_unknown} oob={self.map_filt_oob}"
            )

        self._laser_fov_set = False
        self._map_warned = False
        self._tf_warned = False
        self.get_logger().info(f"Listening to {scan_topic}. Publishing detections to {det_topic}.")

        angle_enabled = self.ignore_angle_min <= self.ignore_angle_max
        self.get_logger().info(
            f"Self-filter (offset circle): radius<{self.self_filter_radius:.2f}m "
            f"around center (x={self.self_filter_x_offset:.2f}, y=0.00). "
            f"Angle filter enabled={angle_enabled}"
        )

    # ------------------------------------------------------------------
    @staticmethod
    def _dilate_bool(mask: np.ndarray, iterations: int) -> np.ndarray:
        """3x3 box dilation repeated `iterations` ori (numpy-only)."""
        if iterations <= 0:
            return mask
        out = mask.copy()
        for _ in range(iterations):
            shifted = out.copy()
            shifted[1:, :]  |= out[:-1, :]
            shifted[:-1, :] |= out[1:, :]
            shifted[:, 1:]  |= out[:, :-1]
            shifted[:, :-1] |= out[:, 1:]
            out = shifted
        return out

    def _on_map(self, msg: OccupancyGrid):
        self._map_info = msg.info
        grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )
        occupied = grid >= self.map_occ_thresh
        if self.map_filt_unknown:
            reject = occupied | (grid == -1)
        else:
            reject = occupied.copy()
        if self.map_infl_cells > 0:
            reject = self._dilate_bool(reject, self.map_infl_cells)
        self._reject_mask = reject
        self.get_logger().info(
            f"Map received: {msg.info.width}x{msg.info.height} @ "
            f"{msg.info.resolution:.3f}m | reject cells "
            f"(after dilation): {int(reject.sum())}/{reject.size}"
        )

    def _apply_map_filter(
        self, dets_xy: np.ndarray, header
    ) -> np.ndarray:
        """Drop detections that fall on occupied / unknown / out-of-map cells."""
        if not self.map_filter_enabled or dets_xy.size == 0:
            return dets_xy
        if self._reject_mask is None or self._map_info is None:
            if not self._map_warned:
                self.get_logger().warn(
                    f"Map filter ON dar niciun /map primit inca — "
                    f"trec detectiile nefiltrat"
                )
                self._map_warned = True
            return dets_xy
        # Time() = latest TF disponibil (evita extrapolation errors in single-threaded
        # executor). Staleness <20ms, irelevanta la 0.15m inflatie.
        try:
            tf = self._tf_buf.lookup_transform(
                self.map_frame,
                header.frame_id,
                Time(),
                timeout=Duration(seconds=self.map_tf_timeout_s),
            )
        except TransformException as e:
            if not self._tf_warned:
                self.get_logger().warn(
                    f"TF {self.map_frame}←{header.frame_id} indisponibil ({e}) "
                    f"— trec detectiile nefiltrat"
                )
                self._tf_warned = True
            return dets_xy
        self._tf_warned = False

        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        xl = dets_xy[:, 0].astype(np.float32)
        yl = dets_xy[:, 1].astype(np.float32)
        mx = tx + cos_t * xl - sin_t * yl
        my = ty + sin_t * xl + cos_t * yl

        info = self._map_info
        res = info.resolution
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        H = info.height
        W = info.width

        col = ((mx - ox) / res).astype(np.int32)
        row = ((my - oy) / res).astype(np.int32)

        oob = (row < 0) | (row >= H) | (col < 0) | (col >= W)

        drop = np.zeros(len(dets_xy), dtype=bool)
        if self.map_filt_oob:
            drop |= oob
        in_map = ~oob
        if in_map.any():
            idx = np.where(in_map)[0]
            cells = self._reject_mask[row[idx], col[idx]]
            drop[idx] |= cells

        n_drop = int(drop.sum())
        if n_drop > 0:
            self.get_logger().info(
                f"[MAP-FILTER] drop {n_drop}/{len(dets_xy)} detectii pe perete/oob/unknown"
            )
        return dets_xy[~drop]

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
        # X-flip: necesar daca senzorul fizic e montat rotit ~π fata de
        # base_footprint (vechiul /scan al caruciorului). Pentru un LIDAR
        # cu yaw=0 in base_footprint (ex: /scan_legs) NU se aplica.
        if self.flip_x_axis and dets_xy.size > 0:
            dets_xy[:, 0] = -dets_xy[:, 0]

        # Ignore cart/robot detections
        dets_xy = self._apply_self_filters(dets_xy)

        # Drop detectiile care cad pe pereti / unknown / OOB in harta SLAM.
        # Asta blocheaza false-positives DR-SPAAM pe colturi de raft.
        dets_xy = self._apply_map_filter(dets_xy, msg.header)

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