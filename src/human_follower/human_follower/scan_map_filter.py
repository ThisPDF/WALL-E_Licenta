#!/usr/bin/env python3
"""
scan_map_filter — mascheaza /scan_legs folosind harta SLAM statica.

Rationalul: DR-SPAAM antrenat pe JRDB confunda colturi de pereti / rafturi cu
picioare. Daca avem deja harta statica a magazinului (slam_toolbox
localization mode), putem proiecta fiecare raza /scan_legs in frame-ul map
si arunca returnarile care cad pe celule OCCUPIED sau UNKNOWN (sau in afara
hartii). Asa DR-SPAAM primeste doar returnari de la obiecte care NU sunt in
harta statica → in mare, oameni si obstacole dinamice.

I/O:
  /scan_legs       (LaserScan, BestEffort)  → input brut din Gazebo bridge
  /map             (OccupancyGrid, Transient Local) → harta slam_toolbox
  /scan_legs_filtered (LaserScan, Reliable) → output catre DR-SPAAM

Detalii implementare:
  - Lookup TF map ← header.frame_id, o singura data pe scan.
  - Endpoint = (range * cos(angle), range * sin(angle)) → transformat in map.
  - Mascam endpoint-urile in celule OCCUPIED (>= occupied_threshold), UNKNOWN
    (-1) sau OUT-OF-MAP. Inflatie configurabila (in celule) ca sa absorbim
    drift TF / zgomot lidar.
  - Daca nu avem inca harta sau TF nu e gata → passthrough (nu blocam DR-SPAAM).
"""
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from tf2_ros import Buffer, TransformListener, TransformException


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


class ScanMapFilter(Node):
    def __init__(self):
        super().__init__("scan_map_filter")

        self.declare_parameter("input_topic",       "/scan_legs")
        self.declare_parameter("output_topic",      "/scan_legs_filtered")
        self.declare_parameter("map_topic",         "/map")
        self.declare_parameter("map_frame",         "map")
        # Pragul de la care o celula e considerata "perete" (0..100 occupancy).
        self.declare_parameter("occupied_threshold", 50)
        # Inflatie a regiunii OCCUPIED+UNKNOWN, in celule. Compenseaza drift
        # TF + zgomot lidar (1-3 celule la rezolutia hartii ~0.05m → ~5-15cm).
        self.declare_parameter("inflation_cells",   2)
        # Daca True, mascam si returnarile pe celule UNKNOWN (-1).
        self.declare_parameter("filter_unknown",    True)
        # Daca True, mascam si returnarile in afara hartii.
        self.declare_parameter("filter_out_of_map", True)
        # Pana primim harta sau TF, ce facem cu scan-ul? True = trecem brut,
        # False = nu publicam (DR-SPAAM ramane orb).
        self.declare_parameter("passthrough_if_no_map", True)
        # Timeout pentru lookup TF (s).
        self.declare_parameter("tf_lookup_timeout", 0.05)

        self.input_topic   = str(self.get_parameter("input_topic").value)
        self.output_topic  = str(self.get_parameter("output_topic").value)
        self.map_topic     = str(self.get_parameter("map_topic").value)
        self.map_frame     = str(self.get_parameter("map_frame").value)
        self.occ_thresh    = int(self.get_parameter("occupied_threshold").value)
        self.inflation     = int(self.get_parameter("inflation_cells").value)
        self.filt_unknown  = bool(self.get_parameter("filter_unknown").value)
        self.filt_oob      = bool(self.get_parameter("filter_out_of_map").value)
        self.passthrough   = bool(self.get_parameter("passthrough_if_no_map").value)
        self.tf_timeout_s  = float(self.get_parameter("tf_lookup_timeout").value)

        # Map state (precalculat: mask 2D bool unde True = celula respinge)
        self._map_info = None
        self._reject_mask: Optional[np.ndarray] = None

        # TF
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # QoS — match bridge-ul Gazebo (BE) la input, Reliable la output.
        in_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=5,
                            durability=DurabilityPolicy.VOLATILE)
        out_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=5,
                             durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pub = self.create_publisher(LaserScan, self.output_topic, out_qos)
        self.sub_scan = self.create_subscription(
            LaserScan, self.input_topic, self._on_scan, in_qos
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, map_qos
        )

        self._count = 0
        self._dropped_total = 0
        self._warned_no_map = False
        self._warned_no_tf = False

        self.get_logger().info(
            f"scan_map_filter: {self.input_topic} → {self.output_topic} | "
            f"map={self.map_topic} | infl={self.inflation} cells | "
            f"occ_thr={self.occ_thresh} | unknown={self.filt_unknown} | "
            f"oob={self.filt_oob}"
        )

    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid):
        self._map_info = msg.info
        grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )
        occupied = grid >= self.occ_thresh
        if self.filt_unknown:
            reject = occupied | (grid == -1)
        else:
            reject = occupied.copy()
        if self.inflation > 0:
            reject = _dilate_bool(reject, self.inflation)
        self._reject_mask = reject
        self.get_logger().info(
            f"Map received: {msg.info.width}x{msg.info.height} @ "
            f"{msg.info.resolution:.3f}m | rejected cells (after dilation): "
            f"{int(reject.sum())}/{reject.size}"
        )

    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        # Fara harta inca: passthrough sau drop
        if self._reject_mask is None or self._map_info is None:
            if self.passthrough:
                self.pub.publish(msg)
            if not self._warned_no_map:
                self.get_logger().warn(
                    f"Niciun /map primit inca → passthrough={self.passthrough}"
                )
                self._warned_no_map = True
            return

        # Lookup TF: scan frame → map frame
        # Folosim Time() (latest TF disponibil) in loc de msg.header.stamp:
        # in executor single-threaded, asteptarea pe stamp exact blocheaza
        # propriul callback TF si expira. Sacrificam ~10ms staleness (negiljabil
        # la 0.05m/celula) si eliminam complet erorile de extrapolare.
        try:
            tf = self._tf_buf.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as e:
            if self.passthrough:
                self.pub.publish(msg)
            if not self._warned_no_tf:
                self.get_logger().warn(
                    f"TF {self.map_frame}←{msg.header.frame_id} indisponibil ({e}) "
                    f"→ passthrough={self.passthrough}"
                )
                self._warned_no_tf = True
            return
        self._warned_no_tf = False

        # Extrage yaw + translatie 2D din transform
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        n = len(msg.ranges)
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        finite = (
            np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= msg.range_max)
        )
        # Inlocuim inf/nan cu 0 pentru calcule (evita NaN warnings + cast errors).
        # Razele non-finite oricum nu intra in masca in_map (sunt filtrate de `finite`).
        ranges_safe = np.where(finite, ranges, 0.0).astype(np.float32)
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        # Endpoint in lidar frame
        xl = ranges_safe * np.cos(angles)
        yl = ranges_safe * np.sin(angles)
        # Endpoint in map frame
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

        drop = np.zeros(n, dtype=bool)
        if self.filt_oob:
            drop |= oob & finite

        # Pentru razele in harta, look up mask precomputat
        in_map = ~oob & finite
        if in_map.any():
            idx = np.where(in_map)[0]
            cells = self._reject_mask[row[idx], col[idx]]
            drop[idx] |= cells

        # Build output: pune inf pe razele drop-uite
        new_ranges = ranges.copy()
        new_ranges[drop] = float("inf")
        n_dropped = int(drop.sum())

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = new_ranges.tolist()
        out.intensities = list(msg.intensities)
        self.pub.publish(out)

        self._count += 1
        self._dropped_total += n_dropped
        if self._count == 1 or self._count % 100 == 0:
            avg = self._dropped_total / self._count
            self.get_logger().info(
                f"Filtered {self._count} scans | avg {avg:.0f} pts dropped/scan "
                f"(occupied/unknown/oob)"
            )


def main():
    rclpy.init()
    try:
        node = ScanMapFilter()
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
