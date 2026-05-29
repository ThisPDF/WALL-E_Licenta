#!/usr/bin/env python3
"""
QoS relay + self-filter pentru /scan.

Publica DOUA topice din /scan (BestEffort de la gz bridge):
  /scan_for_slam  (Reliable, NEFILTRAT) → pentru slam_toolbox.
                  SLAM are nevoie de toate scanurile (inclusiv reflexii
                  de pe rama caruciorului) pentru scan-matching robust;
                  cart-ul apare mereu la aceleasi unghiuri relative, deci
                  e o "trasatura constanta" care nu strica matching-ul.
  /scan_filtered  (Reliable, FILTRAT)   → pentru costmaps + collision_monitor.
                  Cu rama caruciorului eliminata, costmap-ul nu mai are
                  obstacole-fantoma in interiorul robotului, deci nici
                  RPP-ul, nici collision_monitor.FootprintApproach nu
                  mai blocheaza cmd_vel.

Filtru: orice raza cu endpoint-ul in interiorul unui cerc de raza
`self_filter_radius` in jurul originii base_footprint (0,0) este
inlocuita cu inf. Raza implicita 0.75m acopera intregul cart (wheelbase
~0.85x0.66) PLUS LCD-ul ecran (la ~0.68m de base_footprint).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanQoSRelay(Node):
    def __init__(self):
        super().__init__("scan_qos_relay")

        self.declare_parameter("input_topic",       "/scan")
        self.declare_parameter("slam_topic",        "/scan_for_slam")
        self.declare_parameter("filtered_topic",    "/scan_filtered")
        # Filtru pe RANGE (NU pe regiune). Orice filtru pe regiune (cerc/box)
        # creeaza o zona OARBA directionala in care robotul nu vede obstacole
        # reale → coliziuni → stuck → odom desync → harta duplicat.
        # Piesele cart-ului la inaltimea LIDAR (z≈1.07m) sunt TOATE foarte
        # aproape de LIDAR: LCD 0.18m, camera 0.14m, montura ~0m. Filtrand
        # doar return-urile sub self_min_range (0.30m de LIDAR), eliminam
        # self-ul DAR vedem obstacolele reale (mereu >0.30m de LIDAR) in TOATE
        # directiile → robotul nu mai e orb → nu se mai izbeste.
        self.declare_parameter("self_min_range", 0.30)

        in_topic      = str(self.get_parameter("input_topic").value)
        slam_topic    = str(self.get_parameter("slam_topic").value)
        filt_topic    = str(self.get_parameter("filtered_topic").value)

        self._self_min_range = float(self.get_parameter("self_min_range").value)

        in_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_slam = self.create_publisher(LaserScan, slam_topic, out_qos)
        self.pub_filt = self.create_publisher(LaserScan, filt_topic, out_qos)
        self.sub      = self.create_subscription(LaserScan, in_topic, self._on_scan, in_qos)

        self._count = 0
        self._filtered_total = 0
        self.get_logger().info(
            f"scan_qos_relay {in_topic} (BE) -> {slam_topic} (raw) + "
            f"{filt_topic} (self-filter: range < {self._self_min_range:.2f}m)"
        )

    def _on_scan(self, msg: LaserScan):
        # SLAM primeste scanul brut (cu QoS convertit) — neatins.
        self.pub_slam.publish(msg)

        # Filtreaza pentru costmaps + collision_monitor: elimina return-urile
        # foarte apropiate (< self_min_range) = piesele cart-ului langa LIDAR.
        # NU foloseste regiune → fara zona oarba directionala.
        self_min = self._self_min_range
        r_max = msg.range_max

        new_ranges = list(msg.ranges)
        n_filtered = 0
        inf = float("inf")

        for i, r in enumerate(new_ranges):
            if math.isfinite(r) and r < self_min:
                new_ranges[i] = inf
                n_filtered += 1

        # Construim un mesaj NOU cu ranges filtrate (msg.ranges din mesajul
        # publicat la pub_slam ramane astfel intact pentru subscribers care
        # primesc referinta).
        filt_msg = LaserScan()
        filt_msg.header = msg.header
        filt_msg.angle_min = msg.angle_min
        filt_msg.angle_max = msg.angle_max
        filt_msg.angle_increment = msg.angle_increment
        filt_msg.time_increment = msg.time_increment
        filt_msg.scan_time = msg.scan_time
        filt_msg.range_min = msg.range_min
        filt_msg.range_max = msg.range_max
        filt_msg.ranges = new_ranges
        filt_msg.intensities = msg.intensities
        self.pub_filt.publish(filt_msg)

        self._count += 1
        self._filtered_total += n_filtered
        if self._count == 1 or self._count % 100 == 0:
            avg = self._filtered_total / self._count
            self.get_logger().info(
                f"Relayed {self._count} scans | avg {avg:.0f} self-pts filtered/scan"
            )


def main():
    rclpy.init()
    try:
        node = ScanQoSRelay()
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
