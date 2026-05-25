#!/usr/bin/env python3
"""
QoS relay pentru /scan.

Problema: parameter_bridge publica /scan cu QoS Reliable (default),
slam_toolbox subscrie default SensorDataQoS (BestEffort). In practica
qos_overrides nu se aplica intotdeauna corect, iar scanurile NU ajung
la slam_toolbox → nu se genereaza harta.

Solutia: acest nod subscrie /scan cu BestEffort (compatibil cu orice
publisher), re-publica /scan_for_slam cu QoS Reliable explicita. SLAM
subscrie la /scan_for_slam.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanQoSRelay(Node):
    def __init__(self):
        super().__init__("scan_qos_relay")

        self.declare_parameter("input_topic",  "/scan")
        self.declare_parameter("output_topic", "/scan_for_slam")

        in_topic  = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        # BestEffort accepta atat publisheri Reliable cat si BestEffort
        in_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Reliable + KeepLast(5) — match cu default-ul slam_toolbox
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(LaserScan, out_topic, out_qos)
        self.sub = self.create_subscription(LaserScan, in_topic, self._on_scan, in_qos)

        self._count = 0
        self.get_logger().info(
            f"scan_qos_relay {in_topic} (BestEffort) -> {out_topic} (Reliable)"
        )

    def _on_scan(self, msg: LaserScan):
        self.pub.publish(msg)
        self._count += 1
        if self._count == 1 or self._count % 100 == 0:
            self.get_logger().info(f"Relayed {self._count} scans")


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
