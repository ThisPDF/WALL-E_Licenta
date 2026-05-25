#!/usr/bin/env python3
"""
Autonomous map explorer for initial SLAM mapping.

Algoritm simplu: drive forward, avoid obstacles by rotating to clear direction,
after `explore_duration` seconds → call slam_toolbox serialize_map service to
save the map, then exit. After exit, bringup.launch.py auto-starts the
follower nodes via OnProcessExit event handler.

Conventie cmd_vel: cmd.linear.x = -lin (negat, ca-n delivery_manager si
follower_node — fata robotului e in directia opusa lui base_link +X).
"""
import math
import os
import random
import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class State(Enum):
    INIT_ROTATE = 1
    DRIVE       = 2
    AVOID       = 3
    SAVE        = 4
    DONE        = 5


class MapExplorer(Node):
    def __init__(self):
        super().__init__("map_explorer")

        self.declare_parameter("cmd_vel_topic",    "/cmd_vel")
        self.declare_parameter("scan_topic",       "/scan")
        self.declare_parameter("odom_topic",       "/odom")
        self.declare_parameter("map_save_path",    "~/maps/carucior_market")
        self.declare_parameter("explore_duration", 180.0)  # 3 min
        self.declare_parameter("forward_speed",    0.25)
        self.declare_parameter("turn_speed",       0.6)
        self.declare_parameter("safe_dist",        0.55)
        self.declare_parameter("clear_dist",       0.85)
        self.declare_parameter("init_spin_dur",    3.0)  # rotire initiala
        self.declare_parameter("avoid_min_time",   0.6)  # min time in AVOID
        self.declare_parameter("avoid_max_time",   2.5)  # max time in AVOID

        self.cmd_topic      = str(self.get_parameter("cmd_vel_topic").value)
        self.scan_topic     = str(self.get_parameter("scan_topic").value)
        self.odom_topic     = str(self.get_parameter("odom_topic").value)
        self.map_save_path  = os.path.expanduser(
            str(self.get_parameter("map_save_path").value)
        )
        self.explore_dur    = float(self.get_parameter("explore_duration").value)
        self.fwd_speed      = float(self.get_parameter("forward_speed").value)
        self.turn_speed     = float(self.get_parameter("turn_speed").value)
        self.safe_dist      = float(self.get_parameter("safe_dist").value)
        self.clear_dist     = float(self.get_parameter("clear_dist").value)
        self.init_spin_dur  = float(self.get_parameter("init_spin_dur").value)
        self.avoid_min_time = float(self.get_parameter("avoid_min_time").value)
        self.avoid_max_time = float(self.get_parameter("avoid_max_time").value)

        # Asigura ca exista directorul de salvare
        os.makedirs(os.path.dirname(self.map_save_path), exist_ok=True)

        # Stare
        self.state          = State.INIT_ROTATE
        self.state_start    = time.time()
        self.run_start      = time.time()
        self.turn_dir       = 1.0  # +1 = CCW, -1 = CW
        self.avoid_target   = 1.0  # how long to rotate in AVOID
        self.global_min     = float("inf")
        self._scan_seen     = False
        self._save_attempts = 0
        self._save_done     = False

        # ROS
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_sensor)
        self.create_subscription(Odometry,  self.odom_topic, self._on_odom, 10)
        self.create_timer(0.1, self._control)

        # Service client pentru salvare slam_toolbox
        from slam_toolbox.srv import SerializePoseGraph
        self._serialize_srv = SerializePoseGraph
        self.save_client = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map"
        )

        self.get_logger().info(
            f"MapExplorer pornit | save={self.map_save_path} | "
            f"durata={self.explore_dur:.0f}s | "
            f"fwd={self.fwd_speed:.2f}m/s | safe={self.safe_dist:.2f}m"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_scan(self, msg: LaserScan):
        # Min global peste TOATE razele valide (indep. de orientarea lidar)
        gm = float("inf")
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < msg.range_max and r > 0.1:
                if r < gm:
                    gm = r
        self.global_min = gm
        self._scan_seen = True

    def _on_odom(self, msg: Odometry):
        pass  # placeholder; could use odom for distance traveled

    # ── Comenzi ───────────────────────────────────────────────────────────

    def _publish(self, lin: float, ang: float):
        """Publica cmd_vel. Conventie: NEGAT (ca-n follower/delivery_manager)."""
        t = Twist()
        t.linear.x  = -lin
        t.angular.z = -ang
        self.cmd_pub.publish(t)

    def _stop(self):
        self._publish(0.0, 0.0)

    # ── Salvare harta ─────────────────────────────────────────────────────

    def _save_map(self) -> bool:
        if not self.save_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "/slam_toolbox/serialize_map nu raspunde inca, mai astept..."
            )
            return False

        req = self._serialize_srv.Request()
        req.filename = self.map_save_path
        self.get_logger().info(
            f"Apel /slam_toolbox/serialize_map filename={self.map_save_path}"
        )
        future = self.save_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.done():
            resp = future.result()
            ok = (resp is not None and getattr(resp, "result", -1) == 0)
            if ok:
                self.get_logger().info(
                    f"Harta SALVATA cu succes: {self.map_save_path}.posegraph + .data"
                )
            else:
                self.get_logger().error(
                    f"Salvare esuata, raspuns: {resp}"
                )
            return ok
        self.get_logger().error("Timeout salvare harta")
        return False

    # ── Control loop ──────────────────────────────────────────────────────

    def _control(self):
        if not self._scan_seen:
            self._stop()
            return

        now      = time.time()
        elapsed  = now - self.run_start
        in_state = now - self.state_start

        # ── Stop conditions ────────────────────────────────────────────────
        if self.state == State.DONE:
            self._stop()
            return

        if self.state == State.SAVE:
            self._stop()
            if not self._save_done:
                self._save_attempts += 1
                if self._save_map():
                    self._save_done = True
                    self.state = State.DONE
                    self.get_logger().info(
                        "MapExplorer DONE — opresc nodul. Bringup va porni follower-ul."
                    )
                    # Cer rclpy sa iasa din spin
                    rclpy.shutdown()
                elif self._save_attempts >= 5:
                    self.get_logger().error("Salvare esuata dupa 5 incercari")
                    self.state = State.DONE
                    rclpy.shutdown()
            return

        # ── Timeout total → save ───────────────────────────────────────────
        if elapsed >= self.explore_dur:
            self.get_logger().info(
                f"Explore durata atinsa ({elapsed:.1f}s). Salvez harta..."
            )
            self.state = State.SAVE
            self.state_start = now
            self._stop()
            return

        # ── INIT_ROTATE: rotire 360 la start pentru harta initiala ─────────
        if self.state == State.INIT_ROTATE:
            if in_state < self.init_spin_dur:
                self._publish(0.0, self.turn_speed)
            else:
                self.state = State.DRIVE
                self.state_start = now
                self.get_logger().info(
                    f"INIT_ROTATE done dupa {in_state:.1f}s → DRIVE"
                )
            return

        # ── DRIVE: forward, avoid daca apare obstacol ──────────────────────
        if self.state == State.DRIVE:
            if self.global_min < self.safe_dist:
                self.state = State.AVOID
                self.state_start = now
                # Alege directie aleatoare + timp aleator
                self.turn_dir = random.choice([-1.0, 1.0])
                self.avoid_target = random.uniform(
                    self.avoid_min_time, self.avoid_max_time
                )
                self._stop()
                self.get_logger().info(
                    f"OBSTACOL @ {self.global_min:.2f}m → AVOID "
                    f"dir={self.turn_dir:+.0f} timp={self.avoid_target:.1f}s"
                )
            else:
                self._publish(self.fwd_speed, 0.0)
            return

        # ── AVOID: rotire pana cale libera ─────────────────────────────────
        if self.state == State.AVOID:
            cleared = self.global_min >= self.clear_dist
            timed_out = in_state >= self.avoid_target
            if cleared and in_state >= self.avoid_min_time:
                self.state = State.DRIVE
                self.state_start = now
                self.get_logger().info(
                    f"AVOID cleared dupa {in_state:.1f}s "
                    f"(global_min={self.global_min:.2f}m)"
                )
                return
            if timed_out:
                # Inca obstacol → re-incearca cu directie inversa
                self.turn_dir *= -1.0
                self.avoid_target = random.uniform(
                    self.avoid_min_time, self.avoid_max_time
                )
                self.state_start = now
                self.get_logger().info(
                    f"AVOID timeout → schimb directie {self.turn_dir:+.0f}"
                )
            self._publish(0.0, self.turn_dir * self.turn_speed)
            return


def main():
    rclpy.init()
    try:
        node = MapExplorer()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors la iesire
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
