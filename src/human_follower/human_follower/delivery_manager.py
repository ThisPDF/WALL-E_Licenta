#!/usr/bin/env python3
"""
delivery_manager.py
Flow: IDLE → NAVIGATING_TO_USER → AT_USER → FOLLOWING → RETURNING_TO_BASE → IDLE
Coordonate: world frame (offset spawn aplicat automat)
Semn cmd_vel: conventie standard ROS — linear.x>0 = inainte (base_footprint +X),
angular.z>0 = CCW/stanga. odom raporteaza base_footprint, deci NU se negheaza nimic.
"""
import math
import time
import logging
import logging.handlers
import os
import subprocess
import json
import threading
import urllib.request
import urllib.error
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

TARGET_FILE = "/tmp/delivery_target.json"
STOP_FILE   = "/tmp/delivery_stop.json"

# Offset spawn din launch file: -x 0 -y -11.2 -Y 1.57
SPAWN_X   = 0.0
SPAWN_Y   = -11.2
SPAWN_YAW = math.pi / 2  # 1.57 rad

# Fata robotului = base_footprint +X (conventie standard ROS, confirmat teleop).
# odom raporteaza pozitia/orientarea lui base_footprint, deci robot_yaw e deja
# yaw-ul "forward" in world frame (odom_yaw + SPAWN_YAW) — fara offset suplimentar.


class DeliveryManager(Node):
    def __init__(self):
        super().__init__("delivery_manager")

        self.declare_parameter("cmd_vel_topic",           "/cmd_vel")
        self.declare_parameter("odom_topic",              "/odom")
        self.declare_parameter("scan_topic",              "/scan")
        self.declare_parameter("delivery_server_url",     "http://localhost:8080")
        self.declare_parameter("max_linear",              0.5)
        self.declare_parameter("max_angular",             1.2)
        self.declare_parameter("k_lin",                   1.5)
        self.declare_parameter("k_ang",                   2.5)
        self.declare_parameter("arrival_distance",        0.6)
        self.declare_parameter("robot_id",                "robot_001")
        self.declare_parameter("status_publish_interval", 1.0)
        self.declare_parameter("log_file",                "~/delivery_logs/manager.log")
        self.declare_parameter("log_file_max_bytes",      5_000_000)
        self.declare_parameter("log_file_backup_count",   3)

        self.cmd_vel_topic    = str(self.get_parameter("cmd_vel_topic").value)
        self.odom_topic       = str(self.get_parameter("odom_topic").value)
        self.scan_topic       = str(self.get_parameter("scan_topic").value)
        self.server_url       = str(self.get_parameter("delivery_server_url").value)
        self.max_linear       = float(self.get_parameter("max_linear").value)
        self.max_angular      = float(self.get_parameter("max_angular").value)
        self.k_lin            = float(self.get_parameter("k_lin").value)
        self.k_ang            = float(self.get_parameter("k_ang").value)
        self.arrival_distance = float(self.get_parameter("arrival_distance").value)
        self.robot_id         = str(self.get_parameter("robot_id").value)
        self.status_interval  = float(self.get_parameter("status_publish_interval").value)

        # Logging
        log_path = os.path.expanduser(str(self.get_parameter("log_file").value))
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        self._flog = logging.getLogger("delivery_manager")
        self._flog.setLevel(logging.DEBUG)
        self._flog.propagate = False
        if not self._flog.handlers:
            h = logging.handlers.RotatingFileHandler(
                log_path,
                maxBytes=int(self.get_parameter("log_file_max_bytes").value),
                backupCount=int(self.get_parameter("log_file_backup_count").value),
                encoding="utf-8",
            )
            h.setFormatter(logging.Formatter(
                fmt="%(asctime)s.%(msecs)03d  %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            ))
            self._flog.addHandler(h)

        # State
        self.state    = "IDLE"
        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None

        # Poziția în world frame
        self.robot_x   = SPAWN_X
        self.robot_y   = SPAWN_Y
        self.robot_yaw = SPAWN_YAW

        self.human_follower_process = None
        self.last_status_update     = 0.0
        self.last_target_timestamp  = 0
        self.last_stop_timestamp    = 0
        self._last_odom_log         = 0.0
        self._last_nav_log          = 0.0

        # ROS
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry,  self.odom_topic, self.on_odom, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.5, self.poll_files)

        self._log(
            f"DeliveryManager ready | server={self.server_url} | "
            f"spawn=({SPAWN_X}, {SPAWN_Y}) yaw={math.degrees(SPAWN_YAW):.1f}°",
            "info"
        )

    def _log(self, msg, level="info"):
        getattr(self._flog, level)(msg)
        if level == "warn":
            self.get_logger().warn(msg)
        elif level == "error":
            self.get_logger().error(msg)
        else:
            self.get_logger().info(msg)

    # ── Odometry ───────────────────────────────────────────────────────

    def on_odom(self, msg: Odometry):
        odom_x = float(msg.pose.pose.position.x)
        odom_y = float(msg.pose.pose.position.y)

        # Transformare odom → world frame
        cos_s = math.cos(SPAWN_YAW)
        sin_s = math.sin(SPAWN_YAW)
        self.robot_x = SPAWN_X + cos_s * odom_x - sin_s * odom_y
        self.robot_y = SPAWN_Y + sin_s * odom_x + cos_s * odom_y

        # Yaw în world frame
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny, cosy)
        self.robot_yaw = raw_yaw + SPAWN_YAW
        while self.robot_yaw >  math.pi: self.robot_yaw -= 2 * math.pi
        while self.robot_yaw < -math.pi: self.robot_yaw += 2 * math.pi

        now = time.time()
        if (now - self._last_odom_log) > 3.0:
            self._last_odom_log = now
            self._flog.debug(
                f"[ODOM] world=({self.robot_x:.3f}, {self.robot_y:.3f}) "
                f"odom=({odom_x:.3f}, {odom_y:.3f}) "
                f"yaw_world={math.degrees(self.robot_yaw):.1f}°"
            )

    def on_scan(self, msg: LaserScan):
        pass

    # ── File polling ───────────────────────────────────────────────────

    def poll_files(self):
        self._poll_target_file()
        self._poll_stop_file()

    def _poll_target_file(self):
        try:
            if not os.path.exists(TARGET_FILE):
                return
            with open(TARGET_FILE, "r") as f:
                data = json.load(f)

            ts       = data.get("timestamp", 0)
            consumed = data.get("consumed", False)
            if consumed or ts == self.last_target_timestamp:
                return

            self.last_target_timestamp = ts
            tx = float(data.get("x", 0.0))
            ty = float(data.get("y", 0.0))

            data["consumed"] = True
            with open(TARGET_FILE, "w") as f:
                json.dump(data, f)

            self._log(
                f"[FILE] 📍 Target world=({tx:.3f}, {ty:.3f}) "
                f"| robot world=({self.robot_x:.3f}, {self.robot_y:.3f}) "
                f"| dist={math.hypot(tx - self.robot_x, ty - self.robot_y):.3f}m",
                "info"
            )
            self._start_navigation(tx, ty)

        except json.JSONDecodeError:
            pass
        except Exception as e:
            self._flog.debug(f"[FILE] target error: {e}")

    def _poll_stop_file(self):
        try:
            if not os.path.exists(STOP_FILE):
                return
            with open(STOP_FILE, "r") as f:
                data = json.load(f)

            ts       = data.get("timestamp", 0)
            consumed = data.get("consumed", False)
            if consumed or ts == self.last_stop_timestamp:
                return

            self.last_stop_timestamp = ts
            data["consumed"] = True
            with open(STOP_FILE, "w") as f:
                json.dump(data, f)

            self._log("[FILE] 🛑 Stop command — returning to base", "info")
            self._return_to_base()

        except json.JSONDecodeError:
            pass
        except Exception as e:
            self._flog.debug(f"[FILE] stop error: {e}")

    def _start_navigation(self, tx: float, ty: float):
        if self.human_follower_process is not None:
            self._stop_human_follower()
        self.state    = "NAVIGATING_TO_USER"
        self.target_x = tx
        self.target_y = ty
        self._log(
            f"[STATE] NAVIGATING_TO_USER → world=({tx:.3f}, {ty:.3f})",
            "info"
        )

    def _return_to_base(self):
        if self.human_follower_process is not None:
            self._stop_human_follower()
        self.state    = "RETURNING_TO_BASE"
        self.target_x = SPAWN_X
        self.target_y = SPAWN_Y
        self._log(
            f"[STATE] RETURNING_TO_BASE → world=({SPAWN_X}, {SPAWN_Y})",
            "info"
        )

    # ── Control loop ───────────────────────────────────────────────────

    def control_loop(self):
        if self.state == "IDLE":
            pass

        elif self.state == "NAVIGATING_TO_USER":
            self._navigate_to_target()
            self._publish_status_to_server()

        elif self.state == "AT_USER":
            if self.human_follower_process is None:
                self._start_human_follower()
            self.state = "FOLLOWING"
            self._publish_status_to_server()

        elif self.state == "FOLLOWING":
            self._publish_status_to_server()
            if self.human_follower_process is not None:
                if self.human_follower_process.poll() is not None:
                    self._log("[ERROR] human_follower crashed", "error")
                    self.human_follower_process = None

        elif self.state == "RETURNING_TO_BASE":
            self._navigate_to_target()
            self._publish_status_to_server()

    # ── Navigation ─────────────────────────────────────────────────────

    def _navigate_to_target(self):
        if self.target_x is None or self.target_y is None:
            self.cmd_pub.publish(Twist())
            return

        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance = math.hypot(dx, dy)

        # Log la fiecare secundă
        now = time.time()
        if (now - self._last_nav_log) > 1.0:
            self._last_nav_log = now
            angle_to_target = math.atan2(dy, dx)
            # Aceeasi eroare ca in control (jos): bearing - yaw, fara offset.
            angle_error = angle_to_target - self.robot_yaw
            while angle_error >  math.pi: angle_error -= 2 * math.pi
            while angle_error < -math.pi: angle_error += 2 * math.pi
            self.get_logger().info(
                f"[NAV] world=({self.robot_x:.3f}, {self.robot_y:.3f}) "
                f"→ ({self.target_x:.3f}, {self.target_y:.3f}) "
                f"dist={distance:.3f}m "
                f"yaw={math.degrees(self.robot_yaw):.1f}° "
                f"err={math.degrees(angle_error):.1f}°"
            )

        if distance < self.arrival_distance:
            self.cmd_pub.publish(Twist())

            if self.state == "NAVIGATING_TO_USER":
                self._log(
                    f"[ARRIVAL] 🎯 Ajuns la user! "
                    f"world=({self.robot_x:.3f}, {self.robot_y:.3f}) "
                    f"dist_finala={distance:.3f}m",
                    "info"
                )
                self.state = "AT_USER"

            elif self.state == "RETURNING_TO_BASE":
                self._log(
                    f"[ARRIVAL] 🏠 Înapoi la bază! "
                    f"world=({self.robot_x:.3f}, {self.robot_y:.3f})",
                    "info"
                )
                self.state    = "IDLE"
                self.target_x = None
                self.target_y = None
            return

        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.robot_yaw
        while angle_error >  math.pi: angle_error -= 2 * math.pi
        while angle_error < -math.pi: angle_error += 2 * math.pi

        if abs(angle_error) > 1.0:
            lin_speed = 0.0
        else:
            lin_speed = min(self.max_linear, self.k_lin * distance)
            if abs(angle_error) > 0.3:
                lin_speed *= 0.5

        ang_speed = max(-self.max_angular, min(self.max_angular, self.k_ang * angle_error))

        twist = Twist()
        # Conventie standard ROS (dupa fix URDF + flip wheel axes):
        #   linear.x  > 0 → forward, angular.z > 0 → CCW
        # NU mai inversam (era hack pentru URDF original gresit)
        twist.linear.x  = lin_speed
        twist.angular.z = ang_speed
        self.cmd_pub.publish(twist)

    # ── Server communication ───────────────────────────────────────────

    def _publish_status_to_server(self):
        now = time.time()
        if (now - self.last_status_update) < self.status_interval:
            return
        self.last_status_update = now

        payload = {
            "robotId": self.robot_id,
            "x":       round(self.robot_x, 3),
            "y":       round(self.robot_y, 3),
            "state":   self._map_state(self.state),
        }
        threading.Thread(
            target=self._send_robot_status,
            args=(payload,),
            daemon=True
        ).start()

    def _map_state(self, state: str) -> str:
        return {
            "IDLE":               "IDLE",
            "NAVIGATING_TO_USER": "APPROACH",
            "AT_USER":            "WAITING",
            "FOLLOWING":          "WAITING",
            "RETURNING_TO_BASE":  "RETURNING",
        }.get(state, "IDLE")

    def _send_robot_status(self, payload: dict):
        try:
            url  = f"{self.server_url}/api/robot/status"
            body = json.dumps(payload).encode("utf-8")
            req  = urllib.request.Request(
                url, data=body,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=2.0) as resp:
                self._flog.debug(f"[SERVER] ✅ {payload} → {resp.status}")
        except urllib.error.URLError as e:
            self._flog.debug(f"[SERVER] ⚠️ {e.reason}")
        except Exception as e:
            self._flog.debug(f"[SERVER] ⚠️ {e}")

    # ── Human follower ─────────────────────────────────────────────────

    def _start_human_follower(self):
        try:
            self._log("[LAUNCHER] 🚀 Starting human_follower", "info")
            self.human_follower_process = subprocess.Popen(
                ["ros2", "run", "human_follower", "human_follower"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            threading.Thread(
                target=self._read_stderr,
                args=(self.human_follower_process,),
                daemon=True
            ).start()
        except Exception as e:
            self._log(f"[ERROR] {e}", "error")

    def _read_stderr(self, process):
        try:
            for line in process.stderr:
                decoded = line.decode("utf-8", errors="replace").strip()
                if decoded:
                    self._log(f"[human_follower] {decoded}", "error")
        except Exception:
            pass

    def _stop_human_follower(self):
        if self.human_follower_process is None:
            return
        try:
            self._log("[LAUNCHER] 🛑 Stopping human_follower", "info")
            self.human_follower_process.terminate()
            try:
                self.human_follower_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.human_follower_process.kill()
                self.human_follower_process.wait()
            self.human_follower_process = None
        except Exception as e:
            self._log(f"[ERROR] stop: {e}", "error")


def main():
    rclpy.init()
    node = DeliveryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.human_follower_process is not None:
            node._stop_human_follower()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()