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
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose

TARGET_FILE = "/tmp/delivery_target.json"
STOP_FILE   = "/tmp/delivery_stop.json"

# Offset spawn din gazebo.launch.py: -x 0 -y -11.2 -Y 0
# IMPORTANT: cart-ul se spawneaza cu YAW=0 (gazebo.launch.py:138), nu 1.57.
# SLAM aliniaza map frame cu base_footprint → map +X = world +X, map +Y = world +Y
# (pura translatie, fara rotatie). Un SPAWN_YAW=pi/2 vechi rotea pozitia raportata
# cu 90° → robotul aparea rotit pe harta din app. Corect = 0.
SPAWN_X   = 0.0
SPAWN_Y   = -11.2
SPAWN_YAW = 0.0

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
        # Nav2 planeaza in frame `map`. Tinta vine in world; map = world + offset.
        # Validat (vezi uwb_pose_publisher): offset (0, 11.2), yaw 0.
        self.declare_parameter("goal_frame",              "map")
        self.declare_parameter("world_to_map_offset_x",   0.0)
        self.declare_parameter("world_to_map_offset_y",   11.2)
        self.declare_parameter("nav_action",              "navigate_to_pose")
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
        self.goal_frame       = str(self.get_parameter("goal_frame").value)
        self.map_off_x        = float(self.get_parameter("world_to_map_offset_x").value)
        self.map_off_y        = float(self.get_parameter("world_to_map_offset_y").value)
        self.nav_action       = str(self.get_parameter("nav_action").value)

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

        # Nav2 navigare prin actiune NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)
        self._nav_active     = False   # exista un goal in curs
        self._nav_succeeded  = False   # ultimul goal a reusit
        self._goal_handle    = None

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
        self._send_nav_goal(tx, ty)

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
        self._send_nav_goal(SPAWN_X, SPAWN_Y)

    # ── Nav2 ────────────────────────────────────────────────────────────

    def _send_nav_goal(self, world_x: float, world_y: float):
        """Trimite o tinta NavigateToPose la Nav2 (in frame `map`)."""
        # Reset starea ÎNAINTE de orice (nu mosteni rezultatul goal-ului anterior).
        self._nav_active    = True
        self._nav_succeeded = False

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self._log("[NAV2] ❌ action server indisponibil (Nav2 pornit?)", "error")
            self._nav_active = False
            return

        mx = world_x + self.map_off_x
        my = world_y + self.map_off_y

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = mx
        goal.pose.pose.position.y = my
        goal.pose.pose.orientation.w = 1.0   # yaw 0 in `map` (orientarea finala nu e critica)

        self._log(
            f"[NAV2] → goal map=({mx:.3f}, {my:.3f}) (world=({world_x:.3f}, {world_y:.3f}))",
            "info"
        )
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self._log(f"[NAV2] eroare send_goal: {e}", "error")
            self._nav_active = False
            return
        if not handle.accepted:
            self._log("[NAV2] ⚠️ goal RESPINS de Nav2", "warn")
            self._nav_active = False
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        # status 4 = SUCCEEDED (action_msgs/GoalStatus)
        try:
            status = future.result().status
        except Exception as e:
            self._log(f"[NAV2] eroare result: {e}", "error")
            status = -1
        self._nav_active = False
        self._goal_handle = None
        if status == 4:
            self._nav_succeeded = True
            self._log("[NAV2] ✅ goal atins", "info")
        else:
            self._nav_succeeded = False
            self._log(f"[NAV2] goal terminat fara succes (status={status})", "warn")

    # ── Control loop ───────────────────────────────────────────────────

    def control_loop(self):
        if self.state == "IDLE":
            pass

        elif self.state == "NAVIGATING_TO_USER":
            self._publish_status_to_server()
            if not self._nav_active:
                if self._nav_succeeded:
                    self._log(
                        f"[ARRIVAL] 🎯 Ajuns la user! "
                        f"world=({self.robot_x:.3f}, {self.robot_y:.3f})", "info")
                    self.state = "AT_USER"
                else:
                    self._log("[NAV2] navigare esuata → IDLE", "warn")
                    self.state    = "IDLE"
                    self.target_x = None
                    self.target_y = None

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
            self._publish_status_to_server()
            if not self._nav_active:
                if self._nav_succeeded:
                    self._log(
                        f"[ARRIVAL] 🏠 Înapoi la bază! "
                        f"world=({self.robot_x:.3f}, {self.robot_y:.3f})", "info")
                else:
                    self._log("[NAV2] return esuat → IDLE oricum", "warn")
                self.state    = "IDLE"
                self.target_x = None
                self.target_y = None

    # Navigarea se face acum prin Nav2 (vezi _send_nav_goal / _on_goal_result).
    # Controlul direct vechi (_navigate_to_target) a fost inlocuit: Nav2 publica
    # viteza pe /cmd_vel_nav -> cmd_vel_inverter -> /cmd_vel_gz -> Gazebo.

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
            # cmd_vel_topic:=/cmd_vel_nav → trece prin cmd_vel_inverter spre Gazebo
            # (Nav2 e idle in FOLLOWING, deci /cmd_vel_nav e liber pt follower).
            # use_sim_time:=true → ceas sincron cu Gazebo.
            self.human_follower_process = subprocess.Popen(
                ["ros2", "run", "human_follower", "human_follower",
                 "--ros-args",
                 "-p", "use_sim_time:=true",
                 "-p", "cmd_vel_topic:=/cmd_vel_nav"],
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