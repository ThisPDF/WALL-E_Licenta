#!/usr/bin/env python3
"""
uwb_pose_publisher — simulator UWB pentru pozitia oamenului.

Doua moduri de operare:
  1) waypoints (default pentru SIM cu actor Gazebo):
     Interpoleaza liniar pe sim_time intre waypoint-urile traiectoriei
     declarate in world.sdf (sau hardcodate in params). Determinist, sincronizat
     cu animatia Gazebo, nu depinde de bridge-ul gz (care PIERDE numele
     entitatilor cand converteste gz.msgs.Pose_V → tf2_msgs/TFMessage si in plus
     actorii cu animation script nu apar in /world/<w>/dynamic_pose/info).
  2) tf_topic (pentru tag UWB real sau model dinamic in gz):
     Asculta TFMessage pe input_tf_topic, filtreaza pe actor_name.

Optional adauga zgomot Gaussian + dropout pentru a mimica UWB real, si publica
PoseStamped pe /uwb_person_pose la rata fixa, in frame `map`.

In acest proiect map ≡ world (cart spawned la yaw=0, vezi gazebo.launch.py).
"""
import math
import random
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _lerp_angle(a0: float, a1: float, t: float) -> float:
    """Interpolare angulara pe drumul cel mai scurt."""
    delta = _normalize_angle(a1 - a0)
    return _normalize_angle(a0 + delta * t)


class UwbPosePublisher(Node):
    def __init__(self):
        super().__init__("uwb_pose_publisher")

        # mode: 'waypoints' (interp pe sim_time) | 'tf_topic' (subscribe TFMessage)
        self.declare_parameter("mode",                "waypoints")
        self.declare_parameter("output_topic",        "/uwb_person_pose")
        self.declare_parameter("output_frame",        "map")
        self.declare_parameter("publish_rate_hz",     10.0)

        # Zgomot Gaussian + dropout (simuleaza UWB real)
        self.declare_parameter("noise_sigma_xy",      0.05)
        self.declare_parameter("noise_sigma_yaw",     0.0)
        self.declare_parameter("dropout_probability", 0.0)

        # --- Mode 'waypoints' params ---
        # Lista de waypoint-uri ca string CSV: "t,x,y,yaw;t,x,y,yaw;..."
        # Default = scriptul actorului human_94763 din worlds/world.sdf:215-251.
        # Pastram defaultul ca string CSV (ROS2 declare_parameter nu accepta
        # list-of-list nativ; un sir CSV simplu e cea mai portabila reprezentare).
        self.declare_parameter(
            "waypoints_csv",
            "0.0,5.0,-10.6,0.0;"
            "21.0,12.5,-11.2,3.13;"
            "70.0,12.5,8.5,0.0;"
            "87.0,5.6,8.5,-0.0055;"
            "117.0,5.6,-3.19,-1.4998;"
            "152.0,-8.5,-4.2,0.0;"
            "165.0,-8.2,-9.2,0.0"
        )
        self.declare_parameter("loop_trajectory",     True)
        # Daca pornim follower-ul dupa ce sim time a depasit deja un ciclu, vrem
        # ca modulo sa redea corect. Si daca cineva foloseste wall_time (non-sim),
        # folosim sim_time daca e configurat global; altfel ROS time.
        self.declare_parameter("clock_offset_sec",    0.0)

        # World→Map offset. Waypoint-urile din SDF sunt in frame Gazebo "world",
        # dar slam_toolbox initialize "map" frame cu base_footprint la spawn.
        # Cart spawned la world (0, -11.2) → map (0, 0). Deci map_y = world_y + 11.2.
        # Verificat prin extents harta SLAM: world floor y∈[-12,12], map y∈[-0.784, 23.216]
        # → offset y ≈ +11.216 m. Cart spawn x = 0 → offset x = 0.
        # Daca cart-ul se respawneaza la alt loc in gazebo.launch.py, ajusta aici.
        self.declare_parameter("world_to_map_offset_x", 0.0)
        self.declare_parameter("world_to_map_offset_y", 11.2)
        self.declare_parameter("world_to_map_offset_yaw", 0.0)

        # --- Mode 'tf_topic' params ---
        self.declare_parameter("actor_name",      "human_94763")
        self.declare_parameter("input_tf_topic",  "/gz/person_tf")

        self.declare_parameter("stale_warn_sec",  2.0)

        self.mode             = str(self.get_parameter("mode").value)
        out_topic             = str(self.get_parameter("output_topic").value)
        self.output_frame     = str(self.get_parameter("output_frame").value)
        rate_hz               = float(self.get_parameter("publish_rate_hz").value)
        self.sigma_xy         = float(self.get_parameter("noise_sigma_xy").value)
        self.sigma_yaw        = float(self.get_parameter("noise_sigma_yaw").value)
        self.dropout          = float(self.get_parameter("dropout_probability").value)
        self.loop_traj        = bool(self.get_parameter("loop_trajectory").value)
        self.clock_offset_sec = float(self.get_parameter("clock_offset_sec").value)
        self.stale_warn_sec   = float(self.get_parameter("stale_warn_sec").value)
        self.world_to_map_offset_x   = float(self.get_parameter("world_to_map_offset_x").value)
        self.world_to_map_offset_y   = float(self.get_parameter("world_to_map_offset_y").value)
        self.world_to_map_offset_yaw = float(self.get_parameter("world_to_map_offset_yaw").value)

        self._latest_xyz_yaw: Optional[Tuple[float, float, float, float]] = None
        self._last_warn_t = 0.0
        self._publish_count = 0
        self._rng = random.Random(0xC0FFEE)

        # Diagnostics for tf_topic mode
        self._msg_count = 0
        self._seen_frames: set = set()

        self._pub = self.create_publisher(PoseStamped, out_topic, 10)

        if self.mode == "waypoints":
            self.waypoints = self._parse_waypoints_csv(
                str(self.get_parameter("waypoints_csv").value)
            )
            if not self.waypoints:
                self.get_logger().error(
                    "mode=waypoints dar waypoints_csv e vid/invalid — UWB nu va publica"
                )
                self.total_dur = 0.0
            else:
                self.total_dur = self.waypoints[-1][0]
                self.get_logger().info(
                    f"uwb_pose_publisher [mode=waypoints]: "
                    f"{len(self.waypoints)} waypoints, "
                    f"duration={self.total_dur:.1f}s, loop={self.loop_traj}, "
                    f"out={out_topic} frame={self.output_frame} "
                    f"sigma_xy={self.sigma_xy:.3f}m dropout={self.dropout:.2f}"
                )
            self.create_timer(1.0 / max(1.0, rate_hz), self._on_timer_waypoints)
        else:
            self.actor_name = str(self.get_parameter("actor_name").value)
            in_topic = str(self.get_parameter("input_tf_topic").value)
            self._sub = self.create_subscription(TFMessage, in_topic, self._on_tf, 50)
            self.create_timer(1.0 / max(1.0, rate_hz), self._on_timer_tf)
            self.get_logger().info(
                f"uwb_pose_publisher [mode=tf_topic]: actor='{self.actor_name}' "
                f"{in_topic} -> {out_topic} (frame={self.output_frame}) "
                f"rate={rate_hz:.1f}Hz sigma_xy={self.sigma_xy:.3f}m "
                f"dropout={self.dropout:.2f}"
            )

    # ───────────────────────────────────────────────────────────────────────
    @staticmethod
    def _parse_waypoints_csv(csv: str) -> List[Tuple[float, float, float, float]]:
        out: List[Tuple[float, float, float, float]] = []
        for chunk in csv.split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            parts = [p.strip() for p in chunk.split(",")]
            if len(parts) != 4:
                continue
            try:
                t, x, y, yaw = (float(p) for p in parts)
                out.append((t, x, y, yaw))
            except ValueError:
                continue
        out.sort(key=lambda w: w[0])
        return out

    def _pose_at_sim_time(self, t_sim: float) -> Tuple[float, float, float]:
        """Interp liniara pe X/Y si pe drumul cel mai scurt pe yaw."""
        if not self.waypoints:
            return (0.0, 0.0, 0.0)
        if self.loop_traj and self.total_dur > 0.0:
            t_eff = t_sim % self.total_dur
        else:
            t_eff = max(0.0, min(t_sim, self.total_dur))

        # cazul margine: inainte de primul / dupa ultimul waypoint
        if t_eff <= self.waypoints[0][0]:
            _, x, y, yaw = self.waypoints[0]
            return (x, y, yaw)
        if t_eff >= self.waypoints[-1][0]:
            _, x, y, yaw = self.waypoints[-1]
            return (x, y, yaw)

        # caut segmentul
        for i in range(len(self.waypoints) - 1):
            t0, x0, y0, yaw0 = self.waypoints[i]
            t1, x1, y1, yaw1 = self.waypoints[i + 1]
            if t0 <= t_eff <= t1:
                alpha = (t_eff - t0) / max(1e-6, (t1 - t0))
                x = x0 + alpha * (x1 - x0)
                y = y0 + alpha * (y1 - y0)
                yaw = _lerp_angle(yaw0, yaw1, alpha)
                return (x, y, yaw)
        # fallback (n-ar trebui atins)
        _, x, y, yaw = self.waypoints[-1]
        return (x, y, yaw)

    def _on_timer_waypoints(self):
        if not self.waypoints:
            return

        now_msg = self.get_clock().now().to_msg()
        t_sim = float(now_msg.sec) + 1e-9 * float(now_msg.nanosec) + self.clock_offset_sec
        # x,y,yaw sunt in frame "world" Gazebo. Aplicam offset world→map ca
        # PoseStamped sa fie corect in frame slam_toolbox "map".
        x_w, y_w, yaw_w = self._pose_at_sim_time(t_sim)
        cos_o = math.cos(self.world_to_map_offset_yaw)
        sin_o = math.sin(self.world_to_map_offset_yaw)
        x_m = self.world_to_map_offset_x + cos_o * x_w - sin_o * y_w
        y_m = self.world_to_map_offset_y + sin_o * x_w + cos_o * y_w
        yaw_m = _normalize_angle(yaw_w + self.world_to_map_offset_yaw)
        self._publish(now_msg, x_m, y_m, 0.0, yaw_m)

    # ───────────────────────────────────────────────────────────────────────
    # Mode 'tf_topic' (pastrat ca fallback pentru tag UWB real)
    # ───────────────────────────────────────────────────────────────────────
    def _on_tf(self, msg: TFMessage):
        self._msg_count += 1
        target = self.actor_name
        if self._msg_count <= 30:
            for tf in msg.transforms:
                self._seen_frames.add(tf.child_frame_id)
        for tf in msg.transforms:
            cf = tf.child_frame_id
            if (cf == target
                    or cf.endswith("::" + target)
                    or cf.endswith("/" + target)
                    or cf.startswith(target + "::")
                    or cf.startswith(target + "/")
                    or cf.startswith(target + "_")):
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                self._latest_xyz_yaw = (float(t.x), float(t.y), float(t.z), float(yaw))
                return

    def _on_timer_tf(self):
        if self._latest_xyz_yaw is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_warn_t > self.stale_warn_sec:
                self.get_logger().warn(
                    f"[UWB tf_topic] {self._msg_count} msg vazute, niciun match pe "
                    f"actor='{self.actor_name}'. seen={sorted(self._seen_frames)[:10]}"
                )
                self._last_warn_t = now
            return
        now_msg = self.get_clock().now().to_msg()
        x, y, z, yaw = self._latest_xyz_yaw
        self._publish(now_msg, x, y, z, yaw)

    # ───────────────────────────────────────────────────────────────────────
    def _publish(self, stamp_msg, x: float, y: float, z: float, yaw: float):
        if self.dropout > 0.0 and self._rng.random() < self.dropout:
            return
        if self.sigma_xy > 0.0:
            x += self._rng.gauss(0.0, self.sigma_xy)
            y += self._rng.gauss(0.0, self.sigma_xy)
        if self.sigma_yaw > 0.0:
            yaw += self._rng.gauss(0.0, self.sigma_yaw)

        msg = PoseStamped()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.output_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        half = 0.5 * yaw
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self._pub.publish(msg)

        self._publish_count += 1
        if self._publish_count == 1 or self._publish_count % 100 == 0:
            self.get_logger().info(
                f"[UWB] published #{self._publish_count} "
                f"x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg "
                f"[mode={self.mode}]"
            )


def main():
    rclpy.init()
    try:
        node = UwbPosePublisher()
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
