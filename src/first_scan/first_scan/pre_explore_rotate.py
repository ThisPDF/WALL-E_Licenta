#!/usr/bin/env python3
"""
Pre-explore rotation node.

Inainte de a porni explore_lite, roteste robotul ~360 grade la fata locului.
De ce: explore_lite are nevoie de frontiere ca sa trimita goaluri. La start,
harta SLAM e mica (doar ce vede senzorul din pozitia initiala). Robotul ar
trebui sa se miste ca sa expand-eze harta, dar explore_lite renunta dupa
3-6s daca toate goalurile lui sunt blacklistate (toleranta hardcoded 0.25m
intre blacklist si frontiere noi).

Solutia: rotim manual 360° → slam_toolbox adauga noduri pentru fiecare unghi
(minimum_travel_heading: 0.2 rad = 11°) → harta se extinde la 360° in jurul
robotului → explore_lite gaseste frontiere consistente.

Publica pe /cmd_vel_nav (input-ul velocity_smoother), care merge prin chain-ul
oficial nav2 (smoother → collision_monitor → /cmd_vel → gazebo). Astfel nu
intra in conflict cu controller-ul (care e idle cat timp nu are goal).
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PreExploreRotate(Node):
    def __init__(self):
        super().__init__("pre_explore_rotate")

        self.declare_parameter("cmd_topic",      "/cmd_vel_nav")
        self.declare_parameter("angular_speed",  0.5)   # rad/s
        self.declare_parameter("target_radians", 6.5)   # >= 2*pi pentru rotire completa
        self.declare_parameter("publish_rate",   20.0)  # Hz
        self.declare_parameter("startup_delay",  2.0)   # secunde de wait initial

        self.cmd_topic     = str(self.get_parameter("cmd_topic").value)
        self.ang_speed     = float(self.get_parameter("angular_speed").value)
        self.target_rad    = float(self.get_parameter("target_radians").value)
        self.publish_rate  = float(self.get_parameter("publish_rate").value)
        self.startup_delay = float(self.get_parameter("startup_delay").value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.t_start: float = None
        self.t_rotate_start: float = None
        self.duration = self.target_rad / max(0.05, self.ang_speed)

        self.create_timer(1.0 / self.publish_rate, self._tick)

        self.get_logger().info(
            f"[pre_rotate] topic={self.cmd_topic} ang={self.ang_speed}rad/s "
            f"target={math.degrees(self.target_rad):.0f}deg "
            f"durata~{self.duration:.1f}s (startup_delay={self.startup_delay}s)"
        )

    def _tick(self):
        now = time.time()
        if self.t_start is None:
            self.t_start = now
            return

        # Asteapta sa fie ready chain-ul cmd_vel
        if (now - self.t_start) < self.startup_delay:
            return

        if self.t_rotate_start is None:
            self.t_rotate_start = now
            self.get_logger().info("[pre_rotate] START rotatie")

        elapsed = now - self.t_rotate_start
        if elapsed >= self.duration:
            # STOP + iesire (ies cu cod 0 → launch event va declansa explore_lite)
            stop = Twist()
            self.pub.publish(stop)
            self.get_logger().info(
                f"[pre_rotate] DONE dupa {elapsed:.1f}s. Ies."
            )
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        t = Twist()
        t.angular.z = self.ang_speed
        self.pub.publish(t)


def main():
    rclpy.init()
    try:
        node = PreExploreRotate()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # STOP la iesire
            pub = node.create_publisher(Twist, node.cmd_topic, 1)
            pub.publish(Twist())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
