#!/usr/bin/env python3
"""
cmd_vel_inverter.

Adapter intre publisherii de cmd_vel (nav2, follower, delivery_manager) si
bridge-ul Gazebo. Inverseaza componentele necesare pentru a compensa
diferenta intre conventia ROS standard (+X = forward in base_footprint) si
fizica plugin-ului diff_drive (care misca robotul in base_link +Y pentru
positiv cmd_vel.linear.x — convenție originala a URDF-ului dupa rotația
base_footprint→base_link).

Conventie:
  /cmd_vel       (input)  — publishers ROS standard (nav2, follower etc)
  /cmd_vel_gz    (output) — topic-ul caruia ii subscribe gazebo bridge

Inverter face:
  out.linear.x  = -in.linear.x   (compensare rotatie base_link)
  out.angular.z = +in.angular.z  (nemodificat — angular e corect deja)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelInverter(Node):
    def __init__(self):
        super().__init__("cmd_vel_inverter")

        self.declare_parameter("input_topic",   "/cmd_vel")
        self.declare_parameter("output_topic",  "/cmd_vel_gz")
        self.declare_parameter("invert_linear_x",  True)
        self.declare_parameter("invert_linear_y",  False)
        self.declare_parameter("invert_angular_z", False)

        in_topic  = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self.inv_lx = bool(self.get_parameter("invert_linear_x").value)
        self.inv_ly = bool(self.get_parameter("invert_linear_y").value)
        self.inv_az = bool(self.get_parameter("invert_angular_z").value)

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self._on_cmd, 10)

        self.get_logger().info(
            f"cmd_vel_inverter {in_topic} → {out_topic} | "
            f"invert: lx={self.inv_lx} ly={self.inv_ly} az={self.inv_az}"
        )

    def _on_cmd(self, msg: Twist):
        out = Twist()
        out.linear.x  = -msg.linear.x  if self.inv_lx else msg.linear.x
        out.linear.y  = -msg.linear.y  if self.inv_ly else msg.linear.y
        out.linear.z  = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = -msg.angular.z if self.inv_az else msg.angular.z
        self.pub.publish(out)


def main():
    rclpy.init()
    try:
        node = CmdVelInverter()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
