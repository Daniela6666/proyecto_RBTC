#!/usr/bin/env python3
"""Persona C (Santiago Chacón).

Reenvía /cmd_vel (salida de Nav2) al topic que escucha diff_drive_controller.

Con use_ros2_control:=true el controlador lee
/diff_drive_controller/cmd_vel_unstamped y no /cmd_vel (ver README raíz).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input', '/cmd_vel')
        self.declare_parameter('output', '/diff_drive_controller/cmd_vel_unstamped')
        out = self.get_parameter('output').value
        inp = self.get_parameter('input').value
        self.pub = self.create_publisher(Twist, out, 10)
        self.sub = self.create_subscription(Twist, inp, self.pub.publish, 10)
        self.get_logger().info(f'{inp} -> {out}')


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
