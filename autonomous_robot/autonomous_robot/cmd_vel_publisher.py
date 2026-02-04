#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.declare_parameter('linear_x', 0.2)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('rate_hz', 10.0)

        rate = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd_vel)

        self.get_logger().info(
            'Publishing /cmd_vel. Params: linear_x, angular_z, rate_hz'
        )

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = float(self.get_parameter('linear_x').value)
        msg.angular.z = float(self.get_parameter('angular_z').value)

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'/cmd_vel -> linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
