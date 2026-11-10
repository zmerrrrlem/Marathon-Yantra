#!/usr/bin/env python3
"""Publish ramped /cmd_vel from start speed to max speed."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SpeedRamp(Node):
    def __init__(self):
        super().__init__('speed_ramp')
        self.declare_parameter('start_speed', 0.02)
        self.declare_parameter('max_speed', 0.05)
        self.declare_parameter('ramp_rate', 0.001)  # m/s per second
        self.declare_parameter('publish_rate', 10.0)
        self.speed = self.get_parameter('start_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.rate = self.get_parameter('ramp_rate').value
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        period = 1.0 / max(1.0, self.get_parameter('publish_rate').value)
        self.create_timer(period, self.timer_cb)
        self.last_time = self.get_clock().now()

    def timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if self.speed < self.max_speed:
            self.speed = min(self.max_speed, self.speed + self.rate * dt)
        msg = Twist()
        msg.linear.x = float(self.speed)
        self.pub.publish(msg)
        self.get_logger().info(f'speed={self.speed:.4f}', throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = SpeedRamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
