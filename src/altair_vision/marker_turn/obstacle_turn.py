#!/usr/bin/env python3
"""Turn right when an obstacle is within 20 cm in front."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class ObstacleTurn(Node):
    def __init__(self):
        super().__init__('obstacle_turn')
        self.pub = self.create_publisher(Float64, '/altair_turn/request', 10)
        self.sub = self.create_subscription(LaserScan, '/lidar', self.cb, 10)
        self.cooldown = 0

    def cb(self, msg):
        if self.cooldown > 0:
            self.cooldown -= 1
            return
        if not msg.ranges:
            return
        front = msg.ranges[len(msg.ranges)//2]
        if front < 0.20 and front > 0.0:
            req = Float64()
            req.data = -1.5708  # turn right
            self.pub.publish(req)
            self.cooldown = 40
            self.get_logger().info(f'Obstacle at {front:.2f} m -> turn right')


def main():
    rclpy.init()
    node = ObstacleTurn()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
