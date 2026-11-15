#!/usr/bin/env python3
"""Distance tracker for cubic_walk.

Subscribes to /walk_engine_odometry (nav_msgs/Odometry) and accumulates
the total distance traveled. Publishes /walk_distance_m (Float64) and
/walk_distance_completed (Bool). Accepts /walk_distance_reset (Bool).
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool


class DistanceTracker(Node):
    def __init__(self):
        super().__init__('distance_tracker')

        self.declare_parameter('target_distance_m', 400.0)
        self.declare_parameter('reset_threshold_m', 5.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.target_distance = self.get_parameter('target_distance_m').value
        self.reset_threshold = self.get_parameter('reset_threshold_m').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value

        self.total_distance = 0.0
        self.prev_pos = None
        self.completed = False

        self.odom_sub = self.create_subscription(
            Odometry, '/walk_engine_odometry', self.odom_cb, 10
        )
        self.reset_sub = self.create_subscription(
            Bool, '/walk_distance_reset', self.reset_cb, 10
        )

        self.dist_pub = self.create_publisher(Float64, '/walk_distance_m', 10)
        self.completed_pub = self.create_publisher(Bool, '/walk_distance_completed', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)

        self.get_logger().info(
            f'distance_tracker started (target={self.target_distance}m)'
        )

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        if self.prev_pos is not None:
            dx = pos.x - self.prev_pos[0]
            dy = pos.y - self.prev_pos[1]
            dz = pos.z - self.prev_pos[2]
            step = math.sqrt(dx * dx + dy * dy + dz * dz)
            # Filter out tiny movements (noise) and teleport (reset threshold)
            if step < self.reset_threshold and step > 1e-6:
                self.total_distance += step
        self.prev_pos = (pos.x, pos.y, pos.z)

    def reset_cb(self, msg):
        if msg.data:
            self.total_distance = 0.0
            self.prev_pos = None
            self.completed = False
            self.get_logger().info('Distance reset')

    def timer_cb(self):
        if self.total_distance >= self.target_distance:
            self.completed = True

        dist_msg = Float64()
        dist_msg.data = self.total_distance
        self.dist_pub.publish(dist_msg)

        comp_msg = Bool()
        comp_msg.data = self.completed
        self.completed_pub.publish(comp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()