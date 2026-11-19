#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool


class FallDetector(Node):
    """Detects falls from IMU/gyro thresholds and publishes /is_falling.

    Subscribes to /subcon/imu and /subcon/gyr_data, computes linear
    acceleration and angular velocity magnitudes, and publishes a latched
    Bool on /is_falling with configurable confirm/recover hysteresis.
    """

    def __init__(self):
        super().__init__('fall_detector')

        self.declare_parameter('accel_threshold_mps2', 15.0)
        self.declare_parameter('gyro_threshold_rads', 6.0)
        self.declare_parameter('confirm_count', 3)
        self.declare_parameter('recover_count', 10)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.accel_threshold = self.get_parameter('accel_threshold_mps2').value
        self.gyro_threshold = self.get_parameter('gyro_threshold_rads').value
        self.confirm_count = self.get_parameter('confirm_count').value
        self.recover_count = self.get_parameter('recover_count').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value

        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]

        self.fault_counter = 0
        self.recover_counter = 0
        self.falling = False

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.fall_pub = self.create_publisher(Bool, '/is_falling', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)

        self.get_logger().info('fall_detector started')

    def imu_callback(self, msg):
        self.accel = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        self.gyro = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]


    def timer_cb(self):
        accel_mag = math.sqrt(sum(a * a for a in self.accel))
        gyro_mag = math.sqrt(sum(g * g for g in self.gyro))

        fault = accel_mag > self.accel_threshold or gyro_mag > self.gyro_threshold

        if fault:
            self.recover_counter = 0
            self.fault_counter += 1
            if not self.falling and self.fault_counter >= self.confirm_count:
                self.falling = True
                self.get_logger().warn('Fall detected')
        else:
            self.fault_counter = 0
            if self.falling:
                self.recover_counter += 1
                if self.recover_counter >= self.recover_count:
                    self.falling = False
                    self.recover_counter = 0
                    self.get_logger().info('Fall cleared')
            else:
                self.recover_counter = 0

        msg = Bool()
        msg.data = self.falling
        self.fall_pub.publish(msg)


def main():
    rclpy.init()
    node = FallDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
