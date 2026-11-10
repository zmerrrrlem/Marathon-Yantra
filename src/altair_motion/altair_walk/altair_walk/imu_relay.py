#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, Twist
from rcl_interfaces.msg import SetParametersResult


class SimImuRelay(Node):
    """Relay Gazebo IMU into the /subcon/* topics expected by altair_walk.

    Conventions mirror altair_subcon/serial_comms.py:
      - euler_data: ZYX roll/pitch/yaw in radians
      - gyro_data / gyr_data: angular velocity in rad/s
    Also publishes gain_control:
      - linear.y/angular.y/angular.x: pitch P/D/I (knee correction)
      - linear.z/angular.z: yaw P/D (hip yaw heading trim)
    """

    def __init__(self):
        super().__init__('sim_imu_relay')

        self.declare_parameter('pitch_p', 0.4)
        self.declare_parameter('pitch_d', 0.05)
        self.declare_parameter('pitch_i', 0.3)
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('gyro_sign', 1.0)
        self.declare_parameter('yaw_p', 0.1)
        self.declare_parameter('yaw_d', 0.02)
        self.declare_parameter('yaw_sign', 1.0)

        self.imu_sub = self.create_subscription(
            Imu, '/robotis_op3/imu', self.imu_callback, 10
        )
        self.imu_pub = self.create_publisher(Imu, '/subcon/imu', 1)
        self.euler_pub = self.create_publisher(Vector3Stamped, '/subcon/euler_data', 1)
        self.gyr_pub = self.create_publisher(Vector3Stamped, '/subcon/gyr_data', 1)
        self.gyro_pub = self.create_publisher(Vector3Stamped, '/subcon/gyro_data', 1)
        self.gain_pub = self.create_publisher(Twist, 'gain_control', 1)

        self.gain_timer = self.create_timer(0.5, self.publish_gains)

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info('sim_imu_relay started')

    def parameter_callback(self, params):
        for param in params:
            name = param.name
            if name in (
                'pitch_p', 'pitch_d', 'pitch_i', 'pitch_sign', 'gyro_sign',
                'yaw_p', 'yaw_d', 'yaw_sign',
            ):
                self.get_logger().info(f'{name} -> {param.value}')
        return SetParametersResult(successful=True)

    def publish_gains(self):
        yaw_sign = self.get_parameter('yaw_sign').value
        gains = Twist()
        gains.linear.y = self.get_parameter('pitch_p').value
        gains.angular.y = self.get_parameter('pitch_d').value
        gains.angular.x = self.get_parameter('pitch_i').value
        gains.linear.z = yaw_sign * self.get_parameter('yaw_p').value
        gains.angular.z = yaw_sign * self.get_parameter('yaw_d').value
        self.gain_pub.publish(gains)

    def imu_callback(self, msg):
        self.imu_pub.publish(msg)

        q = msg.orientation
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        pitch_sign = self.get_parameter('pitch_sign').value
        gyro_sign = self.get_parameter('gyro_sign').value

        euler_msg = Vector3Stamped()
        euler_msg.header = msg.header
        euler_msg.vector.x = roll
        euler_msg.vector.y = pitch_sign * pitch
        euler_msg.vector.z = yaw
        self.euler_pub.publish(euler_msg)

        gyr_msg = Vector3Stamped()
        gyr_msg.header = msg.header
        gyr_msg.vector.x = gyro_sign * msg.angular_velocity.x
        gyr_msg.vector.y = gyro_sign * msg.angular_velocity.y
        gyr_msg.vector.z = gyro_sign * msg.angular_velocity.z
        self.gyr_pub.publish(gyr_msg)
        self.gyro_pub.publish(gyr_msg)


def main():
    rclpy.init()
    node = SimImuRelay()
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
