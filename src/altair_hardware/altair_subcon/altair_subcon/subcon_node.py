import sys
import time
from math import *

import numpy as np
import serial
import rclpy
from rclpy.node import Node 
from rclpy.logging import get_logger
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8

from altair_subcon.serial_comms import SerialHandler

PORTNAME = "/dev/ttyUSB0"
LOOP_RATE = 125


class Subcon(Node):
    def __init__(self):
        super().__init__('subcon_node')
        self.logger = self.get_logger()
        self.imu_msg = Imu()
        self.quat_msg = QuaternionStamped()
        self.euler_msg = Vector3Stamped()
        self.gyr_msg = Vector3Stamped()
        self.acc_msg = Vector3Stamped()
        self.button_msg = Int8()
        self.imu_pub = self.create_publisher(Imu, "/subcon/imu", 1)
        self.quat_pub = self.create_publisher(QuaternionStamped, "/subcon/quaternion_data", 1)
        self.euler_pub = self.create_publisher(Vector3Stamped, "/subcon/euler_data", 1)
        self.gyro_pub = self.create_publisher(Vector3Stamped, "/subcon/gyro_data", 1)
        self.acc_pub = self.create_publisher(Vector3Stamped, "/subcon/acc_data", 1)
        self.button_pub = self.create_publisher(Int8, "/subcon/button" , 1)
        self.calib_status = self.create_subscription(Bool, "/subcon/calibrate_status", self.calibrate_callback, 1)

        self.serial_handler = None
        self.initialization()
        self.calibrate_status = False
        self.angle_hat = [.0, .0, .0]
        self.setpoint = [.0, .0, .0]
        self.counter = 1
        self.offset = self.calibrate()

        self.timer_period = 1.0 / LOOP_RATE
        self.run_cb = self.create_timer(self.timer_period, self.run_cb)
        self.prev_time = time.perf_counter()

        self.logger.info("SUBCON_NODE INITIALIZED")

    def initialization(self):
        try:
            serial_context = serial.Serial(PORTNAME, baudrate=115200, timeout=0.1)
        except serial.SerialException as e:
            print(e)
            sys.exit(1)

        self.serial_handler = SerialHandler(serial_context, PORTNAME)

    def calibrate(self):
        self.logger.info("CALIBRATING IMU")
        x = 0.0
        y = 0.0
        z = 0.0

        for i in range(100):
            packet = self.serial_handler.read_packet()
            if packet == None:
                continue

            angle = self.serial_handler.get_euler()
            x += angle['x']
            y += angle['y']
            z += angle['z']
            time.sleep(0.01)

        x /= 100
        y /= 100
        z /= 100
        self.logger.info("IMU CALIBRATION FINISHED")

        self.calibrate_status = False
        return {'x': x, 'y': y, 'z': z}

    def calibrate_callback(self, msg):
        self.calibrate_status = msg.data

    def get_imu_data(self):
        angle = self.serial_handler.get_euler()
        gyro = self.serial_handler.get_gyro_data()
        accel = self.serial_handler.get_accel_data()

        # offset is to set the angle to 0 in any attitude we want.
        ang = [angle['x'] - self.offset['x'], angle['y'] - self.offset['y'], angle['z'] - self.offset['z']]
        gyr = [gyro['x'], gyro['y'], gyro['z']]
        acc = [accel['x'], accel['y'], accel['z']]

        return ang, gyr, acc
    
    def run(self, dt):
        if self.calibrate_status == True:
            self.offset = self.calibrate()
            return
            
        packet = self.serial_handler.read_packet()
        if packet == None:
            return
        
        angle, gyro, acc = self.get_imu_data()


        ### estimating euler angle by kalman filter is not used because BNO055 already giving us a clean orientation, 
        ### using kalman filter to estimate the orientation only giving us noise. 
        ### The reference implementation use kalman filter because it is using MPU not BNO.

        quat = self.serial_handler.get_quaternion()
        self.quat_msg.header.stamp = self.get_clock().now().to_msg()
        self.quat_msg.quaternion.x, self.quat_msg.quaternion.y, self.quat_msg.quaternion.z, self.quat_msg.quaternion.w = quat['x'], quat['y'], quat['z'], quat['w']

        self.euler_msg.header.stamp = self.get_clock().now().to_msg()
        self.euler_msg.vector.x, self.euler_msg.vector.y, self.euler_msg.vector.z = angle[0], angle[1], angle[2]

        self.gyr_msg.header.stamp = self.get_clock().now().to_msg()
        self.gyr_msg.vector.x, self.gyr_msg.vector.y, self.gyr_msg.vector.z = gyro[0], gyro[1], gyro[2]

        self.acc_msg.header.stamp = self.get_clock().now().to_msg()
        self.acc_msg.vector.x, self.acc_msg.vector.y, self.acc_msg.vector.z = acc[0], acc[1], acc[2]
        self.button_msg.data = int(packet["button"])
        
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = "imu_link"
        self.imu_msg.orientation = self.quat_msg.quaternion
        self.imu_msg.angular_velocity = self.gyr_msg.vector
        self.imu_msg.linear_acceleration = self.acc_msg.vector

        self.imu_pub.publish(self.imu_msg)
        self.quat_pub.publish(self.quat_msg)
        self.euler_pub.publish(self.euler_msg)
        self.gyro_pub.publish(self.gyr_msg)
        self.acc_pub.publish(self.acc_msg)
        self.button_pub.publish(self.button_msg)

    def run_cb(self):
        current_time = time.perf_counter()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        self.run(dt)
    
def main():
    rclpy.init()
    node = Subcon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except:
        print("ERROR OCCURED")