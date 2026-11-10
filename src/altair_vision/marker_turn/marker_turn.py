#!/usr/bin/env python3
"""Simple OpenCV marker detector for Gazebo camera -> turn requests."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np


class MarkerTurn(Node):
    def __init__(self):
        super().__init__('marker_turn')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Float64, '/altair_turn/request', 10)
        self.sub = self.create_subscription(Image, '/gazebo/camera/image_raw', self.cb, 10)
        # HSV range for a red marker (tune as needed)
        self.lower = np.array([0, 120, 70])
        self.upper = np.array([10, 255, 255])
        self.cooldown = 0

    def cb(self, msg):
        if self.cooldown > 0:
            self.cooldown -= 1
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            # marker in left half -> turn left, right half -> turn right
            req = Float64()
            req.data = 1.5708 if cx < img.shape[1] // 2 else -1.5708
            self.pub.publish(req)
            self.cooldown = 30
            self.get_logger().info(f'Marker detected (area={area}, cx={cx}), turn request sent')
            break


def main():
    rclpy.init()
    node = MarkerTurn()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
