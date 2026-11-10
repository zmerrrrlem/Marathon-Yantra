import sys
import rclpy
import socket
import pickle
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from altair_msgs.msg import BallInfo

class VisionSender(Node):
    def __init__(self, ip="0.0.0.0", port=5005):
        super().__init__('vision_sender')

        # UDP socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (ip, port)
        self.get_logger().info(f'Sending Vision messages to {ip}:{port}')
        self.latest_msg = None

        # Add or modify anything related to the Vision data you want to send to the motion board.
        # Modify the "BallInfo" to add more data in "altair_msgs" packages, because the serialization is easier in one packet directly.
        self.sub = self.create_subscription(BallInfo, '/vision_data', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.latest_msg = msg

    def send_latest(self):
        if self.latest_msg is not None:
            try:
                self.sock.sendto(pickle.dumps(self.latest_msg), self.dest)
            except Exception as e:
                self.get_logger().error(f'Failed to send message: {e}')


def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv
    ip = '127.0.0.1'
    port = 5005
    if len(argv) > 2:
        ip = argv[1]
        port = int(argv[2])
    elif len(argv) > 1:
        ip = argv[1]

    node = VisionSender(ip, port)

    rate = node.create_rate(20.0)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.send_latest()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
