import sys

import rclpy, socket, pickle
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import qos_profile_sensor_data

from altair_msgs.msg import BallInfo

class VisionReceiver(Node):
    def __init__(self, port=5005):
        super().__init__('vision_receiver')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.get_logger().info(f'Receiving Vision messages in port: {port}')

        msg = BallInfo()
        serialized = serialize_message(msg)
        self.byte_size = len(serialized)

        self.pub = self.create_publisher(BallInfo, '/vision_data', qos_profile_sensor_data)

        self.create_timer(1.0/20.0, self.vis_pub)

    def vis_pub(self):
        data, _ = self.sock.recvfrom(65536)
        msg = pickle.loads(data)
        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv
    port = 5005
    if len(argv) > 1:
        port = int(argv[1])

    node = VisionReceiver(port)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

