#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse

class BenchmarkSender(Node):
    def __init__(self, topic, data_size, frequency):
        super().__init__('benchmark_sender')
        self.publisher_ = self.create_publisher(String, topic, 10)
        self.counter = 0
        self.data_size = data_size
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)

    def timer_callback(self):
        # Generate payload of specified size
        payload = 'x' * self.data_size
        msg = String()
        msg.data = f'{self.counter}:{payload}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent counter: {self.counter}')
        self.counter += 1

class BenchmarkReceiver(Node):
    def __init__(self, topic):
        super().__init__('benchmark_receiver')
        self.subscriber_ = self.create_subscription(String, topic, self.callback, 10)
        self.last_counter = None

    def callback(self, msg):
        # Parse counter from incoming message
        try:
            count_str, _ = msg.data.split(':', 1)
            counter = int(count_str)
            self.last_counter = counter
            self.get_logger().info(f'Received counter: {self.last_counter}')
        except ValueError:
            self.get_logger().warn('Received malformed message')


def main():
    parser = argparse.ArgumentParser(
        description='ROS2 Benchmark: send or receive messages of variable size and frequency.'
    )
    parser.add_argument('--mode', choices=['send', 'receive'], required=True,
                        help='Mode: send or receive')
    parser.add_argument('--topic', type=str, default='benchmark_topic',
                        help='Topic name to publish/subscribe')
    parser.add_argument('--data_size', type=int, default=1024,
                        help='Size of data payload in bytes (sender only)')
    parser.add_argument('--frequency', type=float, default=10.0,
                        help='Publish frequency in Hz (sender only)')
    args, remaining = parser.parse_known_args()

    # Initialize rclpy with any additional ROS arguments
    rclpy.init(args=remaining)

    if args.mode == 'send':
        node = BenchmarkSender(args.topic, args.data_size, args.frequency)
    else:
        node = BenchmarkReceiver(args.topic)
        # Wait until the first message arrives
        while rclpy.ok() and node.last_counter is None:
            rclpy.spin_once(node, timeout_sec=1.0)
            node.get_logger().info('Waiting for data...')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
