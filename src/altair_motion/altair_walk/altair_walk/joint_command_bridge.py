#!/usr/bin/env python3
"""Bridge: converts cubic_walk's JointCommand to JointState for OP3 framework.

Subscribes to /walking_motor_goals (altair_msgs/JointCommand) and republishes
as /altair_walk/joint_state (sensor_msgs/JointState) so the existing
altair_walk_module can drive Dynamixel servos.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from altair_msgs.msg import JointCommand


class JointCommandBridge(Node):
    def __init__(self):
        super().__init__('joint_command_bridge')

        self.sub = self.create_subscription(
            JointCommand, '/walking_motor_goals', self.cb, 10
        )
        self.pub = self.create_publisher(JointState, '/altair_walk/joint_state', 10)

        self.get_logger().info('joint_command_bridge started')

    def cb(self, msg):
        js = JointState()
        js.header = msg.header
        js.name = list(msg.joint_names)
        js.position = list(msg.positions)
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()