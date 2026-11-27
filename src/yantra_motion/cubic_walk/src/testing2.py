import rclpy
from rclpy.node import Node
from yantra_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointCommandListener(Node):
    def __init__(self):
        super().__init__('joint_command_listener')

        # Subscriber to the JointCommand topic
        self.subscription = self.create_subscription(
            JointCommand,
            '/walking_motor_goals',  # Replace with your actual topic name
            self.listener_callback,
            10
        )

        # Publisher to the /joint_states topic
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

    def listener_callback(self, msg):
        # Extract joint names and positions from the JointCommand message
        joint_names = msg.joint_names
        joint_positions = msg.positions

        # Create a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = joint_names
        joint_state_msg.position = joint_positions

        # Publish the JointState message
        self.publisher.publish(joint_state_msg)
        self.get_logger().info(f'Published joint states: {joint_names} - {joint_positions}')

def main(args=None):
    rclpy.init(args=args)

    joint_command_listener = JointCommandListener()
    print("Listening to /joint_command_topic...")

    # Spin the node so it can process callbacks
    rclpy.spin(joint_command_listener)

    # Clean up and shut down
    joint_command_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
