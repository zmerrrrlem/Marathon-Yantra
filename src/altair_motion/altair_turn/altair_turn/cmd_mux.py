"""Command mux: single writer to /cmd_vel_input with turn priority."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

TURN_STATES = (1, 2)


class CmdMux(Node):
    """Arbitrate between mission commands and altair_turn commands."""

    def __init__(self):
        """Initialize parameters, topics and the output timer."""
        super().__init__('cmd_mux')

        self.declare_parameter('turn_priority', True)
        self.declare_parameter('mission_timeout_s', 0.5)
        self.declare_parameter('turn_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.mission_cmd = Twist()
        self.mission_time = None
        self.turn_cmd = Twist()
        self.turn_time = None
        self.turn_state = 0
        self.active_source = 'none'

        self.create_subscription(Twist, '/mission/cmd_vel', self.mission_cb, 10)
        self.create_subscription(Twist, '/altair_turn/cmd_vel', self.turn_cb, 10)
        self.create_subscription(Int32, '/altair_turn/state', self.state_cb, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel_input', 10)
        rate = max(1.0, self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self.timer_cb)

    def mission_cb(self, msg):
        """Store the latest mission command."""
        self.mission_cmd = msg
        self.mission_time = self.get_clock().now()

    def turn_cb(self, msg):
        """Store the latest turn command."""
        self.turn_cmd = msg
        self.turn_time = self.get_clock().now()

    def state_cb(self, msg):
        """Track the turn supervisor state."""
        self.turn_state = msg.data

    def fresh(self, stamp, timeout_s):
        """Return True when a message stamp is within timeout_s."""
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= timeout_s

    def timer_cb(self):
        """Publish the highest-priority fresh command, or zero."""
        turn_priority = self.get_parameter('turn_priority').value
        turn_active = self.turn_state in TURN_STATES
        turn_fresh = self.fresh(
            self.turn_time, self.get_parameter('turn_timeout_s').value
        )
        mission_fresh = self.fresh(
            self.mission_time, self.get_parameter('mission_timeout_s').value
        )

        source = 'none'
        out = Twist()
        if turn_priority and turn_active and turn_fresh:
            out = self.turn_cmd
            source = 'turn'
        elif mission_fresh:
            out = self.mission_cmd
            source = 'mission'

        if source != self.active_source:
            self.get_logger().info('cmd_mux source -> %s' % source)
            self.active_source = source

        self.pub.publish(out)


def main(args=None):
    """Run the command mux node."""
    rclpy.init(args=args)
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
