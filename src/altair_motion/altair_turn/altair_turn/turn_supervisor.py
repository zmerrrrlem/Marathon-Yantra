"""Turn supervision: own the yaw target and command channel until a turn settles."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Int32

STATE_IDLE = 0
STATE_TURNING = 1
STATE_SETTLING = 2
STATE_DONE = 3
STATE_ABORTED = 4


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class TurnSupervisor(Node):
    """Sequence relative yaw turns and hold command priority until settled."""

    def __init__(self):
        """Initialize parameters, topics and the 20 Hz control timer."""
        super().__init__('turn_supervisor')

        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('capture_band_deg', 8.0)
        self.declare_parameter('settle_tolerance_deg', 2.0)
        self.declare_parameter('settle_ticks', 10)
        self.declare_parameter('timeout_s', 60.0)
        self.declare_parameter('in_place', True)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('euler_timeout_s', 0.5)
        self.declare_parameter('done_hold_ticks', 10)
        self.declare_parameter('target_publish_hz', 10.0)

        self.state = STATE_IDLE
        self.state_ticks = 0
        self.settle_count = 0
        self.request_angle = 0.0
        self.target = 0.0
        self.walking = False
        self.euler = None
        self.euler_time = None
        self.turn_start_time = None
        self.last_target_publish = None

        self.create_subscription(Float64, '/altair_turn/request', self.request_cb, 10)
        self.create_subscription(Bool, '/altair_turn/cancel', self.cancel_cb, 10)
        self.create_subscription(Vector3Stamped, '/subcon/euler_data', self.euler_cb, 10)
        self.create_subscription(Int32, '/altair_walk/walk_status', self.walk_cb, 10)
        self.create_subscription(Odometry, '/walk_engine_odometry', self.odom_cb, 10)

        self.target_pub = self.create_publisher(Float64, '/target_yaw', 10)
        self.cmd_pub = self.create_publisher(Twist, '/altair_turn/cmd_vel', 10)
        self.state_pub = self.create_publisher(Int32, '/altair_turn/state', 10)

        self.create_timer(1.0 / 20.0, self.timer_cb)

    def euler_cb(self, msg):
        """Store the latest yaw and its arrival time."""
        self.euler = msg.vector.z
        self.euler_time = self.get_clock().now()

    def walk_cb(self, msg):
        """Track whether the walk engine is stepping."""
        self.walking = msg.data == 1

    def odom_cb(self, msg):
        """Cubic walk publishes odometry while active; treat as walking."""
        self.walking = True

    def request_cb(self, msg):
        """Start a turn for the requested relative angle."""
        if self.state in (STATE_TURNING, STATE_SETTLING):
            self.get_logger().warn('Turn already active, request ignored')
            return
        self.start_turn(msg.data)

    def cancel_cb(self, msg):
        """Abort the active turn when requested."""
        if msg.data and self.state in (STATE_TURNING, STATE_SETTLING):
            self.abort('cancel requested')

    def euler_fresh(self):
        """Return True when yaw data is present and recent."""
        if self.euler is None or self.euler_time is None:
            return False
        timeout = self.get_parameter('euler_timeout_s').value
        age = (self.get_clock().now() - self.euler_time).nanoseconds / 1e9
        return age <= timeout

    def publish_state(self):
        """Publish the current supervisor state."""
        msg = Int32()
        msg.data = self.state
        self.state_pub.publish(msg)

    def publish_target(self, force=False):
        """Publish the absolute yaw target on change and at a fixed rate."""
        now = self.get_clock().now()
        period = 1.0 / max(1.0, self.get_parameter('target_publish_hz').value)
        if not force and self.last_target_publish is not None:
            elapsed = (now - self.last_target_publish).nanoseconds / 1e9
            if elapsed < period:
                return
        msg = Float64()
        msg.data = self.target
        self.target_pub.publish(msg)
        self.last_target_publish = now

    def start_turn(self, angle):
        """Validate preconditions and latch a new absolute yaw target."""
        if not self.walking:
            self.get_logger().warn('Turn rejected: walk engine is not walking')
            self.state = STATE_ABORTED
            self.state_ticks = 0
            return
        if not self.euler_fresh():
            self.get_logger().warn('Turn rejected: euler data missing or stale')
            self.state = STATE_ABORTED
            self.state_ticks = 0
            return
        yaw_sign = self.get_parameter('yaw_sign').value
        self.request_angle = angle
        self.target = wrap_angle(self.euler + angle * yaw_sign)
        self.state = STATE_TURNING
        self.state_ticks = 0
        self.settle_count = 0
        self.turn_start_time = self.get_clock().now()
        self.publish_target(force=True)
        self.get_logger().info(
            'Turn started: request %.1f deg, yaw %.1f deg, target %.1f deg'
            % (math.degrees(angle), math.degrees(self.euler), math.degrees(self.target))
        )

    def abort(self, reason):
        """Abort the active turn and release command priority."""
        err = 0.0
        if self.euler is not None:
            err = wrap_angle(self.euler - self.target)
        self.get_logger().warn(
            'Turn aborted (%s) at error %.1f deg' % (reason, math.degrees(err))
        )
        self.state = STATE_ABORTED
        self.state_ticks = 0

    def finish(self):
        """Complete the turn and log its metrics."""
        err = wrap_angle(self.euler - self.target)
        elapsed = 0.0
        if self.turn_start_time is not None:
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
        self.get_logger().info(
            'Turn done: request %.1f deg, target %.1f deg, elapsed %.1f s, error %.2f deg'
            % (
                math.degrees(self.request_angle),
                math.degrees(self.target),
                elapsed,
                math.degrees(err),
            )
        )
        self.state = STATE_DONE
        self.state_ticks = 0

    def turn_command(self):
        """Return the Twist published while turning (yaw is heading_hold's job)."""
        cmd = Twist()
        if not self.get_parameter('in_place').value:
            cmd.linear.x = self.get_parameter('linear_speed').value
        return cmd

    def timer_cb(self):
        """Run the 20 Hz state machine."""
        self.publish_state()

        if self.state in (STATE_TURNING, STATE_SETTLING):
            if not self.walking:
                self.abort('walk engine stopped')
                return
            if not self.euler_fresh():
                self.abort('euler data stale')
                return
            timeout = self.get_parameter('timeout_s').value
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            if elapsed > timeout:
                self.abort('timeout after %.1f s' % elapsed)
                return

            err = wrap_angle(self.euler - self.target)
            capture = math.radians(self.get_parameter('capture_band_deg').value)
            tol = math.radians(self.get_parameter('settle_tolerance_deg').value)

            if self.state == STATE_TURNING and abs(err) < capture:
                self.state = STATE_SETTLING
                self.settle_count = 0
            elif self.state == STATE_SETTLING:
                if abs(err) <= tol:
                    self.settle_count += 1
                else:
                    self.settle_count = 0
                if self.settle_count >= self.get_parameter('settle_ticks').value:
                    self.finish()
                    return

            self.cmd_pub.publish(self.turn_command())
            self.publish_target()
            return

        if self.state in (STATE_DONE, STATE_ABORTED):
            self.cmd_pub.publish(Twist())
            self.state_ticks += 1
            if self.state_ticks >= self.get_parameter('done_hold_ticks').value:
                self.state = STATE_IDLE
                self.state_ticks = 0


def main(args=None):
    """Run the turn supervisor node."""
    rclpy.init(args=args)
    node = TurnSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
