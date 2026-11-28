"""ROS 2 node around HeadingCore: the single writer of the walk engine's cmd_vel.

Wiring (defaults):
    teleop / mission  --/cmd_vel_raw-->  heading_controller  --/cmd_vel-->  quintic_walk

Inputs
    ~input_topic   (geometry_msgs/Twist)   operator / mission command        [/cmd_vel_raw]
    ~imu_topic     (sensor_msgs/Imu)       orientation quaternion + gyro     [/imu_sensor_broadcaster/imu]
    ~walk_status_topic (std_msgs/Int32)    quintic_walk WalkState            [/walk_status]
    /heading/turn      (std_msgs/Float64)  relative turn, radians (+ = CCW)
    /heading/turn_deg  (std_msgs/Float64)  relative turn, degrees
    /heading/cancel    (std_msgs/Empty)    abort the active turn
    /heading/set_target (std_msgs/Float64) absolute heading to hold, radians
Outputs
    ~output_topic  (geometry_msgs/Twist)                                       [/cmd_vel]
    /heading/state (std_msgs/Int32)    0 PASSTHROUGH 1 HOLD 2 TURNING 3 SETTLING 4 DONE 5 ABORTED
    /heading/target_yaw, /heading/error, /heading/yaw (std_msgs/Float64, radians)
"""

from dataclasses import fields
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float64, Int32

from yantra_heading.core import HeadingCore, HeadingParams, STATE_NAMES, Twist2D

# Angle-valued parameters are exposed in degrees for easier tuning
DEGREE_PARAMS = ('hold_deadband', 'capture_band', 'settle_tolerance')


def yaw_from_quaternion(q):
    """Return the yaw (rad) of a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HeadingController(Node):
    """Heading hold + relative turns for any walk engine that takes geometry_msgs/Twist."""

    def __init__(self):
        super().__init__('heading_controller')

        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('imu_topic', '/imu_sensor_broadcaster/imu')
        self.declare_parameter('walk_status_topic', '/walk_status')
        self.declare_parameter('rate_hz', 50.0)

        self.core = HeadingCore()
        for f in fields(HeadingParams):
            value = getattr(self.core.p, f.name)
            name = f.name + ('_deg' if f.name in DEGREE_PARAMS else '')
            if f.name in DEGREE_PARAMS:
                value = math.degrees(value)
            self.declare_parameter(name, value)
        self._apply_params(self.get_parameters([
            f.name + ('_deg' if f.name in DEGREE_PARAMS else '') for f in fields(HeadingParams)]))
        self.add_on_set_parameters_callback(self._on_params)

        gp = self.get_parameter
        self.create_subscription(Twist, gp('input_topic').value, self._cmd_cb, 10)
        self.create_subscription(Imu, gp('imu_topic').value, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Int32, gp('walk_status_topic').value, self._walk_cb, 10)
        self.create_subscription(Float64, '/heading/turn', self._turn_cb, 10)
        self.create_subscription(Float64, '/heading/turn_deg', self._turn_deg_cb, 10)
        self.create_subscription(Empty, '/heading/cancel', self._cancel_cb, 10)
        self.create_subscription(Float64, '/heading/set_target', self._set_target_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, gp('output_topic').value, 10)
        self.state_pub = self.create_publisher(Int32, '/heading/state', 10)
        self.target_pub = self.create_publisher(Float64, '/heading/target_yaw', 10)
        self.error_pub = self.create_publisher(Float64, '/heading/error', 10)
        self.yaw_pub = self.create_publisher(Float64, '/heading/yaw', 10)

        self.last_state = None
        self.create_timer(1.0 / max(1.0, gp('rate_hz').value), self._timer_cb)
        self.get_logger().info('heading_controller: %s -> %s' % (gp('input_topic').value,
                                                                 gp('output_topic').value))

    # --------------------------------------------------------------- params
    def _apply_params(self, params):
        for param in params:
            name = param.name
            if name.endswith('_deg') and name[:-4] in DEGREE_PARAMS:
                setattr(self.core.p, name[:-4], math.radians(param.value))
            elif hasattr(self.core.p, name):
                current = getattr(self.core.p, name)
                setattr(self.core.p, name, type(current)(param.value))

    def _on_params(self, params):
        self._apply_params(params)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ callbacks
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd_cb(self, msg):
        self.core.on_cmd(Twist2D(x=msg.linear.x, y=msg.linear.y, z=msg.linear.z,
                                 stop=msg.angular.x, yaw_rate=msg.angular.z), self._now())

    def _imu_cb(self, msg):
        q = msg.orientation
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or (q.x == q.y == q.z == q.w == 0.0):
            return
        self.core.on_imu(yaw_from_quaternion(q), msg.angular_velocity.z, self._now())

    def _walk_cb(self, msg):
        self.core.on_walk_status(msg.data, self._now())

    def _turn_cb(self, msg):
        self.core.request_turn(msg.data, self._now())

    def _turn_deg_cb(self, msg):
        self.core.request_turn(math.radians(msg.data), self._now())

    def _cancel_cb(self, _msg):
        self.core.cancel_turn(self._now())

    def _set_target_cb(self, msg):
        self.core.set_heading(msg.data, self._now())

    # ---------------------------------------------------------------- timer
    def _timer_cb(self):
        out = self.core.update(self._now())
        if out is not None:
            cmd = Twist()
            cmd.linear.x = out.x
            cmd.linear.y = out.y
            cmd.linear.z = out.z
            cmd.angular.x = out.stop
            cmd.angular.z = out.yaw_rate
            self.cmd_pub.publish(cmd)

        if self.core.last_event:
            self.get_logger().info(self.core.last_event)
            self.core.last_event = ''
        if self.core.state != self.last_state:
            self.last_state = self.core.state
            self.get_logger().debug('state -> %s' % STATE_NAMES[self.core.state])

        self.state_pub.publish(Int32(data=self.core.state))
        if self.core.target is not None:
            self.target_pub.publish(Float64(data=self.core.target))
        if self.core.yaw is not None:
            self.yaw_pub.publish(Float64(data=self.core.yaw))
        self.error_pub.publish(Float64(data=self.core.error))


def main(args=None):
    rclpy.init(args=args)
    node = HeadingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
