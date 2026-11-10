import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from biped_interfaces.msg import Phase


class HeadingHold(Node):
    def __init__(self):
        super().__init__('heading_hold')

        self.declare_parameter('enabled', True)
        self.declare_parameter('kp', 0.1)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('max_angular', 0.02)
        self.declare_parameter('max_correction_rate', 0.1)
        self.declare_parameter('gyro_filter_tau', 0.15)
        self.declare_parameter('euler_filter_tau', 0.5)
        self.declare_parameter('walk_debounce', 34)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('use_gyro', False)

        self.euler = None
        self.euler_f = 0.0
        self.euler_f_init = False
        self.euler_sin_f = 0.0
        self.euler_cos_f = 0.0
        self.gyr_raw = 0.0
        self.gyr_f = 0.0
        self.cmd = Twist()
        self.walking = False
        self.walk_ticks = 0
        self.target_topic = None
        self.prev_correction = 0.0

        self.create_subscription(Twist, '/cmd_vel_input', self.cmd_cb, 10)
        self.create_subscription(Imu, '/imu_sensor_broadcaster/imu', self.imu_cb, 10)
        self.create_subscription(Phase, '/walk_support_state', self.walk_cb, 10)
        self.create_subscription(Float64, '/target_yaw', self.target_cb, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / 20.0, self.timer_cb)

    def cmd_cb(self, msg):
        self.cmd = msg

    def imu_cb(self, msg):
        # Extract yaw from quaternion
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.euler = math.atan2(siny_cosp, cosy_cosp)

        # Extract yaw rate from angular velocity
        self.gyr_raw = msg.angular_velocity.z

    def walk_cb(self, msg):
        # cubic_walk publishes Phase when actively walking
        self.walking = True
        if not self.walking:
            self.walk_ticks = 0

    def target_cb(self, msg):
        self.target_topic = math.atan2(math.sin(msg.data), math.cos(msg.data))

    def timer_cb(self):
        dt = 1.0 / 20.0
        tau = self.get_parameter('gyro_filter_tau').value
        alpha = dt / (tau + dt) if tau > 0.0 else 1.0
        self.gyr_f += alpha * (self.gyr_raw - self.gyr_f)

        tau_e = self.get_parameter('euler_filter_tau').value
        alpha_e = dt / (tau_e + dt) if tau_e > 0.0 else 1.0
        if self.euler is not None:
            s = math.sin(self.euler)
            c = math.cos(self.euler)
            if not self.euler_f_init:
                self.euler_sin_f = s
                self.euler_cos_f = c
                self.euler_f_init = True
            else:
                self.euler_sin_f += alpha_e * (s - self.euler_sin_f)
                self.euler_cos_f += alpha_e * (c - self.euler_cos_f)
            self.euler_f = math.atan2(self.euler_sin_f, self.euler_cos_f)

        out = Twist()
        out.linear.x = self.cmd.linear.x
        out.linear.y = self.cmd.linear.y
        out.linear.z = self.cmd.linear.z
        out.angular.x = self.cmd.angular.x
        out.angular.y = self.cmd.angular.y

        if not self.get_parameter('enabled').value:
            self.pub.publish(out)
            return

        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        scale = self.get_parameter('gain_scale').value
        max_ang = self.get_parameter('max_angular').value
        max_rate = self.get_parameter('max_correction_rate').value
        debounce = self.get_parameter('walk_debounce').value
        if self.target_topic is not None:
            target = self.target_topic
        else:
            target = self.get_parameter('target_yaw').value
            target = math.atan2(math.sin(target), math.cos(target))

        if self.walking:
            self.walk_ticks += 1
        else:
            self.walk_ticks = 0

        desired = 0.0
        if self.walking and self.walk_ticks > debounce and self.euler_f_init:
            err = math.atan2(
                math.sin(self.euler_f - target), math.cos(self.euler_f - target)
            )
            desired -= kp * err * scale
        if (
            self.walking
            and self.walk_ticks > debounce
            and self.get_parameter('use_gyro').value
        ):
            desired -= kd * self.gyr_f * scale

        desired = float(max(-max_ang, min(max_ang, desired)))

        if max_rate > 0.0:
            step = max_rate * dt
            correction = self.prev_correction + float(
                max(-step, min(step, desired - self.prev_correction))
            )
        else:
            correction = desired
        self.prev_correction = correction

        out.angular.z = self.cmd.angular.z + correction
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
