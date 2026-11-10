#!/usr/bin/env python3
"""
Drive /altair_turn/request and report per-turn metrics.

Usage (stack walking, altair_turn launched):
    python3 tools/turn_test.py --angle 90 --count 3
    python3 tools/turn_test.py --angle -90 --count 1 --timeout 70

The robot must already be walking (walk_status == 1). Metrics printed per
turn: elapsed wall time, final error, overshoot, and state result.
"""

import argparse
import math
import time

from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node
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


class TurnTest(Node):
    """Publish turn requests and record yaw/state telemetry."""

    def __init__(self):
        """Initialize topics."""
        super().__init__('turn_test')
        self.yaw = None
        self.state = 0
        self.walking = False

        self.request_pub = self.create_publisher(Float64, '/altair_turn/request', 10)
        self.create_subscription(Vector3Stamped, '/subcon/euler_data', self.euler_cb, 10)
        self.create_subscription(Int32, '/altair_turn/state', self.state_cb, 10)
        self.create_subscription(Int32, '/altair_walk/walk_status', self.walk_cb, 10)

    def euler_cb(self, msg):
        """Store the latest yaw."""
        self.yaw = msg.vector.z

    def state_cb(self, msg):
        """Store the supervisor state."""
        self.state = msg.data

    def walk_cb(self, msg):
        """Track walking status."""
        self.walking = msg.data == 1

    def wait_for(self, predicate, timeout_s):
        """Spin until predicate() is true or the timeout expires."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def run_turn(self, angle_rad, timeout_s):
        """Execute one relative turn and return its metrics."""
        start_yaw = self.yaw
        target = wrap_angle(start_yaw + angle_rad)
        msg = Float64()
        msg.data = angle_rad
        self.request_pub.publish(msg)

        started = time.monotonic()
        saw_active = False
        max_err = 0.0
        min_err = 0.0
        while time.monotonic() - started < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state in (STATE_TURNING, STATE_SETTLING):
                saw_active = True
            if self.yaw is not None:
                err = wrap_angle(self.yaw - target)
                max_err = max(max_err, err)
                min_err = min(min_err, err)
            if not saw_active and self.state == STATE_IDLE:
                continue
            if self.state in (STATE_DONE, STATE_ABORTED):
                break

        elapsed = time.monotonic() - started
        if self.yaw is not None:
            final_err = wrap_angle(self.yaw - target)
        else:
            final_err = float('nan')
        if angle_rad >= 0.0:
            overshoot = max(0.0, max_err)
        else:
            overshoot = max(0.0, -min_err)
        if self.state == STATE_DONE:
            result = 'DONE'
        else:
            result = 'ABORT/TIMEOUT'
        self.wait_for(lambda: self.state == STATE_IDLE, 5.0)
        return elapsed, final_err, overshoot, result


def parse_args():
    """Parse tool arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--angle', type=float, default=90.0, help='relative turn in deg')
    parser.add_argument('--count', type=int, default=1, help='number of turns')
    parser.add_argument('--timeout', type=float, default=70.0, help='per-turn timeout')
    return parser.parse_args()


def main():
    """Run the requested number of turns and print a metrics table."""
    args = parse_args()
    rclpy.init()
    node = TurnTest()
    angle_rad = math.radians(args.angle)
    try:
        ready = node.wait_for(lambda: node.walking and node.yaw is not None, 60.0)
        if not ready:
            node.get_logger().error('Robot is not walking / no yaw data; aborting')
            return 1
        print('')
        print('  #  requested  elapsed_s  final_err_deg  overshoot_deg  result')
        for i in range(args.count):
            elapsed, err, overshoot, result = node.run_turn(angle_rad, args.timeout)
            print(
                '%3d  %8.1f  %9.1f  %13.2f  %13.2f  %s'
                % (i + 1, args.angle, elapsed, math.degrees(err),
                   math.degrees(overshoot), result)
            )
        print('')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
