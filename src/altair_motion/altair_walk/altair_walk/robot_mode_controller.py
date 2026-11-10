#!/usr/bin/env python3
"""Robot mode controller for cubic_walk.

Switches between marathon (0) and sprinting (1) modes by setting
cubic_walk parameters at runtime via `ros2 param set`.

Publishes:
  /altair_walk/robot_mode (Int32) - current mode
  /altair_walk/mode_transition_state (Int32) - 0=idle, 1=transitioning, 2=wind-up, 3=ready
"""
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


# Mode-specific parameters
MODE_PARAMS = {
    0: {  # Marathon (ramp-aware, stable, energy-efficient)
        "engine.trunk_height": 0.28,
        "engine.trunk_pitch": 0.08,
        "engine.trunk_swing": 0.10,
        "engine.foot_rise": 0.03,
        "engine.freq": 1.0,
        "engine.kick_length": 0.12,
        "engine.double_support_ratio": 0.12,
        "engine.arm_shoulder_pitch_amp": 0.15,
    },
    1: {  # Sprinting
        "engine.trunk_height": 0.30,
        "engine.trunk_pitch": 0.15,
        "engine.trunk_swing": 0.08,
        "engine.foot_rise": 0.04,
        "engine.freq": 1.6,
        "engine.kick_length": 0.16,
        "engine.double_support_ratio": 0.08,
        "engine.arm_shoulder_pitch_amp": 0.4,
    },
}

WIND_UP_DURATION = 2.0  # seconds


class RobotModeController(Node):
    def __init__(self):
        super().__init__("robot_mode_controller")

        self.declare_parameter("default_mode", 0)
        self.robot_mode = self.get_parameter("default_mode").value
        self.transition_state = 0  # 0=idle, 1=transitioning, 2=wind-up, 3=ready
        self.transitioning = False

        self.mode_sub = self.create_subscription(
            Int32, "/robot_mode", self.mode_cb, 10
        )
        self.mode_pub = self.create_publisher(Int32, "/altair_walk/robot_mode", 10)
        self.state_pub = self.create_publisher(
            Int32, "/altair_walk/mode_transition_state", 10
        )

        self.timer = self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f"robot_mode_controller started (mode={self.robot_mode})"
        )

    def mode_cb(self, msg):
        new_mode = msg.data
        if new_mode == self.robot_mode or self.transitioning:
            return
        if new_mode not in MODE_PARAMS:
            self.get_logger().warn(f"Unknown mode: {new_mode}")
            return

        self.transitioning = True
        self.transition_state = 1  # transitioning
        self.get_logger().info(
            f"Starting mode transition to "
            f"{'sprinting' if new_mode == 1 else 'marathon'}"
        )

        # Run transition in a separate thread to not block the executor
        import threading
        t = threading.Thread(target=self._do_transition, args=(new_mode,))
        t.daemon = True
        t.start()

    def _do_transition(self, new_mode):
        self.transition_state = 2  # wind-up
        self.get_logger().info("Applied wind-up params, holding for %.1fs" % WIND_UP_DURATION)
        time.sleep(WIND_UP_DURATION)

        self.transition_state = 1  # transitioning
        # Apply new mode params
        params = MODE_PARAMS[new_mode]
        for key, value in params.items():
            try:
                subprocess.run(
                    ["ros2", "param", "set", "/cubic_walk", key, str(value)],
                    timeout=5.0,
                    capture_output=True,
                )
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"Failed to set {key}={value}: timeout")

        self.robot_mode = new_mode
        self.transition_state = 3  # ready
        self.get_logger().info(
            f"Mode transition complete: "
            f"{'sprinting' if new_mode == 1 else 'marathon'} (READY)"
        )
        time.sleep(1.0)
        self.transition_state = 0  # idle
        self.transitioning = False

    def publish_status(self):
        mode_msg = Int32()
        mode_msg.data = self.robot_mode
        self.mode_pub.publish(mode_msg)

        state_msg = Int32()
        state_msg.data = self.transition_state
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotModeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()