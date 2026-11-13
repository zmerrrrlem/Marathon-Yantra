#!/usr/bin/env python3
"""
gait_sweep.py

Parameter sweep for cubic_walk walking engine (marathon vs sprinting modes).

For each parameter combination, this script:
  1. Resets the robot to origin in Gazebo
  2. Sets cubic_walk parameters via ros2 param set
  3. Publishes /cmd_vel to walk forward for a fixed duration
  4. Records max distance traveled, fall detection, walk status
  5. Stops and logs results to CSV

Prerequisites:
  - Source ROS 2:  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  - Launch Gazebo: ros2 launch cubic_walk cubic_walk_integrated.launch.py world:=empty
  - Wait for /walk_engine_odometry, /altair_walk/walk_status, /is_falling topics

Usage:
    python3 src/altair_motion/cubic_walk/scripts/gait_sweep.py --mode marathon --duration 15 --output results_marathon.csv
    python3 src/altair_motion/cubic_walk/scripts/gait_sweep.py --mode sprinting --duration 15 --output results_sprinting.csv
    python3 src/altair_motion/cubic_walk/scripts/gait_sweep.py --mode both --duration 15 --output results_all.csv
"""

import argparse
import csv
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from nav_msgs.msg import Odometry


class TrialCollector(Node):
    """Collects metrics during a single sweep trial."""

    def __init__(self):
        super().__init__('gait_sweep_collector')
        self.max_dist = 0.0
        self.fell = False
        self.walk_status = 2  # IDLE
        self.start_pos = None
        self.trial_start_time = None
        self.trial_active = False

        # Subscribers for telemetry
        self.odom_sub = self.create_subscription(
            Odometry, '/walk_engine_odometry', self._cb_odom, 10)
        self.status_sub = self.create_subscription(
            Int32, '/altair_walk/walk_status', self._cb_status, 10)
        self.fall_sub = self.create_subscription(
            Bool, '/is_falling', self._cb_fall, 10)

        # Publisher for cmd_vel (direct ROS 2 publishing, more reliable than shell)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _cb_odom(self, msg):
        if not self.trial_active:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_pos is None:
            self.start_pos = (x, y)
            self.trial_start_time = self.get_clock().now()
        dx = x - self.start_pos[0]
        dy = y - self.start_pos[1]
        dist = (dx * dx + dy * dy) ** 0.5
        if dist > self.max_dist:
            self.max_dist = dist

    def _cb_status(self, msg):
        self.walk_status = msg.data

    def _cb_fall(self, msg):
        if msg.data:
            self.fell = True

    def start_trial(self):
        self.max_dist = 0.0
        self.fell = False
        self.start_pos = None
        self.trial_start_time = None
        self.trial_active = True

    def stop_trial(self):
        self.trial_active = False

    def get_duration(self):
        if self.trial_start_time is None:
            return 0.0
        return (self.get_clock().now() - self.trial_start_time).nanoseconds / 1e9

    def publish_cmd_vel_for_duration(self, linear_x, angular_z, duration):
        """Publish /cmd_vel continuously for a duration using ROS 2 (10 Hz)."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        end_time = self.get_clock().now() + Duration(seconds=duration)
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok() and self.get_clock().now() < end_time:
            self.cmd_vel_pub.publish(msg)
            try:
                rate.sleep()
            except Exception:
                break

    def stop_walking(self):
        """Send zero velocity to stop walking."""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)


def run_cmd(cmd, timeout=10.0):
    """Run a shell command and return (returncode, stdout, stderr)."""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, '', 'timeout'


def set_param(param_name, value):
    """Set a cubic_walk parameter via ros2 param set."""
    rc, _, err = run_cmd(
        'ros2 param set /cubic_walk ' + param_name + ' ' + str(value), timeout=5.0
    )
    if rc != 0:
        print('      (param set failed: ' + err.strip() + ')')
        return False
    return True


def reset_robot():
    """Reset robot pose to origin in Gazebo."""
    req = ('name: "altair", position: {x: 0, y: 0, z: 0.35}, ' +
           'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}')
    cmd = ('gz service -s /world/empty/set_pose ' +
           '--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean ' +
           '--timeout 3000 --req \'' + req + '\'')
    run_cmd(cmd, timeout=10.0)


def wait_for_topics(timeout=30.0):
    """Wait for required ROS 2 topics to be available."""
    required = ['/walk_engine_odometry', '/altair_walk/walk_status', '/is_falling']
    deadline = time.time() + timeout
    while time.time() < deadline:
        rc, out, _ = run_cmd('ros2 topic list', timeout=5.0)
        topics = out.splitlines()
        if all(t in topics for t in required):
            return True
        time.sleep(1.0)
    return False


def run_trial(collector, params, duration=15.0, linear_x=0.02):
    """Run a single sweep trial with given parameters.

    Returns a dict of metrics.
    """
    # Apply parameters
    for key, value in params.items():
        set_param(key, value)

    # Wait for parameters to take effect
    time.sleep(0.5)

    # Reset robot to origin
    reset_robot()
    time.sleep(1.0)

    # Start data collection
    collector.start_trial()

    # Publish walk command directly via ROS 2 (not shell)
    collector.publish_cmd_vel_for_duration(linear_x, 0.0, duration)

    # Wait for walk to complete
    time.sleep(1.0)

    # Stop walking
    collector.stop_walking()
    collector.stop_trial()

    return {
        'max_dist': collector.max_dist,
        'fell': collector.fell,
        'walk_status': collector.walk_status,
        'duration': collector.get_duration(),
    }


def main():
    parser = argparse.ArgumentParser(
        description='Gait parameter sweep for cubic_walk (marathon vs sprinting)'
    )
    parser.add_argument(
        '--mode', choices=['marathon', 'sprinting', 'both'],
        default='both', help='Walking mode to sweep'
    )
    parser.add_argument(
        '--duration', type=float, default=15.0,
        help='Duration of each trial in seconds (default: 15)'
    )
    parser.add_argument(
        '--linear-x', type=float, default=0.1,
        help='Forward velocity for trials in m/s (default: 0.02)'
    )
    parser.add_argument(
        '--output', default='gait_sweep_results.csv',
        help='Output CSV file path (default: gait_sweep_results.csv)'
    )
    args = parser.parse_args()

    # Initialize ROS 2 for the collector node
    rclpy.init()
    collector = TrialCollector()

    # Spin collector in background thread so callbacks fire during trials
    spin_thread = threading.Thread(target=rclpy.spin, args=(collector,), daemon=True)
    spin_thread.start()

    # Wait for required topics
    print("Waiting for ROS 2 topics...")
    if not wait_for_topics():
        print("ERROR: Required topics not available. Is cubic_walk running in Gazebo?")
        rclpy.shutdown()
        sys.exit(1)
    print("Topics available. Starting sweep.\n")

    # Parameter grids per mode (one-factor-at-a-time sweep)
    mode_grids = {
        'marathon': {
            'engine.freq': [0.8, 1.0, 1.2],
            'engine.foot_rise': [0.02, 0.03, 0.04],
            'engine.trunk_height': [0.26, 0.28, 0.30],
            'engine.double_support_ratio': [0.10, 0.12, 0.14],
            'engine.trunk_pitch': [0.05, 0.08, 0.10],
        },
        'sprinting': {
            'engine.freq': [1.4, 1.6, 1.8],
            'engine.foot_rise': [0.035, 0.04, 0.045],
            'engine.trunk_height': [0.28, 0.30, 0.32],
            'engine.double_support_ratio': [0.06, 0.08, 0.10],
            'engine.trunk_pitch': [0.12, 0.15, 0.18],
        }
    }

    # Base parameters applied before every trial
    base_params = {
        'engine.arm_swing_enabled': 'true',
        'engine.arm_shoulder_pitch_amp': '0.2',
        'node.x_speed_multiplier': '1.0',
    }

    results = []
    modes_to_run = [args.mode] if args.mode != 'both' else ['marathon', 'sprinting']

    for mode in modes_to_run:
        grid = mode_grids[mode]
        print('=' * 60)
        print('SWEEP: ' + mode.upper() + ' mode')
        print('=' * 60)

        # Sweep each parameter individually (one-factor-at-a-time)
        for param_name, values in grid.items():
            print('\n--- Varying ' + param_name + ' ---')
            for val in values:
                params = dict(base_params)
                params[param_name] = str(val)

                print('  Trial: ' + param_name + '=' + str(val), end=' ... ', flush=True)

                try:
                    metrics = run_trial(
                        collector, params,
                        duration=args.duration, linear_x=args.linear_x
                    )

                    row = {
                        'timestamp': datetime.now().isoformat(),
                        'mode': mode,
                        'parameter': param_name,
                        'parameter_value': val,
                        'max_dist_m': round(metrics['max_dist'], 4),
                        'fell': metrics['fell'],
                        'walk_status': metrics['walk_status'],
                        'duration_s': round(metrics['duration'], 2),
                    }
                    results.append(row)

                    status_str = "FELL" if metrics['fell'] else "OK"
                    print(
                        'dist=' + str(round(metrics['max_dist'], 3)) + 'm ' +
                        'status=' + str(metrics['walk_status']) + ' [' + status_str + ']'
                    )

                except Exception as e:
                    print('ERROR: ' + str(e))
                    row = {
                        'timestamp': datetime.now().isoformat(),
                        'mode': mode,
                        'parameter': param_name,
                        'parameter_value': val,
                        'max_dist_m': 0.0,
                        'fell': True,
                        'walk_status': -1,
                        'duration_s': 0.0,
                        'error': str(e),
                    }
                    results.append(row)

                # Brief pause between trials
                time.sleep(1.0)

    # Stop spinning
    collector.stop_trial()
    rclpy.shutdown()
    spin_thread.join(timeout=5.0)

    # Write CSV
    output_path = Path(args.output)
    if results:
        fieldnames = list(results[0].keys())
        with open(output_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)
        print('\nResults written to ' + str(output_path))
    else:
        print('\nNo results collected.')

    # Print summary
    print('\n' + '=' * 60)
    print('SUMMARY')
    print('=' * 60)
    for mode in modes_to_run:
        mode_results = [r for r in results if r['mode'] == mode]
        if not mode_results:
            continue
        # Best = max distance among non-falling trials
        non_falling = [r for r in mode_results if not r['fell']]
        if non_falling:
            best = max(non_falling, key=lambda r: r['max_dist_m'])
            print('\n' + mode.upper() + ':')
            print('  Best parameter: ' + best['parameter'] + '=' + str(best['parameter_value']))
            print('  Best distance: ' + str(round(best['max_dist_m'], 3)) + 'm')
            print('  Falls: ' + str(sum(1 for r in mode_results if r['fell'])) + '/' + str(len(mode_results)) + ' trials')
        else:
            print('\n' + mode.upper() + ': All trials fell or failed.')


if __name__ == '__main__':
    main()
