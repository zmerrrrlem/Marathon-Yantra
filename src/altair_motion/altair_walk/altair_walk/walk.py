import sys
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32, Bool
from geometry_msgs.msg import Vector3, Vector3Stamped, QuaternionStamped, Twist
from sensor_msgs.msg import Imu, JointState

from .preview_controller import *
from .inverse_kinematic import *
from .fuzzy_logic import *


class WalkStatus(Enum):
    STOP = -1
    RESET = 0
    START = 1


# feedback_mode.data = 0 -> no feedback , 1 -> ankle feedback, 2 -> step feedback
class Feedback(Enum):
    NO_FEEDBACK = 0
    ANKLE = 1
    STEP = 2


class AltairWalk(Node):
    def __init__(self):
        super().__init__("altair_walk_node")
        self.logger = self.get_logger()

        self.imu_msg = Imu()
        self.quat_msg = QuaternionStamped()
        self.euler_msg = Vector3Stamped()
        self.gyr_msg = Vector3Stamped()
        self.acc_msg = Vector3Stamped()
        self.button_msg = Int8()
        self.com_msg = Vector3()
        self.cmd_vel = Twist()
        self.walk_status = Int32()
        self.walk_mode = Int32()
        self.feedback_mode = Int32()
        self.walk_state = Int32()
        self.gain_ctrl = Twist()
        self.joint_msg = JointState()
        self.is_falling = False

        self.euler_data = Vector3()
        self.gyr_data = Vector3()
        self.igl_data = np.array([0.0, 0.0, 0.0])

        self.FOOT_DISTANCE = 62 / 1000
        self.COM_HEIGHT = 375 / 1000
        self.X_OFFSET = 30 / 1000
        self.COM_SWING = 10 / 1000

        self.COM = np.array([self.X_OFFSET, 0.0, self.COM_HEIGHT])
        self.LEFT = np.array([0.0, self.FOOT_DISTANCE, 0.0])
        self.RIGHT = np.array([0.0, -self.FOOT_DISTANCE, 0.0])

        self.uCOM = np.array([self.X_OFFSET, 0.0, self.COM_HEIGHT])
        self.uLEFT = np.array([0.0, self.FOOT_DISTANCE, 0.0])
        self.uRIGHT = np.array([0.0, -self.FOOT_DISTANCE, 0.0])

        self.pc = PreviewControl(
            self.COM_HEIGHT, self.COM_SWING, self.X_OFFSET, self.FOOT_DISTANCE
        )
        self.ik = InverseKinematic()
        self.fz = FuzzyLogic()

        self.debug = False
        self.flag = False
        self.step = False
        self.finish = False
        self.JOINTS = np.zeros(12)
        self.joint_msg.name = [
            "l_hip_yaw",
            "l_hip_roll",
            "l_hip_pitch",
            "l_knee",
            "l_ank_pitch",
            "l_ank_roll",
            "r_hip_yaw",
            "r_hip_roll",
            "r_hip_pitch",
            "r_knee",
            "r_ank_pitch",
            "r_ank_roll",
        ]
        self.joint_msg.position = np.zeros(12).tolist()

        # Minimum jerk transition state variables
        self.is_transitioning = False
        self.transition_start_time = None
        self.transition_duration = 2
        self.transition_start_joints = np.zeros(12)
        self.transition_target_joints = np.zeros(12)
        self.previous_walk_status = WalkStatus.STOP.value  # Track status changes
        self.transition_phase = 0  # 0=idle, 1=zeros->stand, 2=stand->walk

        self.declare_parameter('roll_p', 0.042)
        self.declare_parameter('roll_d', 0.037)
        self.declare_parameter('max_cmd_a', 0.0)

        self.start = 0.0
        self.now = 0.0
        self.delta_step = 0.0

        # Tilt blending (Option C): slew ik.TILT toward target instead of instant jump
        self.ik_tilt_current = 10.0
        self.ik_tilt_target = 10.0
        self.ik_tilt_slew_deg_s = 5.0

        self.imu_sub = self.create_subscription(
            Imu, "/subcon/imu", self.imu_callback, 1
        )
        self.euler_sub = self.create_subscription(
            Vector3Stamped, "/subcon/euler_data", self.euler_callback, 1
        )
        self.gyr_sub = self.create_subscription(
            Vector3Stamped, "/subcon/gyr_data", self.gyr_callback, 1
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 1
        )
        self.walk_status_sub = self.create_subscription(
            Int32, "walk_status", self.walk_status_callback, 1
        )
        self.walk_mode_sub = self.create_subscription(
            Int32, "walk_mode", self.walk_mode_callback, 1
        )
        self.feedback_mode_sub = self.create_subscription(
            Int32, "feedback_mode", self.feedback_mode_callback, 1
        )
        self.gain_sub = self.create_subscription(
            Twist, "gain_control", self.gain_callback, 1
        )
        self.falling_sub = self.create_subscription(
            Bool, "is_falling", self.falling_callback, 1
        )

        self.joint_pub = self.create_publisher(JointState, "altair_walk/joint_state", 1)
        self.walk_state_pub = self.create_publisher(Int32, "altair_walk/walk_status", 1)

        self.timer_period = 1.0 / 17.0
        self.run_cb = self.create_timer(self.timer_period, self.process)

        # Start at zeros to match Gazebo's initial pose (straight legs)
        self.COM = np.array([self.X_OFFSET, 0.0, self.COM_HEIGHT])
        self.LEFT = np.array([0.0, self.FOOT_DISTANCE, 0.0])
        self.RIGHT = np.array([0.0, -self.FOOT_DISTANCE, 0.0])
        self.JOINTS = np.zeros(12)



    def imu_callback(self, msg):
        self.imu_msg = msg

    def euler_callback(self, msg):
        self.euler_data = msg.vector

    def gyr_callback(self, msg):
        self.gyr_data = msg.vector

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def walk_status_callback(self, msg):
        self.walk_status = msg

    def walk_mode_callback(self, msg):
        self.walk_mode = msg

    def feedback_mode_callback(self, msg):
        self.feedback_mode = msg

    def gain_callback(self, msg):
        self.gain_ctrl = msg

    def falling_callback(self, msg):
        self.is_falling = msg.data

    def bezier_curve(self, phase, p_start, p_cnt1, p_cnt2, p_end):
        if phase >= 1:
            phase = 1

        # Cubic Bezier: B(t) = (1-t)^3*P0 + 3(1-t)^2*t*P1 + 3(1-t)*t^2*P2 + t^3*P3
        t = np.matrix([1, phase, phase**2, phase**3])
        coef = np.matrix([[1, 0, 0, 0], [-3, 3, 0, 0], [3, -6, 3, 0], [-1, 3, -3, 1]])
        point = np.vstack((p_start, p_cnt1, p_cnt2, p_end))
        path = t * coef * point

        return path

    def minimum_jerk_interpolation(self, phase, q_start, q_end):
        """
        Compute minimum jerk trajectory in joint space.

        Args:
            phase: normalized time (0 to 1)
            q_start: starting joint configuration (numpy array, 12 joints)
            q_end: target joint configuration (numpy array, 12 joints)

        Returns:
            q_current: interpolated joint configuration (numpy array, 12 joints)

        Formula: q(τ) = q0 + (qf - q0) * (10τ³ - 15τ⁴ + 6τ⁵)
        This produces zero velocity and acceleration at start and end points.
        """
        if phase >= 1.0:
            phase = 1.0
        elif phase <= 0.0:
            phase = 0.0

        # Minimum jerk polynomial: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
        s = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5

        # Interpolate: q(τ) = q_start + (q_end - q_start) * s(τ)
        q_current = q_start + (q_end - q_start) * s

        return q_current

    def _validate_joints(self, joints, context="assignment"):
        """Validate JOINTS array has correct shape (12,) and no NaN values."""
        if joints is None:
            self.logger.error(f"JOINTS is None during {context}")
            return False
        if not isinstance(joints, np.ndarray):
            self.logger.error(f"JOINTS is not ndarray during {context}: {type(joints)}")
            return False
        if joints.shape != (12,):
            self.logger.error(f"JOINTS shape invalid during {context}: {joints.shape} (expected (12,))")
            return False
        if np.any(np.isnan(joints)):
            self.logger.error(f"JOINTS contains NaN during {context}")
            return False
        return True

    def initiate_joint_transition(self, target_joints):
        """
        Start a smooth minimum jerk transition to target joint configuration.

        Args:
            target_joints: target joint configuration (numpy array, 12 joints)

        This method can be called at any time, even during an ongoing transition.
        It will smoothly transition from the current joint position to the new target.
        """
        # Store current joint positions as start (handles interrupts gracefully)
        self.transition_start_joints = self.JOINTS.copy()

        # Store target joint positions
        self.transition_target_joints = np.array(target_joints).copy()

        # Record start time
        self.transition_start_time = self.get_clock().now()

        # Mark transition as active
        self.is_transitioning = True

        self.logger.info(
            f"Initiated joint transition (duration: {self.transition_duration}s)"
        )

    def update_joint_transition(self):
        """
        Update ongoing joint space transition using minimum jerk trajectory.

        Returns:
            bool: True if transition is complete, False if still ongoing
        """
        if not self.is_transitioning:
            return True

        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed = (
            current_time - self.transition_start_time
        ).nanoseconds / 1e9  # Convert to seconds

        # Compute phase (normalized time from 0 to 1)
        phase = elapsed / self.transition_duration

        # Check if transition is complete
        if phase >= 1.0:
            # Set final position
            self.JOINTS = self.transition_target_joints.copy()
            if not self._validate_joints(self.JOINTS, "transition complete"):
                return True
            self.logger.info("Joint transition completed")
            return True

        # Interpolate joint positions using minimum jerk trajectory
        self.JOINTS = self.minimum_jerk_interpolation(
            phase, self.transition_start_joints, self.transition_target_joints
        )
        if not self._validate_joints(self.JOINTS, "transition interpolation"):
            return False

        return False

    def set_forward_offset(self, value):
        self.X_OFFSET = value
        self.COM[0] = value
        self.pc.x_offset = value

    def cmd_a_from_angular(self, angular_z):
        cmd_a = -angular_z  # sim yaw response is inverted vs pc frame
        max_cmd_a = self.get_parameter('max_cmd_a').value
        if max_cmd_a > 0.0:
            cmd_a = float(np.clip(cmd_a, -max_cmd_a, max_cmd_a))
        return cmd_a

    def stand(self, ik):
        """
        Transition to stand pose using smooth minimum jerk trajectory.

        Instead of immediately setting joint positions, this initiates a smooth
        transition from the current pose to the stand pose.
        """
        # Update task space references (for potential use in other calculations)
        self.COM = np.array([self.X_OFFSET, 0.0, self.COM_HEIGHT])
        self.LEFT = np.array([0.0, self.FOOT_DISTANCE, 0.0])
        self.RIGHT = np.array([0.0, -self.FOOT_DISTANCE, 0.0])

        # Compute target stand joint configuration using inverse kinematics
        target_joints = ik.solve(self.COM, self.LEFT, self.RIGHT)
        if target_joints is None:
            self.logger.error("IK failed - target unreachable, holding current pose")
            return

        # Initiate smooth transition to stand pose
        self.initiate_joint_transition(target_joints)

        # Reset walk-related flags
        self.finish = False
        self.step = False
        self.delta_step = 0.0

    def stand_immediate(self, ik):
        """
        Immediately set stand pose without transition.
        Used for initialization or emergency stop.
        """
        self.COM = np.array([self.X_OFFSET, 0.0, self.COM_HEIGHT])
        self.LEFT = np.array([0.0, self.FOOT_DISTANCE, 0.0])
        self.RIGHT = np.array([0.0, -self.FOOT_DISTANCE, 0.0])
        joints = ik.solve(self.COM, self.LEFT, self.RIGHT)
        if joints is None or not self._validate_joints(joints, "stand_immediate"):
            self.logger.error("IK failed in stand_immediate, keeping previous joints")
            return

        self.JOINTS = joints
        self.is_transitioning = False

        # Reset walk-related flags
        self.finish = False
        self.step = False
        self.delta_step = 0.0

    def open_loop_walk(self, pc, ik):
        pc.update_walking_pattern()
        # print("pc.walking_ready", pc.walking_ready)
        if pc.walking_ready and not self.debug:
            trunk_yaw = -self.euler_data.z
            joints = ik.solve(
                pc.com_pose, pc.left_foot, pc.right_foot,
                pc.left_yaw, pc.right_yaw, trunk_yaw
            )
        else:
            joints = ik.solve(self.COM, self.LEFT, self.RIGHT)

        if joints is not None and self._validate_joints(joints, "open_loop_walk"):
            self.JOINTS = joints
        else:
            self.logger.warn("IK failed - target unreachable, holding previous joints")

    def push_recovery_static(self, fz, ik):
        time_step = 0.1
        if self.finish == False:
            if (self.is_falling) and not (self.step):
                joints = ik.solve(self.COM, self.LEFT, self.RIGHT)
                if joints is not None and self._validate_joints(joints, "push_recovery_static"):
                    self.JOINTS = joints
                else:
                    self.logger.warn("IK failed in push_recovery_static, holding previous joints")
                self.start = self.get_clock().now()
                self.delta_step = (
                    fz.compute(
                        -self.euler_data.y * 180 / np.pi, self.gyr_data.y * 180 / np.pi
                    )
                    / 100
                )
                self.uCOM = self.COM
                self.uLEFT = self.LEFT
                self.uRIGHT = self.RIGHT
            elif not (self.finish):
                self.now = self.get_clock().now() - self.start
                phase = self.now / time_step
                if phase >= 1:
                    phase = 1
                    self.finish = True
                self.COM = self.bezier_curve(
                    phase,
                    self.uCOM,
                    np.array(
                        [
                            self.X_OFFSET + (self.delta_step / 4.0),
                            -0.03,
                            self.COM_HEIGHT,
                        ]
                    ),
                    np.matrix(
                        [
                            self.X_OFFSET + (self.delta_step / 2.0),
                            -0.005,
                            self.COM_HEIGHT,
                        ]
                    ),
                )
            if True:  # Change the swing leg you want
                self.LEFT = self.bezier_curve(
                    phase,
                    self.uLEFT,
                    np.array([(self.delta_step / 2.0), 48 / 1000, 0.08]),
                    np.array([(self.delta_step), 58 / 1000, 0.08]),
                    np.array([(self.delta_step), 58 / 1000, 0.0]),
                )
            else:
                self.RIGHT = self.bezier_curve(
                    phase,
                    self.uRIGHT,
                    np.array([(self.delta_step / 2.0), -48 / 1000, 0.05]),
                    np.array([(self.delta_step), -48 / 1000, 0.05]),
                    np.array([(self.delta_step), -48 / 1000, 0.05]),
                )

            joints = ik.solve(self.COM, self.LEFT, self.RIGHT)
            if joints is not None and self._validate_joints(joints, "push_recovery_static"):
                self.JOINTS = joints
                self.step = True
            else:
                self.logger.warn("IK failed in push_recovery_static swing, holding previous joints")

    def push_recovery_walk(self, pc, fz, ik):
        time_step = 0.1
        pc.update_walking_pattern()
        if self.finish == False:
            if pc.walking_ready and not self.debug:
                if not (self.is_falling) and not (self.step):
                    # print("WALK")
                    trunk_yaw = -self.euler_data.z
                    joints = ik.solve(
                        pc.com_pose, pc.left_foot, pc.right_foot,
                        pc.left_yaw, pc.right_yaw, trunk_yaw
                    )
                    if joints is not None and self._validate_joints(joints, "push_recovery_walk"):
                        self.JOINTS = joints
                    else:
                        self.logger.warn("IK failed in push_recovery_walk, holding previous joints")
                    self.start = self.get_clock().now()
                    self.delta_step = (
                        fz.compute(
                            -self.euler_data.y * 180 / np.pi,
                            self.gyr_data.y * 180 / np.pi,
                        )
                        / 100
                    )
                    self.uCOM = pc.com_pose
                    self.uLEFT = pc.left_foot
                    self.uRIGHT = pc.right_foot

                elif not (self.finish):
                    # print("STEP")
                    self.now = self.get_clock().now() - self.start
                    phase = self.now / time_step

                    if phase >= 1:
                        phase = 1
                        self.finish = True

                    self.COM = self.bezier_curve(
                        phase,
                        self.uCOM,
                        np.array(
                            [
                                self.X_OFFSET + (self.delta_step / 4.0),
                                -0.03,
                                self.COM_HEIGHT,
                            ]
                        ),
                        np.array(
                            [
                                self.X_OFFSET + (self.delta_step / 2.0),
                                0.01,
                                self.COM_HEIGHT,
                            ]
                        ),
                    )

                    if pc.support_foot == -1:
                        self.LEFT = self.bezier_curve(
                            phase,
                            self.uLEFT,
                            np.array([(self.delta_step / 2.0), 48 / 1000, 0.08]),
                            np.array([(self.delta_step), 62 / 1000, 0.08]),
                            np.array([(self.delta_step), 62 / 1000, 0.0]),
                        )
                    else:
                        self.RIGHT = self.bezier_curve(
                            phase,
                            self.uRIGHT,
                            np.array([(self.delta_step / 2.0), -48 / 1000, 0.08]),
                            np.array([(self.delta_step), -62 / 1000, 0.08]),
                            np.array([(self.delta_step), -62 / 1000, 0.0]),
                        )

                    joints = ik.solve(self.COM, self.LEFT, self.RIGHT)
                    if joints is not None and self._validate_joints(joints, "push_recovery_walk"):
                        self.JOINTS = joints
                        self.step = True
                    else:
                        self.logger.warn("IK failed in push_recovery_walk swing, holding previous joints")
            else:
                self.JOINTS = ik.solve(self.COM, self.LEFT, self.RIGHT)

    def initialize(self):
        pass

    def process(self):
        """
        walk_status.data = 1 -> start , -1 -> stop , 0 -> reset
        walk_mode.data = 1 -> walk , 0 -> static
        feedback_mode.data = 0 -> no feedback , 1 -> ankle feedback, 2 -> step feedback
        """
        # Report actual walking state for external observers (e.g. heading_hold)
        self.walk_state.data = (
            WalkStatus.START.value if self.transition_phase == 2 else WalkStatus.STOP.value
        )
        self.walk_state_pub.publish(self.walk_state)

        # PRIORITY 1: Handle ongoing minimum jerk transitions
        # If a transition is active, update it and publish joints
        # This takes precedence over all other behaviors to ensure smooth transitions
        if self.is_transitioning:
            transition_complete = self.update_joint_transition()

            # Publish joint positions
            self.joint_msg.position = self.JOINTS.tolist()
            self.joint_pub.publish(self.joint_msg)

            # If transition is complete, clear the flag
            if transition_complete:
                self.is_transitioning = False

            # Return early - don't execute other behaviors during transition
            return

        # PRIORITY 3: Regular walking/standing behaviors (only when not transitioning)
        # print("process loop")
        if self.walk_status.data == WalkStatus.STOP.value or self.walk_status.data == WalkStatus.RESET.value:
            # STOP (-1) or RESET (0): stay at current pose (zeros initially), reset preview controller
            # print("WalkStatus.STOP/RESET")
            self.pc = PreviewControl(
                self.COM_HEIGHT, self.COM_SWING, self.X_OFFSET, self.FOOT_DISTANCE
            )
            # Don't call stand() - stay at zeros to match Gazebo
            self.previous_walk_status = self.walk_status.data
            self.transition_phase = 0  # Reset for next start
        elif self.walk_status.data == WalkStatus.START.value:  # 1
            # Phase 1: zeros -> stand pose
            if self.transition_phase == 0 and (
                self.previous_walk_status == WalkStatus.STOP.value or
                self.previous_walk_status == WalkStatus.RESET.value
            ):
                self.logger.info(
                    "Detected STOP/RESET->START transition, initiating transition to stand pose"
                )

                # Compute stand pose
                self.set_forward_offset(30 / 1000)
                self.stand(self.ik)
                self.transition_phase = 1  # zeros -> stand
                self.previous_walk_status = WalkStatus.START.value

                # Return early to let transition complete
                return

            # Phase 2: stand pose -> first walking pose
            if self.transition_phase == 1 and not self.is_transitioning:
                self.logger.info(
                    "Stand pose reached, transitioning to first walking pose"
                )

                # Update preview controller to get first walking pose
                self.set_forward_offset(45 / 1000)
                self.pc.cmd_x = self.cmd_vel.linear.x
                self.pc.cmd_y = self.cmd_vel.linear.y
                self.pc.cmd_a = self.cmd_a_from_angular(self.cmd_vel.angular.z)
                self.pc.update_walking_pattern()

                # Compute first walking joint configuration
                if self.pc.walking_ready:
                    first_walk_joints = self.ik.solve(
                        self.pc.com_pose, self.pc.left_foot, self.pc.right_foot,
                        self.pc.left_yaw, self.pc.right_yaw,
                        0.5 * (self.pc.left_yaw + self.pc.right_yaw)
                    )
                else:
                    # If not ready, use stand pose
                    first_walk_joints = self.ik.solve(self.COM, self.LEFT, self.RIGHT)

                # Initiate smooth transition to first walking pose
                self.initiate_joint_transition(first_walk_joints)
                self.transition_phase = 2  # stand -> walk

                # Return early to let transition complete
                return

            # Phase 3: normal walking behavior (after both transitions complete)
            if self.transition_phase == 2 and not self.is_transitioning:
                pass  # Fall through to normal walking behavior below

            # If still transitioning, return early
            if self.is_transitioning:
                return

            # Normal walking behavior (after initial transition is complete)
            # print("WalkStatus.START")
            self.pc.cmd_x = self.cmd_vel.linear.x
            self.pc.cmd_y = self.cmd_vel.linear.y
            self.pc.cmd_a = self.cmd_a_from_angular(self.cmd_vel.angular.z)

            # Option C: blend ik.TILT toward target instead of instant jump
            if self.walk_mode.data == 1:
                self.ik_tilt_target = 15.0
            else:
                self.ik_tilt_target = 10.0

            dt = self.timer_period
            tilt_error = self.ik_tilt_target - self.ik_tilt_current
            tilt_step = self.ik_tilt_slew_deg_s * dt
            self.ik_tilt_current += float(
                max(-tilt_step, min(tilt_step, tilt_error))
            )
            self.ik.TILT = self.ik_tilt_current

            # Option B: forward offset only set during stand->walk transition
            # (phase 1->2 above), not every cycle in walking branches
            if (self.walk_mode.data == 1) and (
                self.feedback_mode.data == Feedback.STEP.value
            ):
                self.push_recovery_walk(self.pc, self.fz, self.ik)
                # print("push_recovery_walk")
            elif self.walk_mode.data == 1:
                self.open_loop_walk(self.pc, self.ik)
                # print("open_loop_walk")
            elif self.feedback_mode.data == Feedback.STEP.value:
                self.ik.TILT = 10.0
                self.push_recovery_static(self.fz, self.ik)
                # print("push_recovery_static")
            else:
                self.set_forward_offset(30 / 1000)
                self.stand(self.ik)

            if (
                (self.feedback_mode.data == 1) or (self.feedback_mode.data == 2)
            ) and not (self.finish):
                if not self._validate_joints(self.JOINTS, "feedback_control"):
                    self.logger.warn("Skipping feedback - JOINTS invalid")
                else:
                    self.igl_data[1] += self.euler_data.y / 30

                    roll_p = self.get_parameter('roll_p').value
                    roll_d = self.get_parameter('roll_d').value

                    delta_roll = roll_p * self.euler_data.x + roll_d * self.gyr_data.x
                    delta_pitch = (
                        -self.gain_ctrl.linear.y * self.euler_data.y
                        + -self.gain_ctrl.angular.y * self.gyr_data.y
                        + self.gain_ctrl.angular.x * self.igl_data[1]
                    )
                    self.JOINTS[3] += delta_pitch  # l_knee
                    self.JOINTS[5] += delta_roll  # l_ank_roll
                    self.JOINTS[9] -= delta_pitch  # r_knee
                    self.JOINTS[11] -= delta_roll  # r_ank_roll

            if (
                (self.feedback_mode.data == 1)
                or (self.feedback_mode.data == 2)
                or (self.feedback_mode.data == 3)
            ) and not (self.finish):
                if not self._validate_joints(self.JOINTS, "feedback_control"):
                    self.logger.warn("Skipping feedback - JOINTS invalid")
                else:
                    # Heading hold: hip yaw trim on yaw error + yaw rate (P/D)
                    delta_yaw = -(
                        self.gain_ctrl.linear.z * self.euler_data.z
                        + self.gain_ctrl.angular.z * self.gyr_data.z
                    )
                    delta_yaw = float(np.clip(delta_yaw, -0.35, 0.35))
                    self.JOINTS[0] += delta_yaw  # l_hip_yaw
                    self.JOINTS[6] += delta_yaw  # r_hip_yaw

            # Keep track that we're in START mode
            self.previous_walk_status = WalkStatus.START.value

        if self._validate_joints(self.JOINTS, "publish"):
            self.joint_msg.position = self.JOINTS.tolist()
            self.joint_pub.publish(self.joint_msg)
        else:
            self.logger.error("Skipping publish - JOINTS invalid")


def main():
    rclpy.init()
    node = AltairWalk()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
