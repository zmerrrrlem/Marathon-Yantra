"""ROS-free heading hold and relative-turn logic.

Merges the old branch's ``heading_hold`` and ``altair_turn`` (turn supervisor +
cmd mux) into one state machine, so there is exactly one writer of the walk
command and no competing turn implementations.

Modes
-----
PASSTHROUGH  operator commands yaw (or holding is disabled): command forwarded as-is
HOLD         walking straight: yaw is closed on the IMU around a latched heading
TURNING      relative turn requested, large error: drive toward target
SETTLING     inside the capture band: wait until the error stays inside tolerance
DONE/ABORTED short hold-off, then back to HOLD/PASSTHROUGH

All angles are radians, all times are seconds.
"""

from dataclasses import dataclass
import math

PASSTHROUGH = 0
HOLD = 1
TURNING = 2
SETTLING = 3
DONE = 4
ABORTED = 5

STATE_NAMES = {
    PASSTHROUGH: 'PASSTHROUGH',
    HOLD: 'HOLD',
    TURNING: 'TURNING',
    SETTLING: 'SETTLING',
    DONE: 'DONE',
    ABORTED: 'ABORTED',
}

# quintic_walk WalkState enum (walk_utils.hpp):
# PAUSED, INIT, IDLE, START_MOVEMENT, WALKING, STOP_MOVEMENT, START_STEP, STOP_STEP, KICK
WALK_STATE_WALKING = 4
STEPPING_STATES = (3, 4, 6)


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    """Clamp value symmetrically to [-limit, limit]."""
    return max(-limit, min(limit, value))


@dataclass
class Twist2D:
    """The parts of geometry_msgs/Twist the walk engine uses."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0          # linear.z, passed through
    stop: float = 0.0       # angular.x, quintic_walk's "stop walking" signal (!= 0 stops)
    yaw_rate: float = 0.0   # angular.z


@dataclass
class HeadingParams:
    """Tunable parameters (mirrored as ROS parameters by the node)."""

    hold_enabled: bool = True
    hold_kp: float = 0.6               # rad/s of yaw command per rad of error
    hold_ki: float = 0.3               # rad/s per rad*s, removes the steady offset caused by constant drift
    hold_kd: float = 0.0               # rad/s per rad/s of measured yaw rate
    hold_max_rate: float = 0.15        # rad/s, max correction while holding
    hold_deadband: float = math.radians(0.5)
    hold_start_delay: float = 1.0      # s of steady walking before holding starts
    relatch_delay: float = 0.5         # s after the operator stops turning before re-latching heading
    operator_yaw_threshold: float = 1e-3

    turn_kp: float = 1.2
    turn_kd: float = 0.0
    turn_max_rate: float = 0.5         # rad/s
    turn_min_rate: float = 0.05        # rad/s, keeps the robot turning against step friction
    turn_in_place: bool = True         # zero linear command while turning
    capture_band: float = math.radians(8.0)
    settle_tolerance: float = math.radians(2.0)
    settle_time: float = 0.5           # s inside tolerance to finish
    turn_timeout: float = 30.0
    walk_start_grace: float = 3.0      # s allowed for the engine to start stepping after a request
    done_hold_off: float = 0.5         # s spent in DONE/ABORTED before returning

    yaw_sign: float = 1.0              # flip if positive angular.z turns the robot clockwise
    imu_timeout: float = 0.3
    yaw_filter_tau: float = 0.05       # s, low-pass on yaw (sin/cos filtered, wrap-safe)
    gyro_filter_tau: float = 0.05
    rate_limit: float = 2.0            # rad/s^2, max change of the yaw correction


class HeadingCore:
    """Heading/turn state machine. Feed it inputs, call update(now) at a fixed rate."""

    def __init__(self, params=None):
        self.p = params or HeadingParams()
        self.state = PASSTHROUGH
        self.state_since = 0.0
        self.last_event = ''

        self.cmd_in = None              # last operator command (None until the first one arrives)
        self.yaw = None                 # filtered yaw
        self._yaw_s = 0.0
        self._yaw_c = 0.0
        self.yaw_rate = 0.0
        self.imu_time = None
        self.walk_status = None         # None = engine does not publish walk status
        self.walking_since = None

        self.target = None
        self.turn_request = 0.0
        self.turn_start = None
        self.settle_since = None
        self.operator_turn_end = None
        self.correction = 0.0
        self.integral = 0.0
        self.last_update = None
        self.error = 0.0

    # ------------------------------------------------------------------ inputs
    def on_cmd(self, cmd, now):
        """Store an operator command. A stop command (angular.x != 0) cancels any turn."""
        if self.cmd_in is not None and abs(self.cmd_in.yaw_rate) > self.p.operator_yaw_threshold \
                and abs(cmd.yaw_rate) <= self.p.operator_yaw_threshold:
            self.operator_turn_end = now
        self.cmd_in = cmd
        if cmd.stop != 0.0 and self.state in (TURNING, SETTLING):
            self._abort('stop command received', now)

    def on_imu(self, yaw, yaw_rate, now):
        """Store an IMU sample (yaw from the orientation quaternion, yaw rate from the gyro)."""
        dt = 0.0 if self.imu_time is None else max(0.0, now - self.imu_time)
        s, c = math.sin(yaw), math.cos(yaw)
        if self.yaw is None:
            self._yaw_s, self._yaw_c = s, c
            self.yaw_rate = yaw_rate
        else:
            a = dt / (self.p.yaw_filter_tau + dt) if self.p.yaw_filter_tau > 0.0 else 1.0
            self._yaw_s += a * (s - self._yaw_s)
            self._yaw_c += a * (c - self._yaw_c)
            g = dt / (self.p.gyro_filter_tau + dt) if self.p.gyro_filter_tau > 0.0 else 1.0
            self.yaw_rate += g * (yaw_rate - self.yaw_rate)
        self.yaw = math.atan2(self._yaw_s, self._yaw_c)
        self.imu_time = now

    def on_walk_status(self, status, now):
        """Store the walk engine state (quintic_walk /walk_status)."""
        if status == WALK_STATE_WALKING:
            if self.walk_status != WALK_STATE_WALKING:
                self.walking_since = now
        else:
            self.walking_since = None
        self.walk_status = status

    def request_turn(self, angle, now):
        """Start a relative turn of ``angle`` rad (positive = counter-clockwise)."""
        if self.state in (TURNING, SETTLING):
            self.last_event = 'turn request ignored: turn already active'
            return False
        if not self.imu_fresh(now):
            self._set_state(ABORTED, now, 'turn rejected: IMU data missing or stale')
            return False
        if self.cmd_in is not None and self.cmd_in.stop != 0.0:
            self._set_state(ABORTED, now, 'turn rejected: stop command active')
            return False
        self.turn_request = angle
        self.target = wrap_angle(self.yaw + angle)
        self.turn_start = now
        self.settle_since = None
        self._set_state(TURNING, now, 'turn started: %.1f deg -> target %.1f deg'
                        % (math.degrees(angle), math.degrees(self.target)))
        return True

    def cancel_turn(self, now):
        """Abort an active turn."""
        if self.state in (TURNING, SETTLING):
            self._abort('cancel requested', now)

    def set_heading(self, absolute_yaw, now):
        """Set the heading to hold (absolute IMU yaw)."""
        self.target = wrap_angle(absolute_yaw)
        self.last_event = 'heading target set to %.1f deg' % math.degrees(self.target)

    # ----------------------------------------------------------------- helpers
    def imu_fresh(self, now):
        """Return True if a recent IMU sample exists."""
        return self.yaw is not None and self.imu_time is not None and \
            (now - self.imu_time) <= self.p.imu_timeout

    def is_stepping(self):
        """Return True if the engine is stepping (or unknown, for engines without walk status)."""
        if self.walk_status is None:
            return True
        return self.walk_status in STEPPING_STATES

    def _set_state(self, state, now, event=''):
        if state != self.state:
            self.integral = 0.0
        self.state = state
        self.state_since = now
        if event:
            self.last_event = event

    def _abort(self, reason, now):
        err = 0.0 if (self.yaw is None or self.target is None) else wrap_angle(self.target - self.yaw)
        self._set_state(ABORTED, now, 'turn aborted (%s) at error %.1f deg' % (reason, math.degrees(err)))
        # Do not drive back toward an abandoned target; the next hold latches the current heading
        self.target = None

    def _rate_limited(self, desired, dt):
        if self.p.rate_limit > 0.0 and dt > 0.0:
            step = self.p.rate_limit * dt
            desired = self.correction + clamp(desired - self.correction, step)
        self.correction = desired
        return desired

    def _hold_allowed(self, now):
        if not self.p.hold_enabled or self.cmd_in is None or not self.imu_fresh(now):
            return False
        if self.cmd_in.stop != 0.0 or abs(self.cmd_in.yaw_rate) > self.p.operator_yaw_threshold:
            return False
        if self.operator_turn_end is not None and now - self.operator_turn_end < self.p.relatch_delay:
            return False
        if self.walk_status is None:
            return True
        return self.walking_since is not None and now - self.walking_since >= self.p.hold_start_delay

    # ------------------------------------------------------------------ update
    def update(self, now):
        """Advance the state machine. Returns the Twist2D to send, or None to send nothing."""
        dt = 0.0 if self.last_update is None else max(0.0, now - self.last_update)
        self.last_update = now

        if self.state in (TURNING, SETTLING):
            return self._update_turn(now, dt)

        if self.state in (DONE, ABORTED):
            if now - self.state_since >= self.p.done_hold_off:
                self._set_state(PASSTHROUGH, now)
            else:
                # Brief neutral hold-off: keep the operator's translation, no yaw
                self.correction = 0.0
                if self.cmd_in is None:
                    return Twist2D()
                out = Twist2D(**vars(self.cmd_in))
                out.yaw_rate = 0.0
                return out

        if self.cmd_in is None:
            # Nothing commanded yet: do not publish, or a zero Twist would start the engine
            return None

        if self._hold_allowed(now):
            if self.state != HOLD:
                # Latch the current heading unless one is already set (finished turn or set_heading)
                if self.target is None:
                    self.target = self.yaw
                self._set_state(HOLD, now, 'holding heading %.1f deg' % math.degrees(self.target))
            self.error = wrap_angle(self.target - self.yaw)
            err = 0.0 if abs(self.error) < self.p.hold_deadband else self.error
            # Integral with anti-windup: the I share alone can never exceed the correction limit
            if self.p.hold_ki > 0.0:
                self.integral += err * dt
                i_limit = self.p.hold_max_rate / self.p.hold_ki
                self.integral = clamp(self.integral, i_limit)
            desired = self.p.hold_kp * err + self.p.hold_ki * self.integral - self.p.hold_kd * self.yaw_rate
            desired = clamp(desired, self.p.hold_max_rate)
            corr = self._rate_limited(desired, dt)
            out = Twist2D(**vars(self.cmd_in))
            out.yaw_rate = self.cmd_in.yaw_rate + self.p.yaw_sign * corr
            return out

        if self.state != PASSTHROUGH:
            self._set_state(PASSTHROUGH, now)
        # Forget the latched heading whenever the operator steers or stops the robot,
        # so the next hold latches the heading the robot actually has then
        if abs(self.cmd_in.yaw_rate) > self.p.operator_yaw_threshold or self.cmd_in.stop != 0.0:
            self.target = None
        self.correction = 0.0
        self.error = 0.0
        return Twist2D(**vars(self.cmd_in))

    def _update_turn(self, now, dt):
        elapsed = now - self.turn_start
        if not self.imu_fresh(now):
            self._abort('IMU data stale', now)
            return self.update(now)
        if elapsed > self.p.turn_timeout:
            self._abort('timeout after %.1f s' % elapsed, now)
            return self.update(now)
        if not self.is_stepping() and elapsed > self.p.walk_start_grace:
            self._abort('walk engine not stepping', now)
            return self.update(now)

        self.error = wrap_angle(self.target - self.yaw)
        if self.state == TURNING and abs(self.error) < self.p.capture_band:
            self._set_state(SETTLING, now)
            self.settle_since = None
        if self.state == SETTLING:
            if abs(self.error) <= self.p.settle_tolerance:
                if self.settle_since is None:
                    self.settle_since = now
                if now - self.settle_since >= self.p.settle_time:
                    self._set_state(DONE, now, 'turn done: %.1f deg in %.1f s, final error %.2f deg'
                                    % (math.degrees(self.turn_request), elapsed,
                                       math.degrees(self.error)))
                    # Keep holding the new heading afterwards
                    return self.update(now)
            else:
                self.settle_since = None
                if abs(self.error) > 2.0 * self.p.capture_band:
                    self._set_state(TURNING, now)

        desired = self.p.turn_kp * self.error - self.p.turn_kd * self.yaw_rate
        desired = clamp(desired, self.p.turn_max_rate)
        if self.state == TURNING and abs(desired) < self.p.turn_min_rate:
            desired = math.copysign(self.p.turn_min_rate, self.error)
        corr = self._rate_limited(desired, dt)

        base = self.cmd_in if self.cmd_in is not None else Twist2D()
        out = Twist2D(x=base.x, y=base.y, z=base.z, stop=0.0, yaw_rate=self.p.yaw_sign * corr)
        if self.p.turn_in_place:
            out.x = 0.0
            out.y = 0.0
        return out
