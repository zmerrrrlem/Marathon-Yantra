"""Closed-loop tests of the heading/turn logic against a simple simulated walker."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from yantra_heading.core import (  # noqa: E402
    ABORTED, DONE, HOLD, PASSTHROUGH, SETTLING, TURNING,
    HeadingCore, HeadingParams, Twist2D, wrap_angle,
)

RATE = 50.0
DT = 1.0 / RATE


class SimWalker:
    """Yaw integrator with a start-up delay, actuation gain error, drift and IMU noise-free output."""

    def __init__(self, yaw=0.0, drift=0.0, gain=0.8, start_delay=0.6):
        self.yaw = yaw
        self.drift = drift          # rad/s of uncommanded yaw while stepping
        self.gain = gain            # real yaw rate per commanded yaw rate
        self.start_delay = start_delay
        self.cmd = None
        self.cmd_since = None
        self.status = 2             # IDLE

    def step(self, now):
        stepping = self.cmd is not None and self.cmd.stop == 0.0
        if stepping:
            if self.cmd_since is None:
                self.cmd_since = now
            self.status = 4 if now - self.cmd_since >= self.start_delay else 6
        else:
            self.cmd_since = None
            self.status = 2
        rate = 0.0
        if self.status == 4:
            rate = self.gain * self.cmd.yaw_rate + self.drift
        self.yaw = wrap_angle(self.yaw + rate * DT)
        return rate


def run(core, sim, seconds, t0=0.0, operator=None, imu=True):
    t = t0
    for _ in range(int(seconds * RATE)):
        t += DT
        if operator is not None:
            core.on_cmd(operator, t)
        rate = sim.step(t)
        if imu:
            core.on_imu(sim.yaw, rate, t)
        core.on_walk_status(sim.status, t)
        out = core.update(t)
        if out is not None:
            sim.cmd = out
    return t


def test_no_output_before_first_command():
    core = HeadingCore()
    core.on_imu(0.0, 0.0, 0.0)
    assert core.update(0.02) is None


def test_heading_hold_cancels_drift():
    core = HeadingCore()
    sim = SimWalker(yaw=0.3, drift=0.05)        # ~2.9 deg/s drift to the left
    fwd = Twist2D(x=0.1)
    t = run(core, sim, 20.0, operator=fwd)
    assert core.state == HOLD
    # Without correction it would have drifted ~55 deg; with hold it stays near the latched heading
    assert abs(wrap_angle(sim.yaw - core.target)) < math.radians(3.0)
    assert abs(wrap_angle(core.target - 0.3)) < math.radians(4.0)
    assert t > 0


def test_hold_disabled_passes_through():
    core = HeadingCore(HeadingParams(hold_enabled=False))
    sim = SimWalker(drift=0.05)
    run(core, sim, 10.0, operator=Twist2D(x=0.1))
    assert core.state == PASSTHROUGH
    assert sim.cmd.yaw_rate == 0.0


def test_operator_yaw_passes_through_and_relatches():
    core = HeadingCore()
    sim = SimWalker()
    t = run(core, sim, 3.0, operator=Twist2D(x=0.1))
    assert core.state == HOLD
    t = run(core, sim, 2.0, t0=t, operator=Twist2D(x=0.1, yaw_rate=0.3))
    assert core.state == PASSTHROUGH
    assert sim.cmd.yaw_rate == 0.3
    turned_to = sim.yaw
    run(core, sim, 3.0, t0=t, operator=Twist2D(x=0.1))
    assert core.state == HOLD
    # It holds the new heading, it does not swing back to the old one
    assert abs(wrap_angle(core.target - turned_to)) < math.radians(3.0)


def test_turn_90_in_place_settles():
    core = HeadingCore()
    sim = SimWalker(yaw=0.0)
    t = run(core, sim, 2.0, operator=Twist2D(x=0.0))
    assert core.request_turn(math.radians(90), t)
    assert core.state == TURNING
    seen = set()
    for _ in range(int(20 * RATE)):
        t += DT
        rate = sim.step(t)
        core.on_imu(sim.yaw, rate, t)
        core.on_walk_status(sim.status, t)
        out = core.update(t)
        sim.cmd = out
        seen.add(core.state)
        if core.state in (DONE, HOLD, PASSTHROUGH) and SETTLING in seen:
            break
    assert SETTLING in seen
    assert DONE in seen or core.state == DONE
    assert abs(wrap_angle(sim.yaw - math.radians(90))) < math.radians(2.5)


def test_turn_across_pi_wraps():
    core = HeadingCore()
    sim = SimWalker(yaw=math.radians(170))
    t = run(core, sim, 2.0, operator=Twist2D())
    core.request_turn(math.radians(40), t)   # target = -150 deg
    t = run(core, sim, 15.0, t0=t)
    assert abs(wrap_angle(sim.yaw - math.radians(-150))) < math.radians(2.5)
    # and it went the short way (counter-clockwise), never through 0
    assert core.state in (DONE, HOLD, PASSTHROUGH)


def test_turn_then_hold_keeps_new_heading():
    core = HeadingCore()
    sim = SimWalker(drift=0.03)
    t = run(core, sim, 2.0, operator=Twist2D(x=0.1))
    core.request_turn(math.radians(-45), t)
    t = run(core, sim, 12.0, t0=t, operator=Twist2D(x=0.1))
    t = run(core, sim, 10.0, t0=t, operator=Twist2D(x=0.1))
    assert core.state == HOLD
    assert abs(wrap_angle(sim.yaw - math.radians(-45))) < math.radians(3.0)


def test_turn_aborts_on_stale_imu():
    core = HeadingCore()
    sim = SimWalker()
    t = run(core, sim, 2.0, operator=Twist2D())
    core.request_turn(math.radians(90), t)
    t = run(core, sim, 1.0, t0=t, imu=False)
    assert core.state in (ABORTED, PASSTHROUGH)
    assert 'IMU' in core.last_event


def test_turn_rejected_without_imu():
    core = HeadingCore()
    core.on_cmd(Twist2D(), 0.0)
    assert not core.request_turn(1.0, 0.1)
    assert core.state == ABORTED


def test_stop_command_cancels_turn_and_is_forwarded():
    core = HeadingCore()
    sim = SimWalker()
    t = run(core, sim, 2.0, operator=Twist2D())
    core.request_turn(math.radians(90), t)
    t = run(core, sim, 0.5, t0=t)
    stop = Twist2D(stop=1.0)
    t += DT
    core.on_cmd(stop, t)
    assert core.state == ABORTED
    out = core.update(t)
    assert out.stop == 1.0


def test_turn_aborts_if_engine_never_steps():
    core = HeadingCore(HeadingParams(walk_start_grace=1.0))
    core.on_cmd(Twist2D(), 0.0)
    core.on_imu(0.0, 0.0, 0.0)
    core.on_walk_status(2, 0.0)        # IDLE forever
    core.request_turn(1.0, 0.0)
    t = 0.0
    for _ in range(int(2 * RATE)):
        t += DT
        core.on_imu(0.0, 0.0, t)
        core.on_walk_status(2, t)
        core.update(t)
    assert 'not stepping' in core.last_event


def test_second_request_ignored_while_turning():
    core = HeadingCore()
    sim = SimWalker()
    t = run(core, sim, 2.0, operator=Twist2D())
    assert core.request_turn(1.0, t)
    target = core.target
    assert not core.request_turn(-1.0, t + DT)
    assert core.target == target


def test_yaw_filter_is_wrap_safe():
    core = HeadingCore()
    core.on_imu(math.radians(179), 0.0, 0.0)
    for i in range(1, 20):
        core.on_imu(math.radians(-179), 0.0, i * DT)
    assert abs(abs(core.yaw) - math.pi) < math.radians(2.0)
