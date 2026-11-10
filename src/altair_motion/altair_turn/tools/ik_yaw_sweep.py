#!/usr/bin/env python3
"""
Offline validation of the trunk-yaw IK fix.

Run after `altair_start` so altair_walk is importable, then:
    python3 src/altair_motion/altair_turn/tools/ik_yaw_sweep.py

Checks:
- straight invariance: trunk_yaw defaults keep the old straight output
- planted and mid-swing poses on a turning gait stay reachable when the
  matching trunk yaw is passed to the IK
- hip-yaw joints stay below the configured limit on matched poses
- the mismatch reference (trunk_yaw=0 on rotated poses) is reported
"""

import math
import sys

from altair_walk.inverse_kinematic import InverseKinematic
import numpy as np

COM_HEIGHT = 375 / 1000
X_OFFSET = 45 / 1000
FOOT_DISTANCE = 62 / 1000
HIP_YAW_LIMIT_DEG = 15.0
YAW_DEGREES = [0.0, 30.0, 60.0, 90.0, 135.0, 180.0]
CMD_A_VALUES = [0.0, 0.02, 0.04, 0.06, 0.08]


def rotated_foot(yaw, side):
    """Return the sole position of a foot at +/-FOOT_DISTANCE rotated by yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    y = side * FOOT_DISTANCE
    return np.array([-s * y, c * y, 0.0])


def hip_yaw_deg(q):
    """Return the largest hip-yaw joint magnitude in degrees."""
    return max(abs(math.degrees(q[0])), abs(math.degrees(q[6])))


def solve_case(ik, com, left, right, left_yaw, right_yaw, trunk_yaw):
    """Solve one pose and return (reachable, max hip yaw in deg or None)."""
    q = ik.solve(com, left, right, left_yaw, right_yaw, trunk_yaw)
    if q is None or not np.all(np.isfinite(q)):
        return False, None
    return True, hip_yaw_deg(q)


def check_invariance(ik):
    """Verify trunk_yaw default does not change the straight solution."""
    com = np.array([X_OFFSET, 0.0, COM_HEIGHT])
    left = rotated_foot(0.0, 1)
    right = rotated_foot(0.0, -1)
    q_old = ik.solve(com, left, right, 0.0, 0.0)
    q_new = ik.solve(com, left, right, 0.0, 0.0, 0.0)
    if q_old is None or q_new is None or not np.array_equal(q_old, q_new):
        print('FAIL straight invariance')
        return 1
    print('PASS straight invariance (trunk_yaw=0 identical to old call)')
    return 0


def check_yaw(ik, cmd_a, yaw_deg):
    """Check planted and mid-swing poses at one accumulated turn yaw."""
    failures = 0
    yaw = math.radians(yaw_deg)
    com = np.array([X_OFFSET, 0.0, COM_HEIGHT])

    cases = [
        ('planted', rotated_foot(yaw, 1), rotated_foot(yaw, -1), yaw, yaw, yaw),
        (
            'swing-left',
            rotated_foot(yaw, 1),
            rotated_foot(yaw - cmd_a, -1),
            yaw,
            yaw - cmd_a,
            yaw - cmd_a / 2.0,
        ),
        (
            'swing-right',
            rotated_foot(yaw - cmd_a, 1),
            rotated_foot(yaw, -1),
            yaw - cmd_a,
            yaw,
            yaw - cmd_a / 2.0,
        ),
    ]

    for name, left, right, left_yaw, right_yaw, trunk_yaw in cases:
        ok, max_hip = solve_case(ik, com, left, right, left_yaw, right_yaw, trunk_yaw)
        ref_ok, _ = solve_case(ik, com, left, right, left_yaw, right_yaw, 0.0)
        if ok and max_hip is not None and max_hip <= HIP_YAW_LIMIT_DEG:
            status = 'PASS'
        else:
            status = 'FAIL'
            failures += 1
        print(
            '%s cmd_a=%.2f yaw=%6.1f deg case=%-11s matched=%s hip=%s deg ref=%s'
            % (
                status,
                cmd_a,
                yaw_deg,
                name,
                ok,
                '%.1f' % max_hip if max_hip is not None else 'n/a',
                ref_ok,
            )
        )
    return failures


def main():
    """Run the sweep and report pass/fail."""
    ik = InverseKinematic()
    failures = check_invariance(ik)
    for cmd_a in CMD_A_VALUES:
        for yaw_deg in YAW_DEGREES:
            failures += check_yaw(ik, cmd_a, yaw_deg)
    print('')
    if failures:
        print('RESULT: %d FAILURES' % failures)
        return 1
    print('RESULT: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
