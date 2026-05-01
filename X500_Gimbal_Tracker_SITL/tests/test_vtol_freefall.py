#!/usr/bin/env python3
"""
Simple test for the SITL-only freefall simulator in x500_gimbal_proximity_guidance.
"""

import sys
import os
import math

# Ensure the code directory is on sys.path
ROOT = os.path.dirname(os.path.dirname(__file__))
CODE_DIR = os.path.join(ROOT, "code")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

from x500_gimbal_proximity_guidance import simulated_target_response_after_event, VehicleState


def main():
    g = 9.81
    cases = [
        (20.0, 0.0),    # 20 m, stationary vertical
        (10.0, 2.0),    # 10 m, initial downward 2 m/s
        (0.0, 0.0),     # already on ground
        (5.0, -1.0),    # 5 m, slight upward speed (vd_mps negative => up)
    ]

    for alt, vdown in cases:
        vs = VehicleState(0.0, 0.0, -alt, 0.0, 0.0, vdown)
        report = simulated_target_response_after_event(vs)

        if alt <= 0:
            assert report["time_to_impact_s"] == 0.0
            continue

        # analytic solution for 0.5*g*t^2 + vdown*t - alt = 0
        a = 0.5 * g
        b = vdown
        c = -alt
        disc = b * b - 4 * a * c
        assert disc >= 0.0
        t1 = (-b + math.sqrt(disc)) / (2 * a)
        t2 = (-b - math.sqrt(disc)) / (2 * a)
        t_expected = max(t1, t2, 0.0)
        impact_expected = vdown + g * t_expected

        # The simulator integrates in 0.05 s steps, so compare against the
        # continuous solution within one integration step.
        assert abs(report["time_to_impact_s"] - round(t_expected, 3)) <= 0.075
        assert abs(report["impact_velocity_mps"] - round(impact_expected, 3)) <= g * 0.075

    print("ALL TESTS PASSED")


if __name__ == '__main__':
    main()
