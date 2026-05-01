#!/usr/bin/env python3
"""Tests for PN lead-pursuit guidance math."""

from __future__ import annotations

import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))

from vtol_proximity_guidance import (  # noqa: E402
    VehicleState,
    VisionConfirmation,
    VtolGuidanceConfig,
    VtolProximityGuidance,
)


def test_closing_velocity_positive_when_range_closes() -> None:
    guidance = VtolProximityGuidance()
    hunter = VehicleState(0.0, 0.0, -20.0, 0.0, 10.0, 0.0)
    target = VehicleState(0.0, 100.0, -20.0, 0.0, 4.0, 0.0)
    cmd = guidance.command(hunter, target, VisionConfirmation(False, 0.0, 0.0))
    assert cmd.closing_velocity_mps > 5.5


def test_lead_point_ahead_of_moving_target() -> None:
    guidance = VtolProximityGuidance()
    hunter = VehicleState(0.0, 0.0, -20.0, 0.0, 0.0, 0.0)
    target = VehicleState(0.0, 100.0, -20.0, 0.0, 4.0, 0.0)
    cmd = guidance.command(hunter, target, VisionConfirmation(False, 0.0, 0.0))
    assert cmd.predicted_target_e_m > target.east_m
    assert 0.5 <= cmd.lead_time_s <= 8.0


def test_acceleration_limit_caps_speed_step() -> None:
    cfg = VtolGuidanceConfig(max_hunter_speed_mps=15.0, accel_limit_mps2=1.5, control_dt_s=0.2)
    guidance = VtolProximityGuidance(cfg)
    hunter = VehicleState(0.0, 0.0, -20.0, 0.0, 0.0, 0.0)
    target = VehicleState(0.0, 100.0, -20.0, 0.0, 4.0, 0.0)
    cmd1 = guidance.command(hunter, target, VisionConfirmation(False, 0.0, 0.0))
    cmd2 = guidance.command(hunter, target, VisionConfirmation(False, 0.0, 0.0))
    assert math.isclose(cmd1.speed_mps, 0.3, abs_tol=1e-6)
    assert math.isclose(cmd2.speed_mps, 0.6, abs_tol=1e-6)


def test_trigger_requires_vision_lock() -> None:
    guidance = VtolProximityGuidance()
    hunter = VehicleState(0.0, 0.0, -20.0, 0.0, 0.0, 0.0)
    target = VehicleState(0.0, 24.0, -20.0, 0.0, 4.0, 0.0)
    no_lock = guidance.command(hunter, target, VisionConfirmation(False, 0.0, 0.0))
    assert no_lock.event is None
    locked = guidance.command(hunter, target, VisionConfirmation(True, 0.85, 0.0))
    assert locked.event == "target_motor_stop_freefall"


if __name__ == "__main__":
    tests = [
        test_closing_velocity_positive_when_range_closes,
        test_lead_point_ahead_of_moving_target,
        test_acceleration_limit_caps_speed_step,
        test_trigger_requires_vision_lock,
    ]
    for test in tests:
        test()
    print("ALL GUIDANCE TESTS PASSED")
