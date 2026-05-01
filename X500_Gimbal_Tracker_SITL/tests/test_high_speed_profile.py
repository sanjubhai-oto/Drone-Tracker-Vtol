#!/usr/bin/env python3
"""Fast simulation test for 100m, 4m/s target, 15m/s capped hunter."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))

from x500_gimbal_proximity_guidance import VehicleState, VisionConfirmation, VtolProximityGuidance  # noqa: E402


def main() -> None:
    guidance = VtolProximityGuidance()
    hunter = VehicleState(0.0, 0.0, -20.0, 0.0, 0.0, 0.0)
    target = VehicleState(0.0, 100.0, -20.0, 0.0, 4.0, 0.0)
    dt = 0.2
    trigger_step = None
    max_speed = 0.0
    max_closing = 0.0

    for step in range(900):
        range_m = ((target.north_m - hunter.north_m) ** 2 + (target.east_m - hunter.east_m) ** 2) ** 0.5
        vision = VisionConfirmation(range_m < guidance.config.vision_confirm_radius_m, 0.85 if range_m < 35.0 else 0.0, 0.0)
        cmd = guidance.command(hunter, target, vision)
        max_speed = max(max_speed, cmd.speed_mps)
        max_closing = max(max_closing, cmd.closing_velocity_mps)
        if cmd.event == "target_motor_stop_freefall":
            trigger_step = step
            break
        hunter = VehicleState(
            hunter.north_m + cmd.north_velocity_mps * dt,
            hunter.east_m + cmd.east_velocity_mps * dt,
            -20.0,
            cmd.north_velocity_mps,
            cmd.east_velocity_mps,
            0.0,
            cmd.yaw_rad,
        )
        # Same target offboard path as the live controller: east then diagonal.
        if step < 60:
            target_vn, target_ve = 0.0, 4.0
        elif step < 120:
            target_vn, target_ve = 0.8, 4.0
        elif step < 180:
            target_vn, target_ve = -0.8, 4.0
        else:
            target_vn, target_ve = 0.0, 4.0
        target = VehicleState(
            target.north_m + target_vn * dt,
            target.east_m + target_ve * dt,
            -20.0,
            target_vn,
            target_ve,
            0.0,
            target.yaw_rad,
        )

    assert trigger_step is not None, "high-speed guidance did not reach 25m trigger"
    assert max_speed >= 12.0, f"hunter did not ramp toward high speed, max={max_speed:.2f}"
    assert max_speed <= 15.01, f"hunter exceeded cap, max={max_speed:.2f}"
    assert max_closing > 6.0, f"closing velocity too low, max={max_closing:.2f}"
    print(f"HIGH SPEED PROFILE PASS step={trigger_step} max_speed={max_speed:.2f} max_closing={max_closing:.2f}")


if __name__ == "__main__":
    main()
