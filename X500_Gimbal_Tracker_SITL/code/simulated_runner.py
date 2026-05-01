#!/usr/bin/env python3
"""Pure-Python X500 Gimbal Tracker X500 gimbal guidance simulation.

This runner uses the same shared guidance module as the tests. It does not
need PX4, Gazebo, MAVSDK, or QGroundControl.
"""

from __future__ import annotations

import json
from pathlib import Path

from x500_gimbal_proximity_guidance import (
    TargetDrone,
    VehicleState,
    VisionConfirmation,
    VtolProximityGuidance,
    simulate_target_freefall,
)


def target_path_velocity(step: int) -> tuple[float, float]:
    """Same target path phases used by the MAVSDK SITL controller."""
    if step < 60:
        return 0.0, 4.0
    if step < 120:
        return 0.8, 4.0
    if step < 180:
        return -0.8, 4.0
    return 0.0, 4.0


def run_simulation() -> None:
    print("=" * 80)
    print("X500_GIMBAL X500 gimbal INTERCEPTION - PURE GUIDANCE SIMULATION")
    print("=" * 80)

    guidance = VtolProximityGuidance()
    hunter = VehicleState(0.0, 0.0, -20.0, 0.0, 0.0, 0.0)
    target = VehicleState(0.0, 100.0, -20.0, 0.0, 4.0, 0.0)
    telemetry_log: list[dict] = []
    dt = guidance.config.control_dt_s

    print("[INIT] Hunter at N=0, E=0, Alt=20m")
    print("[INIT] Target at N=0, E=100, Alt=20m")
    print("[INIT] Target speed: 4m/s | hunter cap: 15m/s")
    print("[INIT] Trigger radius: 25m with vision confirmation")

    for step in range(1200):
        range_m = ((target.north_m - hunter.north_m) ** 2 + (target.east_m - hunter.east_m) ** 2) ** 0.5
        vision = VisionConfirmation(
            has_lock=range_m <= guidance.config.vision_confirm_radius_m,
            confidence=0.88 if range_m <= guidance.config.vision_confirm_radius_m else 0.0,
            bearing_error_rad=0.0,
        )
        cmd = guidance.command(hunter, target, vision)

        log_entry = {
            "time": round(step * dt, 2),
            "loop": step + 1,
            "hunter_n": round(hunter.north_m, 2),
            "hunter_e": round(hunter.east_m, 2),
            "hunter_alt": round(hunter.altitude_m, 2),
            "target_n": round(target.north_m, 2),
            "target_e": round(target.east_m, 2),
            "target_alt": round(target.altitude_m, 2),
            "range_m": round(range_m, 2),
            "mode": cmd.mode,
            "mission_state": "pure_guidance",
            "vision_lock": vision.has_lock,
            "closing_velocity_mps": round(cmd.closing_velocity_mps, 2),
            "commanded_speed_mps": round(cmd.speed_mps, 2),
            "lead_time_s": round(cmd.lead_time_s, 2),
        }
        telemetry_log.append(log_entry)

        print(
            f"[{step + 1:03d}] range={range_m:6.1f}m "
            f"H=({hunter.north_m:5.1f},{hunter.east_m:6.1f}) "
            f"T=({target.north_m:5.1f},{target.east_m:6.1f}) "
            f"Vc={cmd.closing_velocity_mps:4.1f} speed={cmd.speed_mps:4.1f} "
            f"mode={cmd.mode}"
        )

        if cmd.event == "target_motor_stop_freefall":
            target_drone = TargetDrone(target)
            target_drone.stop_motors_and_freefall()
            freefall = simulate_target_freefall(target_drone, dt=0.05)
            impact = freefall[-1] if freefall else {"time_s": 0.0, "altitude_m": target.altitude_m}
            print()
            print("[EVENT] Trigger reached with vision lock")
            print(f"[EVENT] Range: {range_m:.1f}m")
            print(f"[EVENT] Freefall impact after {impact['time_s']:.2f}s")
            print("[RESULT] Target simulated motor-stop/freefall/disarm; hunter enters breakaway/RTL state")
            break

        hunter = VehicleState(
            hunter.north_m + cmd.north_velocity_mps * dt,
            hunter.east_m + cmd.east_velocity_mps * dt,
            -cmd.altitude_m,
            cmd.north_velocity_mps,
            cmd.east_velocity_mps,
            0.0,
            cmd.yaw_rad,
        )
        target_vn, target_ve = target_path_velocity(step)
        target = VehicleState(
            target.north_m + target_vn * dt,
            target.east_m + target_ve * dt,
            -20.0,
            target_vn,
            target_ve,
            0.0,
            target.yaw_rad,
        )
    else:
        raise RuntimeError("Simulation did not reach trigger radius")

    log_file = Path("simulated_telemetry.json")
    log_file.write_text(json.dumps(telemetry_log, indent=2))

    max_speed = max(row["commanded_speed_mps"] for row in telemetry_log)
    max_closing = max(row["closing_velocity_mps"] for row in telemetry_log)
    min_range = min(row["range_m"] for row in telemetry_log)
    print("=" * 80)
    print("SIMULATION COMPLETE")
    print(f"Rows: {len(telemetry_log)}")
    print(f"Min/trigger range: {min_range:.1f}m")
    print(f"Max commanded speed: {max_speed:.1f}m/s")
    print(f"Max closing velocity: {max_closing:.1f}m/s")
    print(f"Telemetry saved to: {log_file}")
    print("=" * 80)


if __name__ == "__main__":
    run_simulation()
