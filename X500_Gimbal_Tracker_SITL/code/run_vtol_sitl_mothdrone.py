#!/usr/bin/env python3
"""Runner for the X500 Gimbal Tracker X500 gimbal SITL exercise.

Two X500 gimbal vehicles:
- Hunter: pursues target using GPS + vision confirmation.
- Target: flies straight, then motor-stop → freefalls → hits ground → disarms.
"""

from __future__ import annotations

from x500_gimbal_proximity_guidance import (
    VehicleState,
    VisionConfirmation,
    VtolGuidanceConfig,
    VtolProximityGuidance,
    TargetDrone,
    simulate_target_freefall,
)


def main() -> int:
    config = VtolGuidanceConfig()
    guidance = VtolProximityGuidance(config)

    # Initial states at 20 m altitude, target 100 m east moving at 4 m/s
    hunter = VehicleState(0.0, 0.0, -20.0, 0.0, 0.0, 0.0)
    target_initial = VehicleState(100.0, 0.0, -20.0, 0.0, 4.0, 0.0)
    target = TargetDrone(target_initial)

    # --- Phase 1: GPS pursuit loop until trigger ---
    dt = 0.2
    t = 0.0
    print("=== PHASE 1: GPS PURSUIT ===")
    print(f"{'t(s)':>8} {'hunter_N':>10} {'hunter_E':>10} {'target_N':>10} {'target_E':>10} {'range_m':>10} {'mode':>30}")

    for step in range(3000):
        vision = VisionConfirmation(has_lock=True, confidence=0.8, bearing_error_rad=0.0)
        cmd = guidance.command(hunter, target.state, vision)

        # Simple hunter integration using commanded velocity
        hunter = VehicleState(
            north_m=hunter.north_m + cmd.north_velocity_mps * dt,
            east_m=hunter.east_m + cmd.east_velocity_mps * dt,
            down_m=-cmd.altitude_m,
            vn_mps=cmd.north_velocity_mps,
            ve_mps=cmd.east_velocity_mps,
            vd_mps=0.0,
            yaw_rad=cmd.yaw_rad,
        )

        # Target normal flight
        target.step(dt)
        t += dt

        if step % 10 == 0 or cmd.event:
            rel_n = target.state.north_m - hunter.north_m
            rel_e = target.state.east_m - hunter.east_m
            rng = (rel_n ** 2 + rel_e ** 2) ** 0.5
            print(f"{t:8.1f} {hunter.north_m:10.1f} {hunter.east_m:10.1f} {target.state.north_m:10.1f} {target.state.east_m:10.1f} {rng:10.1f} {cmd.mode:>30}")

        if cmd.event == "target_motor_stop_freefall":
            print("\n>>> PROXIMITY EVENT TRIGGERED — STOPPING TARGET MOTORS <<<")
            target.stop_motors_and_freefall()
            break

    # --- Phase 2: Target freefall ---
    print("\n=== PHASE 2: TARGET FREEFALL ===")
    freefall_log = simulate_target_freefall(target, dt=0.05)

    for entry in freefall_log[::20]:  # print every 20th step (about 1 s intervals)
        print(f"  t={entry['time_s']:6.2f}s  alt={entry['altitude_m']:6.2f}m  vd={entry['vd_mps']:6.2f}m/s  disarmed={entry['disarmed']}")

    print("\n=== FINAL TARGET STATUS ===")
    print(target.status())
    print("\n=== HUNTER COMMAND AFTER EVENT ===")
    # One more hunter command after event for breakaway
    vision = VisionConfirmation(has_lock=False, confidence=0.0, bearing_error_rad=0.0)
    breakaway_cmd = guidance.command(hunter, target.state, vision)
    print(breakaway_cmd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
