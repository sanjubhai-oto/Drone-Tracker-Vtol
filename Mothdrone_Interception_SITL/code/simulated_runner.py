#!/usr/bin/env python3
"""
Simulated Mothdrone Runner - No PX4/Gazebo Required
Runs the full interception algorithm in simulation mode for testing/demo.
Shows telemetry, trigger events, and target freefall without needing PX4 SITL.

Usage:
    python3 simulated_runner.py
"""

import math
import time
from pathlib import Path
import json


# Standalone copies of classes (no external deps)
class VehicleState:
    def __init__(self, north_m, east_m, down_m, vn_mps, ve_mps, vd_mps, yaw_rad=0.0):
        self.north_m = north_m
        self.east_m = east_m
        self.down_m = down_m
        self.vn_mps = vn_mps
        self.ve_mps = ve_mps
        self.vd_mps = vd_mps
        self.yaw_rad = yaw_rad

    @property
    def altitude_m(self):
        return -self.down_m


class VisionConfirmation:
    def __init__(self, has_lock, confidence, bearing_error_rad):
        self.has_lock = has_lock
        self.confidence = confidence
        self.bearing_error_rad = bearing_error_rad


class TargetDroneSimulator:
    def __init__(self, initial):
        self.state = initial
        self.motors_armed = True
        self.freefall = False
        self.disarmed = False
        self.g = 9.81
        self.ground_altitude_m = 0.0

    def step(self, dt=0.05):
        if self.freefall:
            self.state = VehicleState(
                self.state.north_m + self.state.vn_mps * dt,
                self.state.east_m + self.state.ve_mps * dt,
                self.state.down_m + self.state.vd_mps * dt,
                self.state.vn_mps,
                self.state.ve_mps,
                self.state.vd_mps + self.g * dt,
                self.state.yaw_rad,
            )
        else:
            self.state = VehicleState(
                self.state.north_m + self.state.vn_mps * dt,
                self.state.east_m + self.state.ve_mps * dt,
                self.state.down_m,
                self.state.vn_mps,
                self.state.ve_mps,
                0.0,
                self.state.yaw_rad,
            )
        if self.state.altitude_m <= self.ground_altitude_m:
            self.state = VehicleState(
                self.state.north_m, self.state.east_m, -self.ground_altitude_m,
                0.0, 0.0, 0.0, self.state.yaw_rad
            )
            self.disarmed = True
            self.motors_armed = False
            self.freefall = False

    def stop_motors_and_freefall(self):
        if not self.disarmed:
            self.motors_armed = False
            self.freefall = True
            print(f"[TARGET] MOTORS STOPPED! Entering freefall from {self.state.altitude_m:.1f}m")

    def status(self):
        return {
            "altitude_m": round(self.state.altitude_m, 2),
            "motors_armed": self.motors_armed,
            "freefall": self.freefall,
            "disarmed": self.disarmed,
            "vd_mps": round(self.state.vd_mps, 2),
        }


class VtolGuidance:
    def __init__(self):
        self.kill_zone_radius_m = 25.0
        self.vision_confirm_radius_m = 35.0
        self.takeoff_altitude_m = 20.0
        self.breakaway_altitude_m = 40.0
        self.max_hunter_speed_mps = 18.0
        self.target_speed_mps = 4.0
        self.pn_navigation_constant = 3.0
        self.min_safe_event_altitude_m = 8.0
        self._event_triggered = False

    def command(self, hunter, target, vision):
        rel_n = target.north_m - hunter.north_m
        rel_e = target.east_m - hunter.east_m
        range_m = math.hypot(rel_n, rel_e)
        bearing = math.atan2(rel_e, rel_n)

        if self._event_triggered:
            return type('Cmd', (), {
                'mode': "hunter_breakaway_rtl",
                'north_velocity_mps': 0.0,
                'east_velocity_mps': 0.0,
                'altitude_m': self.breakaway_altitude_m,
                'yaw_rad': hunter.yaw_rad,
                'event': "breakaway_then_rtl",
            })()

        if self._should_trigger_event(range_m, hunter, target, vision):
            self._event_triggered = True
            return type('Cmd', (), {
                'mode': "proximity_event_confirmed",
                'north_velocity_mps': 0.0,
                'east_velocity_mps': 0.0,
                'altitude_m': self.breakaway_altitude_m,
                'yaw_rad': bearing,
                'event': "target_motor_stop_freefall",
            })()

        speed = self._lead_pursuit_speed(hunter, target, range_m)
        return type('Cmd', (), {
            'mode': "gps_lead_pursuit_visual_gate",
            'north_velocity_mps': speed * math.cos(bearing),
            'east_velocity_mps': speed * math.sin(bearing),
            'altitude_m': self.takeoff_altitude_m,
            'yaw_rad': bearing,
            'event': None,
        })()

    def _should_trigger_event(self, range_m, hunter, target, vision):
        if range_m > self.kill_zone_radius_m:
            return False
        if hunter.altitude_m < self.min_safe_event_altitude_m or target.altitude_m < self.min_safe_event_altitude_m:
            return False
        if range_m <= self.vision_confirm_radius_m:
            return vision.has_lock and vision.confidence >= 0.55
        return False

    def _lead_pursuit_speed(self, hunter, target, range_m):
        rel_vn = target.vn_mps - hunter.vn_mps
        rel_ve = target.ve_mps - hunter.ve_mps
        closing = max(0.0, -(rel_vn * (target.north_m - hunter.north_m) + rel_ve * (target.east_m - hunter.east_m)) / max(range_m, 1e-6))
        commanded = self.target_speed_mps + self.pn_navigation_constant * closing
        return max(self.target_speed_mps, min(self.max_hunter_speed_mps, commanded))


def run_simulation():
    print("="*80)
    print("MOTHDRONE VTOL INTERCEPTION - SIMULATED RUNNER")
    print("No PX4/Gazebo required - pure algorithm demonstration")
    print("="*80)
    print()

    hunter = VehicleState(0.0, 0.0, -20.0, 0.0, 0.0, 0.0)
    target = VehicleState(100.0, 0.0, -20.0, 0.0, 4.0, 0.0)

    guidance = VtolGuidance()
    target_sim = TargetDroneSimulator(target)
    telemetry_log = []

    dt = 0.5
    t = 0.0
    event_triggered = False

    print("[INIT] Hunter at N=0, E=0, Alt=20m")
    print("[INIT] Target at N=100, E=0, Alt=20m, moving East at 4 m/s")
    print("[INIT] Kill zone radius: 25m")
    print("[INIT] Vision confirmation radius: 35m")
    print()
    time.sleep(1)

    while True:
        t += dt

        rel_n = target.north_m - hunter.north_m
        rel_e = target.east_m - hunter.east_m
        range_m = math.hypot(rel_n, rel_e)

        vision = VisionConfirmation(
            has_lock=(range_m < guidance.vision_confirm_radius_m),
            confidence=0.9 if range_m < guidance.vision_confirm_radius_m else 0.0,
            bearing_error_rad=0.0,
        )

        cmd = guidance.command(hunter, target, vision)

        log_entry = {
            "time": round(t, 1),
            "hunter_n": round(hunter.north_m, 1),
            "hunter_e": round(hunter.east_m, 1),
            "hunter_alt": round(hunter.altitude_m, 1),
            "target_n": round(target.north_m, 1),
            "target_e": round(target.east_m, 1),
            "target_alt": round(target.altitude_m, 1),
            "range_m": round(range_m, 1),
            "mode": cmd.mode,
        }
        telemetry_log.append(log_entry)

        print(f"[t={t:5.1f}s] Range: {range_m:6.1f}m | "
              f"Hunter: N={hunter.north_m:7.1f} E={hunter.east_m:7.1f} Alt={hunter.altitude_m:5.1f} | "
              f"Target: N={target.north_m:7.1f} E={target.east_m:7.1f} Alt={target.altitude_m:5.1f} | "
              f"Mode: {cmd.mode}")

        if cmd.event == "target_motor_stop_freefall" and not event_triggered:
            event_triggered = True
            print()
            print("!"*80)
            print("[EVENT] KILL ZONE REACHED - TARGET ACQUIRED")
            print("[EVENT] Vision confirmation: LOCKED (confidence: 90%)")
            print("[EVENT] TRIGGERING MOTOR STOP -> FREEFALL -> DISARM")
            print("!"*80)
            print()
            time.sleep(1)

            target_sim.state = target
            target_sim.stop_motors_and_freefall()

            print("[FREEFALL] Simulating target descent...")
            step = 0
            while not target_sim.disarmed:
                target_sim.step(0.1)
                step += 1
                if step % 5 == 0:
                    status = target_sim.status()
                    print(f"  [FREEFALL] t={step*0.1:4.1f}s | "
                          f"Alt: {status['altitude_m']:5.1f}m | "
                          f"Vd: {status['vd_mps']:5.1f}m/s | "
                          f"Motors: {status['motors_armed']} | "
                          f"Disarmed: {status['disarmed']}")

            print()
            print("[IMPACT] Target hit ground!")
            print(f"[IMPACT] Final altitude: {target_sim.state.altitude_m:.1f}m")
            print("[IMPACT] Target DISARMED")
            print()
            time.sleep(1)

            print("[BREAKAWAY] Hunter climbing to 40m...")
            print("[RTL] Hunter returning to launch position...")
            print("[RTL] Landing sequence initiated")
            print()
            break

        if not event_triggered:
            hunter = VehicleState(
                hunter.north_m + cmd.north_velocity_mps * dt,
                hunter.east_m + cmd.east_velocity_mps * dt,
                -cmd.altitude_m,
                cmd.north_velocity_mps,
                cmd.east_velocity_mps,
                0.0,
                cmd.yaw_rad,
            )
            target = VehicleState(
                target.north_m + target.vn_mps * dt,
                target.east_m + target.ve_mps * dt,
                target.down_m,
                target.vn_mps,
                target.ve_mps,
                0.0,
                target.yaw_rad,
            )
        else:
            target = target_sim.state

        time.sleep(0.05)

    log_file = Path("simulated_telemetry.json")
    with open(log_file, "w") as f:
        json.dump(telemetry_log, f, indent=2)

    print("="*80)
    print("MISSION COMPLETE")
    print("="*80)
    print(f"Simulation time: {t:.1f}s")
    print(f"Final hunter position: N={hunter.north_m:.1f} E={hunter.east_m:.1f}")
    print(f"Final target position: N={target.north_m:.1f} E={target.east_m:.1f}")
    print("Target response: MOTOR STOP -> FREEFALL -> GROUND IMPACT -> DISARM")
    print(f"Telemetry saved to: {log_file}")
    print("="*80)


if __name__ == "__main__":
    run_simulation()
