#!/usr/bin/env python3
"""
Mothdrone VTOL Interception Controller
MAVSDK-based controller for hunter-target VTOL interception with:
- GPS-based proportional navigation
- Vision-based final confirmation
- Target motor-stop / freefall / disarm simulation
- Hunter breakaway and RTL
- Live telemetry streaming to QGroundControl
"""

from __future__ import annotations

import asyncio
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

TARGET_START_EAST_M = 100.0
TARGET_CRUISE_EAST_MPS = 2.0
HUNTER_MAX_SPEED_MPS = 5.0
TAKEOFF_GATE_ALTITUDE_M = 20.0
HUNTER_BREAKAWAY_ALTITUDE_M = 40.0

try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, VelocityNedYaw
    MAVSDK_AVAILABLE = True
except ImportError:
    MAVSDK_AVAILABLE = False
    System = None
    OffboardError = None
    VelocityNedYaw = None


@dataclass
class VehicleState:
    """NED state representation."""
    north_m: float
    east_m: float
    down_m: float
    vn_mps: float
    ve_mps: float
    vd_mps: float
    yaw_rad: float = 0.0

    @property
    def altitude_m(self) -> float:
        return -self.down_m


@dataclass
class VisionConfirmation:
    """Simulated vision system output."""
    has_lock: bool
    confidence: float
    bearing_error_rad: float


@dataclass
class GuidanceCommand:
    """Control command output."""
    mode: str
    north_velocity_mps: float
    east_velocity_mps: float
    altitude_m: float
    yaw_rad: float
    event: Optional[str] = None


class TargetDroneSimulator:
    """
    Simulated target VTOL with motor-stop → freefall → ground impact → disarm.
    Used for SITL demonstration when target motor kill is triggered.
    """

    def __init__(self, initial: VehicleState) -> None:
        self.state = initial
        self.motors_armed = True
        self.freefall = False
        self.disarmed = False
        self.g = 9.81
        self.ground_altitude_m = 0.0

    def step(self, dt: float = 0.05) -> None:
        """Integrate physics forward by dt seconds."""
        if self.freefall:
            self.state = VehicleState(
                north_m=self.state.north_m + self.state.vn_mps * dt,
                east_m=self.state.east_m + self.state.ve_mps * dt,
                down_m=self.state.down_m + self.state.vd_mps * dt,
                vn_mps=self.state.vn_mps,
                ve_mps=self.state.ve_mps,
                vd_mps=self.state.vd_mps + self.g * dt,
                yaw_rad=self.state.yaw_rad,
            )
        else:
            # Normal cruise
            self.state = VehicleState(
                north_m=self.state.north_m + self.state.vn_mps * dt,
                east_m=self.state.east_m + self.state.ve_mps * dt,
                down_m=self.state.down_m,
                vn_mps=self.state.vn_mps,
                ve_mps=self.state.ve_mps,
                vd_mps=0.0,
                yaw_rad=self.state.yaw_rad,
            )

        # Ground contact
        if self.state.altitude_m <= self.ground_altitude_m:
            self.state = VehicleState(
                north_m=self.state.north_m,
                east_m=self.state.east_m,
                down_m=-self.ground_altitude_m,
                vn_mps=0.0,
                ve_mps=0.0,
                vd_mps=0.0,
                yaw_rad=self.state.yaw_rad,
            )
            self.disarmed = True
            self.motors_armed = False
            self.freefall = False

    def stop_motors_and_freefall(self) -> None:
        """Trigger motor stop and freefall."""
        if not self.disarmed:
            self.motors_armed = False
            self.freefall = True
            print(f"[TARGET] MOTORS STOPPED! Entering freefall from {self.state.altitude_m:.1f}m")

    def status(self) -> dict:
        return {
            "altitude_m": round(self.state.altitude_m, 2),
            "motors_armed": self.motors_armed,
            "freefall": self.freefall,
            "disarmed": self.disarmed,
            "vd_mps": round(self.state.vd_mps, 2),
        }


class VtolGuidance:
    """Proportional Navigation guidance for hunter pursuing target."""

    def __init__(self) -> None:
        self.kill_zone_radius_m = 25.0
        self.vision_confirm_radius_m = 35.0
        self.takeoff_altitude_m = 20.0
        self.breakaway_altitude_m = 40.0
        self.max_hunter_speed_mps = 5.0
        self.target_speed_mps = 2.0
        self.pn_navigation_constant = 3.0
        self.min_safe_event_altitude_m = 5.0
        self._event_triggered = False

    def command(self, hunter: VehicleState, target: VehicleState, vision: VisionConfirmation) -> GuidanceCommand:
        rel_n = target.north_m - hunter.north_m
        rel_e = target.east_m - hunter.east_m
        range_m = math.hypot(rel_n, rel_e)
        bearing = math.atan2(rel_e, rel_n)

        if self._event_triggered:
            return GuidanceCommand(
                mode="hunter_breakaway_rtl",
                north_velocity_mps=0.0,
                east_velocity_mps=0.0,
                altitude_m=self.breakaway_altitude_m,
                yaw_rad=hunter.yaw_rad,
                event="breakaway_then_rtl",
            )

        if self._should_trigger_event(range_m, hunter, target, vision):
            self._event_triggered = True
            return GuidanceCommand(
                mode="proximity_event_confirmed",
                north_velocity_mps=0.0,
                east_velocity_mps=0.0,
                altitude_m=self.breakaway_altitude_m,
                yaw_rad=bearing,
                event="target_motor_stop_freefall",
            )

        speed = self._lead_pursuit_speed(hunter, target, range_m)
        return GuidanceCommand(
            mode="gps_lead_pursuit_visual_gate",
            north_velocity_mps=speed * math.cos(bearing),
            east_velocity_mps=speed * math.sin(bearing),
            altitude_m=self.takeoff_altitude_m,
            yaw_rad=bearing,
            event=None,
        )

    def _should_trigger_event(self, range_m: float, hunter: VehicleState, target: VehicleState, vision: VisionConfirmation) -> bool:
        if range_m > self.kill_zone_radius_m:
            return False
        if hunter.altitude_m < self.min_safe_event_altitude_m or target.altitude_m < self.min_safe_event_altitude_m:
            return False
        if range_m <= self.vision_confirm_radius_m:
            return vision.has_lock and vision.confidence >= 0.55
        return False

    def _lead_pursuit_speed(self, hunter: VehicleState, target: VehicleState, range_m: float) -> float:
        if range_m > self.vision_confirm_radius_m:
            return self.max_hunter_speed_mps

        desired_event_range_m = self.kill_zone_radius_m
        range_error = max(0.0, range_m - desired_event_range_m)
        commanded = self.target_speed_mps + 0.8 * range_error
        return max(self.target_speed_mps + 1.0, min(self.max_hunter_speed_mps, commanded))


class MothdroneController:
    """Main controller managing hunter and target VTOLs."""

    def __init__(self, hunter_addr: str = "udpin://127.0.0.1:14540", target_addr: str = "udpin://127.0.0.1:14541"):
        if not MAVSDK_AVAILABLE:
            raise RuntimeError(
                "MAVSDK Python is not installed in this interpreter. "
                "Run with /Users/sanju/.venvs/mothdrone/bin/python or install mavsdk."
            )
        # Use separate MAVSDK gRPC ports. If both auto-start on 50051,
        # target commands can silently route through the hunter server.
        self.hunter = System(port=50051)
        self.target = System(port=50052)
        self.hunter_addr = hunter_addr
        self.target_addr = target_addr
        self.guidance = VtolGuidance()
        self.target_simulator: Optional[TargetDroneSimulator] = None
        self.event_triggered = False
        self.telemetry_log: list[dict] = []
        # PX4 local NED is relative to each vehicle's own home/spawn point.
        # Gazebo spawn pose puts target 100m east of hunter, so convert both
        # telemetry streams into one shared hunter-world NED frame before guidance.
        self.hunter_origin_ned_m = (0.0, 0.0, 0.0)
        self.target_origin_ned_m = (0.0, TARGET_START_EAST_M, 0.0)

    async def connect(self):
        """Connect to both vehicles via MAVSDK."""
        print(f"[MAVSDK] Connecting hunter to {self.hunter_addr}...")
        await self.hunter.connect(system_address=self.hunter_addr)
        print(f"[MAVSDK] Connecting target to {self.target_addr}...")
        await self.target.connect(system_address=self.target_addr)

        # Wait for connection
        async for state in self.hunter.core.connection_state():
            if state.is_connected:
                print("[MAVSDK] Hunter connected")
                break

        async for state in self.target.core.connection_state():
            if state.is_connected:
                print("[MAVSDK] Target connected")
                break

    async def disconnect(self):
        """Disconnect from vehicles."""
        print("[MAVSDK] Disconnecting...")
        # MAVSDK doesn't have explicit disconnect, just stop using

    async def arm_and_takeoff_both(self, altitude_m: float = 20.0):
        """Arm both VTOLs and climb in offboard mode from the start."""
        print("[MISSION] Configuring multicopter velocity limits...")
        await asyncio.gather(
            self._set_speed_profile(self.hunter, role="hunter", xy_max_mps=HUNTER_MAX_SPEED_MPS, xy_cruise_mps=HUNTER_MAX_SPEED_MPS, fw_trim_mps=5.0, fw_max_mps=7.0, fw_thr_max=0.6),
            self._set_speed_profile(self.target, role="target", xy_max_mps=TARGET_CRUISE_EAST_MPS, xy_cruise_mps=TARGET_CRUISE_EAST_MPS, fw_trim_mps=2.0, fw_max_mps=3.0, fw_thr_max=0.6),
        )
        await asyncio.gather(
            self._set_sitl_failsafe_profile(self.hunter, "hunter"),
            self._set_sitl_failsafe_profile(self.target, "target"),
        )

        print("[MISSION] Setting takeoff altitude on both VTOLs...")
        await asyncio.gather(
            self.hunter.action.set_takeoff_altitude(altitude_m),
            self.target.action.set_takeoff_altitude(altitude_m),
        )

        print("[MISSION] Priming offboard setpoints before arming...")
        await asyncio.gather(
            self.hunter.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0)),
            self.target.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0)),
        )

        print("[MISSION] Arming target first; hunter arms after target armed state is detected...")
        await self.target.action.arm()
        await self._wait_until_armed(self.target, "target", timeout_s=10.0)
        print("[MISSION] Target armed detected -> arming hunter now")
        await self.hunter.action.arm()
        await self._wait_until_armed(self.hunter, "hunter", timeout_s=10.0)

        await asyncio.sleep(0.5)

        print("[MISSION] Starting offboard mode on both VTOLs...")
        await asyncio.gather(
            self._ensure_offboard_started(self.hunter, "hunter"),
            self._ensure_offboard_started(self.target, "target"),
        )

        print(f"[MISSION] Offboard climb to {altitude_m}m; no PX4 action takeoff/manual phase...")
        print(f"[MISSION] Waiting until both VTOLs are above {TAKEOFF_GATE_ALTITUDE_M:.0f}m before movement...")
        await self._wait_until_both_above_altitude(min_altitude_m=TAKEOFF_GATE_ALTITUDE_M, timeout_s=60.0)

        print("[MISSION] Forcing both VTOLs to multicopter mode for velocity control...")
        await asyncio.gather(
            self._transition_to_multicopter(self.hunter, "hunter"),
            self._transition_to_multicopter(self.target, "target"),
        )

    async def _set_sitl_failsafe_profile(self, system: System, role: str) -> None:
        """Disable SITL-only link-loss actions that make the standalone target RTL before the demo event."""
        int_params = {
            "NAV_DLL_ACT": 0,
            "COM_OBL_RC_ACT": 0,
        }
        float_params = {
            "COM_DL_LOSS_T": 120.0,
            "COM_OF_LOSS_T": 10.0,
        }
        for name, value in int_params.items():
            try:
                await system.param.set_param_int(name, int(value))
            except Exception as exc:
                print(f"[MISSION] {role} SITL failsafe int param warning {name}={value}: {exc}")
        for name, value in float_params.items():
            try:
                await system.param.set_param_float(name, float(value))
            except Exception as exc:
                print(f"[MISSION] {role} SITL failsafe float param warning {name}={value}: {exc}")

    async def _set_speed_profile(
        self,
        system: System,
        role: str,
        xy_max_mps: float,
        xy_cruise_mps: float,
        fw_trim_mps: float,
        fw_max_mps: float,
        fw_thr_max: float,
    ) -> None:
        """Best-effort PX4 speed tuning for SITL standard VTOL."""
        for name, value in (
            ("MPC_XY_VEL_MAX", xy_max_mps),
            ("MPC_XY_CRUISE", xy_cruise_mps),
            ("FW_AIRSPD_TRIM", fw_trim_mps),
            ("FW_AIRSPD_MAX", fw_max_mps),
            ("FW_THR_MAX", fw_thr_max),
        ):
            try:
                await system.param.set_param_float(name, float(value))
            except Exception as exc:
                print(f"[MISSION] {role} param warning {name}={value}: {exc}")

    async def _transition_to_multicopter(self, system: System, role: str) -> None:
        try:
            await system.action.transition_to_multicopter()
            print(f"[MISSION] {role} in multicopter mode")
        except Exception as exc:
            print(f"[MISSION] {role} multicopter transition warning: {exc}")

    async def _ensure_offboard_started(self, system: System, role: str) -> None:
        await system.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await system.offboard.start()
            print(f"[MISSION] {role} offboard active")
        except Exception as exc:
            print(f"[MISSION] {role} offboard start warning: {exc}")

    async def _wait_until_armed(self, system: System, role: str, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        async for armed in system.telemetry.armed():
            if armed:
                print(f"[MISSION] {role} armed=true")
                return
            if time.monotonic() >= deadline:
                break
        raise RuntimeError(f"{role} did not report armed state within {timeout_s:.1f}s")

    async def _wait_until_both_above_altitude(self, min_altitude_m: float, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            hunter_state = await self.get_telemetry(self.hunter, self.hunter_origin_ned_m)
            target_state = await self.get_telemetry(self.target, self.target_origin_ned_m)
            hunter_vd = -1.5 if hunter_state.altitude_m < min_altitude_m + 1.0 else 0.0
            target_vd = -1.5 if target_state.altitude_m < min_altitude_m + 1.0 else 0.0
            await asyncio.gather(
                self.send_velocity_setpoint(self.hunter, 0.0, 0.0, hunter_vd, hunter_state.yaw_rad),
                self.send_velocity_setpoint(self.target, 0.0, 0.0, target_vd, target_state.yaw_rad),
            )
            print(
                f"[MISSION] Alt check | hunter={hunter_state.altitude_m:.1f}m "
                f"target={target_state.altitude_m:.1f}m"
            )
            if hunter_state.altitude_m >= min_altitude_m and target_state.altitude_m >= min_altitude_m:
                return
            await asyncio.sleep(1.0)
        raise RuntimeError(
            f"Altitude gate failed: hunter and target must both be above {min_altitude_m:.1f}m before guidance starts"
        )

    @staticmethod
    def _target_offboard_path_velocity(loop_count: int) -> tuple[float, float, float]:
        """Simple target offboard path: east leg, diagonal leg, then mild zigzag."""
        if loop_count < 60:
            return 0.0, TARGET_CRUISE_EAST_MPS, 90.0
        if loop_count < 120:
            return 0.8, TARGET_CRUISE_EAST_MPS, 70.0
        if loop_count < 180:
            return -0.8, TARGET_CRUISE_EAST_MPS, 110.0
        return 0.0, TARGET_CRUISE_EAST_MPS, 90.0

    async def get_telemetry(self, system: System, origin_ned_m: tuple[float, float, float]) -> VehicleState:
        """Get current vehicle state from telemetry."""
        async for pv in system.telemetry.position_velocity_ned():
            att_yaw = 0.0
            try:
                async for att in system.telemetry.attitude_euler():
                    att_yaw = math.radians(att.yaw_deg)
                    break
            except Exception:
                pass

            origin_n, origin_e, origin_d = origin_ned_m
            return VehicleState(
                north_m=pv.position.north_m + origin_n,
                east_m=pv.position.east_m + origin_e,
                down_m=pv.position.down_m + origin_d,
                vn_mps=pv.velocity.north_m_s,
                ve_mps=pv.velocity.east_m_s,
                vd_mps=pv.velocity.down_m_s,
                yaw_rad=att_yaw,
            )

        raise RuntimeError("No local NED telemetry received")

    async def send_velocity_setpoint(self, system: System, vn: float, ve: float, vd: float, yaw: float):
        """Send velocity setpoint to vehicle."""
        await system.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, math.degrees(yaw)))

    def _write_telemetry_snapshot(self) -> None:
        """Write telemetry during the mission so the live trajectory page can update."""
        import json

        log_file = Path("mothdrone_telemetry.json")
        tmp_file = log_file.with_suffix(".json.tmp")
        tmp_file.write_text(json.dumps(self.telemetry_log, indent=2))
        tmp_file.replace(log_file)

    async def _hold_hunter_safe_altitude(self, yaw_rad: float, seconds: float, climb: bool = False) -> None:
        """Keep hunter stable after trigger; never command descent while target falls."""
        vd_mps = -1.0 if climb else 0.0
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            await self.send_velocity_setpoint(self.hunter, 0.0, 0.0, vd_mps, yaw_rad)
            await asyncio.sleep(0.2)

    async def _hunter_breakaway_and_rtl(self, yaw_rad: float) -> None:
        """Climb/hold hunter, then hand back to PX4 RTL for home return and landing."""
        print("[BREAKAWAY] Hunter holding altitude while target falls...")
        await self._hold_hunter_safe_altitude(yaw_rad=yaw_rad, seconds=2.0, climb=False)
        print(f"[BREAKAWAY] Hunter climbing toward {HUNTER_BREAKAWAY_ALTITUDE_M:.0f}m before RTL...")
        await self._hold_hunter_safe_altitude(yaw_rad=yaw_rad, seconds=8.0, climb=True)
        try:
            await self.hunter.offboard.stop()
            print("[BREAKAWAY] Hunter offboard stopped; PX4 RTL takes over")
        except Exception as exc:
            print(f"[BREAKAWAY] Hunter offboard stop warning: {exc}")
        print("[RTL] Hunter returning to launch...")
        await self.hunter.action.return_to_launch()
        print("[RTL] RTL command sent. Hunter will return home and land.")

    async def _trigger_target_sitl_motor_stop(self) -> str:
        """Stop the target vehicle in SITL so it falls instead of entering RTL.

        Guarded by MOTHDRONE_SITL_TARGET_KILL so this is never a default real-aircraft path.
        """
        if os.environ.get("MOTHDRONE_SITL_TARGET_KILL") != "1":
            print("[TARGET] SITL target kill disabled; only local freefall model will run")
            return "disabled"

        try:
            await self.target.action.kill()
            print("[TARGET] SITL kill command accepted; target should stop motors and fall")
            return "kill"
        except Exception as kill_exc:
            print(f"[TARGET] SITL kill warning: {kill_exc}")

        try:
            await self.target.action.terminate()
            print("[TARGET] SITL terminate command accepted; target should stop motors and fall")
            return "terminate"
        except Exception as term_exc:
            print(f"[TARGET] SITL terminate warning: {term_exc}")

        try:
            await self.target.offboard.stop()
        except Exception as offboard_exc:
            print(f"[TARGET] target offboard stop warning after failed kill: {offboard_exc}")
        return "failed"

    async def _monitor_post_event_recovery(self, seconds: float = 15.0) -> None:
        """Record post-trigger hunter recovery so the test proves it does not descend."""
        print(f"[VERIFY] Monitoring hunter recovery for {seconds:.0f}s...")
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            hunter_state = await self.get_telemetry(self.hunter, self.hunter_origin_ned_m)
            target_state = await self.get_telemetry(self.target, self.target_origin_ned_m)
            rel_n = target_state.north_m - hunter_state.north_m
            rel_e = target_state.east_m - hunter_state.east_m
            range_m = math.hypot(rel_n, rel_e)
            log_entry = {
                "time": time.time(),
                "loop": len(self.telemetry_log) + 1,
                "hunter_n": round(hunter_state.north_m, 1),
                "hunter_e": round(hunter_state.east_m, 1),
                "hunter_alt": round(hunter_state.altitude_m, 1),
                "target_n": round(target_state.north_m, 1),
                "target_e": round(target_state.east_m, 1),
                "target_alt": round(target_state.altitude_m, 1),
                "range_m": round(range_m, 1),
                "mode": "post_event_rtl_monitor",
                "mission_state": "hunter_recovery",
                "vision_lock": False,
            }
            self.telemetry_log.append(log_entry)
            self._write_telemetry_snapshot()
            print(
                f"[VERIFY] Hunter recovery | alt={hunter_state.altitude_m:.1f}m "
                f"N={hunter_state.north_m:.1f} E={hunter_state.east_m:.1f}"
            )
            await asyncio.sleep(1.0)

    async def run_interception(self):
        """Main interception loop."""
        print("[MISSION] Starting interception loop...")
        print(
            f"[MISSION] Target starts {TARGET_START_EAST_M:.0f}m East, runs offboard target path at ~{TARGET_CRUISE_EAST_MPS:.0f} m/s; "
            f"hunter pursuit capped at ~{HUNTER_MAX_SPEED_MPS:.0f} m/s"
        )
        print("[MISSION] Hunter pursuing with Proportional Navigation...")

        # Initialize target simulator with starting state
        self.target_simulator = TargetDroneSimulator(
            VehicleState(
                north_m=0.0,
                east_m=TARGET_START_EAST_M,
                down_m=-TAKEOFF_GATE_ALTITUDE_M,
                vn_mps=0.0,
                ve_mps=TARGET_CRUISE_EAST_MPS,
                vd_mps=0.0,
            )
        )

        print("[MISSION] Both vehicles remain in offboard after takeoff gate...")
        print("[TARGET] Target offboard path active; hunter listens and follows.")

        loop_count = 0
        while True:
            loop_count += 1

            # Get telemetry
            hunter_state = await self.get_telemetry(self.hunter, self.hunter_origin_ned_m)
            target_state = await self.get_telemetry(self.target, self.target_origin_ned_m)

            # Calculate range for vision simulation
            rel_n = target_state.north_m - hunter_state.north_m
            rel_e = target_state.east_m - hunter_state.east_m
            range_m = math.hypot(rel_n, rel_e)

            # Simulate vision confirmation when in range
            vision = VisionConfirmation(
                has_lock=(range_m < self.guidance.vision_confirm_radius_m),
                confidence=0.85 if range_m < self.guidance.vision_confirm_radius_m else 0.0,
                bearing_error_rad=0.0,
            )

            # Get guidance command
            cmd = self.guidance.command(hunter_state, target_state, vision)
            target_vn, target_ve, target_yaw_deg = self._target_offboard_path_velocity(loop_count)

            # Log telemetry
            log_entry = {
                "time": time.time(),
                "loop": loop_count,
                "hunter_n": round(hunter_state.north_m, 1),
                "hunter_e": round(hunter_state.east_m, 1),
                "hunter_alt": round(hunter_state.altitude_m, 1),
                "target_n": round(target_state.north_m, 1),
                "target_e": round(target_state.east_m, 1),
                "target_alt": round(target_state.altitude_m, 1),
                "range_m": round(range_m, 1),
                "mode": cmd.mode,
                "mission_state": "proximity_trigger" if cmd.event else "hunter_guidance_target_offboard",
                "vision_lock": vision.has_lock,
            }
            self.telemetry_log.append(log_entry)
            self._write_telemetry_snapshot()

            # Print every 2 seconds
            if loop_count % 2 == 0:
                print(f"[{loop_count:3d}] Range: {range_m:6.1f}m | "
                      f"Hunter: N={hunter_state.north_m:7.1f} E={hunter_state.east_m:7.1f} "
                      f"Target: N={target_state.north_m:7.1f} E={target_state.east_m:7.1f} "
                      f"Mode: {cmd.mode}")

            # Handle events
            if range_m <= self.guidance.kill_zone_radius_m and vision.has_lock and not self.event_triggered:
                cmd.event = "target_motor_stop_freefall"
                cmd.mode = "proximity_event_confirmed"

            if cmd.event == "target_motor_stop_freefall" and not self.event_triggered:
                self.event_triggered = True
                print("\n" + "="*70)
                print("[EVENT] KILL ZONE REACHED!")
                print("[EVENT] Vision confirmation: LOCKED")
                print("[EVENT] Triggering target MOTOR STOP → FREEFALL → DISARM")
                print("="*70 + "\n")

                await self._hold_hunter_safe_altitude(cmd.yaw_rad, seconds=1.0, climb=False)

                # Trigger target motor stop and freefall in Gazebo SITL, then mirror it in
                # the local physics log so the output has deterministic event data.
                target_stop_result = await self._trigger_target_sitl_motor_stop()
                self.target_simulator.stop_motors_and_freefall()

                # Simulate freefall until ground impact
                print("[FREEFALL] Simulating target descent...")
                freefall_log = []
                while not self.target_simulator.disarmed:
                    self.target_simulator.step(0.1)
                    status = self.target_simulator.status()
                    freefall_log.append(status)
                    if len(freefall_log) % 10 == 0:
                        print(f"[FREEFALL] Alt: {status['altitude_m']:5.1f}m | "
                              f"Vd: {status['vd_mps']:5.1f}m/s | "
                              f"Motors: {status['motors_armed']} | "
                              f"Disarmed: {status['disarmed']}")

                print(f"\n[IMPACT] Target hit ground at {self.target_simulator.state.altitude_m:.1f}m")
                print(f"[IMPACT] Target DISARMED. Mission complete.\n")

                if target_stop_result not in ("kill", "terminate"):
                    print("[TARGET] Warning: PX4 target motor-stop was not confirmed; check Gazebo target behavior")

                await self._hunter_breakaway_and_rtl(cmd.yaw_rad)
                await self._monitor_post_event_recovery(seconds=15.0)

                break

            # Send velocity setpoint to hunter
            await self.send_velocity_setpoint(
                self.hunter,
                cmd.north_velocity_mps,
                cmd.east_velocity_mps,
                0.0,  # Maintain altitude
                cmd.yaw_rad,
            )
            await self.target.offboard.set_velocity_ned(VelocityNedYaw(target_vn, target_ve, 0.0, target_yaw_deg))

            await asyncio.sleep(0.2)

        # Save telemetry log
        log_file = Path("mothdrone_telemetry.json")
        self._write_telemetry_snapshot()
        print(f"[MISSION] Telemetry saved to {log_file}")

        # Print summary
        print("\n" + "="*70)
        print("MISSION SUMMARY")
        print("="*70)
        print(f"Total loops: {loop_count}")
        print(f"Final range: {range_m:.1f}m")
        print(f"Hunter final position: N={hunter_state.north_m:.1f} E={hunter_state.east_m:.1f}")
        print(f"Target final position: N={target_state.north_m:.1f} E={target_state.east_m:.1f}")
        print(f"Target response: MOTOR STOP → FREEFALL → GROUND IMPACT → DISARM")
        print("="*70)


async def main():
    """Standalone test of the controller."""
    controller = MothdroneController()
    await controller.connect()
    await controller.arm_and_takeoff_both(20.0)
    await controller.run_interception()
    await controller.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
