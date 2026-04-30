"""VTOL proximity-guidance for a two-airframe interception exercise.

This module models:
- target navigation (cooperative target moving at constant speed)
- hunter GPS-based lead pursuit
- vision-based final confirmation
- proximity trigger at <= 25 m radius
- target MOTOR STOP → freefall → ground impact → DISARM
- hunter breakaway and RTL
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VtolGuidanceConfig:
    target_start_east_m: float = 40.0
    kill_zone_radius_m: float = 25.0
    vision_confirm_radius_m: float = 35.0
    takeoff_altitude_m: float = 20.0
    breakaway_altitude_m: float = 40.0
    max_hunter_speed_mps: float = 5.0
    target_speed_mps: float = 2.0
    pn_navigation_constant: float = 3.0
    min_safe_event_altitude_m: float = 8.0


@dataclass(frozen=True)
class VehicleState:
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


@dataclass(frozen=True)
class VisionConfirmation:
    has_lock: bool
    confidence: float
    bearing_error_rad: float


@dataclass(frozen=True)
class GuidanceCommand:
    mode: str
    north_velocity_mps: float
    east_velocity_mps: float
    altitude_m: float
    yaw_rad: float
    event: str | None = None


class TargetDrone:
    """Simulated target VTOL with motor-stop / freefall / disarm states."""

    def __init__(self, initial: VehicleState) -> None:
        self.state = initial
        self.motors_armed: bool = True
        self.freefall: bool = False
        self.disarmed: bool = False
        self.impact_velocity_mps: float | None = None
        self.g = 9.81  # m/s^2

    def step(self, dt: float) -> None:
        """Integrate state forward by dt seconds."""
        if self.freefall:
            # Motors stopped: only gravity acts downward
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
            # Normal flight: target cruises at constant altitude and target_speed_mps east
            self.state = VehicleState(
                north_m=self.state.north_m + self.state.vn_mps * dt,
                east_m=self.state.east_m + self.state.ve_mps * dt,
                down_m=self.state.down_m,
                vn_mps=self.state.vn_mps,
                ve_mps=self.state.ve_mps,
                vd_mps=0.0,
                yaw_rad=self.state.yaw_rad,
            )

        # Ground contact check
        if self.state.altitude_m <= 0.0:
            self.impact_velocity_mps = self.state.vd_mps
            self.state = VehicleState(
                north_m=self.state.north_m,
                east_m=self.state.east_m,
                down_m=0.0,
                vn_mps=0.0,
                ve_mps=0.0,
                vd_mps=0.0,
                yaw_rad=self.state.yaw_rad,
            )
            self.disarmed = True
            self.motors_armed = False
            self.freefall = False

    def stop_motors_and_freefall(self) -> None:
        """Trigger motor stop → freefall."""
        if not self.disarmed:
            self.motors_armed = False
            self.freefall = True

    def status(self) -> dict:
        return {
            "altitude_m": round(self.state.altitude_m, 2),
            "motors_armed": self.motors_armed,
            "freefall": self.freefall,
            "disarmed": self.disarmed,
            "vd_mps": round(self.state.vd_mps, 2),
        }


class VtolProximityGuidance:
    """GPS pursuit + visual confirmation for a cooperative target."""

    def __init__(self, config: VtolGuidanceConfig | None = None) -> None:
        self.config = config or VtolGuidanceConfig()
        self._event_triggered = False

    def command(self, hunter: VehicleState, target: VehicleState, vision: VisionConfirmation) -> GuidanceCommand:
        cfg = self.config
        rel_n = target.north_m - hunter.north_m
        rel_e = target.east_m - hunter.east_m
        range_m = math.hypot(rel_n, rel_e)
        bearing = math.atan2(rel_e, rel_n)

        if self._event_triggered:
            return GuidanceCommand(
                mode="hunter_breakaway_rtl",
                north_velocity_mps=0.0,
                east_velocity_mps=0.0,
                altitude_m=cfg.breakaway_altitude_m,
                yaw_rad=hunter.yaw_rad,
                event="breakaway_then_rtl",
            )

        if self._should_trigger_event(range_m, hunter, target, vision):
            self._event_triggered = True
            return GuidanceCommand(
                mode="proximity_event_confirmed",
                north_velocity_mps=0.0,
                east_velocity_mps=0.0,
                altitude_m=cfg.breakaway_altitude_m,
                yaw_rad=bearing,
                event="target_motor_stop_freefall",
            )

        speed = self._lead_pursuit_speed(hunter, target, range_m)
        return GuidanceCommand(
            mode="gps_lead_pursuit_visual_gate",
            north_velocity_mps=speed * math.cos(bearing),
            east_velocity_mps=speed * math.sin(bearing),
            altitude_m=cfg.takeoff_altitude_m,
            yaw_rad=bearing,
            event=None,
        )

    def _should_trigger_event(
        self,
        range_m: float,
        hunter: VehicleState,
        target: VehicleState,
        vision: VisionConfirmation,
    ) -> bool:
        cfg = self.config
        if range_m > cfg.kill_zone_radius_m:
            return False
        if hunter.altitude_m < cfg.min_safe_event_altitude_m or target.altitude_m < cfg.min_safe_event_altitude_m:
            return False
        if range_m <= cfg.vision_confirm_radius_m:
            return vision.has_lock and vision.confidence >= 0.55
        return False

    def _lead_pursuit_speed(self, hunter: VehicleState, target: VehicleState, range_m: float) -> float:
        cfg = self.config
        if range_m > cfg.vision_confirm_radius_m:
            return cfg.max_hunter_speed_mps

        range_error = max(0.0, range_m - cfg.kill_zone_radius_m)
        commanded = cfg.target_speed_mps + 0.8 * range_error
        return max(cfg.target_speed_mps + 1.0, min(cfg.max_hunter_speed_mps, commanded))


def simulate_target_freefall(target: TargetDrone, dt: float = 0.05) -> list[dict]:
    """Run freefall simulation until ground impact / disarm and return time-series log."""
    log: list[dict] = []
    t = 0.0
    max_steps = 5000
    for _ in range(max_steps):
        target.step(dt)
        t += dt
        log.append(
            {
                "time_s": round(t, 3),
                "altitude_m": round(target.state.altitude_m, 2),
                "vd_mps": round(target.state.vd_mps, 2),
                "disarmed": target.disarmed,
            }
        )
        if target.disarmed:
            break
    return log


def simulated_target_response_after_event(target_state: VehicleState) -> dict:
    """Backward-compatible SITL-only freefall summary for the MAVSDK runner.

    Builds a temporary TargetDrone from a VehicleState, simulates freefall,
    and returns a short summary: time to impact and impact vertical speed.
    This is strictly for SITL/visualization and must NOT be used to stop real
    motors on an actual vehicle.
    """
    if target_state.altitude_m <= 0.0:
        return {"time_to_impact_s": 0.0, "impact_velocity_mps": 0.0, "action": "ALREADY_ON_GROUND"}
    td = TargetDrone(target_state)
    td.stop_motors_and_freefall()
    timeline = simulate_target_freefall(td, dt=0.05)
    if not timeline:
        return {"time_to_impact_s": 0.0, "impact_velocity_mps": 0.0, "action": "no_simulation"}
    last = timeline[-1]
    impact_velocity = td.impact_velocity_mps
    if impact_velocity is None:
        impact_velocity = float(last.get("vd_mps", 0.0))
    return {
        "time_to_impact_s": float(last.get("time_s", 0.0)),
        "impact_velocity_mps": round(float(impact_velocity), 3),
        "action": "SITL_SIMULATED_FREEFALL",
    }
