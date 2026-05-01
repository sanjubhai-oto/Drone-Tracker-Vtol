"""X500 gimbal proximity-guidance for a two-airframe interception exercise.

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
    target_start_east_m: float = 100.0
    kill_zone_radius_m: float = 25.0
    vision_confirm_radius_m: float = 35.0
    takeoff_altitude_m: float = 20.0
    breakaway_altitude_m: float = 40.0
    max_hunter_speed_mps: float = 15.0
    target_speed_mps: float = 4.0
    pn_navigation_constant: float = 3.0
    min_safe_event_altitude_m: float = 8.0
    accel_limit_mps2: float = 1.5
    control_dt_s: float = 0.2


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
    speed_mps: float = 0.0
    closing_velocity_mps: float = 0.0
    lead_time_s: float = 0.0
    predicted_target_n_m: float = 0.0
    predicted_target_e_m: float = 0.0
    event: str | None = None


class TargetDrone:
    """Simulated target X500 gimbal with motor-stop / freefall / disarm states."""

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
        self._commanded_speed_mps = 0.0

    def command(self, hunter: VehicleState, target: VehicleState, vision: VisionConfirmation) -> GuidanceCommand:
        cfg = self.config
        rel_n = target.north_m - hunter.north_m
        rel_e = target.east_m - hunter.east_m
        range_m = math.hypot(rel_n, rel_e)
        bearing = math.atan2(rel_e, rel_n)
        rel_vn = target.vn_mps - hunter.vn_mps
        rel_ve = target.ve_mps - hunter.ve_mps
        closing_velocity = self._closing_velocity(rel_n, rel_e, rel_vn, rel_ve, range_m)

        if self._event_triggered:
            return GuidanceCommand(
                mode="hunter_breakaway_rtl",
                north_velocity_mps=0.0,
                east_velocity_mps=0.0,
                altitude_m=cfg.breakaway_altitude_m,
                yaw_rad=hunter.yaw_rad,
                closing_velocity_mps=closing_velocity,
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
                closing_velocity_mps=closing_velocity,
                event="target_motor_stop_freefall",
            )

        los_n = rel_n / max(range_m, 1e-6)
        los_e = rel_e / max(range_m, 1e-6)
        target_los_speed = target.vn_mps * los_n + target.ve_mps * los_e
        speed = self._lead_pursuit_speed(range_m, closing_velocity, target_los_speed)
        lead_time = self._lead_time(range_m, closing_velocity)
        predicted_n = target.north_m + target.vn_mps * lead_time
        predicted_e = target.east_m + target.ve_mps * lead_time
        lead_n = predicted_n - hunter.north_m
        lead_e = predicted_e - hunter.east_m
        lead_norm = max(math.hypot(lead_n, lead_e), 1e-6)
        lead_unit_n = lead_n / lead_norm
        lead_unit_e = lead_e / lead_norm
        los_rate = self._los_rate(rel_n, rel_e, rel_vn, rel_ve, range_m)
        lateral_speed = max(-4.0, min(4.0, cfg.pn_navigation_constant * closing_velocity * los_rate * 2.0))
        cmd_n = speed * lead_unit_n + lateral_speed * -lead_unit_e
        cmd_e = speed * lead_unit_e + lateral_speed * lead_unit_n
        cmd_norm = max(math.hypot(cmd_n, cmd_e), 1e-6)
        if cmd_norm > speed:
            cmd_n = cmd_n / cmd_norm * speed
            cmd_e = cmd_e / cmd_norm * speed
        return GuidanceCommand(
            mode="pn_lead_pursuit_visual_gate",
            north_velocity_mps=cmd_n,
            east_velocity_mps=cmd_e,
            altitude_m=cfg.takeoff_altitude_m,
            yaw_rad=math.atan2(cmd_e, cmd_n),
            speed_mps=speed,
            closing_velocity_mps=closing_velocity,
            lead_time_s=lead_time,
            predicted_target_n_m=predicted_n,
            predicted_target_e_m=predicted_e,
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

    @staticmethod
    def _closing_velocity(rel_n: float, rel_e: float, rel_vn: float, rel_ve: float, range_m: float) -> float:
        if range_m <= 1e-6:
            return 0.0
        return -((rel_n * rel_vn + rel_e * rel_ve) / range_m)

    @staticmethod
    def _los_rate(rel_n: float, rel_e: float, rel_vn: float, rel_ve: float, range_m: float) -> float:
        if range_m <= 1e-6:
            return 0.0
        return (rel_n * rel_ve - rel_e * rel_vn) / (range_m * range_m)

    def _lead_time(self, range_m: float, closing_velocity: float) -> float:
        effective_close = max(2.0, closing_velocity)
        return max(0.5, min(8.0, range_m / effective_close))

    def _desired_closing_velocity(self, range_m: float) -> float:
        if range_m > 80.0:
            return 11.0
        if range_m > 60.0:
            return 9.0
        if range_m > 45.0:
            return 6.0
        if range_m > self.config.vision_confirm_radius_m:
            return 3.5
        return 1.6

    def _lead_pursuit_speed(self, range_m: float, closing_velocity: float, target_los_speed: float = 0.0) -> float:
        cfg = self.config
        desired_closing = self._desired_closing_velocity(range_m)
        target_speed = target_los_speed + desired_closing
        if range_m <= cfg.vision_confirm_radius_m:
            target_speed = min(target_speed, cfg.target_speed_mps + 2.0)
        if closing_velocity > desired_closing + 2.0 and range_m < 45.0:
            target_speed = min(target_speed, cfg.target_speed_mps + 1.0)
        target_speed = max(cfg.target_speed_mps + 1.0, min(cfg.max_hunter_speed_mps, target_speed))
        max_delta = cfg.accel_limit_mps2 * cfg.control_dt_s
        if target_speed > self._commanded_speed_mps:
            self._commanded_speed_mps = min(target_speed, self._commanded_speed_mps + max_delta)
        else:
            self._commanded_speed_mps = max(target_speed, self._commanded_speed_mps - 2.0 * max_delta)
        return max(0.0, min(cfg.max_hunter_speed_mps, self._commanded_speed_mps))


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
