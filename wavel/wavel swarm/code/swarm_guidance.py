"""Safe Wavel swarm guidance primitives.

This module models formation navigation, mesh communication health, collision
avoidance, and non-destructive vision-based object confirmation.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class SwarmConfig:
    drone_count: int = 5
    cruise_speed_mps: float = 5.0
    max_speed_mps: float = 7.0
    min_separation_m: float = 6.0
    avoidance_gain: float = 7.0
    arrival_radius_m: float = 5.0
    object_confirm_radius_m: float = 45.0
    comms_range_m: float = 80.0
    control_dt_s: float = 0.2
    target_north_m: float = 120.0
    target_east_m: float = 40.0
    target_altitude_m: float = 18.0


@dataclass
class DroneState:
    drone_id: int
    north_m: float
    east_m: float
    altitude_m: float
    vn_mps: float = 0.0
    ve_mps: float = 0.0
    vz_mps: float = 0.0
    yaw_rad: float = 0.0
    armed: bool = False
    mode: str = "idle"
    object_confirmed: bool = False

    def to_dict(self) -> dict:
        data = asdict(self)
        data["yaw_deg"] = math.degrees(self.yaw_rad)
        return data


@dataclass(frozen=True)
class ObjectDetection:
    has_lock: bool
    confidence: float
    bearing_error_rad: float
    range_m: float
    label: str = "building"


class VisionObjectGate:
    """Simple object-confirmation gate for simulation and later OpenCV backends."""

    def __init__(self, config: SwarmConfig) -> None:
        self.config = config

    def detect(self, drone: DroneState) -> ObjectDetection:
        dn = self.config.target_north_m - drone.north_m
        de = self.config.target_east_m - drone.east_m
        rng = math.hypot(dn, de)
        bearing = math.atan2(de, dn)
        bearing_error = _wrap_pi(bearing - drone.yaw_rad)
        in_range = rng <= self.config.object_confirm_radius_m
        in_fov = abs(bearing_error) <= math.radians(35.0)
        confidence = max(0.0, min(0.98, 1.0 - rng / self.config.object_confirm_radius_m))
        has_lock = in_range and in_fov and confidence >= 0.25
        return ObjectDetection(has_lock, confidence if has_lock else 0.0, bearing_error, rng)


class SwarmController:
    """Formation controller with collision avoidance and mesh-comms awareness."""

    def __init__(self, config: SwarmConfig | None = None) -> None:
        self.config = config or SwarmConfig()
        self.formation_offsets = self._formation_offsets(self.config.drone_count)

    def arm_all(self, states: list[DroneState]) -> None:
        for state in states:
            state.armed = True
            state.mode = "offboard_swarm"

    def mesh_links(self, states: list[DroneState]) -> dict[int, list[int]]:
        links: dict[int, list[int]] = {state.drone_id: [] for state in states}
        for i, a in enumerate(states):
            for b in states[i + 1 :]:
                if self.distance_2d(a, b) <= self.config.comms_range_m:
                    links[a.drone_id].append(b.drone_id)
                    links[b.drone_id].append(a.drone_id)
        return links

    def step(self, states: list[DroneState], dt_s: float | None = None) -> list[DroneState]:
        dt = dt_s if dt_s is not None else self.config.control_dt_s
        leader_goal = (self.config.target_north_m, self.config.target_east_m)
        next_states: list[DroneState] = []
        links = self.mesh_links(states)

        for index, state in enumerate(states):
            offset_n, offset_e = self.formation_offsets[index]
            goal_n = leader_goal[0] + offset_n
            goal_e = leader_goal[1] + offset_e
            to_goal_n = goal_n - state.north_m
            to_goal_e = goal_e - state.east_m
            dist = max(math.hypot(to_goal_n, to_goal_e), 1e-6)
            desired_speed = min(self.config.cruise_speed_mps, dist / max(dt, 1e-6))
            cmd_n = to_goal_n / dist * desired_speed
            cmd_e = to_goal_e / dist * desired_speed

            avoid_n, avoid_e = self._avoidance_vector(state, states)
            cmd_n += avoid_n
            cmd_e += avoid_e

            speed = math.hypot(cmd_n, cmd_e)
            if speed > self.config.max_speed_mps:
                scale = self.config.max_speed_mps / speed
                cmd_n *= scale
                cmd_e *= scale

            alt_error = self.config.target_altitude_m - state.altitude_m
            vz = max(-1.5, min(1.5, alt_error))
            object_bearing = math.atan2(
                self.config.target_east_m - state.east_m,
                self.config.target_north_m - state.north_m,
            )
            yaw = object_bearing if dist <= self.config.object_confirm_radius_m else (
                math.atan2(cmd_e, cmd_n) if speed > 0.05 else state.yaw_rad
            )
            arrived = dist <= self.config.arrival_radius_m
            linked = len(links[state.drone_id]) > 0 or len(states) == 1

            next_states.append(
                DroneState(
                    drone_id=state.drone_id,
                    north_m=state.north_m + cmd_n * dt,
                    east_m=state.east_m + cmd_e * dt,
                    altitude_m=max(0.0, state.altitude_m + vz * dt),
                    vn_mps=cmd_n,
                    ve_mps=cmd_e,
                    vz_mps=vz,
                    yaw_rad=yaw,
                    armed=state.armed,
                    mode="hold_at_object" if arrived else ("offboard_swarm" if linked else "comms_degraded_hold"),
                    object_confirmed=state.object_confirmed,
                )
            )
        return next_states

    def _avoidance_vector(self, state: DroneState, states: list[DroneState]) -> tuple[float, float]:
        avoid_n = 0.0
        avoid_e = 0.0
        for other in states:
            if other.drone_id == state.drone_id:
                continue
            dn = state.north_m - other.north_m
            de = state.east_m - other.east_m
            dist = math.hypot(dn, de)
            if 1e-6 < dist < self.config.min_separation_m:
                strength = self.config.avoidance_gain * (self.config.min_separation_m - dist) / self.config.min_separation_m
                avoid_n += dn / dist * strength
                avoid_e += de / dist * strength
        return avoid_n, avoid_e

    @staticmethod
    def distance_2d(a: DroneState, b: DroneState) -> float:
        return math.hypot(a.north_m - b.north_m, a.east_m - b.east_m)

    @staticmethod
    def min_pair_distance(states: list[DroneState]) -> float:
        best = float("inf")
        for i, a in enumerate(states):
            for b in states[i + 1 :]:
                best = min(best, SwarmController.distance_2d(a, b))
        return best if best != float("inf") else 0.0

    @staticmethod
    def _formation_offsets(count: int) -> list[tuple[float, float]]:
        # Ordered to preserve the launch-line east/west ordering and reduce
        # crossing paths during convergence.
        base = [(-16.0, -16.0), (-8.0, -8.0), (0.0, 0.0), (-8.0, 8.0), (-16.0, 16.0)]
        if count <= len(base):
            return base[:count]
        extra = [(float(-8 * (i + 1)), float(18 * ((-1) ** i))) for i in range(count - len(base))]
        return base + extra


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
