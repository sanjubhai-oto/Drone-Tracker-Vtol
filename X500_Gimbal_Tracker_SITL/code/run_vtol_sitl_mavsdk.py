#!/usr/bin/env python3
"""MAVSDK runner: connect to two PX4 SITL systems and run safe proximity event.

Connects to UDP ports 14550 and 14551, arms both, takeoff to 20m, pursues
hunter->target with guidance, triggers safe SITL event (target land) when
within kill zone and simulated vision confirmation.
"""

import asyncio
import math
from mavsdk import System

from x500_gimbal_proximity_guidance import VtolProximityGuidance, VtolGuidanceConfig, VisionConfirmation, VehicleState, simulated_target_response_after_event

KILL_ZONE = 25.0
VISION_CONF_RADIUS = 35.0
TAKEOFF_ALT = 20.0

async def connect(address: str) -> System:
    sys = System()
    print(f"Connecting to {address}...")
    await sys.connect(system_address=address)
    # wait for connection
    for i in range(50):
        try:
            async for state in sys.core.connection_state():
                if state.is_connected:
                    print(f"Connected to {address}")
                    return sys
                break
        except Exception:
            pass
        await asyncio.sleep(0.1)
    raise RuntimeError(f"Failed to connect to {address}")

async def arm_and_takeoff(sys: System, alt_m: float):
    print("Waiting for GUIDED/ARM readiness...")
    await asyncio.sleep(1)
    print("Arming")
    await sys.action.arm()
    await asyncio.sleep(0.5)
    print(f"Taking off to {alt_m}m")
    await sys.action.set_takeoff_altitude(alt_m)
    await sys.action.takeoff()
    await asyncio.sleep(6)

async def get_position(sys: System):
    async for pos in sys.telemetry.position():
        return pos

async def monitor(hunter: System, target: System):
    cfg = VtolGuidanceConfig()
    guidance = VtolProximityGuidance(cfg)
    event_triggered = False

    while True:
        # get simple position (lat/lon/rel alt) and convert basic N/E (naive for demo)
        hpos = await get_position(hunter)
        tpos = await get_position(target)
        # Use latitude (deg) mapped to meters roughly
        # This is an approximation for demo only. For SITL local NED use local_position_ned if available.
        lat_scale = 111319.5  # meters per degree lat
        hn = hpos.latitude_deg * lat_scale
        he = hpos.longitude_deg * (lat_scale * math.cos(math.radians(hpos.latitude_deg)))
        tn = tpos.latitude_deg * lat_scale
        te = tpos.longitude_deg * (lat_scale * math.cos(math.radians(tpos.latitude_deg)))
        halt = hpos.relative_altitude_m
        talt = tpos.relative_altitude_m

        hunter_state = VehicleState(hn, he, -halt, 0.0, 0.0, 0.0)
        target_state = VehicleState(tn, te, -talt, 0.0, 0.0, 0.0)
        # Simulate vision confirmation when range < VISION_CONF_RADIUS
        rel_n = tn - hn
        rel_e = te - he
        rng = math.hypot(rel_n, rel_e)
        vision = VisionConfirmation(has_lock=(rng < VISION_CONF_RADIUS), confidence=0.8 if rng < VISION_CONF_RADIUS else 0.0, bearing_error_rad=0.0)

        cmd = guidance.command(hunter_state, target_state, vision)
        print("Guidance cmd:", cmd)

        if cmd.event == 'simulated_safe_target_termination' and not event_triggered:
            event_triggered = True
            print('Proximity event: commanding target to land (SITL-safe) and hunter RTL')
            # command target to land
            await target.action.land()
            # command hunter return to launch
            await hunter.action.return_to_launch()
            # print simulated freefall report (visualization only)
            print(simulated_target_response_after_event(target_state))
            break

        await asyncio.sleep(1)

async def main():
    # Connect to the two expected endpoints
    hunter_addr = "udp://127.0.0.1:14550"
    target_addr = "udp://127.0.0.1:14551"
    hunter = await connect(hunter_addr)
    target = await connect(target_addr)

    # Arm and takeoff both
    await arm_and_takeoff(hunter, TAKEOFF_ALT)
    await arm_and_takeoff(target, TAKEOFF_ALT)

    # Monitor until event
    await monitor(hunter, target)

if __name__ == '__main__':
    asyncio.run(main())
