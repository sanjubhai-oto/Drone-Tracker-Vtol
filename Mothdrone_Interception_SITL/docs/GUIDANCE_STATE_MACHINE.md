# Mothdrone Guidance State Machine

This is the first implementation target for the VTOL demo. It is SITL-only for the target motor-stop/freefall event.

## Fixed Mission Concept

- Hunter starts at home.
- Target starts 100 m east of hunter.
- Target arms first.
- Hunter arms only after target `armed=true` is detected.
- Both VTOLs use offboard climb support until the altitude gate.
- No horizontal guidance starts until both VTOLs are above 20 m.
- Target stays in offboard and runs a simple path at about 2 m/s.
- Hunter starts pursuing target after the altitude gate.
- Hunter pursuit speed is capped at 5 m/s.
- When range is <= 25 m and vision confirmation is locked, hunter triggers the event signal.
- Target receives a SITL-only motor-stop command and also enters the local deterministic freefall/disarm event model.
- Hunter does not descend with target.
- Hunter holds altitude, climbs toward 40 m, then returns home and lands.

## States

| State | Owner | Condition | Command |
| --- | --- | --- | --- |
| `TARGET_ARM` | target | mission start | arm target and wait for target `armed=true` |
| `HUNTER_ARM_SYNC` | hunter | target armed detected | arm hunter |
| `TAKEOFF_SYNC` | both | both armed | prime/start offboard climb support |
| `ALTITUDE_GATE` | both | after takeoff command | wait until hunter and target altitude >= 20 m |
| `TARGET_OFFBOARD_PATH` | target | altitude gate passed | offboard path velocity at about 2 m/s |
| `HUNTER_GUIDANCE` | hunter | altitude gate passed and range > 25 m | pursue target bearing, max 5 m/s, hold altitude |
| `VISION_CONFIRM` | hunter | range <= 35 m | require lock and confidence >= 0.55 |
| `PROXIMITY_TRIGGER` | both | range <= 25 m and vision locked | emit `target_motor_stop_freefall` |
| `TARGET_FREEFALL_SIM` | target | trigger received | MAVSDK SITL kill/terminate, plus local physics: motors off, gravity descent, disarm at ground |
| `HUNTER_BREAKAWAY_HOLD` | hunter | trigger received | hold altitude, then climb command |
| `RTL_LAND` | hunter | breakaway complete | stop offboard, RTL, land at home |
| `POST_EVENT_VERIFY` | hunter | RTL command sent | log hunter altitude and position for recovery analysis |

## Control Law

The live MAVSDK runner uses shared NED coordinates:

```text
relative_n = target_n - hunter_n
relative_e = target_e - hunter_e
range = sqrt(relative_n^2 + relative_e^2)
bearing = atan2(relative_e, relative_n)
```

Hunter velocity command:

```text
if range > 35 m:
    speed = 5 m/s
else:
    speed = clamp(2 + 0.8 * (range - 25), 3, 5) m/s

vn = speed * cos(bearing)
ve = speed * sin(bearing)
vd = 0
yaw = bearing
```

Trigger condition:

```text
range <= 25 m
and hunter_alt >= 5 m
and target_alt >= 5 m
and vision_lock == true
and vision_confidence >= 0.55
```

## Files

- `launch_mothdrone.py`: starts two PX4 standard VTOL SITL vehicles. Target spawn pose is `0,100,0,0,0,0`.
- `code/mothdrone_controller.py`: live MAVSDK state machine and guidance runner.
- `code/live_trajectory_server.py`: browser live 3D trajectory page.
- `code/plot_trajectory_3d.py`: static 3D trajectory SVG generator.
- `code/vtol_proximity_guidance.py`: dependency-free algorithm model and freefall physics.
- `tests/test_vtol_freefall.py`: unit check for freefall/disarm physics.
- `mothdrone_telemetry.json`: latest run telemetry, including guidance and post-event recovery samples.

## Important Limit

The freefall event is a SITL demo behavior. It is not a real-aircraft motor-stop command path.
The live launcher sets `MOTHDRONE_SITL_TARGET_KILL=1` before the mission controller starts. Without that variable, the controller only logs the local freefall model and will not send the target kill/terminate command.
