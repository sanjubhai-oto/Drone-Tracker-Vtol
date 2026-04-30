# Mothdrone VTOL Interception SITL

Two-standard-VTOL PX4/Gazebo simulation for a Mothdrone-style demo:

- target starts 40 m ahead of hunter
- both VTOLs arm together and enter offboard
- both climb by offboard velocity setpoints
- target flies a simple offboard mission at 2 m/s
- hunter listens to target telemetry and navigates toward it
- at <= 25 m with vision-confirmation gate, hunter triggers the event
- target receives a SITL-only motor-stop/kill command and falls/disarms
- hunter holds/climbs, then hunter-only RTL/land

This package is for simulation only.

## Latest Verified SITL Result

From `mothdrone_telemetry.json`:

- start range: `40.0 m`
- trigger range: `24.8 m`
- trigger state: `proximity_event_confirmed`
- target after trigger: SITL kill accepted, target altitude logged at `0.0 m`
- hunter recovery: climbed from `20.8 m` to `59.5 m`
- result: target did not RTL after trigger; hunter alone performed recovery/RTL

Report:

- `outputs/test_reports/latest.md`

Images:

- `outputs/graphs/mothdrone_trajectory_3d.svg`
- `outputs/graphs/mothdrone_trajectory_path.svg`
- `outputs/graphs/mothdrone_mission_graph.svg`

## Run Full SITL

Prerequisites:

- PX4-Autopilot built at `~/PX4-Autopilot`
- Gazebo Sim working with PX4
- Python with `mavsdk`
- Optional: QGroundControl

On this machine:

```bash
cd "/Users/sanju/EGO-LEVEL-NAVIGATION/VTOL/Mothdrone Interception System (dosan)"
/Users/sanju/.venvs/mothdrone/bin/python launch_mothdrone.py
```

Generic:

```bash
cd "Mothdrone Interception System (dosan)"
python3 -m pip install -r requirements.txt
python3 launch_mothdrone.py
```

## Live 3D Trajectory Graph

Start the live graph server before or during SITL:

```bash
cd "Mothdrone Interception System (dosan)"
python3 code/live_trajectory_server.py
```

Open:

```text
http://127.0.0.1:8790/live_trajectory.html
```

The controller writes `mothdrone_telemetry.json` during flight, so the browser graph updates every 0.5 s. The live page draws a 3D-style perspective path using:

- easting
- northing
- altitude
- target fall-to-ground telemetry
- hunter recovery climb/RTL telemetry

## Main Files

| File | Purpose |
| --- | --- |
| `launch_mothdrone.py` | Starts two PX4 standard VTOL SITL vehicles and runs mission |
| `code/mothdrone_controller.py` | Offboard takeoff, target mission, hunter guidance, event trigger, target kill, hunter RTL |
| `code/live_trajectory_server.py` | Browser live trajectory graph |
| `code/plot_mission_results.py` | Static SVG/HTML graph generator |
| `docs/GUIDANCE_STATE_MACHINE.md` | Mission state machine |
| `docs/ALGORITHM.md` | Algorithm explanation |
| `worlds/` and `models/` | Gazebo/PX4 assets |
| `mothdrone_telemetry.json` | Latest verified telemetry |

## Safety Boundary

The target motor-stop behavior is SITL-only. The launcher sets:

```text
MOTHDRONE_SITL_TARGET_KILL=1
```

That path must not be used as a real-aircraft motor-stop path.
