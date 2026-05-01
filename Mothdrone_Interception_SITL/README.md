# Mothdrone VTOL Interception SITL

Two-standard-VTOL PX4/Gazebo simulation for a Mothdrone-style demo:

- target starts 100 m ahead of hunter
- target arms first
- hunter arms only after target `armed=true` is detected
- both VTOLs use offboard climb support until the altitude gate
- target then flies an offboard path at 4 m/s
- hunter listens to target telemetry and uses PN lead-pursuit guidance toward a predicted intercept point
- hunter speed ramps toward a 15 m/s cap with closing-velocity control
- at <= 25 m with vision-confirmation gate, hunter triggers the event
- target receives a SITL-only motor-stop/kill command and falls/disarms
- hunter holds/climbs, then hunter-only RTL/land
- Ubuntu/PX4 builds with slower estimator startup are handled by pre-arm health waiting and arm retry diagnostics

This package is for simulation only.

## Latest Verified SITL Result

From the latest completed telemetry in `mothdrone_telemetry.json`:

- start range: `99.6 m`
- trigger range: `24.9 m`
- trigger state: `proximity_event_confirmed`
- target moved before trigger: `E=99.7 -> 186.4 m`, `N=0.0 -> 5.6 m`
- hunter moved before trigger: `E=0.1 -> 161.5 m`, `N=0.2 -> 5.0 m`
- hunter commanded speed peak: `13.0 m/s`
- peak closing velocity: `9.4 m/s`
- target after trigger: SITL kill accepted, target altitude logged at `0.0 m`
- hunter recovery: climbed from `20.7 m` at trigger to `59.6 m`
- result: target did not RTL after trigger; hunter alone performed recovery/RTL
- log caveat: no `Accel #0 fail: TIMEOUT` was seen; PX4 still printed startup EKF/airspeed/heading warnings and a non-blocking `COM_DL_LOSS_T` type mismatch warning

Report:

- `outputs/test_reports/latest.md`

Images:

- `outputs/graphs/mothdrone_trajectory_3d.svg`
- `outputs/graphs/mothdrone_trajectory_path.svg`
- `outputs/graphs/mothdrone_mission_graph.svg`

## Visual Result

### 3D Trajectory

![Mothdrone 3D trajectory](outputs/graphs/mothdrone_trajectory_3d.svg)

### Mission Range And Altitude

![Mothdrone mission graph](outputs/graphs/mothdrone_mission_graph.svg)

## Run Full SITL

Ubuntu recommended:

```bash
cd Mothdrone_Interception_SITL
chmod +x scripts/*.sh
./scripts/run_linux_bundle.sh
```

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
| `code/mothdrone_controller.py` | Hunter offboard guidance, target offboard path, event trigger, target kill, hunter RTL |
| `code/live_trajectory_server.py` | Browser live 3D trajectory graph |
| `code/plot_mission_results.py` | Static SVG/HTML graph generator |
| `code/plot_trajectory_3d.py` | Static 3D trajectory SVG generator |
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
