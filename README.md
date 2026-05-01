# PX4 Sim Starter

Starter workspace for PX4 SITL, Gazebo, ROS 2, and multi-vehicle simulation examples.

The repository now includes two packaged PX4/Gazebo demos:

- `Mothdrone_Interception_SITL/`: two standard VTOL vehicles
- `X500_Gimbal_Tracker_SITL/`: two `x500_gimbal` multicopters with the same guidance concept

The Mothdrone package includes:

- PX4/Gazebo standard VTOL launch
- two-airframe hunter + target control
- full offboard takeoff
- target-first arm sync
- moving target offboard path
- hunter guidance toward target telemetry
- 25 m proximity trigger
- SITL-only target motor-stop/kill event
- hunter-only breakaway and RTL
- telemetry logging
- static and live 3D trajectory visualisation
- Ubuntu/PX4 pre-arm health waiting and retry diagnostics for `COMMAND_DENIED` arming failures

## Repository Layout

| Path | Purpose |
| --- | --- |
| `PX4/` | PX4 setup notes |
| `Gazebo/` | Gazebo/GZ setup notes |
| `ROS2Humbel/` | ROS 2 Humble notes |
| `Mothdrone_Interception_SITL/` | Full two-VTOL SITL demo package |
| `X500_Gimbal_Tracker_SITL/` | Two `x500_gimbal` hunter/target SITL package |

## Mothdrone Interception SITL

Go to:

```bash
cd Mothdrone_Interception_SITL
```

Main files:

| File | Purpose |
| --- | --- |
| `launch_mothdrone.py` | Starts two PX4 standard VTOL SITL vehicles and runs mission |
| `code/mothdrone_controller.py` | Target-first arm sync, both-offboard takeoff, moving target path, hunter guidance, trigger, target kill, hunter RTL |
| `code/live_trajectory_server.py` | Live browser 3D trajectory graph |
| `outputs/graphs/mothdrone_trajectory_3d.svg` | Static 3D trajectory image |
| `outputs/graphs/mothdrone_mission_graph.svg` | Path/range/altitude graph |
| `outputs/test_reports/latest.md` | Latest verification report |
| `docs/GUIDANCE_STATE_MACHINE.md` | State machine and control logic |

### Latest Verified Result

From the included telemetry:

- start range: `99.6 m`
- trigger range: `24.9 m`
- target path before trigger: `E=99.7 -> 186.4 m`, `N=0.0 -> 5.6 m`
- hunter path before trigger: `E=0.1 -> 161.5 m`, `N=0.2 -> 5.0 m`
- hunter speed peak: `13.0 m/s`
- peak closing velocity: `9.4 m/s`
- target after trigger: SITL kill accepted, altitude logged at `0.0 m`
- hunter after trigger: climbed from `20.7 m` to `59.6 m`
- recovery: target did not RTL; hunter alone performed breakaway/RTL

### Latest Graphs

![Mothdrone 3D trajectory](Mothdrone_Interception_SITL/outputs/graphs/mothdrone_trajectory_3d.svg)

![Mothdrone mission graph](Mothdrone_Interception_SITL/outputs/graphs/mothdrone_mission_graph.svg)

### Run SITL

Prerequisites:

- PX4-Autopilot built at `~/PX4-Autopilot`
- Gazebo Sim configured for PX4
- Python with `mavsdk`
- Optional: QGroundControl

Run:

```bash
cd Mothdrone_Interception_SITL
chmod +x scripts/*.sh
./scripts/run_linux_bundle.sh
```

On the original tested macOS machine:

```bash
/Users/sanju/.venvs/mothdrone/bin/python launch_mothdrone.py
```

### Live 3D Trajectory

Start this before or during SITL:

```bash
cd Mothdrone_Interception_SITL
python3 code/live_trajectory_server.py
```

Open:

```text
http://127.0.0.1:8790/live_trajectory.html
```

The page refreshes from `mothdrone_telemetry.json` every 0.5 s. The controller writes telemetry during flight.

## Safety Boundary

The Mothdrone target motor-stop behavior is simulation-only. It is guarded in the launcher by:

```text
MOTHDRONE_SITL_TARGET_KILL=1
```

Do not use this as a real-aircraft motor-stop path.

## X500 Gimbal Tracker SITL

Run the X500 package:

```bash
cd X500_Gimbal_Tracker_SITL
chmod +x scripts/*.sh
./scripts/run_linux_bundle.sh
```

Live trajectory and camera panels:

```bash
python3 code/live_trajectory_server.py
```

Open:

```text
http://127.0.0.1:8790/live_trajectory.html
```
