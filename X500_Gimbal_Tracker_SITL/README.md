# X500 Gimbal Drone Tracker SITL

Two `x500_gimbal` PX4/Gazebo vehicles using the same hunter-target guidance concept as the VTOL demo.

- target starts 100 m ahead of hunter
- target arms first; hunter arms only after target `armed=true`
- both vehicles run offboard climb from the start
- target flies a slow path at 4 m/s
- hunter uses lead-pursuit guidance from target telemetry
- simulated visual confirmation gates the 25 m trigger
- target event is SITL-only motor stop/freefall/disarm
- hunter keeps altitude, breaks away, then RTL/land
- live browser page shows 3D trajectory plus hunter/target camera panels

## Run On Ubuntu

```bash
cd X500_Gimbal_Tracker_SITL
chmod +x scripts/*.sh
./scripts/run_linux_bundle.sh
```

Requirements:

- PX4-Autopilot built at `~/PX4-Autopilot`
- PX4 model `Tools/simulation/gz/models/x500_gimbal/model.sdf`
- PX4 SITL binary `build/px4_sitl_default/bin/px4`
- Gazebo Sim `gz`
- Python package `mavsdk`

The launcher uses PX4 airframe `4019` (`gz_x500_gimbal`).

## Live Trajectory And Camera Page

Start this before or during SITL:

```bash
cd X500_Gimbal_Tracker_SITL
python3 code/live_trajectory_server.py
```

Open:

```text
http://127.0.0.1:8790/live_trajectory.html
```

The page refreshes `x500_gimbal_telemetry.json` every 0.5 s and also displays:

- `/camera/hunter.jpg`
- `/camera/target.jpg`

If a camera bridge writes frames to `outputs/camera/hunter_latest.jpg` and `outputs/camera/target_latest.jpg`, the page displays them live. Until then it shows standby camera panels instead of a blank area.

## Gazebo View Control

Automatic Gazebo camera follow is disabled by default so the GUI does not lock onto one vehicle.

```bash
./scripts/follow_gz_model.sh x500_gimbal_0
./scripts/follow_gz_model.sh x500_gimbal_1
./scripts/follow_gz_model.sh any off
```

## Main Files

| File | Purpose |
| --- | --- |
| `launch_x500_gimbal.py` | Starts Gazebo/PX4 for hunter and target, then runs MAVSDK mission |
| `code/x500_gimbal_controller.py` | Offboard arm/takeoff, guidance, 25 m trigger, target freefall simulation, hunter RTL |
| `code/vtol_proximity_guidance.py` | Dependency-free guidance and freefall model |
| `code/live_trajectory_server.py` | Browser trajectory and camera-feed page |
| `scripts/run_linux_bundle.sh` | One-command Ubuntu launcher with diagnostics |
| `scripts/follow_gz_model.sh` | Optional Gazebo GUI camera tracking |
| `outputs/graphs/` | Static 3D trajectory and mission graphs |
| `x500_gimbal_telemetry.json` | Latest telemetry bundle |

## Safety Boundary

The target motor-stop/freefall behavior is simulation-only. It is enabled by `X500_GIMBAL_SITL_TARGET_KILL=1` inside the launcher and must not be used as a real-aircraft motor-stop path.
