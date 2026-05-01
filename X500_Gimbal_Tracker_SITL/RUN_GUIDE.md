# X500 Gimbal Tracker Run Guide

## One-Command Ubuntu Run

```bash
cd X500_Gimbal_Tracker_SITL
chmod +x scripts/*.sh
./scripts/run_linux_bundle.sh
```

The bundle:

- cleans old PX4/Gazebo/MAVSDK processes
- checks PX4, Gazebo, `x500_gimbal`, plugins, and Python `mavsdk`
- starts Gazebo explicitly
- starts hunter as PX4 instance 0 and target as PX4 instance 1
- uses PX4 airframe `4019` / `gz_x500_gimbal`
- disables automatic Gazebo camera follow by default
- runs the MAVSDK mission controller
- writes logs into `logs/`

## Live Browser Page

In another terminal:

```bash
cd X500_Gimbal_Tracker_SITL
python3 code/live_trajectory_server.py
```

Open:

```text
http://127.0.0.1:8790/live_trajectory.html
```

The page shows:

- live 3D trajectory from `x500_gimbal_telemetry.json`
- hunter camera panel from `outputs/camera/hunter_latest.jpg`
- target camera panel from `outputs/camera/target_latest.jpg`

If the image files are missing, standby panels are shown so the UI is never blank.

## Manual Gazebo Camera Tracking

```bash
./scripts/follow_gz_model.sh x500_gimbal_0
./scripts/follow_gz_model.sh x500_gimbal_1
./scripts/follow_gz_model.sh any off
```

## Environment Variables

| Variable | Default | Purpose |
| --- | --- | --- |
| `X500_GIMBAL_PX4_DIR` | `~/PX4-Autopilot` | PX4 source/build path |
| `X500_GIMBAL_EXPLICIT_GZ` | `1` | Start Gazebo before PX4 |
| `X500_GIMBAL_GZ_AUTO_FOLLOW` | `0` | Let PX4/GZ auto-follow one model |
| `X500_GIMBAL_GZ_WORLD` | `default` | PX4 Gazebo world name |

## Safety Boundary

The target motor-stop/freefall event is simulation-only and guarded by `X500_GIMBAL_SITL_TARGET_KILL=1`.
