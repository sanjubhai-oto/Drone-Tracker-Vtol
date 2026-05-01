# X500 Gimbal Tracker Package

Portable package for the two-vehicle `x500_gimbal` PX4/Gazebo SITL demo.

## Included

- `launch_x500_gimbal.py`: two-vehicle PX4/Gazebo launcher.
- `code/x500_gimbal_controller.py`: target-first arm sync, both-offboard climb, target path, hunter guidance, 25 m trigger, SITL-only target freefall, hunter RTL.
- `code/vtol_proximity_guidance.py`: dependency-free guidance/freefall model.
- `code/live_trajectory_server.py`: live browser page with 3D trajectory and both camera panels.
- `scripts/run_linux_bundle.sh`: Ubuntu one-command run script.
- `scripts/check_linux_env.py`: PX4/Gazebo/model preflight checks.
- `outputs/graphs/`: static trajectory and mission graphs.
- `x500_gimbal_telemetry.json`: latest telemetry sample.

## Run

```bash
cd X500_Gimbal_Tracker_SITL
chmod +x scripts/*.sh
./scripts/run_linux_bundle.sh
```

## Live Browser Page

```bash
python3 code/live_trajectory_server.py
```

Open:

```text
http://127.0.0.1:8790/live_trajectory.html
```

The camera panels load `outputs/camera/hunter_latest.jpg` and `outputs/camera/target_latest.jpg` if a camera bridge writes them.

## Safety Note

The target motor-stop/freefall behavior is SITL-only and guarded by `X500_GIMBAL_SITL_TARGET_KILL=1`.
