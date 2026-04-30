# Mothdrone Interception Package

This package contains the Mothdrone VTOL SITL demo code, algorithm docs, world/model files, latest telemetry, graphs, and test report.

## What Is Included

- `launch_mothdrone.py`: main two-VTOL SITL launcher.
- `code/mothdrone_controller.py`: offboard takeoff, target mission, hunter guidance, 25 m trigger, SITL target kill, hunter RTL.
- `docs/GUIDANCE_STATE_MACHINE.md`: guidance state machine.
- `docs/ALGORITHM.md`: algorithm summary.
- `worlds/` and `models/`: local world/model assets.
- `mothdrone_telemetry.json`: latest verified run telemetry.
- `outputs/graphs/mothdrone_trajectory_3d.svg`: two-drone 3D trajectory image.
- `outputs/graphs/mothdrone_trajectory_path.svg`: two-drone top-down trajectory image.
- `outputs/graphs/mothdrone_mission_graph.svg`: path, range, and altitude graph.
- `outputs/test_reports/latest.md`: latest SITL verification report.

## External Requirements

This zip does not include PX4-Autopilot, Gazebo, QGroundControl, or Python virtual environments. The receiving machine needs:

- PX4-Autopilot built at `~/PX4-Autopilot`
- Gazebo Sim compatible with the local PX4 setup
- Python with `mavsdk`
- Optional: QGroundControl

## Run

```bash
cd "Mothdrone_Interception_Package"
python3 -m pip install -r requirements.txt
python3 launch_mothdrone.py
```

On this machine the verified interpreter was:

```bash
/Users/sanju/.venvs/mothdrone/bin/python launch_mothdrone.py
```

## Live 3D Graph

```bash
python3 code/live_trajectory_server.py
```

Open:

```text
http://127.0.0.1:8790/live_trajectory.html
```

## Latest Verified Result

- Start range: 40.0 m.
- Trigger range: 24.8 m.
- Target after trigger: SITL kill accepted, target altitude logged at 0.0 m.
- Hunter after trigger: climbed from 20.8 m to 59.5 m during recovery monitor, then RTL.

## Safety Note

The target motor-stop behavior is SITL-only. It is guarded by `MOTHDRONE_SITL_TARGET_KILL=1` in the launcher and must not be used as a real-aircraft motor-stop path.
