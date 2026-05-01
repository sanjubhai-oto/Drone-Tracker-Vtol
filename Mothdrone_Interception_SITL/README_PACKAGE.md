# Mothdrone Interception Package

This package contains the Mothdrone VTOL SITL demo code, algorithm docs, world file, latest telemetry, graphs, and test report.

## Included

- `launch_mothdrone.py`: one-command two-VTOL PX4/Gazebo launcher.
- `code/mothdrone_controller.py`: target-first arm sync, both-offboard climb, moving target path, hunter guidance, 25 m trigger, SITL-only target kill/freefall, hunter RTL.
- `code/vtol_proximity_guidance.py`: dependency-free shared guidance model used by tests and pure simulation.
- `code/simulated_runner.py`: quick pure-Python run without PX4/Gazebo.
- `code/live_trajectory_server.py`: live browser 3D trajectory graph.
- `docs/GUIDANCE_STATE_MACHINE.md`: state machine and control law.
- `docs/ALGORITHM.md`: algorithm summary.
- `worlds/mothdrone_world.sdf`: Gazebo world asset.
- `mothdrone_telemetry.json`: latest verified SITL telemetry.
- `simulated_telemetry.json`: latest pure-guidance simulation telemetry.
- `outputs/graphs/`: latest trajectory and mission graphs.
- `outputs/test_reports/latest.md`: latest verification report.

## External Requirements

This zip does not include PX4-Autopilot, Gazebo, QGroundControl, or Python virtual environments.

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

On the verified machine:

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

## Latest Verified SITL Result

- Start range: 99.6 m.
- Trigger range: 24.9 m.
- Target path before trigger: E=99.7 m to E=186.4 m.
- Hunter path before trigger: E=0.1 m to E=161.5 m.
- Hunter command speed peak: 13.0 m/s.
- Peak closing velocity: 9.4 m/s.
- Target after trigger: SITL kill accepted, freefall/impact/disarm logged.
- Hunter after trigger: climbed from 20.7 m at trigger to 59.6 m during recovery monitor, then hunter-only RTL.

## Safety Note

The target motor-stop behavior is SITL-only. It is guarded by `MOTHDRONE_SITL_TARGET_KILL=1` in the launcher and must not be used as a real-aircraft motor-stop path.
