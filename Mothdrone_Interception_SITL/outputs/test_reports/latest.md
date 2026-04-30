# Mothdrone 100m Both-Offboard VTOL SITL Report

Result: PASS for GUI launch, target-first arm sync, both-offboard climb, moving target offboard path, hunter guidance from 100 m, 25 m trigger, SITL target kill/fall, and hunter-only RTL.

Key telemetry:
- Spawn: hunter at 0 m, target at 100 m east.
- Arm sync: target armed first; hunter armed only after target `armed=true` was detected.
- Takeoff: both vehicles entered offboard and climbed by offboard velocity setpoints.
- Takeoff gate: guidance started after hunter reached 21.6 m and target reached 22.0 m.
- Target path: target stayed in offboard and moved from E=100.0 m to E=155.2 m, N=0.0 m to N=10.6 m.
- Hunter path: hunter moved from E=0.1 m to E=127.5 m before recovery.
- Start range: 99.9 m.
- Trigger range: 25.0 m.
- Trigger state: `proximity_event_confirmed`.
- Vision gate: `vision_lock=true` at trigger.
- Target after trigger: SITL kill accepted, target altitude logged at 0.0 m.
- Hunter after trigger: held/climbed; altitude rose from 21.5 m at trigger to 59.6 m during recovery monitor.
- Hunter-only recovery: hunter RTL command sent after breakaway.

Artifacts:
- Telemetry: `mothdrone_telemetry.json`
- 3D trajectory SVG: `outputs/graphs/mothdrone_trajectory_3d.svg`
- Mission graph SVG: `outputs/graphs/mothdrone_mission_graph.svg`
- Mission graph HTML: `outputs/graphs/mothdrone_mission_graph.html`
- Controller: `code/mothdrone_controller.py`
- State machine: `docs/GUIDANCE_STATE_MACHINE.md`

Notes:
- This run uses both-offboard control, not PX4 target mission mode.
- Target motor-stop behavior is SITL-only and guarded by `MOTHDRONE_SITL_TARGET_KILL=1`.
