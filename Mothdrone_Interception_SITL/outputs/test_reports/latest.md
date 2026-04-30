# Mothdrone 40m VTOL SITL Report

Result: PASS for offboard takeoff, target mission cruise, hunter guidance, 25 m trigger, SITL target motor-stop/disarm, and hunter-only RTL.

Key telemetry:
- Spawn: hunter at 0 m, target at 40 m east.
- Takeoff: both vehicles armed, entered offboard, and climbed by offboard velocity setpoints.
- Takeoff gate: guidance started only after hunter reached 20.9 m and target reached 21.1 m.
- Start range: 40.0 m.
- Trigger range: 24.8 m.
- Trigger mode: `proximity_event_confirmed`.
- Vision gate: `vision_lock=true` at trigger.
- Target after trigger: SITL kill accepted, PX4 log shows target `Disarmed by external command`, telemetry target altitude reached 0.0 m.
- Hunter after trigger: held/climbed; altitude rose from 20.8 m to 59.5 m during recovery monitor.
- Hunter-only recovery: hunter RTL command sent after breakaway; target did not RTL after trigger.

Artifacts:
- Telemetry: `mothdrone_telemetry.json`
- Graph SVG: `outputs/graphs/mothdrone_mission_graph.svg`
- Graph HTML: `outputs/graphs/mothdrone_mission_graph.html`
- Controller: `code/mothdrone_controller.py`
- State machine: `docs/GUIDANCE_STATE_MACHINE.md`

Notes:
- This is SITL-only target motor-stop behavior, guarded by `MOTHDRONE_SITL_TARGET_KILL=1` in `launch_mothdrone.py`.
- The live target freefall is now coupled to MAVSDK `kill()`; the local physics model remains as deterministic event logging.
