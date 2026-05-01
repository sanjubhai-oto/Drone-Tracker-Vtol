# X500 Gimbal Tracker 100m High-Speed Both-Offboard X500 gimbal SITL Report

Result: PASS for GUI launch, target-first arm sync, both-offboard climb, moving target offboard path, hunter PN lead-pursuit guidance from 100 m, 25 m trigger gate, SITL target kill/fall/disarm, and hunter-only recovery/RTL.

Key telemetry:
- Spawn: hunter at 0 m, target at 100 m east.
- Arm sync: target armed first; hunter armed only after target `armed=true` was detected.
- Takeoff: both vehicles entered offboard and climbed by offboard velocity setpoints.
- Takeoff gate: guidance started after hunter reached 20.9 m and target reached 20.9 m.
- Target path: target stayed in offboard and moved from E=99.7 m to E=186.4 m, N=0.0 m to N=5.6 m before trigger.
- Hunter path: hunter moved from E=0.1 m to E=161.5 m, N=0.2 m to N=5.0 m before trigger.
- Start range: 99.6 m.
- Trigger range: 24.9 m at loop 85.
- Trigger state: `proximity_event_confirmed`.
- Vision gate: `vision_lock=true` at trigger.
- Hunter speed command: ramped to 13.0 m/s peak, then slowed to 5.5 m/s near the trigger radius.
- Closing velocity: peaked at 9.4 m/s and settled to 1.6 m/s at trigger.
- Target speed setting: 4 m/s offboard path.
- Target after trigger: SITL kill accepted, target altitude logged at 0.0 m.
- Hunter after trigger: held/climbed; altitude rose from 20.7 m at trigger to 59.6 m during recovery monitor.
- Hunter-only recovery: hunter RTL command sent after breakaway.

Artifacts:
- Telemetry: `x500_gimbal_telemetry.json`
- 3D trajectory SVG: `outputs/graphs/x500_gimbal_trajectory_3d.svg`
- Mission graph SVG: `outputs/graphs/x500_gimbal_mission_graph.svg`
- Mission graph HTML: `outputs/graphs/x500_gimbal_mission_graph.html`
- Controller: `code/x500_gimbal_controller.py`
- State machine: `docs/GUIDANCE_STATE_MACHINE.md`

Notes:
- This run uses both-offboard control, not PX4 target mission mode.
- Target motor-stop behavior is SITL-only and guarded by `X500_GIMBAL_SITL_TARGET_KILL=1`.
- No `Accel #0 fail: TIMEOUT` was seen in the PX4 logs for this run.
- PX4 did print startup EKF/airspeed/heading arming warnings and a non-blocking `COM_DL_LOSS_T` type mismatch warning.
