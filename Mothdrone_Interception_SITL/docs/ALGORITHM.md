# Mothdrone VTOL Proximity Guidance Algorithm

This folder implements the full logic described in the supplied PDF: two VTOL airframes (hunter + target), GPS-based pursuit, vision-based final confirmation, and a proximity-triggered event that stops the target motors, lets it freefall, and disarms on ground impact.

## Mission Phases

1. Spawn two VTOL SITL vehicles.
   - Hunter at launch origin.
   - Target initially 40 m east of hunter.

2. Sync takeoff.
   - Both arm in SITL.
   - Both climb to 20 m.

3. GPS lead pursuit.
   - Hunter uses target GPS/local position to compute relative range and bearing.
   - Guidance uses lead-pursuit/PN-style speed selection.
   - Target cruises at ~2 m/s.
   - Hunter pursuit is capped at ~5 m/s.

4. Vision final confirmation.
   - When range drops near the event radius, hunter must also have visual lock.
   - Event does not trigger on GPS alone.

5. Proximity event (range <= 25 m).
   - Trigger condition: range <= 25 m, visual lock true, confidence >= 0.55, altitude safe.
   - **Target response:** motors stop → freefall under gravity → ground impact → disarm.
   - **Hunter response:** immediate breakaway to 40 m, then RTL and land.

6. Hunter breakaway and recovery.
   - Hunter first holds altitude while the target freefall simulation completes.
   - Hunter then climbs/pulls away toward 40 m.
   - Hunter executes RTL and lands.

## Control Inputs

- Hunter state: local position, velocity, yaw.
- Target state: local position, velocity, yaw.
- Vision confirmation: lock, confidence, bearing error.
- Mission config: event radius, speed limits, altitude targets.

## Control Outputs

- Hunter velocity setpoint.
- Hunter altitude target.
- Hunter yaw target.
- Event flag: `target_motor_stop_freefall`.

## Current Live Runner Defaults

- Start range: 40 m.
- Trigger radius: 25 m.
- Takeoff gate: both VTOLs must be above 20 m before any horizontal motion.
- Target speed: 2 m/s east.
- Hunter max guidance speed: 5 m/s.
- Hunter post-trigger behavior: hold altitude, climb, stop offboard, RTL, land.

## Target Freefall Physics

`TargetDrone` simulates:
- Motor stop: thrust cut to zero.
- Freefall: vertical acceleration = g (9.81 m/s²), horizontal velocity frozen.
- Ground impact: altitude clamps to 0, all velocities zeroed.
- Disarm: triggered on ground contact.

## Product Requirements Before Flight

- Replace SITL truth with real sensor fusion.
- Add geofence and minimum altitude constraints.
- Add independent manual override.
- Add explicit legal/safety review for any real-world C-UAS work.
- Never test forced motor stop on a real airborne vehicle.
