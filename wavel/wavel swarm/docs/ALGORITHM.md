# Wavel Swarm Algorithm

This package is a safe swarm-navigation and object-inspection simulation.

## Behavior

- 4 to 5 Wavel drones arm together.
- Each vehicle joins a local mesh graph using range-limited communication.
- The swarm travels from point A to point B in a leader-relative formation.
- Pairwise repulsion keeps drones separated.
- Each drone keeps altitude and yaw toward its commanded path.
- A simulated OpenCV-style object gate confirms the building/object near point B.
- The final event is a non-destructive object confirmation/tag, not impact.

## Control Loop

1. Compute each drone formation slot around point B.
2. Generate velocity toward the assigned slot.
3. Add collision-avoidance velocity when another drone is inside `min_separation_m`.
4. Clamp speed to `max_speed_mps`.
5. Update mesh links based on `comms_range_m`.
6. Run the vision gate and mark object confirmation when range/FOV/confidence pass.

## Vision Backend

The included backend is deterministic and simulated. It has the same interface a real OpenCV/RF-DETR/YOLO detector would provide:

- `has_lock`
- `confidence`
- `bearing_error_rad`
- `range_m`
- `label`

RF-DETR Plus can be evaluated later as a detector for benign object confirmation or inspection. It is not wired to weaponize navigation.

