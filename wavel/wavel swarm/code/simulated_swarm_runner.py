#!/usr/bin/env python3
"""Run a safe Wavel swarm object-inspection simulation."""

from __future__ import annotations

import json
from pathlib import Path

from swarm_guidance import DroneState, SwarmConfig, SwarmController, VisionObjectGate


ROOT = Path(__file__).resolve().parents[1]
TELEMETRY = ROOT / "wavel_swarm_telemetry.json"


def initial_states(count: int) -> list[DroneState]:
    spacing = 8.0
    return [
        DroneState(
            drone_id=i + 1,
            north_m=0.0,
            east_m=(i - (count - 1) / 2.0) * spacing,
            altitude_m=0.0,
        )
        for i in range(count)
    ]


def run_simulation(config: SwarmConfig | None = None, steps: int = 460) -> list[dict]:
    cfg = config or SwarmConfig()
    controller = SwarmController(cfg)
    vision = VisionObjectGate(cfg)
    states = initial_states(cfg.drone_count)
    controller.arm_all(states)
    rows: list[dict] = []

    for step in range(steps):
        links = controller.mesh_links(states)
        detections = {state.drone_id: vision.detect(state) for state in states}
        for state in states:
            det = detections[state.drone_id]
            if det.has_lock:
                state.object_confirmed = True
                state.mode = "object_confirmed_hold"

        rows.append(
            {
                "time_s": round(step * cfg.control_dt_s, 3),
                "step": step,
                "min_separation_m": round(controller.min_pair_distance(states), 3),
                "mesh_links": {str(k): v for k, v in links.items()},
                "confirmed_count": sum(1 for s in states if s.object_confirmed),
                "target": {
                    "north_m": cfg.target_north_m,
                    "east_m": cfg.target_east_m,
                    "altitude_m": cfg.target_altitude_m,
                    "label": "building/object",
                },
                "drones": [
                    {
                        **state.to_dict(),
                        "detection": {
                            "has_lock": detections[state.drone_id].has_lock,
                            "confidence": round(detections[state.drone_id].confidence, 3),
                            "range_m": round(detections[state.drone_id].range_m, 3),
                            "bearing_error_deg": round(detections[state.drone_id].bearing_error_rad * 57.2957795, 2),
                            "label": detections[state.drone_id].label,
                        },
                    }
                    for state in states
                ],
            }
        )

        if rows[-1]["confirmed_count"] == cfg.drone_count:
            break
        states = controller.step(states)

    TELEMETRY.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    return rows


def main() -> None:
    rows = run_simulation()
    final = rows[-1]
    print("WAVEL SWARM SIMULATION COMPLETE")
    print(f"steps={len(rows)}")
    print(f"confirmed={final['confirmed_count']}/{len(final['drones'])}")
    print(f"min_separation_m={min(row['min_separation_m'] for row in rows):.2f}")
    print(f"telemetry={TELEMETRY}")


if __name__ == "__main__":
    main()

