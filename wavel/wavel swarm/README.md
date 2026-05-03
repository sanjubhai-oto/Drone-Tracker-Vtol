# Wavel Swarm

Safe multi-Wavel swarm simulation for dynamic following, mesh communication, collision avoidance, and vision-based object confirmation.

This package intentionally implements a non-destructive `object_confirmed` event instead of strike/impact behavior.

## Run

```bash
cd "wavel/wavel swarm"
chmod +x scripts/*.sh
./scripts/run_simulated_swarm.sh
```

Live UI:

```bash
python3 code/live_swarm_server.py
```

Open:

```text
http://127.0.0.1:8795/live_swarm.html
```

## Outputs

- `wavel_swarm_telemetry.json`
- `outputs/graphs/wavel_swarm_path.svg`
- `outputs/graphs/wavel_swarm_path.html`

## Main Files

| File | Purpose |
| --- | --- |
| `code/swarm_guidance.py` | Formation, mesh links, collision avoidance, object gate |
| `code/simulated_swarm_runner.py` | Runs 5-drone swarm simulation |
| `code/live_swarm_server.py` | Browser live trajectory and camera panels |
| `code/plot_swarm_results.py` | Generates SVG/HTML graph |
| `configs/swarm_config.json` | Tunable swarm parameters |
| `docs/ALGORITHM.md` | Control logic explanation |
| `docs/SAFETY_BOUNDARY.md` | Explicit non-weaponization boundary |

## Vision Model

The current detector is a simulated OpenCV-style gate. A real RF-DETR/YOLO/OpenCV backend can be connected later for benign object inspection and confirmation.

