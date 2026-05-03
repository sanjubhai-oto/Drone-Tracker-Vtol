#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

python3 code/simulated_swarm_runner.py
python3 code/plot_swarm_results.py

echo "[WAVEL] telemetry: $ROOT_DIR/wavel_swarm_telemetry.json"
echo "[WAVEL] graph:     $ROOT_DIR/outputs/graphs/wavel_swarm_path.svg"
echo "[WAVEL] live UI:   python3 code/live_swarm_server.py"

