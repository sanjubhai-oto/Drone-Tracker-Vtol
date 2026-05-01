#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

export MOTHDRONE_PX4_DIR="${MOTHDRONE_PX4_DIR:-$HOME/PX4-Autopilot}"
export PYTHONUNBUFFERED=1
export GZ_IP="${GZ_IP:-127.0.0.1}"
export MOTHDRONE_EXPLICIT_GZ="${MOTHDRONE_EXPLICIT_GZ:-1}"

echo "[BUNDLE] Mothdrone Linux SITL bundle"
echo "[BUNDLE] root: $ROOT_DIR"
echo "[BUNDLE] PX4:  $MOTHDRONE_PX4_DIR"
echo "[BUNDLE] explicit Gazebo startup: $MOTHDRONE_EXPLICIT_GZ"

bash scripts/cleanup_linux.sh
python3 scripts/check_linux_env.py

echo "[BUNDLE] installing/checking Python requirements..."
python3 -m pip install --user -r requirements.txt

PX4_GZ_DIR="$MOTHDRONE_PX4_DIR/Tools/simulation/gz"
PX4_BUILD_DIR="$MOTHDRONE_PX4_DIR/build/px4_sitl_default"
PX4_GZ_PLUGINS="$PX4_BUILD_DIR/src/modules/simulation/gz_plugins"
PX4_GZ_SERVER_CONFIG="$MOTHDRONE_PX4_DIR/src/modules/simulation/gz_bridge/server.config"
LOCAL_GZ_STORE="$HOME/.simulation-gazebo"
export GZ_SIM_RESOURCE_PATH="$LOCAL_GZ_STORE/models:$LOCAL_GZ_STORE/worlds:$PX4_GZ_DIR/models:$PX4_GZ_DIR/worlds:$ROOT_DIR/models:$ROOT_DIR/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_GZ_PLUGINS${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_SERVER_CONFIG_PATH="$PX4_GZ_SERVER_CONFIG"
export PX4_GZ_MODELS="$PX4_GZ_DIR/models"
export PX4_GZ_WORLDS="$PX4_GZ_DIR/worlds"
export PX4_GZ_PLUGINS="$PX4_GZ_PLUGINS"

echo "[BUNDLE] resource path: $GZ_SIM_RESOURCE_PATH"
echo "[BUNDLE] plugin path:   $GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "[BUNDLE] server config: $GZ_SIM_SERVER_CONFIG_PATH"
echo "[BUNDLE] launching PX4/Gazebo/MAVSDK mission..."
set +e
python3 launch_mothdrone.py
status=$?
set -e

if [[ "$status" -ne 0 ]]; then
  echo "[BUNDLE] launch failed with exit code $status"
  bash scripts/diagnose_linux_logs.sh || true
  exit "$status"
fi

echo "[BUNDLE] mission ended normally"
