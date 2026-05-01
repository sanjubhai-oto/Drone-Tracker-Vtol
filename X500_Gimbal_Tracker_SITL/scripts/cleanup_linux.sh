#!/usr/bin/env bash
set -euo pipefail

echo "[CLEANUP] stopping old PX4/Gazebo/MAVSDK processes..."
pkill -f "mavsdk_server" 2>/dev/null || true
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*sitl" 2>/dev/null || true
sleep 2

echo "[CLEANUP] remaining sim processes:"
pgrep -af "mavsdk_server|px4_sitl_default/bin/px4|gz sim" || true

