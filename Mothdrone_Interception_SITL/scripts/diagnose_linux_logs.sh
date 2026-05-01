#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "[DIAG] PX4 preflight/sensor errors:"
grep -RniE "Preflight Fail|sensor .*missing|Accelerometer|Gyro|Barometer|No valid|COMMAND_DENIED|arming|denied|timeout|ERROR|WARN" logs/*.log logs/*.err.log 2>/dev/null || true

echo
echo "[DIAG] Running sim processes:"
pgrep -af "mavsdk_server|px4_sitl_default/bin/px4|gz sim" || true

echo
echo "[DIAG] Gazebo server/client log tail:"
tail -80 logs/gazebo.err.log 2>/dev/null || true
tail -80 logs/gazebo.log 2>/dev/null || true

echo
echo "[DIAG] If QGC shows accel/gyro/baro missing, PX4 is running without Gazebo sensor bridge."
echo "[DIAG] Usually fix: close QGC optional, run scripts/cleanup_linux.sh, verify PX4 was built with Gazebo, then run scripts/run_linux_bundle.sh."
echo "[DIAG] The Linux bundle starts Gazebo explicitly first (MOTHDRONE_EXPLICIT_GZ=1), then starts both PX4 instances in standalone mode."
echo "[DIAG] It also exports GZ_SIM_SYSTEM_PLUGIN_PATH to PX4's build gz_plugins directory and GZ_SIM_SERVER_CONFIG_PATH to PX4's gz_bridge server.config."
