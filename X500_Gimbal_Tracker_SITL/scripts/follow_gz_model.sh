#!/usr/bin/env bash
set -euo pipefail

MODEL="${1:-x500_gimbal_0}"
MODE="${2:-follow}"

if [[ "$MODE" == "off" ]]; then
  gz topic -t /gui/track -m gz.msgs.CameraTrack -p "track_mode: NONE"
  echo "[GZ] camera tracking disabled"
  exit 0
fi

gz topic -t /gui/track -m gz.msgs.CameraTrack \
  -p "track_mode: FOLLOW, follow_target: {name: '$MODEL'}, follow_offset: {x: -35, y: -35, z: 28}, follow_pgain: 0.6, track_pgain: 0.6"

echo "[GZ] following model: $MODEL"
echo "[GZ] examples:"
echo "  ./scripts/follow_gz_model.sh x500_gimbal_0"
echo "  ./scripts/follow_gz_model.sh x500_gimbal_1"
echo "  ./scripts/follow_gz_model.sh any off"

