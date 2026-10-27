#!/usr/bin/env bash
# Capture one synchronized RGB/depth/point-cloud group from the running camera.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT="$ROOT/captured_frame"
CAMERA="${CAMERA_CONTAINER:-femto-camera}"
if docker info >/dev/null 2>&1; then D=(docker); else D=(sudo docker); fi
"${D[@]}" ps --format '{{.Names}}' | grep -qx "$CAMERA" || {
  echo "ERROR: camera container $CAMERA is not running" >&2; exit 1;
}
DOMAIN="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$CAMERA" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1 || true)"
DOMAIN="${ROS_DOMAIN_ID:-${DOMAIN:-0}}"
mkdir -p "$OUT/rgb" "$OUT/depth" "$OUT/point"
index=1
while :; do
  name="$(printf '%06d' "$index")"
  [[ ! -e "$OUT/rgb/$name.png" && ! -e "$OUT/depth/$name.png" && ! -e "$OUT/point/$name.ply" ]] && break
  ((index+=1))
done
echo "Capturing synchronized RGB, depth and point cloud from $CAMERA (ROS_DOMAIN_ID=$DOMAIN)..."
"${D[@]}" run --rm --network host --ipc host \
  -e "CAPTURE_UID=$(id -u)" -e "CAPTURE_GID=$(id -g)" \
  -e "ROS_DOMAIN_ID=$DOMAIN" -e ROS_LOCALHOST_ONLY=0 \
  -v "$ROOT/save_dataset_single.py:/tmp/save_dataset_single.py:ro" \
  -v "$OUT:/capture" \
  orbbec-femto-bolt:sorting-local python3 /tmp/save_dataset_single.py /capture "$name"
for path in "$OUT/rgb/$name.png" "$OUT/depth/$name.png" "$OUT/point/$name.ply"; do
  [[ -s "$path" ]] || { echo "ERROR: missing or empty $path" >&2; exit 1; }
done
echo "SUCCESS: $OUT/{rgb,depth,point}/$name (PNG, PNG, PLY)"
