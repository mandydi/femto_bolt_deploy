#!/usr/bin/env bash
# Capture one full-resolution still from the auxiliary Fic760x USB camera.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CAMERA="/dev/v4l/by-id/usb-Image+_Fic760xUsbCamera_SR362-1234-video-index0"
OUT_DIR="$ROOT/captured_usb_camera"

[[ -e "$CAMERA" ]] || { echo "ERROR: USB 相机未连接：$CAMERA" >&2; exit 1; }
command -v gst-launch-1.0 >/dev/null || { echo 'ERROR: 缺少 gst-launch-1.0' >&2; exit 1; }

if pgrep -x cheese >/dev/null 2>&1; then
    echo '关闭 Cheese，释放 USB 相机...'
    pkill -TERM -x cheese
    sleep 2
fi

mkdir -p "$OUT_DIR"
TMP_DIR="$(mktemp -d "$OUT_DIR/.capture.XXXXXXXX")"
trap 'rm -rf -- "$TMP_DIR"' EXIT

echo '采集 USB 相机最大分辨率 4000x3000（MJPEG），保存最后一帧...'
timeout 40 gst-launch-1.0 -q \
    v4l2src device="$CAMERA" num-buffers=5 \
    '!' 'image/jpeg,width=4000,height=3000,framerate=15/1' \
    '!' multifilesink location="$TMP_DIR/frame-%02d.jpg"

frames=("$TMP_DIR"/frame-*.jpg)
[[ -f "${frames[-1]}" ]] || { echo 'ERROR: 相机没有输出 JPEG 帧' >&2; exit 1; }
OUTPUT="$OUT_DIR/usb_$(date +%Y%m%d_%H%M%S).jpg"
python3 - "${frames[-1]}" <<'PY'
from PIL import Image
from pathlib import Path
import sys
p = Path(sys.argv[1])
with Image.open(p) as im:
    im.load()
    assert im.format == 'JPEG' and im.size == (4000, 3000), (im.format, im.size)
PY
mv -- "${frames[-1]}" "$OUTPUT"
echo "SUCCESS: $OUTPUT (4000x3000 JPEG, $(stat -c %s "$OUTPUT") bytes)"
