#!/usr/bin/env bash
# Reproducible Femto Bolt deployment, distilled from running 8001 on 2026-10-26.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if docker info >/dev/null 2>&1; then D=(docker); else D=(sudo docker); fi
CAMERA_IMAGE="${CAMERA_IMAGE:-orbbec-femto-bolt:camera-local}"
SORTING_IMAGE="${SORTING_IMAGE:-orbbec-femto-bolt:sorting-local}"
CAMERA_CONTAINER="${CAMERA_CONTAINER:-femto-camera}"
SORTING_CONTAINER="${SORTING_CONTAINER:-femto-sorting}"
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
    DOMAIN="$ROS_DOMAIN_ID"
else
    DOMAIN="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$CAMERA_CONTAINER" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1 || true)"
    DOMAIN="${DOMAIN:-0}"
fi
ASSETS="${ASSETS_DIR:-$ROOT/assets}"
CONFIG="${CONFIG_DIR:-$ROOT/config}"
fail() { echo "ERROR: $*" >&2; exit 1; }
[[ "$DOMAIN" =~ ^[0-9]{1,3}$ ]] && ((10#$DOMAIN <= 232)) || fail 'ROS_DOMAIN_ID must be 0..232'
usb=(--network host --ipc host --device-cgroup-rule='c 189:* rmw' --mount type=bind,src=/dev/bus/usb,dst=/dev/bus/usb)
if [[ -d /dev/dri ]]; then usb+=(--device-cgroup-rule='c 226:* rmw' --mount type=bind,src=/dev/dri,dst=/dev/dri); fi
require_abs_dirs() {
    [[ "$ASSETS" = /* && -d "$ASSETS" && "$CONFIG" = /* && -d "$CONFIG" ]] || fail 'ASSETS_DIR and CONFIG_DIR must be existing absolute directories'
}
require_new_container() {
    ! "${D[@]}" container inspect "$1" >/dev/null 2>&1 || fail "Container $1 exists. Inspect it and explicitly stop/remove it before replacement."
}
check_runtime() {
    require_abs_dirs
    python3 tools/check_assets.py "$ASSETS"
    "${D[@]}" run --rm --network none --entrypoint /opt/femto/entrypoint.sh \
      -v "$ROOT/runtime:/app:ro" -v "$CONFIG:/config:ro" -v "$ASSETS:/assets:ro" -w /app \
      "$SORTING_IMAGE" python3 -c 'import numpy,cv2,open3d,shapely,numba,torch,ultralytics,rclpy,message_filters; assert hasattr(cv2,"wechat_qrcode"), "OpenCV WeChatQRCode missing"; cv2.wechat_qrcode.WeChatQRCode(); from std_srvs.srv import Trigger; from deployment_config import load_calibration; load_calibration(); import SFsorting; from predict_yolo import get_model; get_model(); print("PASS runtime imports, QR decoder, model load, calibration configuration")'
}
install_host() {
    if ! command -v docker >/dev/null; then
        sudo apt-get update
        sudo apt-get install -y docker.io
    fi
    sudo systemctl enable --now docker
    sudo apt-get install -y python3
    rule="$ROOT/src/orbbec_camera/udev/99-obsensor-libusb.rules"
    [[ -f "$rule" ]] || rule="$ROOT/src/orbbec_camera/scripts/99-obsensor-libusb.rules"
    [[ -f "$rule" ]] || fail 'Cannot find SDK udev rule; inspect src/orbbec_camera before continuing'
    sudo install -m 644 "$rule" /etc/udev/rules.d/99-obsensor-libusb.rules
    sudo udevadm control --reload-rules
    echo 'Docker and udev installed. Replug camera; next import a verified bundle or build images.'
}
import_bundle() {
    local bundle="${1:?Usage: bash deploy.sh import /absolute/path/to/private-bundle}"
    [[ "$bundle" = /* ]] || fail 'Bundle path must be absolute'
    [[ ! -e "$ROOT/assets" ]] || fail 'assets already exists; leave existing assets intact and import to a clean checkout'
    (cd "$bundle" && sha256sum -c SHA256SUMS)
    [[ -f "$bundle/images.tar.gz" ]] || fail 'Bundle has no images; export with --images or use build (fresh build unverified)'
    gzip -dc "$bundle/images.tar.gz" | "${D[@]}" load
    local camera_id sorting_id
    camera_id=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["camera"])' "$bundle/image-ids.json")
    sorting_id=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["sorting"])' "$bundle/image-ids.json")
    "${D[@]}" tag "$camera_id" "$CAMERA_IMAGE"
    "${D[@]}" tag "$sorting_id" "$SORTING_IMAGE"
    cp -a "$bundle/assets" "$ROOT/assets"
    echo 'Images and assets imported. Create this device calibration before server start.'
}
offline_check() {
    python3 tools/check_assets.py "$ASSETS"
    "${D[@]}" run --rm --network none --entrypoint /opt/femto/entrypoint.sh \
      -v "$ROOT/runtime:/app:ro" -v "$ASSETS:/assets:ro" \
      -e NUMBA_CACHE_DIR=/tmp/numba -w /app "$SORTING_IMAGE" \
      python3 -c 'import cv2,numpy,open3d,shapely,numba,torch,ultralytics,rclpy,message_filters; from std_srvs.srv import Trigger; import SFsorting; from predict_yolo import get_model; m=get_model(); print("PASS offline imports and model",torch.__version__,m.names)'
    echo 'Offline environment ready. Camera and this workcell calibration still require onsite acceptance.'
}
case "${1:-help}" in
    install) install_host ;;
    import) import_bundle "${2:?Usage: bash deploy.sh import /absolute/path/to/private-bundle}" ;;
    offline-check) offline_check ;;
    prepare)
        install_host
        import_bundle "${2:?Usage: bash deploy.sh prepare /absolute/path/to/private-bundle}"
        [[ -e "$CONFIG/calibration.json" ]] || cp "$CONFIG/calibration.example.json" "$CONFIG/calibration.json"
        offline_check
        ;;
    build)
        [[ "$(uname -m)" == x86_64 ]] || fail 'Sorting baseline validated only on x86_64'
        "${D[@]}" build --build-arg BUILD_JOBS="${BUILD_JOBS:-2}" -t "$CAMERA_IMAGE" .
        "${D[@]}" build --build-arg CAMERA_IMAGE="$CAMERA_IMAGE" -f Dockerfile.sorting -t "$SORTING_IMAGE" .
        ;;
    devices)
        "${D[@]}" run --rm "${usb[@]}" "$CAMERA_IMAGE" ros2 run orbbec_camera list_devices_node
        ;;
    start)
        serial="${2:-}"
        [[ "$serial" =~ ^[A-Za-z0-9_-]+$ ]] || fail 'Usage: bash deploy.sh start CAMERA_SERIAL'
        require_new_container "$CAMERA_CONTAINER"
        "${D[@]}" run -d --name "$CAMERA_CONTAINER" --restart unless-stopped \
          --log-opt max-size=10m --log-opt max-file=3 "${usb[@]}" \
          -e CAMERA_SERIAL="$serial" -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY=0 \
          -v "$ROOT/camera.launch.py:/opt/femto/camera.launch.py:ro" \
          -v "$ROOT/verify_topics.py:/opt/femto/verify_topics.py:ro" \
          "$CAMERA_IMAGE" camera
        ;;
    verify)
        "${D[@]}" exec "$CAMERA_CONTAINER" /opt/femto/entrypoint.sh python3 /opt/femto/verify_topics.py
        ;;
    check) check_runtime ;;
    server)
        case "${2:-status}" in
            start)
                require_new_container "$SORTING_CONTAINER"
                diagnostic=false
                server_config="$CONFIG"
                server_restart=unless-stopped
                extra_env=()
                if [[ "${3:-}" == --diagnostic ]]; then
                    diagnostic=true
                    server_restart=unless-stopped
                    server_config="$ROOT/private/server-diagnostic/config"
                    capture_dir="$ROOT/private/server-diagnostic/capture"
                    mkdir -p "$capture_dir" "$server_config"
                    "${D[@]}" ps --format '{{.Names}}' | grep -qx "$CAMERA_CONTAINER" || fail 'Camera container must be running for diagnostic start'
                    "${D[@]}" run --rm --network host --ipc host -e "ROS_DOMAIN_ID=$DOMAIN" -e ROS_LOCALHOST_ONLY=0 \
                      -v "$ROOT/tools/capture_frame.py:/tmp/capture_frame.py:ro" -v "$capture_dir:/capture" \
                      "$SORTING_IMAGE" python3 /tmp/capture_frame.py \
                      --output /capture/rgb.jpg --info /capture/camera-info.json > "$ROOT/private/server-diagnostic/capture.log"
                    python3 tools/make_test_calibration.py "$capture_dir/camera-info.json" "$server_config/calibration.json"
                    extra_env=(-e MAX_FRAME_AGE_SEC=10 -e CLOUD_STRIDE=8 -e SYNC_SLOP_SEC=0.5 -e DIAGNOSTIC_ONLY=1)
                else
                    check_runtime
                fi
                "${D[@]}" run -d --name "$SORTING_CONTAINER" --restart "$server_restart" \
                  --log-opt max-size=10m --log-opt max-file=3 --network host --ipc host \
                  -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY=0 \
                  -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=2 "${extra_env[@]}" \
                  -v "$ROOT/runtime:/app:ro" -v "$server_config:/config:ro" -v "$ASSETS:/assets:ro" \
                  -v femto-numba-cache:/tmp/numba -e NUMBA_CACHE_DIR=/tmp/numba \
                  -w /app "$SORTING_IMAGE" python3 SFsorting.py
                if "$diagnostic"; then
                    echo 'DIAGNOSTIC ONLY: camera optical coordinates in mm; old mask is unverified. Do not send results to the robot.'
                fi
                echo 'Container created; wait for "live service ready", then use server test or server client.'
                ;;
            test)
                if "${D[@]}" inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$SORTING_CONTAINER" 2>/dev/null | grep -qx DIAGNOSTIC_ONLY=1; then
                    echo 'DIAGNOSTIC ONLY: coordinates below are camera optical mm, not robot/world coordinates.'
                fi
                "${D[@]}" run --rm --network host --ipc host -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY=0 \
                  -v "$ROOT/test_sorting_client.py:/tmp/test_sorting_client.py:ro" \
                  "$SORTING_IMAGE" python3 /tmp/test_sorting_client.py --timeout 15 --call-timeout 120
                ;;
            client)
                "${D[@]}" run --rm --network host --ipc host -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY=0 \
                  -v "$ROOT/test_sorting_client.py:/tmp/test_sorting_client.py:ro" \
                  "$SORTING_IMAGE" python3 /tmp/test_sorting_client.py --json --timeout 15 --call-timeout 120
                ;;
            logs) "${D[@]}" logs --tail 100 -f "$SORTING_CONTAINER" ;;
            status) "${D[@]}" ps -a --filter "name=^/${SORTING_CONTAINER}$" ;;
            stop) "${D[@]}" stop "$SORTING_CONTAINER" ;;
            resume) "${D[@]}" start "$SORTING_CONTAINER" ;;
            remove) "${D[@]}" rm "$SORTING_CONTAINER" ;;
            *) fail 'server {start [--diagnostic]|client|test|logs|status|stop|resume|remove}' ;;
        esac
        ;;
    status) "${D[@]}" ps -a --filter "name=^/${CAMERA_CONTAINER}$" ;;
    logs) "${D[@]}" logs --tail 100 -f "$CAMERA_CONTAINER" ;;
    stop) "${D[@]}" stop "$CAMERA_CONTAINER" ;;
    resume) "${D[@]}" start "$CAMERA_CONTAINER" ;;
    remove) "${D[@]}" rm "$CAMERA_CONTAINER" ;;
    *) printf '%s\n' 'deploy.sh prepare ABS_BUNDLE | install | import ABS_BUNDLE | offline-check | build' 'deploy.sh devices | start SERIAL | verify | check | status|logs|stop|resume|remove' 'deploy.sh server start [--diagnostic]|client|test|status|logs|stop|resume|remove' ;;
esac
