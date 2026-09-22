#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$SCRIPT_DIR"

IMAGE=orbbec-femto-bolt:humble
CONTAINER=orbbec-femto-bolt

fail() { echo "ERROR: $*" >&2; exit 1; }
docker_cmd() { sudo docker "$@"; }

find_dir() {
    local name="$1"
    for p in "$SCRIPT_DIR/../$name" "$SCRIPT_DIR/$name" "/home/caseim/vision/$name" "/home/caseim/$name"; do
        if [[ -d "$p" ]]; then
            (cd "$p" && pwd)
            return 0
        fi
    done
    return 1
}

YOLO_DIR="$(find_dir yolo26_packet || echo "/home/caseim/vision/yolo26_packet")"
SF_DIR="$(find_dir SF_sorting0916 || echo "/home/caseim/vision/SF_sorting0916")"

usb_args=(--network host --ipc host --device-cgroup-rule='c 189:* rmw'
          --mount type=bind,src=/dev/bus/usb,dst=/dev/bus/usb)
if [[ -d /dev/dri ]]; then
    usb_args+=(--device-cgroup-rule='c 226:* rmw'
               --mount type=bind,src=/dev/dri,dst=/dev/dri)
fi

get_tty_args() {
    if [[ -t 0 && -t 1 ]]; then
        echo "-it"
    else
        echo "-i"
    fi
}

case "${1:-help}" in
    devices)
        docker_cmd run --rm "${usb_args[@]}" "$IMAGE" ros2 run orbbec_camera list_devices_node
        ;;
    start)
        serial="${2:-}"
        domain="${3:-0}"
        [[ "$serial" =~ ^[A-Za-z0-9_-]+$ ]] || fail "Usage: bash deploy.sh start SERIAL [ROS_DOMAIN_ID]"
        [[ "$domain" =~ ^[0-9]{1,3}$ ]] && ((10#$domain <= 232)) || fail "ROS_DOMAIN_ID must be 0..232."
        domain=$((10#$domain))
        docker_cmd image inspect "$IMAGE" >/dev/null

        if docker_cmd container inspect "$CONTAINER" >/dev/null 2>&1; then
            docker_cmd rm -f "$CONTAINER" >/dev/null 2>&1 || true
        fi

        mount_args=(
            -v "$SCRIPT_DIR/camera.launch.py:/opt/femto/camera.launch.py:ro"
            -v "$SCRIPT_DIR/verify_topics.py:/opt/femto/verify_topics.py:ro"
        )
        if [[ -d "$YOLO_DIR" ]]; then
            mount_args+=(-v "$YOLO_DIR:/home/caseim/yolo26_packet")
        fi
        if [[ -d "$SF_DIR" ]]; then
            mount_args+=(-v "$SF_DIR:/home/caseim/SF_sorting0916")
        fi

        docker_cmd run -d --name "$CONTAINER" --restart unless-stopped \
            --log-opt max-size=10m --log-opt max-file=3 \
            "${mount_args[@]}" \
            "${usb_args[@]}" -e "CAMERA_SERIAL=$serial" -e "ROS_DOMAIN_ID=$domain" "$IMAGE" camera
        echo "Autostart configured for camera $serial. Verify actual data: bash deploy.sh verify"
        ;;
    verify)
        docker_cmd exec "$CONTAINER" /opt/femto/entrypoint.sh python3 /opt/femto/verify_topics.py
        ;;
    detect)
        docker_cmd exec "$CONTAINER" /opt/femto/entrypoint.sh python3 /home/caseim/yolo26_packet/test_yolo_detection.py "${@:2}"
        ;;
    sorting)
        if [[ -d "$SF_DIR" ]]; then
            tty_arg=$(get_tty_args)
            docker_cmd run --rm $tty_arg "${usb_args[@]}" \
                -v "$SF_DIR:/home/caseim/SF_sorting0916" \
                -w /home/caseim/SF_sorting0916 \
                "$IMAGE" /opt/femto/entrypoint.sh python3 SFsorting.py "${@:2}"
        else
            fail "Directory SF_sorting0916 not found."
        fi
        ;;
    bash)
        mount_args=()
        if [[ -d "$YOLO_DIR" ]]; then mount_args+=(-v "$YOLO_DIR:/home/caseim/yolo26_packet"); fi
        if [[ -d "$SF_DIR" ]]; then mount_args+=(-v "$SF_DIR:/home/caseim/SF_sorting0916"); fi
        tty_arg=$(get_tty_args)
        docker_cmd run --rm $tty_arg "${usb_args[@]}" "${mount_args[@]}" -w /home/caseim "$IMAGE" bash
        ;;
    rviz)
        if [[ -f "$SCRIPT_DIR/run_rviz.sh" ]]; then
            bash "$SCRIPT_DIR/run_rviz.sh" "${@:2}"
        elif [[ -f "/home/caseim/run_rviz.sh" ]]; then
            bash "/home/caseim/run_rviz.sh" "${@:2}"
        else
            fail "run_rviz.sh not found."
        fi
        ;;
    topics)
        docker_cmd exec "$CONTAINER" /opt/femto/entrypoint.sh ros2 topic list
        ;;
    logs)
        docker_cmd logs --tail 100 -f "$CONTAINER"
        ;;
    status)
        docker_cmd ps -a --filter "name=^/${CONTAINER}$"
        ;;
    stop)
        docker_cmd stop --time 30 "$CONTAINER"
        ;;
    resume)
        docker_cmd start "$CONTAINER"
        ;;
    remove)
        docker_cmd rm -f "$CONTAINER"
        ;;
    *)
        echo "Usage: bash deploy.sh {devices|start SERIAL [DOMAIN]|verify|detect|sorting|bash|rviz|topics|logs|status|stop|resume|remove}"
        ;;
esac
