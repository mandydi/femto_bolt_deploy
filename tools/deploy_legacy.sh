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

run_sorting_client() {
    local mode="$1"
    shift || true
    local client_py=""
    if [[ -f "$SCRIPT_DIR/test_sorting_client.py" ]]; then
        client_py="$SCRIPT_DIR/test_sorting_client.py"
    elif [[ -f "$SF_DIR/test_sorting_client.py" ]]; then
        client_py="$SF_DIR/test_sorting_client.py"
    else
        fail "test_sorting_client.py 未找到 (路径: $SCRIPT_DIR 或 $SF_DIR)"
    fi

    if [[ "$mode" == "client" ]]; then
        # 实际工程生产调用: 纯净输出标准 JSON，无任何多余控制字符
        docker_cmd run --rm -i --network host --ipc host -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
            -v "$client_py:/tmp/test_sorting_client.py:ro" \
            "$IMAGE" /opt/femto/entrypoint.sh python3 /tmp/test_sorting_client.py --json "$@"
    else
        # 测试与性能诊断: 打印完整的端到端耗时分解 (RTT, YOLO, 3D拟合, 等待帧) 与目标坐标
        local tty_arg
        tty_arg=$(get_tty_args)
        docker_cmd run --rm $tty_arg --network host --ipc host -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
            -v "$client_py:/tmp/test_sorting_client.py:ro" \
            "$IMAGE" /opt/femto/entrypoint.sh python3 /tmp/test_sorting_client.py "$@"
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
    location)
        [[ -f "$SCRIPT_DIR/aruco_location.py" ]] || fail "Missing aruco_location.py"
        [[ -f "$SCRIPT_DIR/aruco_reference.json" ]] || fail "Missing aruco_reference.json"
        docker_cmd cp "$SCRIPT_DIR/aruco_location.py" "$CONTAINER:/tmp/aruco_location.py"
        docker_cmd cp "$SCRIPT_DIR/aruco_reference.json" "$CONTAINER:/tmp/aruco_reference.json"
        docker_cmd exec "$CONTAINER" /opt/femto/entrypoint.sh python3 -u /tmp/aruco_location.py \
            --reference /tmp/aruco_reference.json "${@:2}"
        ;;
    sorting)
        if [[ -d "$SF_DIR" ]]; then
            mount_args=()
            if [[ -d "$YOLO_DIR" ]]; then mount_args+=(-v "$YOLO_DIR:/home/caseim/yolo26_packet"); fi
            if [[ -d "$SF_DIR" ]]; then mount_args+=(-v "$SF_DIR:/home/caseim/SF_sorting0916"); fi
            tty_arg=$(get_tty_args)
            docker_cmd run --rm $tty_arg "${usb_args[@]}" "${mount_args[@]}" \
                -w /home/caseim/SF_sorting0916 \
                "$IMAGE" /opt/femto/entrypoint.sh python3 SFsorting.py "${@:2}"
        else
            fail "Directory SF_sorting0916 not found."
        fi
        ;;
    server|sorting_server|sorting-server)
        action="${2:-start}"
        srv_container="sf_sorting_server"
        case "$action" in
            start)
                if docker_cmd container inspect "$srv_container" >/dev/null 2>&1; then
                    docker_cmd rm -f "$srv_container" >/dev/null 2>&1 || true
                fi
                mount_args=()
                if [[ -d "$YOLO_DIR" ]]; then mount_args+=(-v "$YOLO_DIR:/home/caseim/yolo26_packet"); fi
                if [[ -d "$SF_DIR" ]]; then mount_args+=(-v "$SF_DIR:/home/caseim/SF_sorting0916"); fi
                docker_cmd run -d --name "$srv_container" --restart unless-stopped \
                    --log-opt max-size=10m --log-opt max-file=3 \
                    "${usb_args[@]}" "${mount_args[@]}" \
                    -w /home/caseim/SF_sorting0916 \
                    "$IMAGE" /opt/femto/entrypoint.sh python3 SFsorting.py --mode service "${@:3}"
                echo "=========================================================="
                echo "🚀 顺丰分拣常驻服务 (sf_sorting_server) 已启动成功！"
                echo "=========================================================="
                echo "  • 查看实时日志: bash deploy.sh server logs"
                echo "  • 测试性能诊断: bash deploy.sh server test"
                echo "  • 生产工程调用: bash deploy.sh server client"
                echo "  • 查看运行状态: bash deploy.sh server status"
                echo "  • 停止常驻服务: bash deploy.sh server stop"
                echo "=========================================================="
                ;;
            stop)
                docker_cmd rm -f "$srv_container" >/dev/null 2>&1 || true
                echo "顺丰分拣服务已停止。"
                ;;
            status)
                docker_cmd ps -a --filter "name=^/${srv_container}$"
                ;;
            logs)
                docker_cmd logs --tail 100 -f "$srv_container"
                ;;
            client)
                run_sorting_client client "${@:3}"
                ;;
            test)
                run_sorting_client test "${@:3}"
                ;;
            run)
                tty_arg=$(get_tty_args)
                mount_args=()
                if [[ -d "$YOLO_DIR" ]]; then mount_args+=(-v "$YOLO_DIR:/home/caseim/yolo26_packet"); fi
                if [[ -d "$SF_DIR" ]]; then mount_args+=(-v "$SF_DIR:/home/caseim/SF_sorting0916"); fi
                docker_cmd run --rm $tty_arg "${usb_args[@]}" "${mount_args[@]}" \
                    -w /home/caseim/SF_sorting0916 \
                    "$IMAGE" /opt/femto/entrypoint.sh python3 SFsorting.py --mode service "${@:3}"
                ;;
            *)
                echo "Usage: bash deploy.sh server {start|stop|status|logs|client|test|run}"
                ;;
        esac
        ;;
    client)
        run_sorting_client client "${@:2}"
        ;;
    test)
        run_sorting_client test "${@:2}"
        ;;
    server-start)
        "$0" server start "${@:2}"
        ;;
    server-stop)
        "$0" server stop "${@:2}"
        ;;
    server-status)
        "$0" server status "${@:2}"
        ;;
    server-logs)
        "$0" server logs "${@:2}"
        ;;
    bash)
        mount_args=()
        if [[ -d "$YOLO_DIR" ]]; then mount_args+=(-v "$YOLO_DIR:/home/caseim/yolo26_packet"); fi
        if [[ -d "$SF_DIR" ]]; then mount_args+=(-v "$SF_DIR:/home/caseim/SF_sorting0916"); fi
        docker_cmd run --rm -it "${usb_args[@]}" "${mount_args[@]}" -w /home/caseim/SF_sorting0916 "$IMAGE" /opt/femto/entrypoint.sh bash
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
        echo "================================================================================"
        echo "顺丰视觉系统统一部署与调用工具 (deploy.sh)"
        echo "================================================================================"
        echo "【一、相机驱动管理】"
        echo "  bash deploy.sh devices                      - 列出连接的 Femto Bolt 相机与序列号"
        echo "  bash deploy.sh start <SERIAL> [DOMAIN]     - 启动相机驱动常驻服务"
        echo "  bash deploy.sh verify                       - 验证相机彩色、深度与点云话题"
        echo "  bash deploy.sh status / logs / stop         - 相机服务 状态 / 实时日志 / 停止"
        echo ""
        echo "【二、分拣算法常驻服务 (Server)】"
        echo "  bash deploy.sh server start                 - 启动分拣常驻计算服务 (sf_sorting_server)"
        echo "  bash deploy.sh server stop                  - 停止分拣常驻计算服务"
        echo "  bash deploy.sh server status                - 查看分拣常驻服务容器状态"
        echo "  bash deploy.sh server logs                  - 查看分拣常驻服务日志 (实时输出)"
        echo ""
        echo "【三、实际工程生产调用 (Production Workflow)】"
        echo "  bash deploy.sh server client                - 生产模式：输出标准 JSON 结果供 PLC/主控程序解析"
        echo "  (快捷别名: bash deploy.sh client)"
        echo ""
        echo "【四、测试与诊断工作流 (Testing & Benchmark)】"
        echo "  bash deploy.sh server test                  - 测试模式：打印完整的端到端耗时分解与目标抓取详情"
        echo "  (快捷别名: bash deploy.sh test)"
        echo "================================================================================"
        ;;
esac
