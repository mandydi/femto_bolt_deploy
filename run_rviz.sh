#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 允许本地 root 访问 X11 显示
xhost +local:root 2>/dev/null || xhost + 2>/dev/null || true

DISP="${DISPLAY:-:0}"

RVIZ_CFG=""
if [ -f "$SCRIPT_DIR/femto_bolt.rviz" ]; then
    RVIZ_CFG="$SCRIPT_DIR/femto_bolt.rviz"
elif [ -f "$SCRIPT_DIR/femto_bolt_deploy.rviz" ]; then
    RVIZ_CFG="$SCRIPT_DIR/femto_bolt_deploy.rviz"
fi

CONFIG_ARG=""
MOUNT_ARG=()
if [ -n "$RVIZ_CFG" ]; then
    CONFIG_ARG="-d /opt/femto/config.rviz"
    MOUNT_ARG=(-v "$RVIZ_CFG:/opt/femto/config.rviz:ro")
fi

echo "=================================================="
echo "正在使用显示器 ($DISP) 启动 RViz2..."
echo "配置: ${RVIZ_CFG:-默认配置}"
echo "=================================================="

sudo docker run --rm -it \
    --network host \
    --ipc host \
    -e DISPLAY="$DISP" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/dri:/dev/dri \
    ${MOUNT_ARG[@]+"${MOUNT_ARG[@]}"} \
    --device-cgroup-rule='c 226:* rmw' \
    orbbec-femto-bolt:rviz2 \
    /opt/femto/entrypoint.sh rviz2 ${CONFIG_ARG} "$@"
