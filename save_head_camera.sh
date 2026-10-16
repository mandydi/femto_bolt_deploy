#!/usr/bin/env bash
# =========================================================
# 一键单帧画面采集脚本:
# 采集 1 帧 4K RGB、D2C 深度图、三维点云并退出
# 保存路径: /home/caseim/femto_bolt_deploy/captured_frame/
# 分为三个子文件夹: rgb/ , depth/ , point/
# 命名自动自增: 000001.png, 000002.png ...
# =========================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="/home/caseim/femto_bolt_deploy/captured_frame"

# 退出信号捕获
cleanup() {
    docker exec orbbec-femto-bolt pkill -9 -f save_dataset 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 启动前清理任何可能残留的旧采集进程
docker exec orbbec-femto-bolt pkill -9 -f save_dataset 2>/dev/null || true

mkdir -p "$OUT_DIR/rgb" "$OUT_DIR/depth" "$OUT_DIR/point"

echo "========================================================="
echo "执行单帧画面采集 (4K RGB + 16-bit D2C深度 + 三维点云)..."
echo "保存目录: ${OUT_DIR}"
echo "========================================================="

docker cp "$SCRIPT_DIR/save_dataset_single.py" orbbec-femto-bolt:/opt/femto/save_dataset_single.py
docker exec -t orbbec-femto-bolt /opt/femto/entrypoint.sh python3 /opt/femto/save_dataset_single.py /home/caseim/yolo26_packet/captured_frame
