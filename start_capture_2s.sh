#!/usr/bin/env bash
# =========================================================
# 一键启动 2s 周期连续采集头部相机 4K RGB、D2C 深度图、三维点云
# 保存路径: /home/caseim/femto_bolt_deploy/captured_frame/
# 分为三个子文件夹: rgb/ , depth/ , point/
# =========================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="/home/caseim/femto_bolt_deploy/captured_frame"
INTERVAL="${1:-2.0}"

# 退出信号捕捉：保证 Ctrl+C 时容器内的后台 python 采集进程被彻底杀死
cleanup() {
    echo -e "\n[INFO] 正在停止容器内的采集进程..."
    docker exec orbbec-femto-bolt pkill -9 -f save_dataset_continuous.py 2>/dev/null || true
    echo "[OK] 采集已彻底停止！"
}
trap cleanup EXIT INT TERM

# 启动前先清理历史可能遗留的旧采集进程
docker exec orbbec-femto-bolt pkill -9 -f save_dataset_continuous.py 2>/dev/null || true

echo "========================================================="
echo "启动连续画面采集: 每 ${INTERVAL} 秒一帧"
echo "保存目录: ${OUT_DIR}"
echo "子目录: rgb/ (3840x2160), depth/ (3840x2160 D2C), point/ (.ply)"
echo "按 Ctrl+C 可立即停止采集（容器内进程会自动清理退出）"
echo "========================================================="

mkdir -p "$OUT_DIR/rgb" "$OUT_DIR/depth" "$OUT_DIR/point"

docker cp "$SCRIPT_DIR/save_dataset_continuous.py" orbbec-femto-bolt:/opt/femto/save_dataset_continuous.py
# 使用 -t 传递 TTY 信号，确保 Ctrl+C 能够直达容器内的 Python 进程
docker exec -t orbbec-femto-bolt /opt/femto/entrypoint.sh python3 /opt/femto/save_dataset_continuous.py /home/caseim/yolo26_packet/captured_frame "$INTERVAL"
