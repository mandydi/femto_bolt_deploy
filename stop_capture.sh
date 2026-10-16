#!/usr/bin/env bash
# =========================================================
# 一键彻底停止所有正在运行的画面采集进程
# =========================================================
echo "正在检查并停止后台采集任务..."
docker exec orbbec-femto-bolt pkill -9 -f save_dataset_continuous.py 2>/dev/null || true
pkill -9 -f start_capture_2s.sh 2>/dev/null || true
pkill -9 -f save_head_camera.sh 2>/dev/null || true
echo "已全部停止！"
