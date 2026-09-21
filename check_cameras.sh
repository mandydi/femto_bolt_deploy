#!/usr/bin/env bash
# =========================================================
# USB 相机一键检测与高画质采集工具
# 支持 4K 最大分辨率与自动曝光 (AEC) 预热，避免首帧全黑
# =========================================================
set -e

OUT_DIR="${1:-/tmp/cam_test}"
mkdir -p "$OUT_DIR"

echo "================================================="
echo "        USB 相机状态检测与高质量截图"
echo "  保存目录: $OUT_DIR"
echo "================================================="

# 临时停止 pipewire 避免独占 /dev/video1
systemctl --user stop wireplumber pipewire 2>/dev/null || true
sleep 0.5

python3 - << EOF
import cv2
import os
import time

cams = [
    ('/dev/video1', 3840, 2160, 'video1.jpg', 'Fic760xUsbCamera #1'),
    ('/dev/video2', 3840, 2160, 'video2.jpg', 'Fic760xUsbCamera #2'),
    ('/dev/video4', 3840, 2160, 'video4.jpg', 'Fic760xUsbCamera #3'),
    ('/dev/video6', 3840, 2160, 'video6.jpg', 'Fic760xUsbCamera #4'),
    ('/dev/video9', 1920, 1080, 'video9.jpg', 'ARC International Camera'),
]

out_dir = "$OUT_DIR"
for dev, req_w, req_h, fname, label in cams:
    if not os.path.exists(dev):
        print(f"[-] {dev} ({label}): 节点不存在")
        continue

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[-] {dev} ({label}): 无法打开设备（可能被独占）")
        continue

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, req_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, req_h)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 预热 20 帧使自动曝光/增益稳定，避免画面全黑
    frame = None
    for _ in range(20):
        ret, tmp = cap.read()
        if ret:
            frame = tmp
        time.sleep(0.02)
    cap.release()

    if frame is not None:
        save_file = os.path.join(out_dir, fname)
        cv2.imwrite(save_file, frame)
        mean_b = frame.mean()
        status = "正常" if mean_b > 15 else "偏暗/需补光"
        sz_kb = int(os.path.getsize(save_file) / 1024)
        print(f"[+] {dev} ({label}): 采集成功! 分辨率: {actual_w}x{actual_h}, 亮度: {mean_b:.1f} ({status}), 大小: {sz_kb}KB -> {save_file}")
    else:
        print(f"[-] {dev} ({label}): 抓取失败 (未读到有效帧)")

EOF

# 恢复 pipewire
systemctl --user start pipewire wireplumber 2>/dev/null || true

echo "================================================="
echo "检测完成！"
echo "================================================="
