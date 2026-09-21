#!/usr/bin/env bash
# =========================================================
# 一键从 Orbbec Femto Bolt 容器中提取并保存当前头部相机 4K RGB 图像
# =========================================================
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT_FILE="${1:-$SCRIPT_DIR/head_camera.jpg}"

echo "正在从 ROS 话题中提取头部相机当前 4K 画面..."

cat << 'INNER_EOF' | docker exec -i orbbec-femto-bolt /opt/femto/entrypoint.sh python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sys

class Saver(Node):
    def __init__(self):
        super().__init__('head_cam_saver_4k')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.sub = self.create_subscription(
            CompressedImage,
            '/vision/top_camera/color/image_raw/compressed',
            self.cb,
            qos
        )

    def cb(self, msg):
        with open('/tmp/head_camera.jpg', 'wb') as f:
            f.write(bytes(msg.data))
        sys.exit(0)

rclpy.init()
node = Saver()
try:
    rclpy.spin(node)
except (SystemExit, KeyboardInterrupt):
    pass
rclpy.shutdown()
INNER_EOF

docker cp orbbec-femto-bolt:/tmp/head_camera.jpg "$OUT_FILE" 2>/dev/null
if [ -s "$OUT_FILE" ]; then
    sz=$(ls -lh "$OUT_FILE" | awk '{print $5}')
    echo "[OK] 头部相机 4K 图像保存成功: $OUT_FILE ($sz)"
else
    echo "[FAIL] 保存失败，请确认 orbbec-femto-bolt 容器正在运行且话题正在发布！"
fi
