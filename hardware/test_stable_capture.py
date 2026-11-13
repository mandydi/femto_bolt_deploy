import cv2
import time
import os

devices = [
    (0, "cam_bottom_0"),
    (6, "cam_side_6"),
    (8, "cam_front_8"),
    (10, "cam_side_10"),
    (12, "cam_extra_12")
]

print("=== 测试顺序无锁单线程稳定抓拍 ===")
for dev_id, name in devices:
    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"❌ 相机 {name} (index {dev_id}) 无法打开")
        continue
    
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    # 读取两帧
    ret, frame = cap.read()
    cap.release()
    time.sleep(0.05)
    
    if ret and frame is not None:
        print(f"✅ {name}: 成功读取 {frame.shape[1]}x{frame.shape[0]}, 均值: {frame.mean():.1f}")
    else:
        print(f"❌ {name}: 读取失败")
print("=== 稳定抓拍测试完成 ===")
