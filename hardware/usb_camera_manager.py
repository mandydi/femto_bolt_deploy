"""
安全单线程/按需顺序相机管理器 (彻底根除崩溃转储)
路径: /home/caseim/femto_bolt_deploy/usb_scanner/usb_camera_manager.py
设计原则:
1. 严禁 5 个相机同时在后台无休止抓流 (避免撑爆 USB 2.0 控制器导致内核驱动越界损坏)
2. 按需单次抓拍 (Sequential Open-Read-Close)
3. 纯单线程内存安全，绝不触发 glibc free corruption
"""
import os
import time
import cv2
from barcode_engine import decode_barcode_robust

DEFAULT_CAMERAS = [
    {"dev_id": 0, "name": "cam_bottom_0", "desc": "底部/低位相机"},
    {"dev_id": 6, "name": "cam_side_6", "desc": "侧面俯视滑槽相机"},
    {"dev_id": 8, "name": "cam_front_8", "desc": "正视传送带纸箱相机"},
    {"dev_id": 10, "name": "cam_side_10", "desc": "侧视传送带相机"},
    {"dev_id": 12, "name": "cam_extra_12", "desc": "传送带/滑槽近景相机"},
]

def capture_single_camera(dev_id, width=1920, height=1080):
    """
    安全抓取单张 1080P 画面
    打开 -> 设置 -> 读帧 -> 立即释放设备
    """
    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(dev_id)
    if not cap.isOpened():
        return None

    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # 丢掉缓冲帧获取最新帧
        for _ in range(2):
            cap.grab()
        ret, frame = cap.read()
        return frame if ret and frame is not None else None
    except Exception:
        return None
    finally:
        cap.release()


class USBCameraManager:
    def __init__(self, camera_configs=None):
        self.configs = camera_configs or DEFAULT_CAMERAS

    def scan_camera_by_id(self, dev_id, save_debug_dir=None):
        """扫描指定单个相机"""
        cfg = next((c for c in self.configs if c["dev_id"] == dev_id), None)
        name = cfg["name"] if cfg else f"cam_{dev_id}"
        desc = cfg["desc"] if cfg else "未知相机"

        frame = capture_single_camera(dev_id)
        if frame is None:
            return []

        t0 = time.time()
        hits = decode_barcode_robust(frame)
        cost_ms = (time.time() - t0) * 1000

        results = []
        for h in hits:
            item = {
                "waybill_id": h.get("data"),
                "type": h.get("type"),
                "camera": name,
                "desc": desc,
                "cost_ms": round(cost_ms, 1),
                "engine": h.get("engine")
            }
            if save_debug_dir:
                os.makedirs(save_debug_dir, exist_ok=True)
                fname = f"{int(time.time()*1000)}_{name}_{h.get('data')}.jpg"
                fpath = os.path.join(save_debug_dir, fname)
                cv2.imwrite(fpath, frame)
                item["saved_path"] = fpath
            results.append(item)
        return results

    def scan_all_cameras(self, save_debug_dir=None):
        """
        顺序安全轮询所有在线相机，杜绝总线拥塞与内存崩溃
        一旦任意相机命中单号，可立即返回
        """
        all_hits = []
        for cfg in self.configs:
            res = self.scan_camera_by_id(cfg["dev_id"], save_debug_dir)
            if res:
                all_hits.extend(res)
        return all_hits

    def capture_all_previews(self, save_dir):
        """保存所有相机的当前预览图"""
        os.makedirs(save_dir, exist_ok=True)
        saved = {}
        for cfg in self.configs:
            frame = capture_single_camera(cfg["dev_id"])
            if frame is not None:
                p = os.path.join(save_dir, f"latest_{cfg['name']}.jpg")
                cv2.imwrite(p, frame)
                saved[cfg["name"]] = p
        return saved
