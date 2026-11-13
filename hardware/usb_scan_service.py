#!/usr/bin/env python3
"""
近景 USB 高清相机后台扫描微服务 (异步极速缓存版)
路径: /home/caseim/femto_bolt_deploy/usb_scanner/usb_scan_service.py

核心特点:
1. 后台以单线程安全轮询相机，遇到单号立即缓存
2. 提供 /scan 接口: 毫秒级直接返回最近捕获的有效单号，对分拣 RTT 零耗时增加
3. 容器 (sf_sorting_server) 通过 http://127.0.0.1:8005/scan 瞬时读取
"""
import os
import sys
import json
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from usb_camera_manager import capture_single_camera
from barcode_engine import decode_barcode_robust

# 优先扫描正视传送带与近景的核心相机 (优先级排序)
SCAN_TARGETS = [
    {"dev_id": 8, "name": "cam_front_8", "desc": "正视传送带纸箱相机"},
    {"dev_id": 12, "name": "cam_extra_12", "desc": "传送带/滑槽近景相机"},
    {"dev_id": 6, "name": "cam_side_6", "desc": "侧面俯视滑槽相机"},
    {"dev_id": 10, "name": "cam_side_10", "desc": "侧视传送带相机"},
    {"dev_id": 0, "name": "cam_bottom_0", "desc": "底部/低位相机"},
]

CAPTURES_DIR = os.path.join(CURRENT_DIR, "captures")
os.makedirs(CAPTURES_DIR, exist_ok=True)

# 全局共享状态
_LOCK = threading.Lock()
_LATEST_HITS = []
_LAST_HIT_TIME = 0.0
_RUNNING = True

def background_scan_loop():
    global _LATEST_HITS, _LAST_HIT_TIME, _RUNNING
    print("[*] 异步后台条码监听循环已就绪...", flush=True)
    
    while _RUNNING:
        for target in SCAN_TARGETS:
            if not _RUNNING:
                break
            try:
                # 仅对单张画面进行轻量安全单线程捕获
                frame = capture_single_camera(target["dev_id"])
                if frame is None:
                    continue

                hits = decode_barcode_robust(frame)
                if hits:
                    now = time.time()
                    hit_list = []
                    for h in hits:
                        hit_list.append({
                            "waybill_id": h.get("data"),
                            "type": h.get("type"),
                            "camera": target["name"],
                            "desc": target["desc"],
                            "timestamp": now
                        })
                        print(f"[{time.strftime('%H:%M:%S')}] 🎯 捕获新单号: {h.get('data')} (来自 {target['name']})", flush=True)

                    with _LOCK:
                        _LATEST_HITS = hit_list
                        _LAST_HIT_TIME = now
            except Exception:
                pass
            time.sleep(0.1)

        time.sleep(0.2)


class ScanRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/health":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')
            return

        if self.path.startswith("/scan"):
            now = time.time()
            valid_hits = []
            with _LOCK:
                # 缓存有效期 5 秒 (5秒内扫到的包裹单号均有效)
                if (now - _LAST_HIT_TIME) <= 5.0:
                    valid_hits = list(_LATEST_HITS)

            resp_data = {
                "success": len(valid_hits) > 0,
                "hits": valid_hits,
                "cache_age_sec": round(now - _LAST_HIT_TIME, 2) if _LAST_HIT_TIME > 0 else 999.0
            }
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(resp_data, ensure_ascii=False).encode("utf-8"))
            return

        self.send_response(404)
        self.end_headers()


def run_server(port=8005):
    t = threading.Thread(target=background_scan_loop, daemon=True)
    t.start()

    server_address = ("127.0.0.1", port)
    httpd = HTTPServer(server_address, ScanRequestHandler)
    print("=" * 60)
    print(f"🚀 USB 近景扫码极速服务已就绪: http://127.0.0.1:{port}")
    print(f"   后台自动轮询监听，前端 0ms 闪电响应")
    print("=" * 60, flush=True)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        global _RUNNING
        _RUNNING = False
        httpd.server_close()


if __name__ == "__main__":
    run_server()
