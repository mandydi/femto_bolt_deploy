#!/usr/bin/env python3
"""
多路 USB 高清相机单号扫描测试工具 (安全无崩溃版)
路径: /home/caseim/femto_bolt_deploy/usb_scanner/test_live_scanner.py
"""
import os
import sys
import time
import argparse

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from usb_camera_manager import USBCameraManager

CAPTURES_DIR = os.path.join(CURRENT_DIR, "captures")
os.makedirs(CAPTURES_DIR, exist_ok=True)


def run_once(manager):
    print("=" * 60)
    print("🚀 正在对所有连接的高清相机执行【单次安全扫描】...")
    print("=" * 60)

    # 1. 保存各相机当前视野预览
    print("正在保存各相机的最新视场画面至 captures/ 目录...")
    saved_dict = manager.capture_all_previews(CAPTURES_DIR)
    for name, p in saved_dict.items():
        print(f"  📷 {name}: 画面已保存至 {p}")

    # 2. 执行安全扫码
    t_start = time.time()
    results = manager.scan_all_cameras(save_debug_dir=CAPTURES_DIR)
    total_cost = (time.time() - t_start) * 1000

    print("\n" + "=" * 60)
    print(f"扫描总耗时: {total_cost:.1f}ms | 检出单号总数: {len(results)}")
    if results:
        print("🎉【成功识别到快递单号】:")
        for idx, r in enumerate(results, 1):
            print(f"  [{idx}] 单号: {r['waybill_id']}")
            print(f"      码制类型: {r['type']}")
            print(f"      捕获相机: {r['camera']} ({r['desc']})")
            print(f"      解码耗时: {r['cost_ms']}ms")
            print(f"      证据图片: {r.get('saved_path')}")
    else:
        print("ℹ️ 当前各个相机视野内暂未检测到条码/单号。")
        print("   提示: 请确认相机视野内是否有面单，或拿包裹在镜头前测试。")
    print("=" * 60)
    return results


def run_loop(manager, interval=0.3):
    print("=" * 60)
    print("🔄 进入【实时多目条码安全监听模式】(按 Ctrl+C 退出)")
    print("   请在任意 USB 高清相机前移动或放置包裹进行测试...")
    print("=" * 60)
    
    seen_cache = {}
    try:
        while True:
            results = manager.scan_all_cameras(save_debug_dir=CAPTURES_DIR)
            now = time.time()
            for r in results:
                w_id = r["waybill_id"]
                if w_id not in seen_cache or (now - seen_cache[w_id]) > 3.0:
                    seen_cache[w_id] = now
                    print(f"[{time.strftime('%H:%M:%S')}] 🎯 捕获单号: {w_id} | 码制: {r['type']} | 相机: {r['camera']} ({r['desc']}) | 耗时: {r['cost_ms']}ms")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n监听已安全退出。")


def main():
    parser = argparse.ArgumentParser(description="多路 USB 高清相机近景单号扫描测试工具")
    parser.add_argument("--once", action="store_true", help="单次抓拍并扫描")
    parser.add_argument("--loop", action="store_true", help="实时循环监听扫描")
    parser.add_argument("--cam", type=int, default=None, help="仅扫描指定相机设备号 (如 --cam 8)")
    parser.add_argument("--interval", type=float, default=0.3, help="循环扫描间隔 (秒)")
    args = parser.parse_args()

    manager = USBCameraManager()

    if args.cam is not None:
        print(f"正在仅扫描相机 /dev/video{args.cam}...")
        hits = manager.scan_camera_by_id(args.cam, save_debug_dir=CAPTURES_DIR)
        print("扫描结果:", hits)
        return

    if args.loop:
        run_loop(manager, interval=args.interval)
    else:
        run_once(manager)


if __name__ == "__main__":
    main()
