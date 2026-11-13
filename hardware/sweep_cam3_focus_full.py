#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cam3 4000x3000 全画幅无裁剪焦距遍历测试程序
功能:
  1. 连接 /dev/camera_3 (3号相机)
  2. 强制锁定 MJPG 4000x3000 最高物理分辨率 (绝不裁剪)
  3. 关闭自动对焦 (focus_automatic_continuous=0)
  4. 遍历指定焦距 (0 ~ 1023)，每档对焦等待马达稳定并丢弃残帧
  5. 保存完整 4000x3000 大图到输出目录
  6. 实时分析全图清晰度与泡沫包裹区域清晰度
  7. 测试完成后自动复原相机到自动聚焦模式

使用方式:
  # 1. 默认全面巡检 (涵盖远景、传送带、特写等 20 个关键焦距)
  python3 sweep_cam3_focus_full.py

  # 2. 自定义密集步长扫描 (例如在 340 到 440 之间每隔 10 拍一张)
  python3 sweep_cam3_focus_full.py --start 340 --end 440 --step 10

  # 3. 指定任意焦距列表
  python3 sweep_cam3_focus_full.py --values 360,380,390,400,420
"""

import os
import sys
import time
import argparse
import cv2
import numpy as np

DEFAULT_FOCUS_LIST = [
    0, 100, 150, 200, 250, 300, 
    330, 350, 370, 380, 390, 400, 410, 430, 450, 
    480, 520, 600, 700, 850
]

def parse_args():
    parser = argparse.ArgumentParser(description="CAM3 4000x3000 Full-Frame Focus Sweep")
    parser.add_argument("--device", default="/dev/camera_3", help="Camera device path (default: /dev/camera_3)")
    parser.add_argument("--outdir", default="./captures/cam3_focus_sweep", help="Output directory for full images")
    parser.add_argument("--values", type=str, default=None, help="Comma-separated focus values, e.g. '300,350,400'")
    parser.add_argument("--start", type=int, default=None, help="Range start (0-1023)")
    parser.add_argument("--end", type=int, default=None, help="Range end (0-1023)")
    parser.add_argument("--step", type=int, default=20, help="Range step (default: 20)")
    parser.add_argument("--width", type=int, default=4000, help="Width (default: 4000)")
    parser.add_argument("--height", type=int, default=3000, help="Height (default: 3000)")
    return parser.parse_args()

def main():
    args = parse_args()

    # 确定焦距遍历列表
    if args.values:
        focus_list = [int(v.strip()) for v in args.values.split(",") if v.strip().isdigit()]
    elif args.start is not None and args.end is not None:
        focus_list = list(range(args.start, args.end + 1, args.step))
    else:
        focus_list = DEFAULT_FOCUS_LIST

    out_dir = os.path.abspath(args.outdir)
    os.makedirs(out_dir, exist_ok=True)

    print("=" * 75)
    print("📸 CAM3 4000x3000 全画幅无裁剪遍历对焦测试")
    print(f"  设备节点: {args.device}")
    print(f"  分辨率要求: {args.width} x {args.height} (无裁剪 100% 原始保存)")
    print(f"  保存目标目录: {out_dir}")
    print(f"  遍历对焦列表 ({len(focus_list)} 档): {focus_list}")
    print("=" * 75)

    # 1. 打开相机
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"❌ 无法打开相机设备: {args.device}")
        sys.exit(1)

    # 2. 强制配置 4000x3000 MJPG
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # 3. 关闭自动对焦
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    time.sleep(0.3)

    # 预热并核验真实输出尺寸
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ 相机就绪! 实际输入分辨率: {actual_w} x {actual_h}")
    if actual_w != args.width or actual_h != args.height:
        print(f"⚠️ 警告: 实际分辨率 ({actual_w}x{actual_h}) 与期望 ({args.width}x{args.height}) 不完全一致!")

    results = []

    try:
        for idx, f_val in enumerate(focus_list, 1):
            # 设置对焦
            cap.set(cv2.CAP_PROP_FOCUS, f_val)
            # 等待电机机械对焦与曝光收敛 (0.35s)
            time.sleep(0.35)

            # 丢弃缓冲区中之前对焦的旧帧
            for _ in range(3):
                cap.read()

            ret, frame = cap.read()
            if not ret or frame is None:
                print(f"[{idx:2d}/{len(focus_list):2d}] ❌ 焦距 {f_val:4d} 抓拍失败!")
                continue

            h, w = frame.shape[:2]
            # 严格断言图片分辨率未被缩小或裁剪
            assert w == actual_w and h == actual_h, f"尺寸异常: {w}x{h}"

            # 保存完整无裁剪大图
            filename = f"cam3_focus_{f_val:04d}_4000x3000.jpg"
            save_path = os.path.join(out_dir, filename)
            # 以最高 JPEG 质量 (98) 保存，杜绝二次压缩损伤
            cv2.imwrite(save_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 98])
            file_size_mb = os.path.getsize(save_path) / (1024 * 1024)

            # 计算清晰度指标 (Laplacian 方差)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 1. 全图整体清晰度
            full_sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())

            # 2. 泡沫箱区域清晰度 (y: 1550~2350, x: 850~1500)
            foam_roi = gray[1550:2350, 850:1500]
            foam_sharp = float(cv2.Laplacian(foam_roi, cv2.CV_64F).var()) if foam_roi.size > 0 else 0.0

            results.append({
                "focus": f_val,
                "file": filename,
                "size_mb": file_size_mb,
                "full_sharp": full_sharp,
                "foam_sharp": foam_sharp
            })

            print(f"[{idx:2d}/{len(focus_list):2d}] 🎯 焦距={f_val:4d} | 全图清晰度={full_sharp:7.1f} | 泡沫箱清晰度={foam_sharp:7.1f} | 保存: {filename} ({file_size_mb:.2f} MB)")

    finally:
        # 安全复原：恢复自动连续对焦
        print("\n🔄 正在复原相机设置: 开启自动聚焦 (focus_automatic_continuous=1)...")
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
        cap.release()
        print("✅ 相机设备已安全释放。")

    # 打印排名前列的最优焦距榜单
    if results:
        print("\n" + "=" * 75)
        print("🏆 清晰度综合评测榜单 (前 5 名):")
        print("=" * 75)
        # 按泡沫箱清晰度排序
        by_foam = sorted(results, key=lambda x: x["foam_sharp"], reverse=True)
        print("【泡沫包裹区域最清晰 Top 3】:")
        for rank, item in enumerate(by_foam[:3], 1):
            print(f"  Top {rank}: 焦距 = {item['focus']:4d} (清晰度得分: {item['foam_sharp']:7.1f}) -> 文件: {item['file']}")

        # 按全图清晰度排序
        by_full = sorted(results, key=lambda x: x["full_sharp"], reverse=True)
        print("\n【全图整体最清晰 Top 3】:")
        for rank, item in enumerate(by_full[:3], 1):
            print(f"  Top {rank}: 焦距 = {item['focus']:4d} (清晰度得分: {item['full_sharp']:7.1f}) -> 文件: {item['file']}")
        print("=" * 75)

if __name__ == "__main__":
    main()
