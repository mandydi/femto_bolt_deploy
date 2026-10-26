#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
顺丰包裹视觉分拣 ROS 2 客户端调用脚本 (测试与生产统一入口)

功能说明:
1. 测试工作流 (交互诊断模式):
   python3 test_sorting_client.py
   --> 打印完整的端到端耗时 (RTT)、服务端耗时细项分解 (YOLO/3D拟合/等待帧)、抓取坐标与位姿

2. 实际工程工作流 (生产 JSON 模式):
   python3 test_sorting_client.py --json
   --> 纯净输出服务端返回的标准 JSON 字符串，无任何多余打印，供机械臂/PLC/外部主控程序直接捕获解析
"""

import sys
import time
import json
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def format_timing_report(cost_sec, data):
    timing = data.get("timing", {})
    server_total_ms = timing.get("server_total_ms")
    wait_frame_ms = timing.get("wait_frame_ms")
    parse_ms = timing.get("parse_ms")
    yolo_ms = timing.get("yolo_ms")
    fit_3d_ms = timing.get("fit_3d_ms")

    report = []
    report.append("=" * 60)
    report.append("📦 顺丰分拣服务测试调用报告 (SF Sorting Test Report)")
    report.append("=" * 60)
    report.append(f"[*] 接口地址: /sf_sorting/get_result (std_srvs/srv/Trigger)")
    
    is_success = data.get("success", False)
    status_str = "✅ 成功 (已识别包裹并拟合抓取点)" if is_success else "⚠️  未检测到有效包裹目标"
    report.append(f"[*] 执行状态: {status_str}")
    report.append("")
    report.append("⏱️  【调用耗时统计 (Timing Breakdown)】")
    report.append(f"  • 客户端实际总耗时 (RTT):   {cost_sec:.3f} s  ({cost_sec*1000:.1f} ms)  <-- 客户端拿到数据的完整实际时间")
    
    if server_total_ms is not None:
        comm_ms = max(0.0, cost_sec * 1000 - server_total_ms)
        report.append(f"    ├─ 服务端本次请求总耗时:   {server_total_ms/1000:.3f} s  ({server_total_ms:.1f} ms)  (已包含在 RTT 内，无需叠加)")
        if wait_frame_ms is not None:
            report.append(f"    │  ├─ 相机帧同步等待:     {wait_frame_ms/1000:.3f} s  ({wait_frame_ms:.1f} ms)")
        if parse_ms is not None:
            report.append(f"    │  ├─ 图像与点云格式转换: {parse_ms/1000:.3f} s  ({parse_ms:.1f} ms)")
        if yolo_ms is not None:
            report.append(f"    │  ├─ YOLO 目标与多边形:  {yolo_ms/1000:.3f} s  ({yolo_ms:.1f} ms)")
        if fit_3d_ms is not None:
            report.append(f"    │  └─ 3D点云抽取与拟合:   {fit_3d_ms/1000:.3f} s  ({fit_3d_ms:.1f} ms)")
        report.append(f"    └─ ROS 2 网络通信与开销:   {comm_ms/1000:.3f} s  ({comm_ms:.1f} ms)")

    if timing.get('preprocess_ms') is not None:
        report.append(f"  • 后台同步帧预处理:         {timing['preprocess_ms']:.1f} ms (请求前执行，不计入 RTT)")
        report.append(f"  • 算法开始时输入帧龄:       {timing['frame_age_ms']:.1f} ms")

    # 评估指标
    if cost_sec < 1.0:
        report.append(f"  • 耗时指标评估:             ✅ 优于指标 (< 1.0s)")
    else:
        report.append(f"  • 耗时指标评估:             ⚠️  超过 1.0s 指标阈值")

    report.append("")
    report.append("🎯 【目标识别与抓取结果 (Target Info)】")
    if is_success:
        report.append(f"  • 包裹类别:   {data.get('best_label')}")
        report.append(f"  • 中心坐标:   X = {data.get('center_world', [None]*3)[0]} mm,  Y = {data.get('center_world', [None]*3)[1]} mm,  Z = {data.get('center_world', [None]*3)[2]} mm")
        quad_3d = data.get("box_8pts_world")
        if quad_3d and len(quad_3d) == 8:
            report.append("  • 内接包络长方体八点 (mm):")
            for idx, pt in enumerate(quad_3d):
                report.append(f"      P{idx+1}: [{pt[0]:>8.2f}, {pt[1]:>8.2f}, {pt[2]:>8.2f}]")
    else:
        report.append("  • 未在有效抓取区域 (ROI) 内检测到包裹，或点云深度质量不足")
    
    report.append("=" * 60)
    if data.get("error"): report.append(f"错误: {data['error']}")
    return "\n".join(report)


def call_sorting_service(timeout_sec=5.0, json_only=False, exit_on_fail=False, call_timeout_sec=30.0):
    rclpy.init()
    node = rclpy.create_node('sf_sorting_client')
    client = node.create_client(Trigger, '/sf_sorting/get_result')

    if not json_only:
        print(f"[*] 正在连接分拣服务 /sf_sorting/get_result (超时时间: {timeout_sec}s)...", flush=True)

    if not client.wait_for_service(timeout_sec=timeout_sec):
        if not json_only:
            print(f"\n[❌ 错误] 无法连接到服务 /sf_sorting/get_result (超时 {timeout_sec}s)!", flush=True)
            print("[!] 请检查：", flush=True)
            print("    1. 分拣服务端是否已启动：bash deploy.sh server status", flush=True)
            print("    2. 若未启动，请执行：bash deploy.sh server start", flush=True)
            print("    3. 检查两台机 ROS_DOMAIN_ID 是否一致 (默认 0)", flush=True)
        else:
            print(json.dumps({
                "success": False,
                "error": f"Service /sf_sorting/get_result unavailable (timeout {timeout_sec}s)"
            }, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not json_only:
        print("[*] 服务已连接，正在请求抓取点计算...", flush=True)

    t0 = time.time()
    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=call_timeout_sec)

    if not future.done():
        future.cancel()
        failure = {"success": False, "error": "Service response timeout; execution result unknown"}
        print(json.dumps(failure, ensure_ascii=False), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if future.result() is not None:
        res = future.result()
        cost_sec = time.time() - t0

        if json_only:
            # 纯工程生产模式: 剥离 timing 调试字段，纯净输出业务所需的数据结构
            try:
                data = json.loads(res.message)
                data.pop("timing", None)
                print(json.dumps(data, ensure_ascii=False), flush=True)
            except Exception:
                print(res.message, flush=True)
        else:
            try:
                data = json.loads(res.message)
                print(format_timing_report(cost_sec, data), flush=True)
            except Exception:
                print(f"[*] 收到服务端响应 (耗时 {cost_sec:.2f}s):", flush=True)
                print(f"    success: {res.success}", flush=True)
                print(f"    原始文本: {res.message}", flush=True)

        node.destroy_node()
        rclpy.shutdown()
        if exit_on_fail and not res.success:
            sys.exit(1)
        sys.exit(0)
    else:
        cost_sec = time.time() - t0
        if json_only:
            print(json.dumps({
                "success": False,
                "error": "Service call failed (future result is None)"
            }, ensure_ascii=False))
        else:
            print(f"[❌ 错误] 服务调用失败 (耗时 {cost_sec:.2f}s)", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="顺丰包裹视觉分拣客户端 (测试与生产统一调用)")
    parser.add_argument("--json", action="store_true", help="生产模式: 仅输出标准 JSON 结果供外部工程/PLC捕获")
    parser.add_argument("--timeout", type=float, default=5.0, help="服务连接超时时间(秒，默认 5.0)")
    parser.add_argument("--exit-on-fail", action="store_true", help="当未检测到包裹时返回非0退出码 (默认不开启，返回0并输出 success: false)")
    parser.add_argument("--call-timeout", type=float, default=30.0, help="服务响应超时秒数（不自动重试）")
    args = parser.parse_args()

    call_sorting_service(timeout_sec=args.timeout, json_only=args.json, exit_on_fail=args.exit_on_fail, call_timeout_sec=args.call_timeout)
