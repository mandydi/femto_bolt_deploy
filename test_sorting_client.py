#!/usr/bin/env python3
"""
顺丰分拣 ROS 2 客户端调用脚本
可在局域网任意工控机上运行 (需设置相同的 ROS_DOMAIN_ID)
无需安装任何自定义消息包，直接使用 ROS 2 标准 std_srvs/srv/Trigger

支持命令行参数：
  python3 test_sorting_client.py          # 友好调试输出
  python3 test_sorting_client.py --json   # 仅输出原始 JSON 字符串 (便于其它程序 subprocess 捕获解析)
  python3 test_sorting_client.py --timeout 10.0
"""
import sys
import time
import json
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

def call_sorting_service(timeout_sec=5.0, json_only=False):
    rclpy.init()
    node = rclpy.create_node('sf_sorting_test_client')
    client = node.create_client(Trigger, '/sf_sorting/get_result')

    if not json_only:
        print(f"[*] 正在连接分拣服务 /sf_sorting/get_result ...", flush=True)

    if not client.wait_for_service(timeout_sec=timeout_sec):
        if not json_only:
            print(f"[!] 无法连接到服务 /sf_sorting/get_result (超时 {timeout_sec}s)", flush=True)
            print("[!] 请检查：", flush=True)
            print("    1. 分拣服务端是否已启动 (bash deploy.sh server start)", flush=True)
            print("    2. 两台工控机是否在同一局域网网段，且 ROS_DOMAIN_ID 相同 (默认 0)", flush=True)
        else:
            print(json.dumps({"success": False, "error": f"Service unavailable (timeout {timeout_sec}s)"}))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not json_only:
        print("[*] 服务已连接，正在发送抓取点计算请求...", flush=True)

    t0 = time.time()
    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        res = future.result()
        cost = time.time() - t0
        if json_only:
            # 纯 JSON 模式直接输出服务端传回的 JSON 消息
            print(res.message, flush=True)
        else:
            print(f"[*] 收到服务端响应 (耗时 {cost:.2f}s):", flush=True)
            print(f"    success: {res.success}", flush=True)
            try:
                data = json.loads(res.message)
                print("    解析 JSON 返回结果:", flush=True)
                print(json.dumps(data, indent=2, ensure_ascii=False), flush=True)
                if data.get("success"):
                    print(f"\n[+] 抓取目标结果:", flush=True)
                    print(f"    类别: {data.get('best_label')}", flush=True)
                    print(f"    Z最小抓取点: X={data.get('x')}, Y={data.get('y')}, Z={data.get('z')} mm", flush=True)
                    print(f"    倾斜角: theta_x={data.get('theta_x_deg')}°, theta_y={data.get('theta_y_deg')}°", flush=True)
                    if data.get("quad_3d"):
                        print(f"    四边形3D顶点 (mm): {data.get('quad_3d')}", flush=True)
            except Exception:
                print(f"    原始返回文本: {res.message}", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if res.success else 1)
    else:
        if json_only:
            print(json.dumps({"success": False, "error": "Call failed"}))
        else:
            print("[!] 服务调用失败", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="顺丰分拣客户端")
    parser.add_argument("--json", action="store_true", help="只输出标准 JSON 格式，便于外部工程程序捕获")
    parser.add_argument("--timeout", type=float, default=5.0, help="连接等待超时时间(秒)")
    args = parser.parse_args()

    call_sorting_service(timeout_sec=args.timeout, json_only=args.json)
