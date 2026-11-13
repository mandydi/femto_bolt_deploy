"""
多路 USB 高清相机唯一标识与固定调用 API
(支持防色彩污染流式稳定 + 0秒实验模式 + 相机留空优雅容错 + 各机位独立目录保存)

适用平台: 8001 / 8002 及后续标准工位机
硬件与系统固定映射 (通过 /etc/udev/rules.d/99-usb-cameras.rules 锁定 USB 物理插槽，重启永不漂移):
  1号相机: /dev/camera_1 -> 底部/低位相机 (8002 留空未接)
  2号相机: /dev/camera_2 -> 侧面俯视滑槽相机
  3号相机: /dev/camera_3 -> 正视传送带纸箱相机
  4号相机: /dev/camera_4 -> 侧视传送带相机
  5号相机: /dev/camera_5 -> 传送带/滑槽近景相机
"""
import os
import sys
import time
import argparse
from datetime import datetime
import cv2

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from barcode_engine import decode_barcode_robust

# 相机逻辑角色与固定软链接定义
CAMERA_ROLES = {
    1: {"dev": "/dev/camera_1", "name": "cam_bottom_1", "role": "1号机 - 底部/低位相机"},
    2: {"dev": "/dev/camera_2", "name": "cam_chute_2", "role": "2号机 - 侧面俯视滑槽相机"},
    3: {"dev": "/dev/camera_3", "name": "cam_front_3", "role": "3号机 - 正视传送带纸箱相机"},
    4: {"dev": "/dev/camera_4", "name": "cam_side_4", "role": "4号机 - 侧视传送带相机"},
    5: {"dev": "/dev/camera_5", "name": "cam_near_5", "role": "5号机 - 传送带/滑槽近景相机"},
}

BASE_CAPTURES_DIR = os.path.join(CURRENT_DIR, "captures")
os.makedirs(BASE_CAPTURES_DIR, exist_ok=True)

# 确保 1~5 号相机都有各自独立的保存子目录
for cid in CAMERA_ROLES.keys():
    os.makedirs(os.path.join(BASE_CAPTURES_DIR, f"cam_{cid}"), exist_ok=True)


def get_cam_dir(cam_id: int) -> str:
    """获取指定相机的独立归档目录 (如 captures/cam_2/)"""
    cam_dir = os.path.join(BASE_CAPTURES_DIR, f"cam_{cam_id}")
    os.makedirs(cam_dir, exist_ok=True)
    return cam_dir


def get_camera_info(cam_id: int) -> dict:
    """获取指定相机的拓扑与状态信息"""
    if cam_id not in CAMERA_ROLES:
        raise ValueError(f"无效的相机ID: {cam_id}，可选范围为 1 ~ 5")
    
    info = dict(CAMERA_ROLES[cam_id])
    dev_path = info["dev"]
    is_connected = os.path.exists(dev_path)
    info["connected"] = is_connected

    if is_connected:
        real_dev = os.path.realpath(dev_path)
        base_name = os.path.basename(real_dev)
        info["real_dev"] = real_dev
        
        # 探测底层 USB 物理端口
        usb_port = "未知"
        try:
            sys_path = os.path.realpath(f"/sys/class/video4linux/{base_name}")
            parts = sys_path.split("/")
            ports = [p for p in parts if p.startswith("1-") or p.startswith("2-")]
            if ports:
                usb_port = ports[0]
        except Exception:
            pass
        info["usb_port"] = usb_port
        info["desc"] = f"{info['role']} ({base_name} @ USB {usb_port})"
    else:
        info["real_dev"] = None
        info["usb_port"] = None
        info["desc"] = f"{info['role']} [未接入/留空]"

    return info


def get_camera_frame(cam_id: int, width: int = 4000, height: int = 3000, stream_seconds: float = 1.8):
    """
    【算法调用接口 1: 拍摄单张高清画面】
    特性:
      1. stream_seconds > 0: 流式推流预热，彻底杜绝 JPEG 截断花屏、绿条与黄斑。
      2. stream_seconds == 0: 零延迟即时快照模式，适合无色彩污染环境极速抓拍。
      3. 相机未接时直接返回 None，不阻塞、不抛致命崩溃。
    输入: cam_id (1, 2, 3, 4, 5)
    输出: BGR 图像 numpy 数组 (失败返回 None)
    """
    info = get_camera_info(cam_id)
    if not info["connected"]:
        return None

    dev_path = info["dev"]
    cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(dev_path)
    if not cap.isOpened():
        return None

    try:
        # 设置 MJPG 与高清分辨率
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 模式 A: 0秒实验模式 (即时抓拍)
        if stream_seconds <= 0:
            ret, frame = cap.read()
            if ret and frame is not None:
                return frame
            return None

        # 模式 B: 预热防色彩污染模式 (持续推流丢弃前置过渡帧)
        t_start = time.time()
        latest_clean_frame = None

        while (time.time() - t_start) < stream_seconds:
            ret, frame = cap.read()
            if ret and frame is not None:
                latest_clean_frame = frame
            time.sleep(0.03)

        return latest_clean_frame
    except Exception as e:
        print(f"相机 {cam_id} 读取异常: {e}", flush=True)
        return None
    finally:
        cap.release()
        # 总线释放安全间隔
        time.sleep(0.25)


def scan_camera(cam_id: int, save_debug: bool = False, stream_seconds: float = 1.8):
    """
    【算法调用接口 2: 扫描指定单个相机】
    输入: cam_id (1, 2, 3, 4, 5)
    输出: 若扫到返回单号字符串 (如 'SF5146342231083')，若未扫到返回 None
    """
    info = get_camera_info(cam_id)
    if not info["connected"]:
        return None

    frame = get_camera_frame(cam_id, stream_seconds=stream_seconds)
    if frame is None:
        return None

    hits = decode_barcode_robust(frame)
    if hits:
        waybill_id = hits[0].get("data")
        if save_debug and waybill_id:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            cam_dir = get_cam_dir(cam_id)
            out_p = os.path.join(cam_dir, f"cam_{cam_id}_{ts}_{waybill_id}.jpg")
            cv2.imwrite(out_p, frame)
        return waybill_id
    return None


def scan_cameras_in_order(cam_ids=None, save_debug: bool = True, stream_seconds: float = 1.8):
    """
    【业务组长逻辑接口: 顺序遍历各个相机扫码】
    逻辑: 依次调用 USB 相机，类似这个扫不出来就遍历下一个，直到有单号或全部扫不到
    输入: cam_ids (默认 [1, 2, 3, 4, 5]，会自动跳过未接入的机位)
    输出: (命中相机ID, 命中单号字符串) 或 None
    """
    if cam_ids is None:
        cam_ids = [1, 2, 3, 4, 5]

    for cid in cam_ids:
        info = get_camera_info(cid)
        if not info["connected"]:
            print(f"[  跳过] 相机 {cid} 号 ({info['desc']}) 未接入，跳过检查...", flush=True)
            continue

        t0 = time.time()
        w_id = scan_camera(cid, save_debug=save_debug, stream_seconds=stream_seconds)
        cost_ms = (time.time() - t0) * 1000
        desc = info["desc"]

        if w_id:
            print(f"[✅ 命中] 相机 {cid} 号 ({desc}) 成功识别到单号: {w_id} (耗时: {cost_ms:.1f}ms)", flush=True)
            return cid, w_id
        else:
            print(f"[  跳过] 相机 {cid} 号 ({desc}) 未发现单号 ({cost_ms:.1f}ms)，继续检查下一个...", flush=True)

    print("[⚠️ 结束] 遍历全部有效相机，均未发现有效单号。", flush=True)
    return None


def _get_next_index(cam_id: int) -> int:
    """获取该相机在专属独立目录下的下一个自增序号"""
    cam_dir = get_cam_dir(cam_id)
    prefix = f"cam_{cam_id}_"
    existing = [f for f in os.listdir(cam_dir) if f.startswith(prefix) and f.endswith(".jpg")]
    return len(existing) + 1


def main():
    parser = argparse.ArgumentParser(description="多路 USB 相机固定标识与调用工具")
    parser.add_argument("--cam", type=int, choices=[1, 2, 3, 4, 5], help="指定只测试拍照某个相机 (1~5)")
    parser.add_argument("--all", action="store_true", help="所有已连接相机各拍一张测试照，分别归档到各自文件夹")
    parser.add_argument("--scan", action="store_true", help="执行组长逻辑: 顺序遍历相机扫码")
    parser.add_argument("--stream-time", type=float, default=1.8, help="推流稳定等待秒数 (默认1.8s稳定防残损，设为 0 即为即时快照模式)")
    args = parser.parse_args()

    print("=" * 72)
    print("📋 USB 高清相机硬件拓扑状态表:")
    print("=" * 72)
    for cid in range(1, 6):
        info = get_camera_info(cid)
        if info["connected"]:
            target = info["real_dev"]
            cam_folder = f"captures/cam_{cid}/"
            print(f"  [{cid}号相机] 软链接: {info['dev']:<14} -> 设备: {target:<10} | 保存目录: {cam_folder:<18} | {info['desc']}")
        else:
            print(f"  [{cid}号相机] 软链接: {info['dev']:<14} -> [未接入/留空]   | 预留目录: captures/cam_{cid}/    | {info['desc']}")
    print("=" * 72 + "\n")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if args.cam:
        cid = args.cam
        info = get_camera_info(cid)
        if not info["connected"]:
            print(f"⚠️ 相机 {cid} 号 ({info['desc']}) 未接入硬件，软链接 {info['dev']} 不存在。")
            return

        cam_dir = get_cam_dir(cid)
        seq = _get_next_index(cid)
        filename = f"cam_{cid}_{timestamp}_{seq:03d}.jpg"
        save_p = os.path.join(cam_dir, filename)

        mode_str = f"推流预热 {args.stream_time}s" if args.stream_time > 0 else "0s 极速快照模式"
        print(f"📸 正在对 {cid} 号相机 ({info['desc']}) 拍照 ({mode_str})...")
        frame = get_camera_frame(cid, stream_seconds=args.stream_time)
        if frame is not None:
            cv2.imwrite(save_p, frame)
            latest_p = os.path.join(cam_dir, f"cam_{cid}_latest.jpg")
            cv2.imwrite(latest_p, frame)
            print(f"✅ 抓拍成功! 图像尺寸: {frame.shape[1]}x{frame.shape[0]} (序号: #{seq:03d})")
            print(f"   • 专属归档: {save_p}")
            print(f"   • 最新指针: {latest_p}")
        else:
            print(f"❌ 抓拍失败: {info['dev']}")
        return

    if args.all:
        mode_str = f"每台推流稳定 {args.stream_time}s" if args.stream_time > 0 else "0s 极速快照模式"
        print(f"📸 正在对已接入相机拍照 (批次时间戳: {timestamp}，{mode_str})...")
        for cid in [1, 2, 3, 4, 5]:
            info = get_camera_info(cid)
            if not info["connected"]:
                print(f"  ⚪ [{cid}号机留空/跳过]: {info['desc']}")
                continue

            cam_dir = get_cam_dir(cid)
            seq = _get_next_index(cid)
            filename = f"cam_{cid}_{timestamp}_{seq:03d}.jpg"
            save_p = os.path.join(cam_dir, filename)

            frame = get_camera_frame(cid, stream_seconds=args.stream_time)
            if frame is not None:
                cv2.imwrite(save_p, frame)
                latest_p = os.path.join(cam_dir, f"cam_{cid}_latest.jpg")
                cv2.imwrite(latest_p, frame)
                print(f"  ✅ [{cid}号机已就绪] 尺寸: {frame.shape[1]}x{frame.shape[0]} | 保存至: captures/cam_{cid}/{filename}")
            else:
                print(f"  ❌ [{cid}号机抓取失败]: {info['dev']}")
        print(f"\n全部测试照已分类保存至各自专属目录: {BASE_CAPTURES_DIR}/cam_*/")
        return

    if args.scan:
        print("🔍 正在按组长逻辑执行【多机位遍历扫码测试】...")
        hit = scan_cameras_in_order(stream_seconds=args.stream_time)
        if hit:
            print(f"\n🎉 最终返回: 相机编号 = {hit[0]}, 单号 = {hit[1]}")
        else:
            print("\n未检测到单号。")
        return

    print("提示: 可执行 python3 camera_id_api.py --all 拍摄所有相机，或 --cam 2 单独拍摄，或 --scan。")


if __name__ == "__main__":
    main()
