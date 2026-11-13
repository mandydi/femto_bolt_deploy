# 多路 USB 高清相机硬件拓扑与唯一标识调用系统

## 一、物理拓扑固定绑定规则 (udev)
基于 Linux USB 物理拓扑端口 (`by-path`) 绝对死锁绑定，即使系统重启、设备热插拔，软链接永久不变：

### 8001 机器 (5路完整相机)
| 相机编号 | 软链接 | 物理 USB 端口 | 工位视角角色 | 保存归档目录 |
| :--- | :--- | :--- | :--- | :--- |
| **1号机** | `/dev/camera_1` | `1-3:1.0` | 底部/低位相机 | `captures/cam_1/` |
| **2号机** | `/dev/camera_2` | `1-4:1.0` | 侧面俯视滑槽相机 | `captures/cam_2/` |
| **3号机** | `/dev/camera_3` | `1-6:1.0` | 正视传送带纸箱相机 | `captures/cam_3/` |
| **4号机** | `/dev/camera_4` | `1-7:1.0` | 侧视传送带相机 | `captures/cam_4/` |
| **5号机** | `/dev/camera_5` | `1-8:1.0` | 传送带/滑槽近景相机 | `captures/cam_5/` |

### 8002 机器 (4路相机，1号机留空)
| 相机编号 | 软链接 | 物理 USB 端口 | 工位视角角色 | 保存归档目录 |
| :--- | :--- | :--- | :--- | :--- |
| **1号机** | `/dev/camera_1` | *（留空未接）* | 底部/低位相机 [未接入/留空] | `captures/cam_1/` (预留) |
| **2号机** | `/dev/camera_2` | `1-6:1.0` | 侧面俯视滑槽相机 | `captures/cam_2/` |
| **3号机** | `/dev/camera_3` | `1-1:1.0` | 正视传送带纸箱相机 | `captures/cam_3/` |
| **4号机** | `/dev/camera_4` | `1-5:1.0` | 侧视传送带相机 | `captures/cam_4/` |
| **5号机** | `/dev/camera_5` | `1-3:1.0` | 传送带/滑槽近景相机 | `captures/cam_5/` |

---

## 二、测试命令说明

### 1. 查看当前相机拓扑与连接状态
```bash
python3 camera_id_api.py
```

### 2. 所有已接入相机各拍一张测试照（独立目录分类归档）
```bash
# 默认 1.8s 推流预热（100% 杜绝 JPEG 截断花斑与残帧）
python3 camera_id_api.py --all

# 0 秒实验模式（零等待即时快照）
python3 camera_id_api.py --all --stream-time 0
```

### 3. 单独测试指定编号相机
```bash
python3 camera_id_api.py --cam 2
python3 camera_id_api.py --cam 2 --stream-time 0
```

### 4. 业务组长遍历扫码模式（带命中单号自动返回）
```bash
python3 camera_id_api.py --scan
```

---

## 三、算法与业务代码导入调用方式 (Python)

```python
from camera_id_api import get_camera_frame, scan_camera, scan_cameras_in_order

# 1. 明确按编号拍摄指定相机画面 (如 2号机)
# stream_seconds 可传入 1.8 (防花斑稳定) 或 0 (零等待极速快照)
frame = get_camera_frame(cam_id=2, stream_seconds=1.8)
if frame is not None:
    # 处理高清帧 4000x3000
    pass

# 2. 对指定相机直接扫码 (如 3号机)
waybill_id = scan_camera(cam_id=3)
if waybill_id:
    print(f"识别到单号: {waybill_id}")

# 3. 业务组长逻辑：根据头部空间坐标指定相机优先级遍历扫码
# 自动跳过留空/未连接的机位，扫到即返回 (命中相机ID, 命中单号)
res = scan_cameras_in_order(cam_ids=[2, 3, 4, 5])
if res:
    cam_id, waybill_id = res
    print(f"在 {cam_id} 号相机成功匹配到包裹单号: {waybill_id}")
```
