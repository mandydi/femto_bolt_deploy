# 工控机视觉相机驱动与开机自启部署标准 SOP (沉淀指南)

本指南总结了机器人工控机（包括机器人 1、机器人 2 及新工控机）上各类相机驱动的部署逻辑、关键配置要点与一键部署脚本的使用方法。

---

## 一、 核心要点与排坑经验

### 1. 奥比中光 (Orbbec) USB 3.0 相机 (如 Gemini 305 / 330 系列)
* **Linux 权限与 udev 规则**：
  - 非 root 用户运行 ROS 2 驱动时，必须将 `/etc/udev/rules.d/99-obsensor-libusb.rules` 安装到位。
  - 驱动默认将设备所有权赋予 `video` 组。必须将当前运行用户（如 `caseim`）加入 `video` 和 `plugdev` 组：
    ```bash
    sudo usermod -aG video,plugdev $USER
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
* **多相机绑定与硬件识别**：
  - 双臂或多路 USB 相机由于开机 USB 枚举顺序不固定，**严禁使用总线索引**，必须通过**序列号 (Serial Number)** 绑定相机与 ROS 2 节点（参数 `serial_number:=<SN>`）。
* **开机自启防抢占与冷启动保护**：
  - systemd 服务启动前需加入硬件在线轮询守卫：
    ```ini
    ExecStartPre=/bin/bash -c 'until grep -qsx "<SERIAL>" /sys/bus/usb/devices/*/serial; do sleep 1; done'
    ```

### 2. 奥比中光千兆网口相机 (如 Gemini 435Le 头部相机)
* **IP 规划与网卡绑定**：
  - 相机默认 IP 通常为 `192.168.1.10:8090`。
  - 工控机相连的网卡（如 `enp2s0`）必须配置为同一网段静态 IP（如 `192.168.1.23/24`）。
* **开机自启双就绪守卫**：
  - 启动驱动前必须确保 DDS 网卡 IP（如 `192.168.1.22`）和相机网卡 IP（`192.168.1.23`）均已 UP，否则 CycloneDDS 绑定失败或 SDK 广播找不到相机：
    ```ini
    ExecStartPre=/bin/bash -c 'until ip addr show | grep -q "192.168.1.22" && ip addr show | grep -q "192.168.1.23"; do sleep 1; done'
    ```

### 3. 海康威视 USB 3.0 工业相机 (如 MV-CB016-10UC-S 双手相机)
* **USBFS 内存限制扩展**：
  - 双路高分辨率 USB 3.0 相机同时采集会超过 Linux 默认的 16MB usbfs 缓冲上限，必须开机执行 `/opt/MVS/bin/set_usbfs_memory_size.sh`（已封装为 `hik-usbfs.service`）。
* **相机内部硬转码**：
  - 硬件默认输出 Bayer 格式，若在 CPU 做软转码开销达 150%+。驱动节点需下发 `PixelType_Gvsp_RGB8_Packed` 让相机 ISP 完成硬件转码，驱动直接 zero-copy memmove 到 ROS 消息 buffer。

### 4. ROS 2 通信域 (DDS Domain ID)
* 新工控机（`gkjs-ai`）运行在 **`ROS_DOMAIN_ID=16`**（机器人 2 为 `8`，机器人 1 为 `6`），底层采用 CycloneDDS。
* 任何诊断、`ros2 topic list` 或 `ros2 topic hz` 必须显式声明环境：
  ```bash
  export ROS_DOMAIN_ID=16
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
  source /opt/ros/humble/setup.bash
  ```

---

## 二、 一键部署脚本使用方法 (`deploy_camera.sh`)

脚本已部署在工控机路径：`/home/caseim/comorobot_ws/tools/deploy_vision/deploy_camera.sh`

### 1. 交互式模式（带菜单向导）
直接运行脚本，脚本会自动检测 udev 规则、扫描总线上的全部设备与序列号，并引导创建 systemd 服务：
```bash
sudo bash /home/caseim/comorobot_ws/tools/deploy_vision/deploy_camera.sh
```

### 2. 命令行静默部署模式
适用于批量脚本化安装：

* **部署奥比中光 USB 深度相机（如右臂/左臂）**：
  ```bash
  sudo bash /home/caseim/comorobot_ws/tools/deploy_vision/deploy_camera.sh \
      --type orbbec-usb \
      --name right_arm_rgbd \
      --serial CV2T66100016 \
      --domain 16
  ```

* **部署奥比中光千兆网口头部相机**：
  ```bash
  sudo bash /home/caseim/comorobot_ws/tools/deploy_vision/deploy_camera.sh \
      --type orbbec-net \
      --domain 16
  ```

* **仅扫描并列出系统相机与序列号**：
  ```bash
  sudo bash /home/caseim/comorobot_ws/tools/deploy_vision/deploy_camera.sh --scan
  ```

* **仅安装 Orbbec udev 规则与权限**：
  ```bash
  sudo bash /home/caseim/comorobot_ws/tools/deploy_vision/deploy_camera.sh --install-udev-only
  ```

---

## 三、 标准 ROS 2 话题命名契约

| 相机部位 | 颜色图像 (Color) | 压缩图像 (Compressed) | 深度图像 (Depth) | 内参信息 (CameraInfo) |
| :--- | :--- | :--- | :--- | :--- |
| **头部相机 (Head)** | `/vision/camera/head_rgbd/color/image_raw` | `/vision/camera/head_rgbd/color/image_raw/compressed` | `/vision/camera/head_rgbd/depth/image_raw` | `/vision/camera/head_rgbd/color/camera_info` |
| **左臂/左手 (Left)** | `/vision/camera/left_hand_rgbd/color/image_raw` (或 `left_arm_rgbd`) | `.../color/image_raw/compressed` | 视硬件（海康无，奥比有） | `.../color/camera_info` |
| **右臂/右手 (Right)**| `/vision/camera/right_hand_rgbd/color/image_raw` (或 `right_arm_rgbd`) | `.../color/image_raw/compressed` | 视硬件（海康无，奥比有） | `.../color/camera_info` |

---

## 四、 常用维护与验证命令速查

```bash
# 1. 检查开机自启与运行状态
systemctl status orbbec-camera.service hik-cameras.service webrtc-server.service

# 2. 检查日志流
journalctl -u orbbec-camera.service -f
journalctl -u hik-cameras.service -f

# 3. 实时测试话题发布频率 (注意 DOMAIN_ID)
ROS_DOMAIN_ID=16 ros2 topic hz /vision/camera/head_rgbd/color/image_raw
ROS_DOMAIN_ID=16 ros2 topic hz /vision/camera/left_hand_rgbd/color/image_raw/compressed
ROS_DOMAIN_ID=16 ros2 topic hz /vision/camera/right_hand_rgbd/color/image_raw/compressed
```
