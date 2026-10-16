# Ubuntu 24.04：Femto Bolt 开机发布彩色、深度、点云

此包基于 `W:\Github\OrbbecSDK_ROS2` 的相机驱动源码，以及
`vision_rightarm/launch/vision_rightarm_launch.py` 的命名空间结构制作。
宿主机 Ubuntu 24.04，容器 Ubuntu 22.04 + ROS 2 Humble；宿主机无需安装 ROS。
只构建 `orbbec_camera_msgs`、`orbbec_camera`，不安装视觉检测、模型或 CUDA。
完整压缩包包含驱动源码与 SDK，首次安装仍需联网下载 Docker、基础镜像和 apt 依赖。

## 1. 安装环境

将整个部署包解压到工控机，例如 `~/femto_bolt_deploy`。用有 sudo 权限的普通用户执行：

```bash
cd ~/femto_bolt_deploy
bash deploy.sh install
```

支持 x86_64 / aarch64，默认编译并行度 2；内存较小时可用 `BUILD_JOBS=1 bash deploy.sh install`。
如果下载 Docker Hub 或 apt 失败，保留具体报错后处理网络；安装失败时脚本会退出。

## 2. 获取序列号并启动

给 Femto Bolt 接好电源和 USB 3 数据线，确认没有其他相机进程占用设备：

```bash
bash deploy.sh devices
# 从输出中找 Femto Bolt 的 Serial Number，替换下方示例文字：
bash deploy.sh start YOUR_FEMTO_BOLT_SERIAL
bash deploy.sh logs
```

Ctrl+C 只退出日志查看，不会停止相机。查询 SN 的命令如果持续运行，读到 SN 后 Ctrl+C 退出。
序列号只需部署时查一次，Docker 保存该配置，后续每次开机自动按此 SN 连接。
不会使用原来的 Gemini 305 序列号，也不会在目标未连接时选其他相机。

`install` 启用 Docker 开机启动；`start` 创建 `unless-stopped` 重启策略的容器。
相机进程退出会结束 launch，让 Docker 重启；等待 USB/重连由已有驱动处理。
这不是话题断流看门狗，进程存活但流异常时应查看日志、重新启动并复测。

## 3. 话题与验收

| 输出 | 话题 |
|---|---|
| 彩色图 | `/vision/camera/right_arm_rgbd/color/image_raw` |
| 深度图 | `/vision/camera/right_arm_rgbd/depth/image_raw` |
| XYZ 点云 | `/vision/camera/right_arm_rgbd/depth/points` |

保留原右臂话题前缀。彩色设为 1280×720 @ 30 FPS，原生深度设为 640×576 @ 30 FPS；
深度使用 SW 对齐彩色，发布尺寸以验收输出为准。点云为 XYZ，不含 RGB。

```bash
bash deploy.sh topics
bash deploy.sh verify
```

验收脚本用 sensor-data QoS 同时订阅三路数据 30 秒，要求每路至少 5 帧非空数据、
时间戳递增且结束前 3 秒内仍收到数据，打印尺寸、编码、frame_id 和实际接收频率。
PASS 仅证明持续收到了消息；还需实际观察图像内容和点云几何，不能代替标定精度验收。

初次三路均 PASS 后，重启工控机（先保存其他工作）：

```bash
sudo reboot
# 重新登录后：
cd ~/femto_bolt_deploy
bash deploy.sh status
bash deploy.sh verify
```

只有重启后无需手动 start，三路仍 PASS，才算开机自启实机验收完成。

## 4. 维护与连接其他 ROS 节点

```bash
bash deploy.sh stop      # 停止并保持停用；之后重启主机也不会自动启动
bash deploy.sh resume    # 恢复运行和后续开机自启
bash deploy.sh logs
# 更换相机 SN 或 ROS_DOMAIN_ID：
bash deploy.sh stop
bash deploy.sh remove    # 仅删除本部署容器，保留镜像、源码
bash deploy.sh start YOUR_FEMTO_BOLT_SERIAL 0
```

默认 `ROS_DOMAIN_ID=0`，使用主机网络、主机 IPC。其他 Humble 节点使用相同 domain；
跨机器还需网络允许 DDS 发现及数据流。宿主 24.04 若原生安装 Jazzy，不能把跨 ROS
发行版互通当作已验证能力，优先让消费者也运行在 Humble 环境。

容器映射 USB 总线及存在的 `/dev/dri` 图形设备，不使用 `--privileged`。
Femto Bolt SDK 的 depthengine 链接 `libEGL.so.1`，镜像已补齐 Mesa/EGL 运行库；
若工控机仅有 NVIDIA 专有驱动或无可用图形设备，需要根据实际 EGL 初始化日志
检查其容器 GPU 运行环境，尚未验证所有 GPU 配置。日志轮转上限约 30 MB。
原有右臂识别服务与 launch 文件不修改。若有其他服务占用相机，先确认来源再停止。

## 验证状态与依据

当前为离线准备的部署包，未在新工控机或真实 Femto Bolt 上构建、启动、重启验收。
已通过 Bash/Python 语法检查，并在 WSL 临时副本中验证了 SDK 符号链接恢复，
核对了 depthengine 的 ELF 依赖。最终以目标工控机的构建及上述实际订阅结果为准。

- 原始脚本：`vision_rightarm/launch/vision_rightarm_launch.py`
- 型号参数：本仓库 `orbbec_camera/launch/femto_bolt.launch.py`
- [ROS 2 Humble 官方安装说明（Ubuntu 22.04）](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Orbbec 官方仓库](https://github.com/orbbec/OrbbecSDK_ROS2)

## 5. 顺丰现场集成：3D 点云识别与法向量提取 (`sf_sorting`)

现场配套集成了包裹分拣的 3D 点云识别流水线：
- **位置**：`sf_sorting/SFsorting.py`
- **主要功能**：
  1. 订阅 Femto Bolt 发布的点云与图像话题；
  2. 结合 YOLO 实例分割与深度点云，定位目标包裹表面；
  3. 提取指定近距离范围内最近物体的表面法向量及相机三维坐标 `(X, Y, Z)`；
  4. 支持与机械臂抓取/吸盘位姿控制协议（如 ASCS 协议）对接。

