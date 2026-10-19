# Ubuntu 24.04：Femto Bolt 开机发布彩色、深度、点云

此包基于 \W:\Github\OrbbecSDK_ROS2\ 的相机驱动源码，以及
\ision_rightarm/launch/vision_rightarm_launch.py\ 的命名空间结构制作。
宿主机 Ubuntu 24.04，容器 Ubuntu 22.04 + ROS 2 Humble；宿主机无需安装 ROS。
只构建 \orbbec_camera_msgs\、\orbbec_camera\，不安装视觉检测、模型或 CUDA。
完整压缩包包含驱动源码与 SDK，首次安装仍需联网下载 Docker、基础镜像和 apt 依赖。

## 1. 安装环境

将整个部署包解压到工控机，例如 \~/femto_bolt_deploy\。用有 sudo 权限的普通用户执行：

\\\ash
cd ~/femto_bolt_deploy
bash deploy.sh install
\\\

支持 x86_64 / aarch64，默认编译并行度 2；内存较小时可用 \BUILD_JOBS=1 bash deploy.sh install\。
如果下载 Docker Hub 或 apt 失败，保留具体报错后处理网络；安装失败时脚本会退出。

## 2. 获取序列号并启动

给 Femto Bolt 接好电源和 USB 3 数据线，确认没有其他相机进程占用设备：

\\\ash
bash deploy.sh devices
# 从输出中找 Femto Bolt 的 Serial Number，替换下方示例文字：
bash deploy.sh start YOUR_FEMTO_BOLT_SERIAL
bash deploy.sh logs
\\\

Ctrl+C 只退出日志查看，不会停止相机。查询 SN 的命令如果持续运行，读到 SN 后 Ctrl+C 退出。
序列号只需部署时查一次，Docker 保存该配置，后续每次开机自动按此 SN 连接。
不会使用原来的 Gemini 305 序列号，也不会在目标未连接时选其他相机。

\install\ 启用 Docker 开机启动；\start\ 创建 \unless-stopped\ 重启策略的容器。
相机进程退出会结束 launch，让 Docker 重启；等待 USB/重连由已有驱动处理。
这不是话题断流看门狗，进程存活但流异常时应查看日志、重新启动并复测。

## 3. 话题与验收

| 输出 | 话题 |
|---|---|
| 彩色图 | \/vision/camera/right_arm_rgbd/color/image_raw\ |
| 深度图 | \/vision/camera/right_arm_rgbd/depth/image_raw\ |
| XYZ 点云 | \/vision/camera/right_arm_rgbd/depth/points\ |

保留原右臂话题前缀。彩色设为 1280×720 @ 30 FPS，原生深度设为 640×576 @ 30 FPS；
深度使用 SW 对齐彩色，发布尺寸以验收输出为准。点云为 XYZ，不含 RGB。

\\\ash
bash deploy.sh topics
bash deploy.sh verify
\\\

验收脚本用 sensor-data QoS 同时订阅三路数据 30 秒，要求每路至少 5 帧非空数据、
时间戳递增且结束前 3 秒内仍收到数据，打印尺寸、编码、frame_id 和实际接收频率。
PASS 仅证明持续收到了消息；还需实际观察图像内容和点云几何，不能代替标定精度验收。

初次三路均 PASS 后，重启工控机（先保存其他工作）：

\\\ash
sudo reboot
# 重新登录后：
cd ~/femto_bolt_deploy
bash deploy.sh status
bash deploy.sh verify
\\\

只有重启后无需手动 start，三路仍 PASS，才算开机自启实机验收完成。

## 4. 维护与连接其他 ROS 节点

\\\ash
bash deploy.sh stop      # 停止并保持停用；之后重启主机也不会自动启动
bash deploy.sh resume    # 恢复运行和后续开机自启
bash deploy.sh logs
# 更换相机 SN 或 ROS_DOMAIN_ID：
bash deploy.sh stop
bash deploy.sh remove    # 仅删除本部署容器，保留镜像、源码
bash deploy.sh start YOUR_FEMTO_BOLT_SERIAL 0
\\\

默认 \ROS_DOMAIN_ID=0\，使用主机网络、主机 IPC。其他 Humble 节点使用相同 domain；
跨机器还需网络允许 DDS 发现及数据流。宿主 24.04 若原生安装 Jazzy，不能把跨 ROS
发行版互通当作已验证能力，优先让消费者也运行在 Humble 环境。

容器映射 USB 总线及存在的 \/dev/dri\ 图形设备，不使用 \--privileged\。
Femto Bolt SDK 的 depthengine 链接 \libEGL.so.1\，镜像已补齐 Mesa/EGL 运行库；
若工控机仅有 NVIDIA 专有驱动或无可用图形设备，需要根据实际 EGL 初始化日志
检查其容器 GPU 运行环境，尚未验证所有 GPU 配置。日志轮转上限约 30 MB。
原有右臂识别服务与 launch 文件不修改。若有其他服务占用相机，先确认来源再停止。

## 5. 顺丰现场集成：3D 点云识别与法向量提取 (\sf_sorting\)

现场配套集成了包裹分拣的 3D 点云识别流水线：
- **位置**：\sf_sorting/SFsorting.py\
- **主要功能**：
  1. 订阅 Femto Bolt 发布的点云与图像话题；
  2. 结合 YOLO 实例分割与深度点云，定位目标包裹表面；
  3. 提取指定近距离范围内最近物体的表面法向量及相机三维坐标 \(X, Y, Z)\；
  4. 支持与机械臂抓取/吸盘位姿控制协议（如 ASCS 协议）对接。

## 6. 顺丰智能分拣视觉感知系统与服务工作流架构

全链路视觉分拣管线现已实现毫秒级（~0.3s - 0.7s）实时响应，测试工作流与实际工程生产工作流已完全解耦统一：

### 1. 服务生命周期管理 (Server Lifecycle)
\\\ash
# 启动常驻分拣计算服务 (sf_sorting_server)
bash deploy.sh server start

# 查看服务运行状态
bash deploy.sh server status

# 实时查看服务端日志 (包含帧同步、YOLO检测、3D拟合耗时明细)
bash deploy.sh server logs

# 停止分拣服务
bash deploy.sh server stop
\\\

### 2. 实际工程生产调用 (Production Workflow)
供生产调度、PLC、机械臂工控程序自动化集成调用：
\\\ash
# 调用服务，纯净返回标准 JSON (无任何控制台多余噪音，便于管道/外部脚本捕获)
bash deploy.sh server client
# 或快捷别名:
bash deploy.sh client
# 兼容旧工程脚本:
bash call_sorting_client.sh
\\\

**返回 JSON 结构示例**：
\\\json
{
   success: true,
  timestamp: 1792389913.065,
  best_label: carton,
  x: -443.86,
  y: 137.60,
  z: 1018.00,
  quad_3d: [
    [-614.13, -112.51, 922.15],
    [-355.29, -198.46, 1117.37],
    [-248.93, 418.30, 1160.59],
    [-515.47, 441.91, 963.23]
  ],
  theta_x_deg: 36.3310,
  theta_y_deg: 3.2485,
  timing: {
    wait_frame_ms: 0.0,
    parse_ms: 13.7,
    yolo_ms: 164.1,
    fit_3d_ms: 130.4,
    server_total_ms: 308.3
  }
}
\\\

### 3. 测试与性能诊断工作流 (Testing & Benchmark Workflow)
供现场算法工程师一键评估全链路时延与点云拟合精度：
\\\ash
# 一键触发测试诊断，打印格式化性能耗时仪表盘与抓取位姿详情
bash deploy.sh server test
# 或快捷别名:
bash deploy.sh test
\\\
