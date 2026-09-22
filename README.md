# Ubuntu 24.04：Femto Bolt 开机发布彩色、深度、点云

此包基于 `W:\Github\OrbbecSDK_ROS2` 的相机驱动源码，
宿主机 Ubuntu 24.04，容器 Ubuntu 22.04 + ROS 2 Humble。
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
| 彩色图 | `/vision/top_camera/rgb` |
| 深度图 | `/vision/top_camera/depth` |
| XYZ 点云 | `/vision/top_camera/points` |
彩色,深度话题设为 1280×720 @ 30 FPS；

```bash
bash deploy.sh topics
bash deploy.sh verify
```

验收脚本用 sensor-data QoS 同时订阅三路数据 30 秒，要求每路至少 5 帧非空数据、
时间戳递增且结束前 3 秒内仍收到数据，打印尺寸、编码、frame_id 和实际接收频率。
PASS 仅证明持续收到了消息；还需实际观察图像内容和点云几何，不能代替标定精度验收。


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
发行版互通当作已验证能力，优先运行Humble 环境。

- [ROS 2 Humble 官方安装说明（Ubuntu 22.04）](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Orbbec 官方仓库](https://github.com/orbbec/OrbbecSDK_ROS2)
