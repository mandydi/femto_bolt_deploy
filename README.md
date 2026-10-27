# Femto Bolt 相机与八点分拣服务部署

本仓库的部署入口为 `deploy.sh`。2026-10-26 根据 **8001 正在运行的环境**重新整理，见 [基线与验证记录](docs/DEPLOYMENT_BASELINE.md)。

新设备按 [固定部署与接相机验收流程](docs/NEW_WORKSTATION_SOP.md) 执行。私有包放入仓库内被 Git 忽略的 `private/femto-YYYYMMDD/` 后，可运行 `bash deploy.sh prepare "$PWD/private/femto-YYYYMMDD"` 一次完成安装、导入和离线检查。

新机未标定时先用 `bash deploy.sh server start --diagnostic`、`bash deploy.sh server client` 验证相机到分拣服务的回包；返回的坐标仅在相机系，不能交给机器人。现场三路静态采样可运行 `bash save_head_camera.sh`。另接的 Fic760x USB 相机可运行 `bash save_usb_camera.sh`，保存 4000×3000 JPEG 到 `captured_usb_camera/`；脚本会先关闭占用设备的 Cheese。

它部署相机和视觉服务，不连接机器人、不发送动作。输入为彩色/深度/点云，输出为包裹类别、中心及八个三维顶点。**代码可从 GitHub 获取；模型、镜像和现场资产通过私有迁移包传递，不存入公开仓库。克隆仓库不是完整部署。**

## 推荐：离线迁移到新工控机

已验证的参考平台为 x86_64、Docker、容器内 Ubuntu 22.04 + ROS 2 Humble。新主机先核对 Ubuntu 版本、磁盘、USB 3 和相机型号；宿主无需安装 ROS。ARM 的驱动源码存在，但本次完整分拣迁移未验证 ARM。

### 1. 在 8001 导出一次私有迁移包

在独立的新检出目录执行，不要覆盖正在挂载的生产目录：

```bash
bash tools/export_8001.sh /home/caseim/deployment_exports/femto-YYYYMMDD --images
```

输出目录必须不存在。导出读取两个容器的镜像 ID，并以 `docker commit --pause=false` 保存分拣容器的可写层（其中有 numba），**不停止或重启生产容器**。它仍会消耗磁盘/CPU，建议现场空闲时执行。镜像包含 x86_64 二进制，只用于同架构迁移。迁移包包含：

- `images.tar.gz`：实际相机镜像和分拣容器快照，通常数 GB；
- `image-ids.json`：镜像 ID，避免同名标签漂移；
- `assets/packet_best.pt`、`assets/mask1.png`、哈希清单；
- `observed-pip-freeze.txt`、`SHA256SUMS`。

用公司批准的文件传输方式复制整个目录到新机器，不上传公开 Git/Release。不需要复制生产图片、数据集、凭据或 FRP 配置。

### 2. 在新工控机安装并导入

```bash
git clone https://github.com/mandydi/femto_bolt_deploy.git
cd femto_bolt_deploy
bash deploy.sh install
bash deploy.sh import /absolute/path/to/femto-YYYYMMDD
bash deploy.sh devices
```

`install` 安装 Docker 和仓库提供的 Orbbec udev 规则；需要 apt 网络与 sudo。已有 Docker 不替换安装方式。导入校验哈希，不自动覆盖已有 assets。不要在 8001 的生产目录执行新 start/import。

### 3. 启动相机并验收

接好 Femto Bolt 电源与 USB 3，记录新相机 SN：

```bash
bash deploy.sh start NEW_CAMERA_SERIAL
bash deploy.sh logs
# 另一终端：
bash deploy.sh verify
```

默认 `ROS_DOMAIN_ID=0`。多套工位应规划不同 domain，客户端与服务必须相同，例如每次命令前 `export ROS_DOMAIN_ID=17`。SSH/FRP 不会自动转发 ROS 2 DDS 数据，跨主机调用还需现场局域网互通。

固定基线：彩色 3840×2160@30，原生深度 640×576@30，SW D2C 对齐；以实际消息为准。发布：

- `/vision/top_camera/rgb`
- `/vision/top_camera/depth`
- `/vision/top_camera/points`

`verify` 观察 15 秒，要求各路至少5帧、时间戳递增、末3秒仍有消息。PASS 仅证明数据连续，仍需检查画面、深度与点云几何。

### 4. 配置新工位的几何关系，再启动分拣

```bash
cp config/calibration.example.json config/calibration.json
# 用实际标定结果填写 camera_matrix、dist_coeffs、camera_to_world_mm、world_frame。
# 为新工位重新生成 assets/mask1.png，并更新 assets/manifest.json 中对应 SHA256。
# 独立几何验证通过后，才将 validated_for_deployment 改为 true。
bash deploy.sh check
bash deploy.sh server start
bash deploy.sh server logs
# 等待初始化/JIT/相机同步，再调用：
bash deploy.sh server client
```

**示例参数来自旧工位，默认禁止分拣启动。不能复制旧外参后只改 validated=true 就交给机器人。** `camera_to_world_mm` 将相机坐标（mm）变到已确认世界坐标（mm）；工作区 mask 必须匹配3840×2160图像。更改分辨率须重新验证算法与内参。模型为 CPU 推理，实际耗时需新设备测量。

客户端输出 JSON，必须检查 `success`。成功字段为：

```json
{"success":true,"best_label":"carton","center_world":[0,0,0],"box_8pts_world":[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]],"waybill_dict":{},"timing":{}}
```

上例仅展示结构，零坐标不代表有效结果。实际中心为3个有限数，轮廓为8×3有限数，单位mm。`world_frame` 来自设备配置，消费者必须与配置同时管理。八点语义、顺序见 runtime/SFsorting.py 的 box_8pts 构造；对接机器人前用已知物体核验，不把它当旧四点 quad_3d。2026-10-27 已同步 8001 的面单二维码识别分支：仅在检测到的二维码与所选包裹重叠并成功解码时填写 `waybill_dict`，否则为 `{}`；这不等于已经向顺丰平台查询。

### 5. 重启验收

在允许重启的窗口保存其他工作后重启。重新登录，**不手动 start**，验证：

```bash
bash deploy.sh status
bash deploy.sh server status
bash deploy.sh verify
bash deploy.sh server client
```

相机、分拣容器都有 unless-stopped 策略；手动 stop 后重启不会恢复，需要 resume。容器创建成功不等于服务就绪。

## 维护

```bash
bash deploy.sh stop                # 相机停用
bash deploy.sh resume              # 相机恢复
bash deploy.sh server stop
bash deploy.sh server resume
```

脚本不强制删除同名容器。更换配置/挂载需要明确 stop、remove、start；`remove` 只删除已停止容器，不删除源码与资产。新容器名为 femto-camera/femto-sorting，不沿用生产容器名，**仍不得在同一相机上重复启动第二套驱动**。

## 从源码重建（备用，尚未完成干净构建验收）

```bash
bash deploy.sh install
bash deploy.sh build
```

Dockerfile 构建驱动，Dockerfile.sorting 按 requirements.sorting.txt 及观察到的 CPU torch/torchvision 版本构建算法环境。包源版本是否仍可下载、全新镜像兼容性尚未验证；失败应保留原始错误，不临时换版本后宣称等价。近期交付优先使用实机快照。

旧 `CAMERA_DEPLOYMENT_SOP.md` 是其他机器人原生 ROS 工作空间的历史资料，不能作为本 Docker 工位的执行指南。旧部署脚本只在 tools/deploy_legacy.sh 留存供比对，已经归档，不再作为执行入口。根目录旧 SFsorting.py 和 sf_sorting/ 同为历史版本；本入口只运行 runtime/SFsorting.py。
