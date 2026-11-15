# SF-Vision: 顺丰物流视觉感知与机器人协同系统 (码垛与分拣)

本仓库整合了顺丰智慧物流场景下两大核心视觉业务模块——**智能码垛（Palletizing/Depalletizing）** 与 **高速分拣（Sorting）** 的视觉算法、工控机部署服务与相机硬件驱动。

---

## 目录结构与模块说明

```text
sf-vision/
├── palletizing/                     # 【码垛场景】笼车、托盘与包裹垛型视觉处理
│   ├── infer_seg_depth_demo.py      # 点云与深度图融合分割推理演示（YOLO-seg + 深度切片）
│   └── stack_cage_dataset.yaml      # 笼车/垛型多目标检测与分割数据集配置
│
├── sorting/                         # 【分拣场景】高速供包台/分拣工作站视觉服务
│   ├── SFsorting.py                 # 当前生产级分拣算法服务（含三级点云缓存、长驻常驻模型、吸附点解算）
│   ├── SFsorting_fusion_1105.py     # 深度/彩色融合与双相机多模态位姿解算
│   └── history/                     # 历史版本追溯 (SFsorting_1022.py, SFsorting_minimal_qr_1026.py)
│
├── hardware/                        # 【相机与硬件接口】
│   ├── camera_id_api.py             # 多路 USB/工业相机持久化拓扑与序列号映射 API
│   ├── barcode_engine.py            # 高分辨率面单条码与二维码解码引擎
│   ├── sweep_cam3_focus_full.py     # 相机对焦及多档分辨率（高达 4000x3000）扫描测试工具
│   ├── usb_camera_manager.py        # USB 相机生命周期与守护进程管理
│   ├── usb_scan_service.py          # 独立扫码子服务
│   └── udev/                        # Linux udev 静态端口绑定规则 (99-usb-cameras.rules 等)
│
├── config/                          # 【运行配置】
│   ├── cameras.json                 # 多相机内参、视场角参数与拓扑配置文件
│   └── calibration.example.json     # 标定矩阵与世界坐标转换模板
│
├── docs/                            # 【技术方案与标准作业程序 (SOP)】
│   ├── 分拣系统坐标转换与流程设计.md # 机器人坐标系、相机坐标系与吸附点解算标准文档
│   ├── 分拣脚本更新工作流.md         # 算法热更新、版本回退与生产测试验证规范
│   ├── NEW_WORKSTATION_SOP.md       # 新工位部署 SOP
│   └── DEPLOYMENT_BASELINE.md       # 生产环境基线参数与软硬件依赖
│
├── deploy.sh                        # Docker 容器化一键部署与服务管理入口
└── runtime/                         # 容器运行期挂载脚本与核心入口
```

---

## 核心业务场景与技术架构

### 1. 智能码垛场景 (Palletizing)
- **视觉目标**：识别装笼、托盘及垛位中的不同规格包裹，提取表面点云几何平面，计算最优抓取吸附位姿。
- **专用模型**：训练 `stack_cage_yolo26n` 针对高反光编织袋、笼车网格边缘进行针对性分割，实现 mAP50 99.5% 高可靠度识别。
- **相机选型**：顶置大视场深度相机（Femto Bolt）获取全局垛型点云，辅以高分辨率面单扫码相机识别运单信息。

### 2. 高速分拣场景 (Sorting)
- **视觉目标**：实时处理来料包裹，返回分类类别、包裹三维中心坐标以及 8 顶点三维轮廓 (`box_8pts_world`)。
- **性能优化**：
  - **三级点云缓存机制**：内存高速缓存 (`_PLY_CACHE`) -> 磁盘降采样体素缓存 (`.voxel{mm}.npy`) -> 原始点云异步加载，彻底消除大点云引起的工控机内存崩溃与服务超时。
  - **常驻模型预热**：彻底解决每次请求重新加载 YOLO 带来的 6-7 秒服务卡顿。
  - **背景抗干扰优化**：通过更换专用哑光对比度工作台画布，消除边界反射噪点；结合收缩掩膜点云切片 (`get_shrunk_mask_prism_fit_plane`) 杜绝多包裹粘连引起的点云收敛误差。

---

## 快速部署与使用

本仓库部署入口为 `deploy.sh`，基于 Docker 容器化环境运行：
```bash
# 启动相机与视觉分拣服务
bash deploy.sh start <CAMERA_SERIAL>
bash deploy.sh server start

# 运行客户端测试视觉反馈数据
bash deploy.sh server client
```
详细配置与故障排查请参阅 [docs/NEW_WORKSTATION_SOP.md](docs/NEW_WORKSTATION_SOP.md)。

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
git clone http://gkjs.synology.me:5080/sf-express/sf-vision.git
cd sf-vision
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
