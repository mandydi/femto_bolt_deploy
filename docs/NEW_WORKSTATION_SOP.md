# 新工控机固定部署与接相机验收

适用：与 8001 同为 x86_64、Ubuntu 22.04 的 Femto Bolt 工位。仓库是代码，私有迁移包包含镜像与模型；单独克隆仓库不能启动视觉服务。本流程不触发机器人动作。

## 交付前一次性准备

1. 在 8001 按 README 的 `tools/export_8001.sh ... --images` 导出私有包，并保留一份经 SHA-256 校验的副本。不要每装一台就重新导出生产容器。
2. 优先用现场移动硬盘复制数 GiB 的迁移包。远程 FRP 链路只作备选：8009 本次约 2.48 GiB 镜像传输约一小时，耗时主要在传输，并非安装或导入。
3. 私有包不传公开 GitHub。新机上把它放在仓库内的 `private/femto-YYYYMMDD/`；`private/` 已被 Git 忽略。

## 新机一次性部署

```bash
git clone https://github.com/mandydi/femto_bolt_deploy.git
cd femto_bolt_deploy
# 从移动硬盘复制私有包到 ./private/femto-YYYYMMDD/，检查含 SHA256SUMS、images.tar.gz、assets/
bash deploy.sh prepare "$PWD/private/femto-YYYYMMDD"
```

`prepare` 顺序执行 Docker/udev 安装、包哈希校验与镜像导入、资产校验、隔离容器依赖和模型加载；任何一步失败即停止。它复制未验证的标定模板，但不启动相机或分拣服务。完成后记录 `git rev-parse HEAD`、镜像 ID 与 PASS 输出。确认镜像和资产仍在且报告已保存后，可以删除仓库内 `private/femto-YYYYMMDD/`；公共仓库和 Docker 镜像不受影响。源端保留迁移包，以备别的设备重装。

若某步骤曾成功而后续失败，不重复运行 `prepare`。依据首个失败点修复，然后分别执行 `bash deploy.sh install`、`bash deploy.sh import 绝对路径` 或 `bash deploy.sh offline-check`；`import` 会拒绝覆盖已导入资产。

## 现场接相机

1. 先接 Femto Bolt 电源，再接工控机 USB 3 口。重新插拔一次以应用 udev 规则。
2. 在仓库执行 `bash deploy.sh devices`，记录它识别到的实际 SN。若显示多台，按机身标签和拔插前后列表确认目标机；不能沿用 8001 的 SN。无设备时先查电源、线缆、USB 口和 `lsusb`，不要猜 SN 启动。
3. 执行 `bash deploy.sh start 实际SN`，再运行 `bash deploy.sh status` 与 `sudo docker logs --tail 100 femto-camera`。要求容器运行、日志识别到同一 SN，且没有持续 USB/启动错误。
4. 执行 `bash deploy.sh verify`。要求 RGB、depth、points 三路均 `PASS`：15 秒内各至少五帧、时间戳递增、末三秒仍有新帧。若任一路 FAIL，保留原始日志和帧数，不算通过。8001 的一次抽样出现 RGB 后三秒无新帧，新机必须独立复测。
5. 在现场查看 RGB、深度、点云和已知物体的位置关系。脚本的三路 PASS 只能证明有连续数据，不能证明几何正确。

需要留存现场输入时，运行 `bash save_head_camera.sh`。它从当前 `femto-camera` 的 ROS 域订阅一组三路时间戳相差不超过 500 ms 的消息，保存到仓库内 `captured_frame/{rgb,depth,point}/`。RGB 和深度为 PNG，点云为二进制 PLY；文件写入、尺寸和点数检查都通过后才打印 `SUCCESS`。这是静态现场采样，不是运动中抓取的同步验收。

### 未标定时先验证 SORTING 通信链路

当新工位外参和 mask 尚未完成，可显式启动隔离的相机坐标诊断服务：

```bash
bash deploy.sh server start --diagnostic
bash deploy.sh server status
sudo docker logs --tail 50 femto-sorting   # 等待 live service ready
bash deploy.sh server test                # 含耗时明细
bash deploy.sh server client              # JSON 回包
```

这个入口从本机 RGB `CameraInfo` 读取内参，以相机光学坐标系为临时坐标系，使用当前旧 mask 并对点云抽样以适应新机内存。JSON 回包保持 8001 的业务字段，包括 `waybill_dict`；诊断说明只显示在启动和 `server test` 的终端提示里。此时 `center_world`、`box_8pts_world` 的数值实际是相机坐标毫米，不能发送机器人。8009 诊断模式允许三路时间戳最多相差 500 ms，仅用于静态场景联通测试；正式运动抓取须重新验收同步时差和处理时延。关闭诊断服务后执行 `bash deploy.sh server stop`、`bash deploy.sh server remove`。诊断容器无自动重启策略。

## 分拣服务与重启验收

新工位完成实际内外参及工作区 mask 标定，再填写 `config/calibration.json` 与 `assets/mask1.png`，更新 `assets/manifest.json` 的 mask SHA-256。用已知物体独立验证世界坐标、单位 mm、轮廓八点顺序后，才把 `validated_for_deployment` 改为 `true`。

```bash
bash deploy.sh check
bash deploy.sh server start
bash deploy.sh server status
sudo docker logs --tail 100 femto-sorting
bash deploy.sh server client
```

客户端必须返回 `success: true`，中心为 3 个有限数、轮廓为 8×3 个有限数；其数值要与已知物体现场测量相符。启动成功或 JSON 格式正确都不等于几何验收。若返回失败，保留 JSON 和容器日志。

在允许重启的窗口重启工控机。重新登录后不手动 `start` 或 `resume`，执行：

```bash
cd ~/femto_bolt_deploy
bash deploy.sh status
bash deploy.sh server status
bash deploy.sh verify
bash deploy.sh server client
```

两容器均在运行、三路仍 PASS、分拣客户端再次成功且坐标合理，才记录为重启后验收通过。若分拣尚未标定，只验收相机重启恢复，不宣称分拣或机器人联调完成。
