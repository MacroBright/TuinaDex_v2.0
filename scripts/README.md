# scripts: 自动化运维、环境管理与闭环验证工具箱

`scripts/` 目录汇聚了 TuinaDex v2.0 项目的 **基础设施、环境自检、标定向导、遥操运行、数据采集与自动化 E2E 回归测试脚本**。

所有的脚本均遵循零假设可执行原则，具备工业级退出码检查、错误诊断拦截与输出防爆设计。

---

## 快速索引表 (Script Quick Reference)

| 分类 | 脚本文件名 | 运行环境 | 核心功能与使用场景 |
| :--- | :--- | :--- | :--- |
| **Unified CLI** | [`bin/tuina`](../bin/tuina) | Auto (自感知) | **八大域系统级命令行总控门面**（配 [`tuina-completion.bash`](../bin/tuina-completion.bash) Tab 补全）。 |
| **环境基座** | [`activate_tuinadex.sh`](activate_tuinadex.sh) | Bash | 局部环境快速切换：`source scripts/activate_tuinadex.sh [control\|vla]`。 |
| | [`verify_all_envs.sh`](verify_all_envs.sh) | Bash | **环境准入门禁**。十项全能自动化自检（Python、ROS 2、Pinocchio、MuJoCo、CUDA、DDS）。 |
| | [`env/`](env/) | Bash | 宿主机与边缘端环境初始化脚本集（`check_env.sh`, `init_dev_pc.sh`, `init_orangepi.sh`, `setup_cyclonedds.sh`）。 |
| **标定向导** | [`00_preflight_calibration.py`](00_preflight_calibration.py) | `tuinadex-control` | **起飞前三合一交互式标定向导**（桌面物理平面 + 6 轴手腕 IMU + 灵巧手活动度）。 |
| | [`calibrate_imu.py`](calibrate_imu.py) | `tuinadex-control` | 手腕 6 轴 IMU 3 姿态正交对齐与零位校准工具（默认 `/dev/myimu`）。 |
| | [`teach_arm_ready_pose.py`](teach_arm_ready_pose.py) | `tuinadex-control` | **6-DOF reBot 无动力拖动示教脚本**，记录角度并写入 `configs/teleop_real_posture.yaml`。 |
| | [`test_goto_ready_pose.py`](test_goto_ready_pose.py) | `tuinadex-control` | 平滑五次多项式驱动 reBot 机械臂移动至 `ready_pose_deg` 实测验证工具。 |
| **遥操执行** | [`run_sim_teleop.py`](run_sim_teleop.py) | `tuinadex-control` | 22 维一体化实时视-惯遥操独立 3D 交互预览与 OpenCV HUD 仪表盘。 |
| | [`ros2_teleop_bridge.py`](ros2_teleop_bridge.py) | ROS 2 (`lyrical`) | **跨 Python 版本双向低延迟 UDP 桥**（`:9876 TD22` 指令、`:9877 HOME/HAND_HOLD`、`:9878 TS22` 实测状态）。 |
| **数据采集** | [`collect_teleop_data.py`](collect_teleop_data.py) | `tuinadex-control` | 多模态具身数据采集 CLI（支持 Web UI Hub、纯终端录制、`--env real` 物理 UVC 相机配置）。 |
| | [`run_data_convert.sh`](run_data_convert.sh) | `tuinadex-vla` / Docker | 离线将 `rosbag2` 转换为标准 `LeRobotDataset v3.0` 容器化封装脚本。 |
| **调试与测试** | [`mock_command_publisher.py`](mock_command_publisher.py) | ROS 2 (`lyrical`) | 命令行测试指令发布器，支持任意位置、持续时间与关节模式。 |
| | [`sim_loop_probe.py`](sim_loop_probe.py) | `tuinadex-control` | 物理仿真步进与控制循环高精度耗时与抖动剖析探针。 |
| **E2E 门禁** | [`e2e_unified_robot_check.sh`](e2e_unified_robot_check.sh) | Bash + ROS 2 | **整机协同端到端自检**：22D 协同、500Hz 物理积分、50Hz 通信与 300ms 看门狗。 |
| | [`e2e_hub_collection_check.sh`](e2e_hub_collection_check.sh) | Bash + Conda | **数据采集端到端自检**：真实 EGL 离屏渲染三机位视频 + H.264 导出全链路。 |
| | [`e2e_sim_check.sh`](e2e_sim_check.sh) | Bash + ROS 2 | 纯仿真环境自动化回归测试脚本。 |
| | [`e2e_mock_check.sh`](e2e_mock_check.sh) | Bash + ROS 2 | 脱机 Mock 模式自动化回归测试脚本。 |

---

## 常用脚本调用指令速查

### 0. 跨主机环境初始化 (PC 与 香橙派 Orange Pi / aarch64)
```bash
# 1. 运行系统级环境与硬件探针 (自动识别 CPU 架构、ROS 2、总线与环境状态)
./bin/tuina sys env

# 2. 标准 PC 开发机环境初始化 (安装 Udev 规则与 CycloneDDS)
sudo bash scripts/env/init_dev_pc.sh --install-rules
# 或通过 CLI:
sudo ./bin/tuina sys init-pc --install-rules

# 3. 香橙派 / ARM64 边缘主机初始化 (自动配置 1Mbps SocketCAN 开机自启服务与 Udev 规则)
sudo bash scripts/env/init_orangepi.sh --install-all
# 或通过 CLI:
sudo ./bin/tuina sys init-edge --install-all
```

### 1. 环境激活与健康检查
```bash
# 激活控制开发基座
source scripts/activate_tuinadex.sh control

# 运行全系统多端异构依赖准入门禁自检 (发起 PR 前必跑)
bash scripts/verify_all_envs.sh
```

### 2. 交互式三合一标定向导
```bash
# 运行可视化交互式起飞前标定
conda run -n tuinadex-control python scripts/00_preflight_calibration.py

# 纯手腕 IMU 快速标定 (默认串口 /dev/myimu，避免与 LEAP Hand /dev/ttyUSB0 冲突)
conda run -n tuinadex-control python scripts/calibrate_imu.py --port /dev/myimu

# 6-DOF reBot 机械臂无动力拖动示教 ready_pose 并测试平顺就位
./bin/tuina arm teach --channel can0
./bin/tuina arm goto-ready --channel can0
```

### 3. 多模态 Web UI 采集工作台
```bash
# 启动 22D 臂手协同仿真与 Web UI 工作台 (自动开启 3 机位视频与 UDP 9876 接收)
conda run -n tuinadex-control python scripts/collect_teleop_data.py \
    --env sim \
    --mode arm_hand \
    --cameras overhead,wrist,side \
    --hub \
    --fps 50
```

### 4. 自动化 E2E 验收与门禁测试
```bash
# 1. 22 维一体化机器人双模回环自检
bash scripts/e2e_unified_robot_check.sh

# 2. 三机位 EGL 离屏渲染与 LeRobotDataset v3.0 流式转码 E2E 自检
bash scripts/e2e_hub_collection_check.sh
```

---

## 系统级 CLI 统一调用入口 (`tuina`)

所有上述脚本与真机工具均已汇总封装为八大语义域全局命令 `tuina`（全局软链 `~/.local/bin/tuina -> bin/tuina`，提供 `-y/--yes` 跳过确认与 `--dry-run` 预检）：

| 语义域 | 典型系统级指令 | 核心覆盖脚本与功能 |
| :--- | :--- | :--- |
| **`launch`** | `tuina launch --sim --gui` / `--real` / `--target hand` | `robot.launch.py`, `hand.launch.py`, `arm.launch.py`, `sim.launch.py` |
| **`teleop`** | `tuina teleop hand` / `composite` (`real`) / `sim` / `node` | 16-DOF & 22-DOF 真机/仿真遥操与 HUD |
| **`collect`** | `tuina collect hub --env real` / `terminal` / `check` | `collect_teleop_data.py`, `e2e_hub_collection_check.sh` |
| **`calibrate`** | `tuina calibrate all` / `imu` / `hand` / `hand-limits` / `arm-ready` / `arm-goto` | 三合一标定、IMU 对齐、手部零点/限位、臂准备位示教与检验 |
| **`arm`** | `tuina arm cli` / `status` / `watch` / `move` / `zero` / `teach` / `can up` | `rebot.cli`、拖动示教与 1Mbps SocketCAN 管理 |
| **`hand`** | `tuina hand control` / `calibrate` / `diagnose` / `teleop` | `leap-control`, `leap-calibrate`, `leap-diagnose`, `leap-teleop` |
| **`data`** | `tuina data convert --bag ... --out ...` / `inspect` / `vla-cloud` | `tuinadex_dataset.cli`, `run_data_convert.sh`, `launch_vla_cloud.sh` |
| **`sys`** | `tuina sys env` / `init-pc` / `init-edge` / `verify` / `build` / `test` / `e2e` | 环境探针、PC/香橙派边缘初始化、colcon 编译、分域单测、E2E 与补全 |


