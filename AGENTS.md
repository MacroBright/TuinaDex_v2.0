# AGENTS.md — TuinaDex v2.0 (中医推拿机械臂-灵巧手具身智能系统)

> **定位**: 本文件是 TuinaDex v2.0 代码库的顶层架构宪章与开发协作规范。
> 面向所有克隆本仓库的开发者及其 AI 编码助手（Claude Code, Cursor, Codex, Windsurf, Copilot, Antigravity）。
> 任何人在新增功能、重构代码或发起 Pull Request 前，必须严格遵循本规范，确保系统架构不变量不被破坏。

---

## 1. 项目定位与核心不变量 (Mission & Core Invariants)

TuinaDex 是面向中医推拿/理疗场景构建的高性能具身智能机器人系统。
当前阶段（MVP）实现了 **16-DOF LEAP Hand 灵巧手 + 6-DOF reBot 机械臂（总自由度 22-DOF）** 的联合控制与端到端闭环：
`ROS2 运行时 -> 遥操作示教 / 三机位数据采集 Web Hub -> LeRobotDataset v3.0 转换 -> 具身策略学习 (SmolVLA/ACT) -> ROS2 仿真/真机推理部署`。

### 黄金不变量 (Golden Invariants · 严禁破坏)
1. **全链路统一单位契约 (SI Radian)**:
   - ROS2 接口、控制循环、MuJoCo 动力学仿真与 LeRobot 数据集强制全链路统一使用 **SI 国际标准单位：弧度 (rad)**，角速度为 `rad/s`，力矩为 `N·m`，法向接触力为 `N`；
   - 以实测标定后的全开平伸姿态 (`OPEN_POSE`) 为 **0 rad 基准零位**，手指各关节向内弯曲屈曲为正 ($q > 0$)。
2. **动作空间规范 (Absolute Joint Position)**:
   - 动作空间强制使用 **22 维（或 16 维单手模式）绝对关节位置 (Absolute Joint Position)**，绝对禁止使用增量 delta。
3. **状态与动作严格解耦**:
   - 目标指令：`/hand/joint_command` & `/arm/joint_command` (`sensor_msgs/JointState`)，反映上层目标期望；
   - 反馈状态：`/hand/joint_states` & `/arm/joint_states` (`sensor_msgs/JointState`)，反映真实物理关节状态。
4. **双域架构与驱动隔离**:
   - ROS2 负责机器人运行时（通信、控制循环、硬件抽象、安全看门狗）；
   - LeRobot 负责策略学习（离线数据集处理、模仿学习训练、离线评测）；
   - 策略层严禁绕过 ROS2 直接访问驱动或串口/CAN 总线。
5. **硬件安全红线 (Safety Guardrails)**:
   - 通信中断或主端手势丢失超过 **300ms** 时，系统必须无条件触发安全看门狗，平滑阻尼回退至全开平伸姿态 (`Relax-to-OPEN`)；
   - 灵巧手舵机工作电流上限硬锁定为 **150mA**，杜绝堵转过热或暴力夹捏；
   - 严禁无物理/软件限位保护向电机发送越界绝对目标。

---

## 2. 五层分层架构全景与边界契约 (5-Layer Architecture)

本项目遵循严格的五层分层架构，**严禁跨层调用，严禁反向依赖**：

```
┌────────────────────────────────────────────────────────┐
│ Layer 5: AI / Policy Layer (LeRobot / SmolVLA / ACT)   │  -> tuinadex-vla (CUDA 12.8)
└───────────────────────────┬────────────────────────────┘
                            │ Observation / Action (22D rad)
┌───────────────────────────▼────────────────────────────┐
│ Layer 4: ROS2 Middleware Layer (Nodes, Hub, Teleop)    │  -> tuinadex-control
└───────────────────────────┬────────────────────────────┘
                            │ /joint_command & /joint_states
┌───────────────────────────▼────────────────────────────┐
│ Layer 3: Control & Safety Layer (Watchdog FSM, Clamp)  │  -> tuinadex_core
└───────────────────────────┬────────────────────────────┘
                            │ Verified Radian Array / Safe Hold+Ramp
┌───────────────────────────▼────────────────────────────┐
│ Layer 2: Driver & Bridge Layer (LeapDriver, SimBridge) │  -> tuinadex_hand/arm/sim
└───────────────────────────┬────────────────────────────┘
                            │ Dynamixel 2.0 (4Mbps) / SocketCAN (500kbps) / 500Hz Substep
┌───────────────────────────▼────────────────────────────┐
│ Layer 1: Hardware & Simulation (LEAP Hand, reBot, Sim) │  -> U2D2 / USB2CAN / tuina_scene.xml
└────────────────────────────────────────────────────────┘
```

### 分层职责与调用禁区
1. **Layer 3 (`tuinadex_core`) 纯粹性**:
   - `tuinadex_core` 为控制内核与安全数学算法，**严禁引入 ROS2 运行时依赖**（如 `rclpy`、`sensor_msgs`），必须保证可在脱机原生 Python 环境下被独立单测。
2. **单一事实源 (SSOT) 话题原则**:
   - 任一物理量或状态话题在同一时刻**只能有唯一主控发布者**；
   - 复合仿真环境下，机械臂状态 `/arm/joint_states` 独占发布者为 `ArmNode`，仿真桥处于跟随订阅模式更新动力学位姿，严禁话题双发竞争。
3. **数据事实源契约**:
   - `rosbag2` 是实验原始数据的唯一事实源；`LeRobotDataset` 是下游学习数据产品，由独立离线工具 `tuinadex_dataset` 转换生成。

---

## 3. 文件结构全景与框架分层映射 (Directory Mapping)

代码库各功能包、目录与框架层次的严谨映射关系如下：

| 顶层目录 / 子包 | 框架层次 | 主要文件类型 | 职责与功能定位 |
| :--- | :--- | :--- | :--- |
| `src/tuinadex_bringup/` | **Layer 4** (Middleware) | `.launch.py`, `.yaml`, `.py` | **整机统一部署与协同启动入口**。<br>`launch/robot.launch.py` 提供单手/复合臂手、真机/仿真/Mock 一键统一启动。 |
| `src/tuinadex_teleop/` | **Layer 4** (Middleware) | `.py`, `.launch.py`<br>`.html`, `.css`, `.js`<br>`.yaml`, `.xml` | **遥操作示教与前端数据采集 Hub**。<br>包含三机位独立 EGL 渲染相机阵列 (`cameras/`)、零依赖 Web UI 采集工作台 (`hub/`)、动作重定向与逆运动学求解 (`kinematics/`)、位姿感知 (`perception/`)。 |
| `src/tuinadex_recorder/` | **Layer 4** (Middleware) | `.py`, `package.xml` | **实验原始数据事实源记录节点**。<br>负责订阅全量状态与图像并高效落盘为标准 `rosbag2` 格式。 |
| `src/tuinadex_core/` | **Layer 3** (Control & Safety) | `.py` (纯 Python 无 ROS2 依赖) | **核心控制与安全守护内核**。<br>`watchdog.py` (300ms 安全状态机)、`command_processing.py` (22D/16D 弧度校验与阻尼平滑)、`names.py` (标准关节命名映射)。 |
| `src/tuinadex_hand/` | **Layer 2 & 1** (Driver/Bridge) | `.py`, `.launch.py`, `.yaml` | **16-DOF LEAP Hand 灵巧手物理驱动与节点**。<br>`driver/leap_driver.py` 实现 Dynamixel Protocol 2.0 4Mbps 高速串口通信，`leap_hand_node.py` 对接 ROS2 接口。 |
| `src/tuinadex_arm/` | **Layer 2 & 1** (Driver/Bridge) | `.py`, `.urdf`, `.stl`<br>`.launch.py`, `.yaml` | **6-DOF reBot 机械臂物理驱动与节点**。<br>`arm_node.py` 提供机械臂 ROS2 控制环，`description/` 提供 URDF 与 3D 网格模型。 |
| `src/rebot_control/` | **Layer 2** (Driver Layer) | `.py`, `.yaml` | **reBot 机械臂底层通信库与 CLI 调试工具**。<br>基于 Linux SocketCAN 实现 500kbps 电机通信、MIT 模式控制算法与状态监控。 |
| `src/tuinadex_sim/` | **Layer 2 & 1** (Sim Engine) | `.xml`, `.urdf`, `.stl`<br>`.py`, `.launch.py` | **MuJoCo 动力学物理仿真系统**。<br>`worlds/tuina_scene.xml` 提供推拿台全景孪生，`sim_bridge.py` 驱动 500Hz 物理子步解算，`leap_hand_sim_node.py` 对接 ROS2。 |
| `dataset/tuinadex_dataset/` | **Layer 5** (AI Policy Prep) | `.py`, `setup.py` | **LeRobot 数据集离线转换与流式导出工具**。<br>解析 rosbag2，执行时空同步对齐与质量校验，流式导出包含 H.264 三路视频与 22D 动作/力矩/法向力的 `LeRobotDataset v3.0`。 |
| `scripts/` | 全局基础设施 | `.sh`, `.py`<br>`.yml`, `.rules`, `.xml` | **环境管理、统一配置与自动化验证**。<br>`activate_tuinadex.sh` (局部切换)、`verify_all_envs.sh` (准入门禁)、`collect_teleop_data.py` (采集 CLI)、各 `e2e_*.sh` 验证脚本。 |
| `docs/` | 架构与决策资产 | `.md`, `.html`<br>`.json`, `.png` | **技术设计文档与决策记录**。<br>`architecture/` (架构全景交互图)、`adrs/` (架构决策记录)、`specs/` (环境与接口规范)。 |
| `tests/` | 全局质量防线 | `.py`, `pytest.ini` | **自动化单元测试与集成测试套件**。<br>包含 `teleop/` (81 项算法单测)、ROS2 节点集成单测、Hub 单测、VLA 导出单测与环境自检单测。 |

---

## 4. 各层次数据传输方式与接口契约 (Data Flow & Contracts)

### 4.1 实时控制闭环 (50Hz 控制周期 / 500Hz 动力学子步)
- **指令话题 (Command Topics)**:
  - 灵巧手指令：`/hand/joint_command` (`sensor_msgs/msg/JointState`)
  - 机械臂指令：`/arm/joint_command` (`sensor_msgs/msg/JointState`)
  - **契约要求**：`position` 字段必须包含完整维度绝对弧度（手 16D，臂 6D），`name` 字段必须与标准关节名严格对齐。
- **状态反馈话题 (State Topics)**:
  - 灵巧手状态：`/hand/joint_states` (`sensor_msgs/msg/JointState`)
  - 机械臂状态：`/arm/joint_states` (`sensor_msgs/msg/JointState`)
  - **契约要求**：必须包含实测物理位置 (`position`)、速度 (`velocity`) 与力矩/电流 (`effort`)。

### 4.2 多模态视觉与数据采集链路
- **标准三机位视觉拓扑**:
  - `overhead` (俯视固定机位, 640x480, fovy=60°): 工作台上方偏后，下倾 50° 俯视全景；
  - `wrist` (腕部随动机位, 640x480, fovy=75°): 绑定 `leap_hand_palm`，视角朝指尖前方偏掌心；
  - `side` (侧向固定机位, 640x480, fovy=45°): 工作台侧面，镜头平行于接触表面。
- **数据通道与缓冲解耦**:
  - 专用 `CameraProducerThread` 独占 EGL 离屏渲染，彻底与动力学控制回路解耦；
  - 内存图像统一缓存为 **JPEG 字节流**（单试次内存控制在 135MB 以内，防止 OOM）；
  - 试次落盘流式多路并行编码为 **H.264 MP4 视频流**，数值特征对齐写入 **Parquet 表格**。
- **全模态特征字典 (Dataset Features)**:
  - `observation.state`: 22 维实测关节角度 (float32[22])；
  - `observation.effort`: 22 维实测关节力矩/电流 (float32[22])；
  - `observation.contact_forces`: 4 指尖法向接触力 (float32[4])；
  - `action`: 22 维目标关节位置 (float32[22])；
  - `observation.images.{overhead,wrist,side}`: 3 路 640x480 H.264 视频流；
  - 元数据：`timestamp`, `frame_index`, `episode_index`, `phase_index`, `next.done`。

### 4.3 Web UI Hub 交互链路
- **轻量零依赖原则**: 原生 Python `http.server` 运行于 8080 端口，禁止 npm 构建链；
- **通信模式**:
  - REST API (`/api/status`, `/api/start`, `/api/stop`, `/api/save`, `/api/abort`): 控制采集状态机；
  - SSE 实时事件流 (`/api/sse`, 25Hz): 推送关节角度、力矩、指尖力与丢帧遥测数据；
  - MJPEG 多相机流 (`/stream/overhead`, `/stream/wrist`, `/stream/side`): 低延迟图传。

---

## 5. 代码编写规范与分层分包准则 (Coding Standards)

所有协作者与 AI Agent 在向代码库新增或修改代码时，必须严格遵守以下规范：

### 5.1 命名规范 (Naming Conventions)
- **文件与模块命名**:
  - Python / C++ 源文件：全小写下划线 `snake_case.py` / `snake_case.cpp`；
  - 头文件：`snake_case.hpp` 并强制包含 `#pragma once`；
  - ROS2 Launch 文件：`name.launch.py`；
  - 配置文件：`name_params.yaml` 或 `name_config.yaml`。
- **符号与标识符命名**:
  - 类名：大驼峰 `CamelCase`（如 `WatchdogFSM`, `MujocoSimBridge`, `TuinaCollector`）；
  - 函数与方法：小写下划线 `snake_case()`；模块私有函数以单个下划线开头 `_private_helper()`；
  - 全局常量：全大写下划线 `UPPER_SNAKE_CASE`（如 `CANONICAL_JOINT_NAMES`, `OPEN_POSE_RAD`）；
  - 局部变量：具有明确物理含义的 `snake_case`，禁止无意义单字母命名（标准物理量除外，如 `q`, `dq`, `tau`）。
- **ROS2 实体命名**:
  - 节点名：名词性 `snake_case`（如 `leap_hand_node`, `arm_node`, `teleop_composite_node`）；
  - 话题名：分层语义斜杠命名（如 `/hand/joint_command`, `/arm/joint_states`, `/teleop/status`）；
  - 参数名：`snake_case`，物理参数须附带单位后缀（如 `publish_rate_hz`, `watchdog_timeout_ms`）。
- **关节与自由度统一命名**:
  - 16-DOF 灵巧手：严格对齐 `tuinadex_core.names.CANONICAL_JOINT_NAMES`（`joint_0` 至 `joint_15`）；
  - 6-DOF 机械臂：严格对齐 `tuinadex_core.names.CANONICAL_ARM_JOINT_NAMES`（`joint1` 至 `joint6`）。

### 5.2 分包与放置准则 (Code Placement Discipline)
- **核心算法代码**：若不依赖 ROS2 消息与调度器，**强制写入 `tuinadex_core`**，并编写纯 Python 单元测试；
- **底层驱动实现**：总线协议解析封装在各硬件包的 `driver/` 子目录，严禁在 ROS2 节点回调中直接写原生套接字/串口通信；
- **静态资源管理**：URDF 放入 `urdf/`，MJCF 放入 `worlds/` 或 `configs/mjcf/`，3D 资产（STL/OBJ）放入 `meshes/` 并保持小写。

---

## 6. 双基座环境管理与依赖变更门禁 (Environment Management)

根据《多端异构环境管理架构设计规范》(`docs/specs/ENVIRONMENT_MANAGEMENT_SPEC.md`)，本项目采用双基座硬隔离方案：

### 6.1 双环境边界与作用域
- **`tuinadex-control` (Python 3.10.20)**:
  - 覆盖范围：ROS2 节点、6-DOF 臂/16-DOF 手底层驱动、MuJoCo 仿真、Pinocchio 动力学、手势视觉遥操、Web UI 采集 Hub。
- **`tuinadex-vla` (Python 3.10.20 + CUDA 12.8)**:
  - 覆盖范围：具身策略学习、Hugging Face LeRobot、PyTorch、SmolVLA / ACT 模型训练与评测。

### 6.2 局部作用域与防污染准则
- **严禁全局污染**：禁止在宿主机 `~/.bashrc` 中直接写入特定 Conda 环境的自动激活；
- **按需局部切换**:
  - 控制/仿真/采集开发：`source scripts/activate_tuinadex.sh control`
  - 策略学习/数据导出：`source scripts/activate_tuinadex.sh vla`
- **动态链接库隔离最佳实践**:
  - 在涉及 C++ 依赖（如 Pinocchio, Boost, OpenCV）的环境中，确保 Conda 内部依赖库路径优先于系统库，防止出现系统动态库符号未定义冲突。

### 6.3 依赖变更准入门禁 (Invariant 24)
- 任何新增依赖、升级包版本操作，必须同步导出锁定文件：
  `conda env export --no-builds > scripts/env/tuinadex-control.yml`
- 提交前必须执行全套自动化环境验证：
  `bash scripts/verify_all_envs.sh`
- 只有诊断报告全部为 `[PASS]` 时才允许合入主干。

---

## 7. 团队协作工作流程 (Team Collaboration Workflow)

团队采用基于 **GitHub Flow + 具身智能测试驱动门禁 (TDD & Quality Gates)** 的规范工作流程：

```
1. 克隆与环境自检 (verify_all_envs.sh)
   │
2. 签出特性分支 (feat/<module>-<description>)
   │
3. TDD 增量开发 (先在 tests/ 写用例，再写实现)
   │
4. 本地四重门禁全量回归 (colcon build & pytest & e2e)
   │
5. 规范提交 (Conventional Commits: feat/fix/test/docs)
   │
6. 提交 Pull Request 与代码复核 (回答四要素，双轴复核后合入)
```

### 协作执行细则：
1. **环境准备与基线自检**:
   ```bash
   git clone https://github.com/MacroBright/TuinaDex_v2.0.git
   cd TuinaDex_v2.0
   source scripts/activate_tuinadex.sh control
   bash scripts/verify_all_envs.sh
   ```
2. **分支命名规范**:
   - 功能开发：`feat/<module>-<description>`
   - 缺陷修复：`fix/<module>-<issue>`
   - 文档改动：`docs/<description>`
3. **AI 辅助协作与 TDD 约束**:
   - **最小改动原则 (YAGNI)**：改动必须能直接追溯到具体需求，严禁路过式重构或顺手格式化；
   - **真实性审计防线 (Anti-Mock & Anti-Skip)**：
     - 严禁空断言与硬编码假期望；
     - 严禁为了通过 CI 对核心物理动力学与渲染使用 Mock；
     - 严禁使用 `@pytest.mark.skip` 掩盖未解决的问题；
   - **命令输出防爆安全带**: 全量测试与构建长日志命令**强制使用管道截断过滤**（如 `| head -n 30` 或 `| grep -E "FAIL|ERROR|Summary"`），严禁裸命令刷屏淹没上下文。
4. **提交信息规范 (Conventional Commits)**:
   - 格式：`<type>(<scope>): <subject>`
   - 示例：`feat(hub): add MJPEG streaming endpoint for wrist camera`
5. **本地准入四重门禁 (发起 PR 前必跑)**:
   - 环境检查：`bash scripts/verify_all_envs.sh`
   - ROS2 构建：`source /opt/ros/lyrical/setup.bash && colcon build --symlink-install`
   - 单元测试：`pytest tests/ -q`
   - E2E 双模回环：`bash scripts/e2e_hub_collection_check.sh` 与 `bash scripts/e2e_unified_robot_check.sh`
6. **PR 提交流程**:
   - PR 描述中必须清晰回答四要素：改动点 (What changed)、测试命令 (How tested)、实证依据 (Evidence)、未验证边界 (Unverified borders)。

---

## 8. 常用全套验证命令速查表

| 验证目标 | 推荐执行命令 | 预期指标 |
| :--- | :--- | :--- |
| **系统级 CLI 解析与安全门禁** | `./bin/tuina sys test --suite cli` | **37 passed** |
| **全环境与依赖准入检查** | `./bin/tuina sys verify` *(或 `bash scripts/verify_all_envs.sh`)* | **10 项全 PASSED** |
| **核心安全与限位算法单测** | `./bin/tuina sys test --suite core` | **23 passed** |
| **遥操运动学与感知核心单测** | `./bin/tuina sys test --suite teleop` | **107 passed** |
| **数据采集 Hub 与 API 单测** | `./bin/tuina sys test --suite hub` | **34 passed** |
| **驱动、真实相机与标定扩展单测** | `source scripts/activate_tuinadex.sh control && pytest tests/test_real_camera_rig.py tests/test_env_management.py tests/test_leap_cli_and_pose_manager.py tests/test_leap_teleop_cli.py tests/test_leap_driver.py tests/test_arm_driver.py tests/test_sim_bridge.py tests/test_sim_core.py -q` | **53 passed** |
| **LeRobot v3.0 数据流式导出单测** | `./bin/tuina sys test --suite dataset` | `3 passed, 2 skipped` *(vla 环境跳过 rosbag2 用例)* |
| **ROS 2 节点与启动集成测试** | `ROS_DOMAIN_ID=177 ./bin/tuina sys test --suite ros2` | **8 passed** *(CycloneDDS 隔离域)* |
| **复合仿真节点集成测试** | `python3 -m pytest tests/test_teleop_sim_composite_node.py -q` | **7 passed** |
| **22D 臂手联合控制双模回环** | `./bin/tuina sys e2e --suite unified` *(或 `bash scripts/e2e_unified_robot_check.sh`)* | ALL PASS |
| **三机位采集 Hub 端到端 E2E** | `./bin/tuina collect check` *(或 `bash scripts/e2e_hub_collection_check.sh`)* | PASS (真实 EGL + H.264) |
| **整机统一启动入口** | `./bin/tuina launch --sim` *(或 `ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=true mock:=false`)* | 节点拓扑正常，/hand 与 /arm 话题正常发布 |
| **数据采集 Web Hub 入口** | `./bin/tuina collect hub --env sim --mode arm_hand` | Web Hub 服务就绪于 8080 端口 |

---

## 9. 多端异构主机与边缘端 (Linux x86_64 / 香橙派 Orange Pi aarch64) 克隆、自愈与运行规范

当其他 AI Agent 或开发者在任意 Linux 环境（如标准开发机 x86_64、香橙派 Orange Pi AI Pro / Orange Pi 5 等 aarch64 边缘单板计算机，或无硬件 CI 机器）克隆本代码库后，**强制执行以下自愈与运行 SOP，杜绝盲目调试与环境污染**：

### 9.1 第一动作：自感知与环境探针 (First-Contact Probe Protocol)
克隆仓库后，严禁未经检测直接修改代码或运行驱动。第一步必须执行环境探针：
```bash
git clone https://github.com/MacroBright/TuinaDex_v2.0.git
cd TuinaDex_v2.0

# 运行系统级环境与硬件探针
./bin/tuina sys env
# 或
bash scripts/env/check_env.sh
```
探针将输出：
- **CPU 架构**：`x86_64`（PC/工作站）或 `aarch64`（香橙派/ARM SBC）；
- **ROS 2 发行版与 CLI 状态**：`lyrical`, `humble`, `jazzy`；
- **Conda / Python 虚拟环境状态**：`tuinadex-control` 与 `tuinadex-vla` 路径；
- **物理硬件总线**：串口设备 `/dev/ttyUSB*`、Udev 别名 `/dev/tuinadex_hand`、SocketCAN `can0` 状态、IMU 别名 `/dev/myimu`。

### 9.2 跨架构双基座环境就绪指南 (Dual Conda/Venv Setup by Arch)
根据探针识别的 CPU 架构选择对应配置路径：

#### 路径 A: 标准 Linux x86_64 开发机 / 服务器
1. **安装 Conda 控制与策略环境**:
   ```bash
   # 控制与驱动基座 (Python 3.10)
   conda env create -f scripts/env/tuinadex-control.yml
   # 具身策略学习基座 (Python 3.10 + CUDA + LeRobot)
   conda env create -f scripts/env/tuinadex-vla.yml
   ```
2. **初始化开发机权限与 DDS**:
   ```bash
   # 安装 Udev 规则 (U2D2 / USB2CAN 免 root) 与 CycloneDDS
   sudo bash scripts/env/init_dev_pc.sh --install-rules
   # 确保当前用户在 dialout 组
   sudo usermod -aG dialout $USER
   ```

#### 路径 B: 香橙派 / ARM64 边缘单板计算端 (Orange Pi AI Pro 20T / Orange Pi 5 aarch64)
> [!IMPORTANT]
> aarch64 环境下**严禁**直接通过 `tuinadex-control.yml` 导入（包含 `linux-64` 特定二进制），必须采用原生 aarch64 Miniforge 构建：
1. **轻量化 Conda 环境构建**:
   ```bash
   # 推荐安装 aarch64 Miniforge3
   conda create -n tuinadex-control python=3.10 -y
   conda activate tuinadex-control
   
   # 安装核心运行时依赖
   pip install numpy scipy pyyaml opencv-python pinocchio dynamixel-sdk pyserial can-utils pytest
   ```
2. **香橙派一键硬件初始化 (SocketCAN + Udev + CycloneDDS)**:
   ```bash
   # 自动安装 1Mbps SocketCAN 开机自启服务与硬件别名
   sudo bash scripts/env/init_orangepi.sh --install-all
   ```
   该脚本会自动配置 `/etc/systemd/system/tuinadex-can.service`，开机自启 1Mbps `can0`，并建立 `/dev/tuinadex_hand` 与 `/dev/myimu` 软链。

### 9.3 工作区编译构建 (Workspace Build)
无论在哪种架构上，均通过系统级 CLI 编译 ROS 2 工作空间，无需手动纠结环境变量与动态库冲突：
```bash
./bin/tuina sys build
# 等价于: source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install
```

### 9.4 跨主机运行三模态自愈指南 (Three Operational Modes)
根据当前主机连接的硬件状态，Agent 必须自动选择适配的运行模态：

#### 模态 1: 无硬件纯软件仿真孪生 (Zero-Hardware Twin Mode · 开发机与远程体验)
*适用场景：无任何机械臂/灵巧手/相机外设，需在 MuJoCo 孪生中进行推拿动作验证与算法联调。*
```bash
# 1. 启动 22-DOF MuJoCo 仿真物理孪生
./bin/tuina launch --sim --gui

# 2. 启动 22-DOF 视-惯遥控 HUD 界面 (使用虚拟手势与键盘热键)
./bin/tuina teleop sim --mock

# 3. 启动多机位 Web UI 采集工作台
./bin/tuina collect hub --env sim --mode arm_hand
# 浏览器访问: http://127.0.0.1:8080
```

#### 模态 2: 边缘端真实机器人 (Edge Physical Mode · 香橙派/物理主机连真实硬件)
*适用场景：香橙派或工控机外接 U2D2 + USB2CAN + USB 相机阵列，执行真机推拿与示教采集。*
```bash
# 1. 确认 SocketCAN 1Mbps 接口就绪
./bin/tuina arm can up --channel can0 --bitrate 1000000

# 2. 运行真机准备姿态示教与回位
./bin/tuina arm teach --channel can0
./bin/tuina arm goto-ready --channel can0

# 3. 运行多设备综合标定向导 (包含桌面、IMU 与灵巧手)
./bin/tuina calibrate all

# 4. 启动真机协同控制节点
./bin/tuina launch --real

# 5. 启动真机多机位采集 Web Hub
./bin/tuina collect hub --env real --mode arm_hand --cameras overhead,side
# 局域网内任意 PC 打开浏览器访问: http://<orangepi-ip>:8080 即可远程监控并录制试次
```

#### 模态 3: 无外设快速 Mock 验证 (CI / Headless Mode)
*适用场景：CI 自动化构建流水线、无 GPU 无显示器远程服务器、快速端到端冒烟自检。*
```bash
# 1. 启动轻量化 Mock 节点 (无需物理电机与总线)
./bin/tuina launch --mock

# 2. 执行端到端双模回环全量自检
./bin/tuina sys e2e --suite unified
```

### 9.5 常见故障排查自愈矩阵 (Agent Self-Healing Matrix)

| 故障现象 | 根本诱因 (Root Cause) | Agent 自愈排障 SOP |
| :--- | :--- | :--- |
| **`can0: interface not active`** | 未接入 USB2CAN 或 SocketCAN 接口未拉起 | 1. 真实硬件：运行 `./bin/tuina arm can up` 或 `sudo systemctl restart tuinadex-can.service`；<br>2. 无硬件模拟：运行 `./bin/tuina arm can vcan` 自动载入虚拟总线 `vcan0`。 |
| **`Permission denied: /dev/ttyUSB*`** | 当前用户不在 `dialout` 组或缺少 Udev 规则 | 运行 `sudo usermod -aG dialout $USER && sudo bash scripts/env/init_dev_pc.sh --install-rules`，重新登录生效。 |
| **香橙派 Headless 无显示器报错** | OpenCV / OpenGL 尝试在无 X11/Wayland 桌面创建窗口 | 1. 在遥操启动命令中加入 `--no-gui` 或脱机使用 Web Hub 采集；<br>2. 导出 EGL 离屏渲染环境变量：`export MUJOCO_GL=egl`。 |
| **ROS 2 话题跨主机无法发现** | DDS 域 ID 不一致或多网卡广播隔离 | 1. 确保所有主机 `echo $ROS_DOMAIN_ID` 均为 `42`；<br>2. 确认 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 且 CycloneDDS XML 生效。 |
| **Conda 环境激活找不到 `conda` 函数** | 子 Shell 中未预加载 Conda 初始化脚本 | 运行 `source scripts/activate_tuinadex.sh control`（内部已适配 miniforge/miniconda 全局候选路径）。 |
