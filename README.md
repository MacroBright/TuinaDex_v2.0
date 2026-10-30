# TuinaDex v2.0 — 中医推拿机械臂-灵巧手具身智能系统 (LEAP Hand MVP & MuJoCo Digital Twin)

[![ROS2](https://img.shields.io/badge/ROS2-Lyrical%20%2F%20Humble-blue.svg)](https://docs.ros.org)
[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.14-green.svg)](https://www.python.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.13-purple.svg)](https://mujoco.org)
[![LeRobot](https://img.shields.io/badge/LeRobot-v0.4.4-orange.svg)](https://github.com/huggingface/lerobot)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

TuinaDex 是面向中医推拿/理疗场景构建的具身智能机器人系统。
当前阶段专注于 **6-DOF 机械臂 + 16-DOF LEAP Hand 灵巧手 (共 22-DOF)** 的高可靠软硬件与数字孪生闭环系统：
`ROS2 运行时 / MuJoCo 仿真 -> 遥操作示教 -> rosbag2 数据记录 -> 离线对齐与校验 -> LeRobot 策略学习 -> ROS2 闭环部署`。

---

## 1. 系统架构与数据流图 (Architecture & Dataflow)

```mermaid
flowchart TD
    subgraph Teleoperation ["遥操作与手势捕捉 (tuinadex_teleop)"]
        Cam["RGB 相机"] --> Tracker["MediaPipe 手势追踪器 (21 Keypoints)"]
        Tracker --> Gate["利手过滤 (Handedness Gate)"]
        Gate --> Filter["OneEuroFilter 抖动滤波"]
        Filter --> Mapper["JointMapper (几何解算)"]
        Mapper --> TeleopNode["TeleopNode 发布 50Hz 目标指令"]
    end

    subgraph Core ["核心安全与共享逻辑 (tuinadex_core)"]
        TeleopNode -->|"/hand/joint_command (JointState)"| Align["align_command (Canonical 重排 + 限位门)"]
        Align --> FSM{"WatchdogStateMachine\n(FOLLOWING / RELAXING)"}
        FSM --> RouteSwitch{"模式路由选择\n(use_sim:=true/false)"}
    end

    subgraph RealRuntime ["分支 A: 物理硬件执行体 (tuinadex_hand, use_sim:=false)"]
        RouteSwitch -->|Real / Mock| LeapDriver["Dynamixel Protocol 2.0 (4Mbps, 150mA)"]
        LeapDriver --> RealHand["16-DOF LEAP Hand 实体舵机"]
        RealHand --> RealStatePub["LeapHandNode 发布 50Hz 实测状态"]
    end

    subgraph SimRuntime ["分支 B: MuJoCo 数字孪生执行体 (tuinadex_sim, use_sim:=true)"]
        RouteSwitch -->|Sim| SimCore["SimulationCore (Hold+Ramp 回退)"]
        SimCore --> MuJoCoBridge["MujocoSimBridge (10x 0.002s 子步进)"]
        MuJoCoBridge --> Scene["推拿床与人体背部假人刚体世界 (tuina_scene)"]
        Scene --> SimStatePub["LeapHandSimNode 发布 50Hz 仿真状态"]
    end

    subgraph Bus ["统一 ROS 2 话题总线 (Topic Parity)"]
        RealStatePub --> JointStates["/hand/joint_states (sensor_msgs/JointState)"]
        SimStatePub --> JointStates
    end

    subgraph Recorder ["示教数据采集 (tuinadex_recorder)"]
        JointStates --> BagRec["RecorderNode (rosbag2_py 进程内写入)"]
        TeleopNode -.->|"/hand/joint_command"| BagRec
        SrvCtrl["Trigger 服务 (/recorder/start & /stop)"] --> BagRec
        BagRec --> RawBag[("rosbag2 原始事实源 (sqlite3/db3)")]
    end

    subgraph OfflinePipeline ["离线数据适配器 (tuinadex_dataset)"]
        RawBag --> BagReader["bag_reader (反序列化双话题)"]
        BagReader --> Sync["TimeSynchronizer (50Hz 时间网格插值对齐)"]
        Sync --> Validator{"DatasetValidator\n(SPEC_JOINT_LIMITS 校验)"}
        Validator -->|Pass| Adapter["adapter.save_to_lerobot"]
        Adapter --> LeRobotDS[("LeRobotDataset (v3.0 Parquet)")]
    end

    subgraph Policy ["策略学习与推理 (LeRobot / SmolVLA)"]
        LeRobotDS --> Train["离线模仿学习 / 策略训练"]
        Train --> PolicyNode["ROS2 推理部署节点"]
        PolicyNode -.->|"/hand/joint_command"| Align
    end
```

---

## 2. 核心技术规范与架构不变量 (Invariants)

1. **统一弧度制与零位契约 (Invariant 1)**:
   - ROS2 接口与 LeRobot 数据集全链路强制 **SI 国际标准单位：弧度 (rad)**；
   - 以实测标定后的 `OPEN_POSE`（全开平伸）为 **0 rad 基准零位**，手指各关节向内屈曲弯曲为正 ($q > 0$)。
2. **Canonical 16-DOF 排序**:
   - 顺序固定为：食指(侧摆/俯仰/近指/远指) 4DOF、中指 4DOF、无名指 4DOF、拇指(回转/俯仰/屈曲/指尖) 4DOF。
   - 详见 [`docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`](docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md)。
3. **话题完全等价性 (Invariant 13)**:
   - 仿真主控 `leap_hand_sim_node` 与物理主控 `leap_hand_node` 对外完全共享 `/hand/joint_command` 与 `/hand/joint_states`；上层遥操、采集与策略节点零感知切换。
4. **多速率时钟与物理步进解耦 (Invariants 14 & 16)**:
   - MuJoCo 内部物理动力学以 500Hz (`dt = 0.002s`) 运行以保障接触积分稳定；
   - ROS 2 仿真节点采用单线程 50Hz Timer，每 tick 内锁步执行 10 次子步进 (`10 x 0.002s = 20ms`)，发布端严格降采样至 50Hz，杜绝后台线程竞态。
5. **共享纯逻辑下沉与无环依赖 (Invariant 17)**:
   - `tuinadex_core` 承载 `align_command`、`JOINT_SPEC_LIMITS` 与 `WatchdogStateMachine`，彻底切断 `tuinadex_hand` 与 `tuinadex_sim` 循环依赖。
6. **双模看门狗与平滑回退 (Invariants 15 & 18)**:
   - 两端均内置 300ms Watchdog；失联时复刻 `0.3s hold + 0.6s ramp` 阻尼回退至全开平伸姿态 (`Relax-to-OPEN`)，杜绝机械冲击与仿真冲量发散。

---

## 3. 环境准备与多环境矩阵 (Environment Matrix)

由于 Ubuntu 26.04 系统 ROS2 (Lyrical Luth / Python 3.14) 与 LeRobot (Python 3.10) 的环境隔离要求，本项目采用分层环境矩阵：

| 环境 | 路径 / 工具 | 适用模块 | 验证方式 |
| :--- | :--- | :--- | :--- |
| **ROS2 主机环境** | `/opt/ros/lyrical/setup.bash` + `/usr/bin/python3` (MuJoCo 3.13) | `tuinadex_core`, `tuinadex_hand`, `tuinadex_sim`, `tuinadex_recorder` 编译与 ROS2 集成测试 | `colcon build`, `pytest tests/test_leap_hand_node_integration.py tests/test_sim_node_integration.py` |
| **日常控制基座环境 (tuinadex-control)** | `~/win_office/conda/envs/tuinadex-control/bin/python` (Python 3.10.20) | 6-DOF 臂、16-DOF 手驱动、MuJoCo 动力学、Co_Teleop 视觉遥操 (MediaPipe/OpenCV/RealSense) | `python -m pytest tests/ -v` |
| **具身学习与训练环境 (tuinadex-vla)** | `~/win_office/conda/envs/tuinadex-vla/bin/python` (Python 3.10.20 + CUDA 12.8) | `tuinadex_dataset` 离线保存与 Parquet 导出、SmolVLA/ACT 训练 | `python -m pytest tests/test_dataset_adapter.py -v` |

---

## 4. 构建与快速上手 (Quickstart)

### 4.1 编译 ROS2 功能包 (8 包全量)
```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4.2 臂手协同双模式一键启动 (Unified Dual-Mode Launch)

通过 `tuinadex_bringup` 提供的顶层统一入口 `robot.launch.py`，实现 6-DOF 臂与 16-DOF 手的协同调度：

#### 模式 1: 真实物理硬件 / Mock 驱动启动 (`use_sim:=false`)
```bash
# 1. 22 维臂手协同真机启动 (连接 USB-TTL 串口与 SocketCAN can0)
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=false hand_port:=/dev/ttyUSB0 arm_channel:=can0

# 2. 22 维脱机 Mock 安全回环启动 (CI 与无物理硬件调试推荐)
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=false mock:=true

# 3. 单独启动 16 维灵巧手或 6 维机械臂
ros2 launch tuinadex_bringup robot.launch.py mode:=hand_only mock:=true
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_only mock:=true
```

#### 模式 2: MuJoCo 数字孪生仿真启动 (`use_sim:=true`)
```bash
# 1. 启动 22 维协同数字孪生仿真并弹出 3D 渲染窗口
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=true gui:=true

# 2. 纯后台无头模式运行 (Headless, 适用于自动化脚本与 CI)
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=true
```

#### 独立单手/单臂 Launch (兼容既有习惯)
```bash
ros2 launch tuinadex_hand hand.launch.py use_sim:=true|false mock:=true
ros2 launch tuinadex_arm arm.launch.py mock:=true
ros2 launch tuinadex_teleop teleop.launch.py
```

### 4.3 自动化端到端回环验证 (E2E Checks)

- **22 维臂手协同与双模式启动终验自检**:
  ```bash
  bash scripts/e2e_unified_robot_check.sh
  ```
  *验证指标: Mock 硬件启动、MuJoCo 仿真启动、22 维指令分发与状态聚合、300ms 看门狗联合回退、直通 LeRobotDataset v3.0 22 维多模态导出。*

- **MuJoCo 仿真统一入口闭环自检**:
  ```bash
  bash scripts/e2e_sim_check.sh
  ```
  *验证指标: 50Hz 控制发布、MuJoCo 10 步/tick 积分、130+ 帧状态接收、周期中位数 19.97ms、物理收敛并平滑回退 OPEN。*

- **Mock 驱动示教录制与数据集转换闭环自检**:
  ```bash
  bash scripts/e2e_mock_check.sh
  ```
  *验证指标: Mock 控制节点 + 示教录制 + 50Hz 离线时间对齐 + LeRobot Parquet 数据集落地。*

- **多模态数据采集与 LeRobotDataset v3.0 闭环自检**:
  ```bash
  bash scripts/e2e_collection_check.sh
  ```
  *验证指标: 仿真环境、双目相机 (head, wrist) 视频流分块编码、50Hz 关节状态对齐、生成原生 LeRobotDataset v3.0 目录 (MP4 + Parquet)。*

- **三机位 Web Hub 采集与 H.264 导出自检**:
  ```bash
  bash scripts/e2e_hub_collection_check.sh
  ```
  *验证指标: 真实 EGL 三路渲染、22D Mock 遥操、REST/SSE/MJPEG Hub、异步保存、三路 H.264 与 LeRobot v3.0 Parquet 产物。*

### 4.4 多模态数据采集快速上手 (Data Collection Quickstart)
```bash
# 激活具身基座环境
source scripts/activate_tuinadex.sh control

# 1. 启动三机位 Web 采集 Hub（默认仅绑定 127.0.0.1）
python scripts/collect_teleop_data.py --env sim --mode arm_hand --mock-teleop \
  --hub --headless --port 8080 --repo-id data/hub_run

# 2. 运行 16D 手部终端采集
python scripts/collect_teleop_data.py --env sim --mode hand_only \
  --cameras head,wrist --repo-id data/tuina_demo

# 3. 一键切换至真机数据采集 (上层录制与 LeRobot 代码零改动)
python scripts/collect_teleop_data.py --env real --mode hand_only \
  --cameras head,wrist --repo-id data/tuina_real_demo
```

Hub 启动后 stdout 输出 `HUB_READY port=<n>`。浏览器打开
`http://127.0.0.1:<n>` 即可查看 `overhead / wrist / side` 三路画面、
22D 关节与 4 指接触力遥测，并通过 REST 命令完成
`start / pause / resume / stop / save / discard / label` 操作。

### 4.5 运行多矩阵单元测试
```bash
# 1. 基础逻辑、遥操映射与仿真桥接测试 (tuinadex-control 环境)
~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/ -v

# 2. LeRobot 数据集适配器与学习模型测试 (tuinadex-vla 环境)
~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_dataset_sync_validator.py tests/test_dataset_adapter.py -v

# 3. ROS2 节点级集成与双模测试 (ROS2 主机环境)
source /opt/ros/lyrical/setup.bash
python3 -m pytest tests/test_leap_hand_node_integration.py tests/test_recorder_node.py tests/test_sim_node_integration.py -v
```

---

## 5. 项目结构 (Repository Structure)

```text
TuinaDex_v2.0/
├── .agent/                      # 多 Agent 协作接力黑板与状态 (SESSION_STATE.md)
├── docs/
│   ├── specs/                   # 冻结的 16-DOF 接口与通信规范
│   ├── adrs/                    # 架构决策记录 (ADR 0001: MVP, ADR 0002: Simulation)
│   ├── architecture/            # 交互式架构全景图 (tuinadex_sim_digital_twin.html)
│   └── superpowers/plans/       # Phase 3 实施规划与审查记录
├── dataset/                     # 离线数据处理与 LeRobot 适配器包 (tuinadex_dataset)
│   └── tuinadex_dataset/        # bag_reader, synchronizer, validator, adapter, cli
├── src/                         # ROS2 源码空间 (8 个模块化功能包)
│   ├── tuinadex_core/           # 共享纯逻辑包 (规范关节名、逐关节限位、align_command、Watchdog FSM)
│   ├── rebot_control/           # reBot B601-RS 机械臂底层电机与平滑算法控制 SDK
│   ├── tuinadex_arm/            # 6-DOF 机械臂 ROS2 运行时、SocketCAN 驱动与看门狗
│   ├── tuinadex_hand/           # 16-DOF 物理灵巧手运行时、Dynamixel 串口驱动
│   ├── tuinadex_sim/            # MuJoCo 数字孪生包 (URDF/STL 资产、tuina_scene.xml、MujocoSimBridge)
│   ├── tuinadex_bringup/        # 臂手协同整机统一启动 Launch 包 (robot.launch.py)
│   ├── tuinadex_recorder/       # 进程内 rosbag2 示教采集节点
│   └── tuinadex_teleop/         # 统一多模态环境、三机位采集、Web Hub 与遥操示教
├── tests/                       # 全量测试套件 (80+ 纯逻辑单测 + 12 ROS 2 集成测试覆盖臂手双模)
├── scripts/                     # 自动化测试与辅助脚本 (e2e_sim_check.sh, e2e_mock_check.sh)
├── AGENTS.md                    # 项目专属规则层与硬件红线
└── README.md                    # 本文档
```

---

## 6. 核心架构决策记录 (Architecture Decision Records)
- [ADR 0001: LEAP Hand MVP 核心架构决策记录](docs/adrs/0001-architecture-decisions-mvp.md) (状态机选型、rosbag2 采集器、双域解耦、双重限位护栏)
- [ADR 0002: MuJoCo 仿真数字孪生与真机统一切换架构决策记录](docs/adrs/0002-architecture-decisions-sim.md) (tuinadex_core 下沉、单线程 50Hz/500Hz 步进、Hold+Ramp 回退保真、统一启动切换)
