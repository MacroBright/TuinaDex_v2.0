# TuinaDex v2.0 多端异构环境管理架构设计规范 (ENVIRONMENT_MANAGEMENT_SPEC)

> **文档标识**: `SPEC-ENV-2026-10-14`  
> **生效状态**: `Approved / Architecture Frozen`  
> **适用范围**: 本地开发机 (Legion Y9000P)、香橙派 AI Pro 20T (边缘中枢)、云端 GPU 服务器、全线 ROS 2 功能包与数据管道

---

## 1. 架构目标与核心不变量 (Core Principles & Invariants)

### 1.1 总体架构目标
TuinaDex v2.0 是一套面向中医推拿/理疗场景的具身智能机械臂-灵巧手系统。本规范旨在建立一套**标准化、低维护摩擦、支持长期演进**的跨端多环境管理体系：
1. **一个主代码仓库**：整机核心 ROS 2 源码、Xacro/URDF 描述、环境构建脚本与测试集统一受控；
2. **算力与实时性分级自治**：
   - **重算力层（云端 GPU）**：承载 VLA 大模型推理（Action Chunking 推流）与 LeRobot 离线策略训练；
   - **中枢执行层（香橙派 AI Pro 20T）**：承载 30fps 昇腾 NPU 穴位识别、SocketCAN/USB 电机硬件闭环以及毫秒级安全看门狗；
   - **示教开发层（本地开发 PC）**：承载专家示教遥操作录制、MuJoCo 仿真与数据适配；
3. **跨 Ubuntu/ROS2 发行版透明通信**：抹平本地开发机（Ubuntu 26.04 / Lyrical）与香橙派（Ubuntu 22.04 / Humble）之间的版本鸿沟。

### 1.2 核心架构不变量 (Invariants)
- **[INV-01] DDS 线路兼容不变量**：跨设备通信全网统一使用 `Eclipse CycloneDDS`（`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`），锁定 `ROS_DOMAIN_ID=42`。跨设备网络严禁传递未经严格双端版本测试的复杂自定义消息，全部使用 ROS 2 标准接口（`sensor_msgs`、`geometry_msgs`、`std_msgs`）。
- **[INV-02] 硬件与单位契约不变量**：
  - 机械臂（6-DoF reBot-DevArm RS）与灵巧手（16-DoF LEAP Hand）在 ROS 2 与 LeRobot 数据流中全链路强制使用 **SI 国际标准单位：弧度 (rad)**；
  - 灵巧手以实测标定 `OPEN_POSE`（全开平伸）为 $0\text{ rad}$ 基准，弯曲屈曲为正 ($q > 0$)，顺序严格固定为 16 维 Canonical Ordering。
- **[INV-03] 软硬安全看门狗不变量**：
  - 云端 VLA 动作 Chunk 丢失或网络抖动超过 `300 ms` 时，香橙派本地控制循环必须无条件熔断外部指令，自动进入阻尼减速回退态（`Relax-to-OPEN`）；
  - 灵巧手电机电流硬件阈值锁死在 `150 mA`；机械臂运动限制在安全工作空间（$\le 70\%$ 臂展）。
- **[INV-04] 数据集事实源不变量**：
  - 真实物理实验与示教记录以 `rosbag2` 为唯一事实源（Source of Truth）；
  - `LeRobotDataset` 为下游衍生产品，由独立容器化工具从 rosbag2 单向转换生成。

---

## 2. 节点与环境划分矩阵 (Environment Matrix)

系统划分为五套严格职责隔离的运行时环境：

```
                               ┌─────────────────────────────────────────────────────┐
                               │             [ENV-CLOUD] 云端 GPU 服务器             │
                               │  Docker (nvidia-docker, CUDA 12.4, PyTorch 2.4)     │
                               │  • VLA 推理服务 (gRPC / Zenoh Action Streaming)     │
                               │  • LeRobot 策略离线训练                             │
                               └──────────────────────────▲──────────────────────────┘
                                                          │ gRPC / Action Chunks (10~15Hz)
                                                          ▼
┌───────────────────────────────────────┐      ┌─────────────────────────────────────┐
│       [ENV-DEV] 本地开发机 (Legion)    │      │    [ENV-EDGE] 香橙派 AI Pro 20T     │
│  Ubuntu 26.04 + ROS 2 Lyrical (原生)  │      │  Ubuntu 22.04 aarch64 + CANN 8.x    │
│                                       │      │                                     │
│  • ENV-DEV-HW (Conda arm_robot/leap)  │      │  • 昇腾 NPU 穴位识别 (pyACL, 30fps) │
│    - 专家遥操示教采集 (Teleop)        │◄────►│  • 底层总线: SocketCAN + USB-TTL    │
│    - 硬件低延迟直连联调               │ DDS  │  • 统一机器人控制中枢                │
│  • ENV-DEV-SIM                        │ (42) │  • 本地安全看门狗 (100Hz)           │
│    - MuJoCo 物理仿真 / RViz 运动学    │      │  • VLA 动作平滑插值执行             │
│  • ENV-DATA (Docker)                  │      │                                     │
│    - rosbag2 -> LeRobotDataset 转换   │      │                                     │
└───────────────────────────────────────┘      └─────────────────────────────────────┘
```

### 2.1 详细技术配置表

| 环境代码 | 所在物理主机 | OS / 架构 | ROS 2 发行版 | Python / 工具链 | 核心依赖包 | 隔离管理方式 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`ENV-DEV-HW`** | 本地 PC | Ubuntu 26.04<br>x86_64 | ROS 2 Lyrical<br>(/opt/ros/lyrical) | Python 3.10 (Conda)<br>+ Python 3.14 (系统) | `motorbridge`<br>`ros-lyrical-pinocchio`<br>`pyserial`<br>`sensor_msgs` | **Conda 虚拟环境** (`arm_robot`, `leap_hand`)，直通物理串口与 SocketCAN |
| **`ENV-DEV-SIM`** | 本地 PC | Ubuntu 26.04<br>x86_64 | ROS 2 Lyrical | Python 3.14 / 3.12 | `mujoco 3.x`<br>`rviz2`<br>`moveit2` | **宿主机原生环境**，复用本地 RTX 3060 OpenGL 硬件加速 |
| **`ENV-DATA`** | 本地 PC / 服务器 | Linux x86_64 | 可选 ROS 2 | Python 3.10<br>(Docker 锁死) | `lerobot`<br>`rosbags`<br>`ffmpeg 7.1` (libsvtav1) | **Docker 容器** (`tuinadex-data:v2.0`)，规避本地 Python 3.14 对旧版 PyTorch 的兼容冲突 |
| **`ENV-CLOUD`** | 云端 GPU 节点 | Linux x86_64<br>(RTX 3090/A100) | 无需 ROS 2 | Python 3.10+<br>CUDA 12.4 | `lerobot`<br>`transformers`<br>`grpcio`<br>`zenoh-python` | **Docker 容器** (`tuinadex-vla:latest`)，挂载 GPU 提供远程动作生成服务 |
| **`ENV-EDGE`** | 香橙派 AI Pro 20T | 官方 Ubuntu 22.04<br>aarch64 | ROS 2 Humble<br>(/opt/ros/humble) | Python 3.10 (系统)<br>CANN 8.x | `AscendCL (pyACL)`<br>`ros-humble-pinocchio`<br>`motorbridge`<br>`cyclonedds` | **宿主机原生专有环境**，直通 `/dev/davinci*`、`/dev/ttyUSB*`、`can0` |

---

## 3. 物理总线与设备流转迁移规范

### 3.1 硬件外设连接规范

| 设备名称 | 物理电气接口 | 协议与波特率 | 开发测试阶段连接宿主 | 最终落地阶段连接宿主 | 驱动模块路径 |
| :--- | :--- | :--- | :---: | :---: | :--- |
| **reBot-DevArm RS**<br>(6-DoF 机械臂) | USB-CAN 适配器<br>(SocketCAN `can0`) | RobStride 协议<br>`1,000,000 bps` | **本地 PC** | **香橙派 AI Pro** | `motorbridge` + `reBotArm_control_py` |
| **LEAP Hand**<br>(16-DoF 灵巧手) | USB-TTL 串口<br>(`/dev/ttyUSB*`) | Dynamixel Protocol 2.0<br>`4,000,000 bps` | **本地 PC** | **香橙派 AI Pro** | `src/tuinadex_hand/driver/` |
| **RGB-D 相机**<br>(腕部/全局相机) | USB 3.0 Type-C | UVC / V4L2 视频流 | **本地 PC** (示教录制) | **香橙派 AI Pro** | 官方 ROS 2 相机驱动 |

### 3.2 阶段流转机制
1. **阶段 1：本地闭环示教阶段**  
   - 机械臂与灵巧手全部插入本地 PC；
   - 启动本地硬件驱动节点，下发目标指令，录制高质量 `rosbag2` 示教数据；
   - 香橙派独立挂载相机调试穴位识别，通过 DDS 广播穴位坐标 `/perception/acupoints`。
2. **阶段 2：边缘中枢一体化落地阶段**  
   - 机械臂 CAN 盒与灵巧手 USB 转接线拔出，直接插入香橙派 AI Pro 的 USB 接口；
   - 在香橙派上启动整机中枢 Launch，本地 PC 转为只读监视器（RViz 可视化）。

---

## 4. 跨端通信与接口协议契约

### 4.1 跨 Ubuntu / ROS 2 版本 DDS 互通机制
- **网络互联要求**：本地 PC 与香橙派接入同一局域网（有线以太网或 5GHz 专网 Wi-Fi）；
- **DDS 配置规范**：
  在两端主机的 `~/.bashrc` 中声明：
  ```bash
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='eth0'/></Interfaces></General></Domain></CycloneDDS>"
  ```
- **核心数据话题**：
  - `/perception/acupoints` (`geometry_msgs/PoseArray`, 30Hz): 穴位 3D 位姿；
  - `/hand/joint_states` (`sensor_msgs/JointState`, 50Hz): 灵巧手 16 维反馈；
  - `/arm/joint_states` (`sensor_msgs/JointState`, 50Hz): 机械臂 6 维反馈；
  - `/hand/joint_command` (`sensor_msgs/JointState`, 50Hz): 灵巧手目标控制；
  - `/arm/joint_command` (`sensor_msgs/JointState`, 50Hz): 机械臂目标控制。

### 4.2 云端 VLA 远程推流契约 (Cloud-to-Edge Action Streaming)
- **协议选型**：高性能异步 **gRPC** 或 **Zenoh** 传输层（Payload 经 Protobuf 序列化）；
- **接口模型**：**Action Chunking 机制**
  - **请求 (Edge $\to$ Cloud)**：当前时刻 RGB 图像帧 + 深度信息 + 当前 22 维关节实际状态 + 穴位坐标；
  - **响应 (Cloud $\to$ Edge)**：未来 $K$ 步（通常 $K=15\sim 30$，对应 $0.5\sim 1.0\text{ s}$）的目标关节轨迹 Chunk $\mathbf{A} \in \mathbb{R}^{K \times 22}$。
- **边缘端平滑插值引擎**：
  香橙派内置 100Hz 局部样条插值器（Cubic Spline），即使云端推流频率仅为 10Hz，底层执行也能保持 100Hz 的丝滑平顺运动。

---

## 5. 项目工程化目录与构建规范

主仓库根目录下统一组织多端配置资产：

```text
TuinaDex_v2.0/
├── .agent/
│   └── SESSION_STATE.md            # 多 Agent 协同与开发状态黑板
├── docker/
│   ├── data.Dockerfile             # ENV-DATA: LeRobot 数据清洗与转换镜像
│   └── vla_cloud.Dockerfile        # ENV-CLOUD: 云端 VLA 推理服务镜像
├── scripts/
│   ├── env/
│   │   ├── init_dev_pc.sh          # 本机 Ubuntu 26.04 环境一键检测与配置
│   │   └── init_orangepi.sh        # 香橙派 Ubuntu 22.04 一键配置脚本
│   ├── run_data_convert.sh         # 一键唤起 Docker 进行数据集转换
│   └── setup_cyclonedds.sh         # 跨端 CycloneDDS 快速配置脚本
├── docs/
│   ├── specs/
│   │   ├── LEAP_HAND_MVP_INTERFACE_SPEC.md
│   │   └── ENVIRONMENT_MANAGEMENT_SPEC.md  # 本规范文档 (软链接)
│   └── superpowers/specs/
│       └── 2026-10-14-environment-management-architecture-design.md
└── src/
    ├── tuinadex_core/               # 核心常量、安全看门狗、数学转换
    ├── tuinadex_hand/               # 16-DOF LEAP Hand 纯 Python 运行时
    ├── tuinadex_arm/                # 6-DOF reBot-DevArm RS 控制节点 (待集成)
    ├── tuinadex_teleop/             # 专家示教遥操作
    ├── tuinadex_recorder/           # rosbag2 自动录制调度器
    ├── tuinadex_sim/                # MuJoCo 仿真与数字孪生
    └── tuinadex_bringup/            # 整机多模式联合启动 Launch
```

---

## 6. 环境健康度自检与验证基准 (Health Check & Verification)

每个环境在投入使用前，必须执行以下基准验证命令并全绿通过：

### 6.1 本地 PC 开发机 (ENV-DEV-HW & SIM)
```bash
# 1. 验证 Lyrical 基础功能包编译
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select tuinadex_core tuinadex_hand tuinadex_sim

# 2. 纯逻辑与硬件 Mock 单元测试
~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v

# 3. MuJoCo 仿真与 ROS2 节点集成回归
python3 -m pytest tests/test_leap_hand_node_integration.py tests/test_sim_node_integration.py -v
```

### 6.2 香橙派边缘端 (ENV-EDGE)
```bash
# 1. 检查昇腾 NPU 状态与驱动
npu-smi info

# 2. 检查 SocketCAN 接口
ip -details link show can0

# 3. 验证 CycloneDDS 跨端联通测试 (与本机双向 ping)
ros2 topic pub /ping std_msgs/String "data: 'hello from orangepi'"
```

### 6.3 云端推理容器 (ENV-CLOUD)
```bash
# 1. 检查 CUDA 与 PyTorch 加速
python3 -c "import torch; assert torch.cuda.is_available(); print('CUDA OK')"

# 2. 验证 gRPC VLA 测速服务
python3 -m pytest tests/test_vla_service_latency.py -v
```

---

## 7. 动态维护与演进机制 (Evolution & Governance)

1. **项目黑板状态同步**：后续新增硬件外设（如足底踏板、脚部微动、触觉阵列传感器）时，必须在 `.agent/SESSION_STATE.md` 中记录其总线分配，并同步更新本规范。
2. **多 Agent 接力准则**：后续无论是 Claude Code 编写实施方案，还是 Codex 编写底层 SocketCAN 驱动，均严格以本规范中的端口定义、话题契约、安全看门狗阈值为最高准则，不得随意私自增加跨端非标依赖。
