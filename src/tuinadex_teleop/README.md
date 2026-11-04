# tuinadex_teleop: 机械臂-灵巧手协同仿真与视-惯正交遥操作系统

`tuinadex_teleop` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **协同仿真与多模态遥操作系统（Layer 4: Middleware Layer）**。

本包针对 **reBot-DevArm RS 机械臂 (6-DOF) + LEAP Hand 仿生灵巧手 (16-DOF)** 构成的 22 自由度复合机器人系统，提供视-惯正交融合传感（Intel RealSense 深度相机空间定位 + 腕部 6 轴 IMU 姿态解算）、Pink/Pinocchio QP-IK 机械臂实时逆解、16-DOF 灵巧手运动学几何重定向、4 级安全门限守护、60Hz 高速低延迟通信以及高保真 MuJoCo 数字孪生协同仿真环境。

---

## 快速开始 (Quick Start)

> [!TIP]
> 推荐直接使用 TuinaDex 统一 CLI 工具 **`tuina`** 进行交互（无需手动繁琐激活 Conda 与加载环境变量，自带环境自感知能力）。同时支持纯仿真 `--mock` 离线体验模式。

### 1. 方式 A：统一 CLI 启动（推荐，极简零门槛）

```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0

# 1. 启动 22D 实时协同遥操预览 (真实传感器: RealSense D455 + 手腕 IMU)
./bin/tuina teleop sim

# 2. 离线虚拟体验 (无需物理硬件，自动生成人手测试轨迹，带 3D 物理画面与 OpenCV HUD 仪表盘)
./bin/tuina teleop sim --mock

# 3. 纯终端无头运行 (适用于 SSH 远程开发或 CI 自动化回归测试)
./bin/tuina teleop sim --mock --no-gui
```

### 2. 方式 B：交互式标定向导 (Pre-Flight Calibration Wizard)
在进行高精度遥操作示教前，一键运行交互式标定向导，自动生成统一标定配置 `configs/session_calibration.yaml`：

```bash
# 启动桌面物理平面 + 手腕 IMU 姿态零位 + 灵巧手活动度 (ROM) 交互式标定
./bin/tuina teleop calibrate

# 或仅对手腕 6 轴 IMU 进行 3 姿态正交对齐标定 (中立平放 -> 俯仰上翘 -> 向右翻滚)
./bin/tuina teleop calibrate --imu-only --imu-port /dev/ttyUSB0
```

### 3. 方式 C：ROS 2 复合节点启动 (60Hz 高频通信)
适合**接入 ROS 2 通信网络，供整个系统协同调度与数据录制**：

```bash
# 途径 1: 通过 tuina CLI 统一拉起 60Hz 遥操节点
./bin/tuina teleop node --rate-hz 60.0 --camera-fps 60

# 途径 2: 原生 ros2 launch 拉起 (真实硬件连接)
source /opt/ros/lyrical/setup.bash && source install/setup.bash
ros2 launch tuinadex_teleop teleop_sim.launch.py use_sim:=true mock:=false mock_imu:=false gui:=true

# 途径 3: 纯仿真 Mock 模式拉起 (无需硬件，自动合成 60Hz 信号)
ros2 launch tuinadex_teleop teleop_sim.launch.py use_sim:=true mock:=true mock_imu:=true gui:=true
```

### 4. 方式 D：一边仿真遥操一边数据采集 (双 Hub 联动工作流)
系统支持 **ROS 2 话题总线** 与 **本地零延迟 UDP 桥 (127.0.0.1:9876)** 两种高速直通链路，全模态对齐 Hugging Face LeRobotDataset v3.0 规范：

```bash
# 终端 1：启动 60Hz 视-惯遥操作节点 (弹出 OpenCV 交互仪表盘，下发 22D 动作目标)
./bin/tuina teleop node --rate-hz 60.0

# 终端 2：启动 Web UI 采集工作台 (自动监听遥操指令 + 3机位 50Hz 实时视频流与遥测)
./bin/tuina collect hub --env sim --mode arm_hand --cameras overhead,wrist,side --fps 50
```
- 打开浏览器访问 👉 **`http://127.0.0.1:8080`**；
- 监视 3 机位视频墙（俯视 60°、腕部 75°、侧向 45°）与 22 维实时关节/接触力曲线；
- 点击页面中 **【开始录制 (Start)】** 开始采集；示教完成后点击 **【停止录制 (Stop)】** $\rightarrow$ **【保存试次 (Save Episode)】**；
- 系统自动多路流式编码生成包含 H.264 MP4 视频与 Parquet 表格的标准 `LeRobotDataset v3.0`。

---

## 核心高频快捷键速查卡 (HUD Hotkeys)

在遥操作视窗激活时（`tuina teleop sim` 或 `teleop_sim_composite_node` 的 OpenCV 视窗），可敲击以下快捷键实时操控：

| 按键 | 核心功能 | 详细行为说明 |
| :---: | :--- | :--- |
| **`[空格]`** / **`[C]`** | **离合器开关 (Clutch)** | **核心控制键**。在 `ENGAGED` (接管遥控) 与 `DISENGAGED` (挂起安全) 间切换；接管瞬间自动锚定当前人手位置与 IMU 零位，实现**无冲击平滑接管**。 |
| **`[H]`** | **安全就绪姿态 (Safe Home)** | 机械臂末端平滑返回推拿预备位姿 (`DEFAULT_READY_ARM_Q`)，手掌朝下正对推拿床假人背部。 |
| **`[S]`** | **安全防护门限切换** | **一键热切换**：在 `ACTIVE` (有安全门限守护，120N 力/防跳跃保护) 与 `BYPASS` (无安全门限自由模式) 之间随时切换。 |
| **`[Z]`** | **灵巧手平手归零复位** | **人手自然张开平放时按下**：动态固化当前平手姿态为零偏基准并保存，同时令仿真端 16 关节全部复位展平（0.00 rad），彻底消除残余弯曲。 |
| **`[I]`** | **IMU 姿态对齐归零** | 将手腕当前朝向锁定为机械臂末端的对应姿态基准。 |
| **`[M]`** | **机械臂姿态跟踪模式循环** | 循环切换：`TRACK_IMU` (全姿态跟随) $\rightarrow$ `FIXED_DOWNWARD` (垂直向下) $\rightarrow$ `TRACK_WRIST` (纯视觉) $\rightarrow$ `POSITION_ONLY` (仅控位置)。 |
| **`[A]`** | **位移映射轴向切换** | 实时轮换位置正交对齐（默认 `-y,+x,+z` 适配 90° 床侧基座旋转安装）。 |
| **`[X]`** | **手腕 IMU 姿态轴向切换** | 实时轮换手腕 IMU 姿态轴向（大写 `[X]` 为反向轮换，严格保证 $\det=+1$）。 |
| **`[1]`/`[2]`/`[3]`** | **灵巧手解算模式切换** | 1: Hybrid 混合优化模式 (默认)；2: 纯几何映射模式；3: DexPilot 模式。 |
| **`[R]`** | **仿真一键重置** | 复位离合器、滤波器及 MuJoCo 仿真模型至 Ready 就绪姿态。 |
| **`[Q]`** / **`[ESC]`** | **安全退出** | 安全释放相机与 IMU 串口资源并关闭所有窗口。 |

---

## 1. 系统架构与数据流拓扑

```
[人手空间定位 (RealSense D455)] ────► [WristTracker (Scheme C)] ──┐
                                                                 │
[手腕 6 轴姿态 (Yahboom IMU)]  ────► [WristImuTracker]          ├──► [ClutchController]
                                                                 │        │
[人手 21 个骨骼关节点]         ────► [HandRetargeter (16-DOF)]  │        ▼
                                                                 ├──► [SafetySupervisor (4-Gate)]
                                                                 │        │
                                     [ArmIKSolver (Pink QP)]    ◄┘        ▼
                                                                    /arm/joint_command (6D, 60Hz)
                                                                    /hand/joint_command (16D, 60Hz)
                                                                          │
                                                      ┌───────────────────┴───────────────────┐
                                                      ▼                                       ▼
                                           [MujocoSimAdapter (500Hz)]             [UDP Teleop Bridge :9876]
                                                      │                                       │
                                                      ▼                                       ▼
                                           /arm/joint_states (6D)               [Web UI Collection Hub]
                                           /hand/joint_states (16D)             [LeRobotDataset v3.0]
```

1. **正交传感融合 (Orthogonal Sensing)**:
   - 位置信息由深度相机基于桌面 ArUco 物理平面提取，经 One-Euro 滤波消除抖动；
   - 姿态信息由高采样率 6 轴 IMU 提供，彻底解决手掌翻转时视觉严重遮挡与关键点翻转跳变缺陷。
2. **60Hz 高帧率流水线与双模传输**:
   - 遥操主频升级至 **60Hz**，USB3 相机自动启用 60FPS 采集，实现端到端 $<25\text{ms}$ 极低延迟响应；
   - 支持 ROS 2 标准话题与原生 UDP 二进制桥，彻底解决跨 Python 3.10 / 3.14 环境通信壁垒。
3. **4 级安全门限守护 (SafetySupervisor)**:
   - **Gate 1 (感知丢失检测)**: 手部丢失超 800ms 触发安全悬停；
   - **Gate 2 (位姿跳跃防护)**: 拦截帧间位移 $>0.18\text{m}$ 或角度 $>60^\circ$ 的奇异跳变；
   - **Gate 3 (末端接触力限制)**: 接触总力超过 $120\text{N}$（单指 $>70\text{N}$）触发柔顺卸力回退；
   - **Gate 4 (奇异点防御)**: 逆解雅可比奇异值低于门限时自动阻尼锁定。

---

## 2. 节点与 Launch 接口契约

### 节点: `teleop_sim_composite_node`
- **执行入口**: `ros2 run tuinadex_teleop teleop_sim_composite_node`
- **默认控制频率**: 60 Hz (`publish_rate_hz: 60.0`)

#### 发布话题 (Published Topics)
| 话题名 | 消息类型 | 频率 (Hz) | 语义说明 |
| :--- | :--- | :--- | :--- |
| `/arm/joint_command` | `sensor_msgs/msg/JointState` | 60 Hz | 机械臂 6 关节目标角度 (SI rad) |
| `/hand/joint_command` | `sensor_msgs/msg/JointState` | 60 Hz | 灵巧手 16 关节目标角度 (SI rad) |
| `/teleop/status` | `std_msgs/msg/String` | 10 Hz | 离合状态、帧率、安全门报告 JSON |

#### 订阅话题 (Subscribed Topics)
| 话题名 | 消息类型 | 频率 (Hz) | 语义说明 |
| :--- | :--- | :--- | :--- |
| `/arm/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz | 机械臂实测物理位置反馈 (供 IK 求解参考) |
| `/hand/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz | 灵巧手实测物理状态反馈 |
| `/teleop/home` | `std_msgs/msg/Empty` | 事件驱动 | 远程触发机械臂安全回零 |

---

## 3. 目录与配置资产结构

```text
src/tuinadex_teleop/
├── configs/
│   ├── mjcf/
│   │   ├── tuina_sim_scene.xml      # 22-DOF 一体化推拿全场景动力学模型
│   │   └── rebot_arm_leap_hand.xml  # 22-DOF 臂手一体纯机器人模型
│   ├── urdf/
│   │   ├── rebot_arm_rs.urdf        # 6-DOF 机械臂运动学模型 (供 Pink IK 求解)
│   │   └── leap_hand_right.urdf     # 16-DOF 灵巧手运动学模型
│   ├── session_calibration.yaml     # 桌面、IMU 与灵巧手全集成统一标定配置文件
│   ├── table_calibration.yaml       # 桌面物理平面 ArUco 标定参数
│   ├── imu_calibration.yaml         # 腕部 IMU 姿态对齐标定参数
│   ├── teleop_config.yaml           # 遥操作全局安全阈值与时钟配置
│   └── leap_retargeting_config.yaml # 灵巧手关节优化权重与几何缩放配置
├── launch/
│   ├── teleop_sim.launch.py         # 22 维复合仿真遥操 ROS 2 启动入口 (支持 60Hz)
│   └── teleop.launch.py             # 纯手势遥操启动入口
├── tuinadex_teleop/
│   ├── adapters/                    # MujocoSimAdapter, MockSimAdapter
│   ├── core/                        # clutch.py, safety.py, types.py, names.py
│   ├── kinematics/                  # arm_ik_solver.py (Pink QP), hand_retargeter.py
│   ├── perception/                  # wrist_tracker.py, imu_stream.py, realsense_stream.py
│   ├── ui/                          # teleop_hud.py (高密度 OpenCV 实时仪表盘)
│   ├── hub/                         # Web UI 数据采集工作台前端与 SSE 推送
│   ├── collection/                  # data_hub_controller.py (多模态采集与视频转码)
│   ├── teleop_sim_composite_node.py # 22-DOF 复合仿真遥操 ROS 2 主节点
│   └── teleop_gesture_node.py       # 纯手部视觉遥操节点
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 4. 自动化测试与验证

```bash
# 1. 运行遥操与算法核心单元测试 (81 项全绿，覆盖 IK、重定向、离合器与安全门)
source scripts/activate_tuinadex.sh control
pytest tests/teleop/ -v

# 2. 运行 ROS 2 复合节点通信与物理闭环集成测试
source /opt/ros/lyrical/setup.bash && source install/setup.bash
python3 -m pytest tests/test_teleop_sim_composite_node.py -v

# 3. 运行全系统 22 维联合控制与启动端到端 E2E 自检
bash scripts/e2e_unified_robot_check.sh
```
