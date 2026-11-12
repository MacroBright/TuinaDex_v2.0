# tuinadex_hand: 16-DOF LEAP Hand 物理驱动、标定工具箱与 ROS 2 运行时

`tuinadex_hand` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **16 自由度 LEAP Hand 仿生灵巧手物理驱动与 ROS 2 节点功能包（Layer 2 & 1: Driver & Hardware Layer）**。

本包负责通过 U2D2 适配器与 16 个 Dynamixel XC330 舵机建立 4Mbps 高速串口通信，提供 50Hz 闭环位置控制、实测状态遥测发布、电流硬限位保护以及 300ms 看门狗安全平滑回退；同时完整内置了灵巧手专属的**交互控制终端 (`leap-control`)、零点标定与推拿手法姿态录制器 (`leap-calibrate`) 以及物理限位扫描体检诊断工具 (`leap-diagnose`)**。

---

## 快速开始 (Quick Start)

### 1. 硬件准备与串口权限配置
连接 U2D2 至电脑 USB 端口，确保当前用户具备串口读写权限：
```bash
# 赋予当前用户 dialout 权限 (首次配置需重新登录生效)
sudo usermod -a -G dialout $USER

# 确认串口设备节点存在 (默认映射为 /dev/ttyUSB0)
ls -l /dev/ttyUSB0
```

### 2. 环境激活与工作空间加载
```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0

# 方式 A: 激活 Python 控制环境 (推荐，支持直接敲 CLI 命令)
source scripts/activate_tuinadex.sh control

# 方式 B: 加载 ROS 2 工作空间
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

### 3. 硬件调试与视觉遥操工具箱 (CLI Tools)

| 工具命令 | 核心定位与功能 | 典型使用命令 |
| :--- | :--- | :--- |
| **`leap-control`** | 交互式逐指/手势控制台 (支持相对弧度、手势切换、电流与状态监控) | `leap-control --port /dev/ttyUSB0`<br>*(脱机演示: `leap-control --mock`)* |
| **`leap-calibrate`** | 硬件零点 (0 rad 全开位) 校准与推拿手法姿态录制 | `leap-calibrate --action 1`<br>*(脱机演示: `leap-calibrate --mock`)* |
| **`leap-diagnose`** | 16 舵机物理限位扫描体检与 $2\pi$ 回绕解算 | `leap-diagnose --port /dev/ttyUSB0`<br>*(脱机演示: `leap-diagnose --mock`)* |
| **`leap-teleop`** | 实时视觉手势遥操终端 (复用底层驱动 + 仿真验证关键点检测与重定向，带 OpenCV HUD) | `leap-teleop --port /dev/ttyUSB0`<br>*(脱机演示: `leap-teleop --mock --mock-camera`)* |

> [!TIP]
> 以上四项调试命令在 `tuinadex-control` Conda 环境激活后可直接在终端中调用；在 ROS 2 环境中亦可使用 `ros2 run tuinadex_hand <command>` 调用。

#### A. 实时视觉遥操终端 (`leap-teleop`) 使用示例
```bash
# 真实相机 + 真实灵巧手
leap-teleop --port /dev/ttyUSB0 --camera-id 0

# 脱机全模拟调试 (无硬件与无真实相机)
leap-teleop --mock --mock-camera
```
HUD 控制快捷键：
- `SPACE`: 离合切换（暂停 / 继续向舵机下发手势控制指令）；
- `L`: 锁定当前手势（保持当前关节姿态）；
- `O`: 立即回归全开平伸位姿（`OPEN_POSE`）；
- `Q` / `ESC`: 安全平滑复位至全开位、释放扭矩并安全退出。

#### B. 交互式控制台 (`leap-control`) 使用示例
```bash
leap-control --port /dev/ttyUSB0
```
进入 REPL 提示符后常用命令：
```text
LEAP> state                # 显示 16 关节实测角度、全开基准、相对弧度与电流
LEAP> open                 # 立即回归全开平伸位 (0 rad)
LEAP> fist                 # 切换至预设姿势: 全握拳
LEAP> index mcp 0.5        # 食指 MCP 前后向手心弯曲 0.5 rad (正值弯曲)
LEAP> thumb mcp 0.3        # 拇指 MCP 关节弯曲 0.3 rad
LEAP> middle all 0.4       # 中指整指所有关节弯曲 0.4 rad
LEAP> poses                # 列出姿态库中已注册的所有姿态 (含中医推拿揉法)
LEAP> raw 1 4.50           # 调试模式: 直接向 1 号电机写入绝对硬件弧度
LEAP> help                 # 打印命令帮助清单
LEAP> quit                 # 安全复位至全开位并退出
```

#### C. 零点基准校准与推拿手法录制 (`leap-calibrate`)
```bash
# 模式 1: 零点校准 (重录 OPEN_POSE 作为系统 0 rad 绝对基准)
leap-calibrate --action 1

# 模式 2: 推拿手法姿势录制 (循环录制或新增自定义手法，持久化保存至 config/poses.json)
leap-calibrate --action 2
```
操作流程：转矩自动关闭 $\rightarrow$ 手动摆好推拿手型 $\rightarrow$ 按 Enter 键确认 $\rightarrow$ 驱动自动写入 `config/poses.json`，立即全局生效。

#### D. 16 舵机物理限位体检诊断 (`leap-diagnose`)
```bash
# 全量扫描 0-15 号电机物理限位
leap-diagnose --port /dev/ttyUSB0

# 仅诊断指定电机 (如电机 4)
leap-diagnose --port /dev/ttyUSB0 --motor 4
```
安全特性：转矩安全关闭，手动轻推手指到伸展与屈曲软停档位按 Enter，工具自动计算 $2\pi$ 编码器跨圈回绕，检验当前 OPEN_POSE 是否在极限区间内，并将数据落盘为 `config/motor_limits.json`。退出前自动恢复转矩并回到全开位。

---

### 4. 运行 ROS 2 节点

#### 模式 A：真实硬件启动 (连接实体 LEAP Hand)
> [!CAUTION]
> 实体上电前请确保灵巧手活动范围内无机械干涉，并确认工作电流上限锁定为 150mA。

```bash
ros2 launch tuinadex_hand hand.launch.py port:=/dev/ttyUSB0 mock:=false
```

#### 模式 B：离线 Mock 启动 (无物理硬件体验)
用于脱机开发、算法联调与 CI 自动化测试：
```bash
ros2 launch tuinadex_hand hand.launch.py mock:=true
```

### 5. 监听物理状态与下发测试指令
```bash
# 监听 50Hz 灵巧手实测关节弧度、速度与力矩反馈
ros2 topic echo /hand/joint_states

# 在新终端下发 16 关节弯曲测试指令 (持续 2 秒)
python3 scripts/mock_command_publisher.py --mode hand --position 0.4 --duration-sec 2.0
```

---

## 1. 节点与通信契约 (Node Interface Contract)

### 节点名称: `leap_hand_node`
- **执行体入口**: `ros2 run tuinadex_hand leap_hand_node`
- **默认控制频率**: 50 Hz (`control_rate_hz: 50.0`)
- **内部设计模式**: 普通 Node + 纯逻辑 `WatchdogStateMachine` (遵循 `tuinadex_core`)

#### 订阅话题 (Subscribed Topics)
| 话题名称 | 消息类型 | 频率 (Hz) | 语义与单位规范 |
| :--- | :--- | :--- | :--- |
| `/hand/joint_command` | `sensor_msgs/msg/JointState` | 50 Hz | 目标关节角 (16 维绝对弧度, SI rad)。经 `align_command` 进行 Canonical 重排与限位门过滤。 |

#### 发布话题 (Published Topics)
| 话题名称 | 消息类型 | 频率 (Hz) | 语义与单位规范 |
| :--- | :--- | :--- | :--- |
| `/hand/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz | 实测关节状态。包含 16 维物理位置 (rad)、角速度 (rad/s) 与负载力矩/电流 (mA)。 |

---

## 2. 参数配置 (`config/leap_hand_params.yaml`)

| 参数名 | 类型 | 默认值 | 物理单位 | 作用与安全含义 |
| :--- | :--- | :--- | :--- | :--- |
| `port` | `string` | `"/dev/ttyUSB0"` | - | U2D2 / USB-RS485 串口设备路径 |
| `baudrate` | `int` | `4000000` | bps | Dynamixel Protocol 2.0 通信波特率 (4 Mbps) |
| `curr_lim` | `int` | `150` | mA | **舵机工作电流硬上限** (防堵转过热与大力夹捏) |
| `kP` | `int` | `300` | - | 舵机内置位置 PID 比例增益 (Position P Gain) |
| `kI` | `int` | `0` | - | 舵机内置积分增益 (Position I Gain) |
| `kD` | `int` | `100` | - | 舵机内置微分增益 (Position D Gain) |
| `control_rate_hz` | `double` | `50.0` | Hz | ROS 2 定时器控制与状态发布频率 |
| `watchdog_timeout_sec` | `double` | `0.30` | s | **指令失联安全超时门限** (300ms) |
| `watchdog_hold_sec` | `double` | `0.30` | s | 超时后原位保持缓冲时间 (0.3s) |
| `watchdog_ramp_sec` | `double` | `0.60` | s | 超时后平滑阻尼回零过渡时间 (0.6s) |
| `shutdown_dwell_sec` | `double` | `0.5` | s | 节点安全退出前驻留归零等待时间 |
| `mock_mode` | `bool` | `false` | - | 是否启用内部软件 Mock 仿真驱动 |

---

## 3. 目录与代码拓扑

```
src/tuinadex_hand/
├── config/
│   ├── leap_hand_params.yaml     # 硬件通信参数、PID 与安全阈值配置
│   ├── poses.json                # 实测手势与中医推拿手法姿态库 (数据事实源)
│   └── motor_limits.json         # 16 舵机物理限位实测测量表
├── launch/
│   └── hand.launch.py            # 支持真机/Mock/Sim 多模自适应启动脚本
├── resource/
│   └── tuinadex_hand             # ROS 2 ament 索引
├── tuinadex_hand/
│   ├── __init__.py               # Python SDK 顶级导出
│   ├── leap_hand_node.py         # 50Hz ROS 2 主控节点与看门狗回路
│   ├── cli/                      # 硬件调试与标定命令行工具集
│   │   ├── __init__.py
│   │   ├── control.py            # leap-control: 逐指/手势交互控制终端
│   │   ├── calibrate.py          # leap-calibrate: 零点与推拿姿态录制器
│   │   └── diagnostics.py        # leap-diagnose: 物理限位扫描体检工具
│   ├── core/                     # 核心姿态与运动学算法 (纯 Python，解耦)
│   │   ├── __init__.py
│   │   ├── pose_manager.py       # 姿态持久化、有效性校验与 2pi 回绕解算
│   │   ├── kinematics_limits.py  # 运动学角度限位与安全截断
│   │   └── hand_controller.py   # LeapNode & HandController 统一操作句柄
│   └── driver/                   # 硬件通信协议与驱动层
│       ├── __init__.py
│       ├── dynamixel_client.py   # Dynamixel SDK Protocol 2.0 底层封装
│       └── leap_driver.py        # 16-DOF 硬件抽象层、自闭环标定绑定与阻尼平滑器
├── package.xml                   # 功能包依赖清单 (rclpy, sensor_msgs 等)
└── setup.py                      # 构建与可执行命令 entry_points 注册
```

---

## 4. 黄金架构不变量与硬件安全红线

> [!WARNING]
> **硬件安全红线 (Hardware Guardrails)**:
> 1. **150mA 电流硬锁定**: 电机驱动层初始化时强制将舵机控制表的 `Current Limit` 写入为 150mA，杜绝发生过载烧毁；
> 2. **300ms 看门狗平滑阻尼回退 (Relax-to-OPEN)**: 上层主端手势或算法节点一旦出现网络卡顿或异常退出超过 300ms，系统无条件启动阻尼平滑插值，在 0.9s 内将手指展开至 0 rad 全开姿态；
> 3. **统一弧度制契约 (SI Radian)**: 外部话题交互 100% 采用弧度制，底层仅在与 Dynamixel 寄存器对接处执行 `rad <-> raw ticks` 线性映射；
> 4. **零位事实源唯一性**: 硬件驱动初始化时优先读取 `config/poses.json` 中的实测全开位作为物理 0 rad 锚点，确保标定工具与 ROS 2 运行时绝对闭环自洽。

---

## 5. 单元测试与闭环验证

在本地环境下运行驱动级与节点级集成测试：

```bash
# 隔离 ROS 2 域 ID 并加载环境
export ROS_DOMAIN_ID=99
source /opt/ros/lyrical/setup.bash
source install/setup.bash

# 运行 LEAP Hand 驱动、姿态管理与 ROS 2 节点集成测试
python3 -m pytest tests/test_leap_cli_and_pose_manager.py tests/test_leap_driver.py tests/test_leap_hand_node_integration.py -v
```

**预期输出指标**:
```
tests/test_leap_cli_and_pose_manager.py (9 passed)
tests/test_leap_driver.py (4 passed)
tests/test_leap_hand_node_integration.py (1 passed)
============================== 14 passed in 2.60s ===============================
```
