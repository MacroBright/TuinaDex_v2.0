# tuinadex_hand: 16-DOF LEAP Hand 物理驱动与 ROS 2 运行时

`tuinadex_hand` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **16 自由度 LEAP Hand 仿生灵巧手物理驱动与 ROS 2 节点功能包（Layer 2 & 1: Driver & Hardware Layer）**。

本包负责通过 U2D2 适配器与 16 个 Dynamixel XC330 舵机建立 4Mbps 高速串口通信，提供 50Hz 闭环位置控制、实测状态遥测发布、电流硬限位保护以及 300ms 看门狗安全平滑回退。

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

### 2. 独立编译与环境加载
```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0
source /opt/ros/lyrical/setup.bash

# 独立构建 tuinadex_hand 功能包
colcon build --symlink-install --packages-select tuinadex_hand
source install/setup.bash
```

### 3. 运行节点

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

### 4. 监听物理状态与下发测试指令
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
│   └── leap_hand_params.yaml     # 硬件通信参数、PID 与安全阈值配置
├── launch/
│   └── hand.launch.py            # 支持真机/Mock/Sim 多模自适应启动脚本
├── resource/
│   └── tuinadex_hand             # ROS 2 包索引
├── tuinadex_hand/
│   ├── __init__.py
│   ├── leap_hand_node.py         # 50Hz ROS 2 主控节点与看门狗回路
│   └── driver/
│       ├── __init__.py
│       ├── dynamixel_client.py   # Dynamixel SDK Protocol 2.0 底层封装
│       └── leap_driver.py        # 16-DOF 硬件抽象层与阻尼归零平滑器
├── package.xml                   # 功能包依赖清单 (rclpy, sensor_msgs 等)
└── setup.py                      # 构建与可执行文件注册
```

---

## 4. 黄金架构不变量与硬件安全红线

> [!WARNING]
> **硬件安全红线 (Hardware Guardrails)**:
> 1. **150mA 电流硬锁定**: 电机驱动层初始化时强制将舵机控制表的 `Current Limit` 写入为 150mA，杜绝发生过载烧毁；
> 2. **300ms 看门狗平滑阻尼回退 (Relax-to-OPEN)**: 上层主端手势或算法节点一旦出现网络卡顿或异常退出超过 300ms，系统无条件启动阻尼平滑插值，在 0.9s 内将手指展开至 0 rad 全开姿态；
> 3. **统一弧度制契约 (SI Radian)**: 外部话题交互 100% 采用弧度制，底层仅在与 Dynamixel 寄存器对接处执行 `rad <-> raw ticks` 线性映射。

---

## 5. 单元测试与闭环验证

在本地环境下运行驱动级与节点级集成测试：

```bash
# 隔离 ROS 2 域 ID 并加载环境
export ROS_DOMAIN_ID=99
source /opt/ros/lyrical/setup.bash

# 运行 LEAP Hand 驱动与 ROS 2 节点集成测试
python3 -m pytest tests/test_leap_driver.py tests/test_leap_hand_node_integration.py -v
```

**预期输出指标**:
```
tests/test_leap_driver.py::test_driver_init_and_mock_mode PASSED
tests/test_leap_driver.py::test_driver_current_limit_invariant PASSED
tests/test_leap_driver.py::test_relax_to_open_interpolation PASSED
tests/test_leap_hand_node_integration.py::test_hand_mock_loop_and_watchdog PASSED
============================== 4 passed in 2.10s ===============================
```
