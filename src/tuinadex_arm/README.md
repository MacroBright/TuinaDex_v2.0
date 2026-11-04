# tuinadex_arm: 6-DOF reBot 机械臂物理驱动与 ROS 2 运行时

`tuinadex_arm` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **6 自由度 reBot-DevArm RS 机械臂物理驱动、模型描述与 ROS 2 节点功能包（Layer 2 & 1: Driver & Hardware Layer）**。

本包负责通过 Linux SocketCAN 接口（默认 1Mbps 高速 CAN 总线）驱动 6 个 RobStride 关节电机（J1-J3: RS06，J4-J6: RS00），提供 50Hz 闭环位置控制、实测动力学状态遥测、使能与回零触发服务、热过载多级熔断保护以及 300ms 看门狗安全平滑回零（Relax-to-HOME）。同时包含机械臂高保真 URDF 与 3D 碰撞网格资源。

---

## 快速开始 (Quick Start)

### 1. CAN 总线接口配置 (Linux SocketCAN)
确保硬件 USB2CAN 适配器已接入，并完成 CAN 网络接口初始化：
```bash
# 1. 真实硬件 CAN 总线配置 (1 Mbps)
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0

# 2. 或建立虚拟 CAN 接口用于离线调试 (vcan0)
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### 2. 独立编译与环境加载
```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0
source /opt/ros/lyrical/setup.bash

# 独立构建 tuinadex_arm 功能包
colcon build --symlink-install --packages-select tuinadex_arm
source install/setup.bash
```

### 3. 运行节点

#### 模式 A：连接物理机械臂运行
> [!CAUTION]
> 实体上电前必须确认机械臂急停开关 (E-Stop) 处于随手可触状态，工作空间 1.0 米内无人体或障碍物！

```bash
ros2 launch tuinadex_arm arm.launch.py channel:=can0 mock:=false
```

#### 模式 B：离线 Mock 模式启动 (无物理硬件)
适用于脱机开发、算法集成与 CI 测试：
```bash
ros2 launch tuinadex_arm arm.launch.py mock:=true
```

### 4. 监听状态反馈与下发服务调用
```bash
# 1. 监听 50Hz 机械臂真实关节角度、速度与力矩反馈
ros2 topic echo /arm/joint_states

# 2. 调用服务安全回零位 (0 rad HOME)
ros2 service call /arm/safe_home std_srvs/srv/Trigger

# 3. 在新终端下发 6 关节测试运动指令 (持续 2 秒)
python3 scripts/mock_command_publisher.py --mode arm --arm-position 0.2 --duration-sec 2.0
```

---

## 1. 节点与通信契约 (Node Interface Contract)

### 节点名称: `arm_node`
- **执行体入口**: `ros2 run tuinadex_arm arm_node`
- **默认控制频率**: 50 Hz (`control_rate_hz: 50.0`)
- **设计模式**: 与 `leap_hand_node` 保持严格对称的 Node + `WatchdogStateMachine` 架构

#### 订阅话题 (Subscribed Topics)
| 话题名称 | 消息类型 | 频率 (Hz) | 语义与单位规范 |
| :--- | :--- | :--- | :--- |
| `/arm/joint_command` | `sensor_msgs/msg/JointState` | 50 Hz | 目标关节角 (6 维绝对弧度, SI rad)。经 `align_arm_command` 严格校验限位门并按 `joint1`~`joint6` 重排。 |

#### 发布话题 (Published Topics)
| 话题名称 | 消息类型 | 频率 (Hz) | 语义与单位规范 |
| :--- | :--- | :--- | :--- |
| `/arm/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz | 实测关节状态。包含 6 维物理位置 (rad)、速度 (rad/s) 与电机力矩 (N·m)。 |

#### 服务接口 (Services)
| 服务名称 | 服务类型 | 请求/响应 | 功能描述 |
| :--- | :--- | :--- | :--- |
| `/arm/enable` | `std_srvs/srv/Trigger` | Client $\rightarrow$ Server | 建立 CAN 连接并完成电机使能 |
| `/arm/disable` | `std_srvs/srv/Trigger` | Client $\rightarrow$ Server | 安全下使能并断开 CAN 通信 |
| `/arm/safe_home` | `std_srvs/srv/Trigger` | Client $\rightarrow$ Server | 驱动机械臂平滑运动回到 0 rad 基准安全 HOME 位姿 |

---

## 2. 参数配置 (`config/arm_params.yaml`)

| 参数名 | 类型 | 默认值 | 物理单位 | 作用与安全含义 |
| :--- | :--- | :--- | :--- | :--- |
| `channel` | `string` | `"can0"` | - | SocketCAN 接口名称 (如 `can0`, `can1`, `vcan0`) |
| `bitrate` | `int` | `1000000` | bps | CAN 通信波特率 (1 Mbps) |
| `host_id` | `int` | `253` | - | 主机控制节点 CAN ID (`0xFD`) |
| `control_rate_hz` | `double` | `50.0` | Hz | ROS 2 控制回路与状态发布频率 |
| `watchdog_timeout_sec` | `double` | `0.30` | s | **指令失联安全超时门限** (300ms) |
| `watchdog_hold_sec` | `double` | `0.30` | s | 超时后原位保持缓冲时间 (0.3s) |
| `watchdog_ramp_sec` | `double` | `0.60` | s | 超时后平滑阻尼回零过渡时间 (0.6s) |
| `default_speed_deg_s` | `double` | `30.0` | deg/s | 默认轨迹运动角速度限制 |
| `temp_alarm_c` | `double` | `110.0` | °C | 电机线圈高温告警阈值 |
| `temp_return_zero_c` | `double` | `125.0` | °C | 电机超温降级强制平滑回零阈值 |
| `temp_disconnect_c` | `double` | `140.0` | °C | **电机危险超温下电硬熔断阈值** |
| `mock_mode` | `bool` | `false` | - | 是否启用内部软件 Mock 仿真驱动 |

---

## 3. 目录与资产拓扑

```
src/tuinadex_arm/
├── config/
│   └── arm_params.yaml           # CAN 端口、控制频率与安全温度配置
├── description/                  # 机械臂模型资产库
│   ├── DM/                       # reBot-DevArm DM 达妙电机版本
│   │   ├── meshes/               # STL 可视化与碰撞网格
│   │   └── urdf/ReBot_Arm_DM.urdf
│   └── RS/                       # reBot-DevArm RS 睿思电机版本 (当前主力)
│       ├── meshes/               # STL 可视化与碰撞网格 (含 MuJoCo 优化版)
│       └── urdf/ReBot_Arm_RS.urdf
├── launch/
│   └── arm.launch.py             # 机械臂节点启动调度脚本
├── resource/
│   └── tuinadex_arm              # ROS 2 包索引
├── tuinadex_arm/
│   ├── __init__.py
│   ├── arm_node.py               # 50Hz ROS 2 主控节点与服务接口
│   └── driver/
│       ├── __init__.py
│       └── arm_driver.py         # 6-DOF SocketCAN 驱动抽象与平滑器
├── package.xml                   # 包依赖声明 (rclpy, sensor_msgs, std_srvs)
└── setup.py                      # 安装与节点脚本入口注册
```

---

## 4. 黄金架构不变量与硬件安全红线

> [!WARNING]
> **机械臂硬件安全红线**:
> 1. **全行程限位保护 (ARM_SPEC_LIMITS)**:
>    - `joint1`: $[-2.80, 2.80]\text{ rad}$
>    - `joint2`: $[0.00, 3.14]\text{ rad}$
>    - `joint3`: $[0.00, 3.14]\text{ rad}$
>    - `joint4`: $[-1.57, 1.57]\text{ rad}$
>    - `joint5`: $[-1.57, 1.57]\text{ rad}$
>    - `joint6`: $[-3.14, 3.14]\text{ rad}$
>    所有超出上述区间的指令会被驱动底层 `np.clip` 强行截断，绝不向电机下发越界目标；
> 2. **300ms 看门狗安全回零 (Relax-to-HOME)**: 指令中断超过 300ms 时，执行 `0.3s hold + 0.6s ramp` 平滑过渡至全 0 rad 安全推拿待命位姿；
> 3. **热保护三级防护网**: $110^\circ\text{C}$ 告警 $\rightarrow$ $125^\circ\text{C}$ 强制慢速回零 $\rightarrow$ $140^\circ\text{C}$ 切断供电下使能。

---

## 5. 单元测试与闭环验证

在本地环境下运行驱动级与节点级集成测试：

```bash
# 隔离 ROS 2 域 ID 并加载环境
export ROS_DOMAIN_ID=99
source /opt/ros/lyrical/setup.bash

# 运行机械臂驱动与节点集成单测
python3 -m pytest tests/test_arm_driver.py tests/test_arm_node_integration.py -v
```

**预期输出指标**:
```
tests/test_arm_driver.py::test_arm_driver_init_mock PASSED
tests/test_arm_driver.py::test_arm_driver_limits_clamp PASSED
tests/test_arm_driver.py::test_arm_driver_relax_to_open PASSED
tests/test_arm_node_integration.py::test_arm_mock_loop_and_watchdog PASSED
============================== 4 passed in 2.30s ===============================
```
