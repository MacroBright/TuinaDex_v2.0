# tuinadex_recorder: 示教数据事实源录制节点

`tuinadex_recorder` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **实验原始数据事实源录制功能包（Layer 4: Middleware Layer）**。

本包基于 ROS 2 原生 `rosbag2_py` 实现进程内高性能数据捕获，采用 CDR 原始字节流透传机制（Zero-Deserialization），将 22 自由度机械臂与灵巧手的主端控制指令、真实物理状态以及时序数据高保真无损落盘为标准 `rosbag2` (sqlite3) 格式，作为下游模仿学习与具身策略训练的**唯一事实源 (Single Source of Truth, SSOT)**。

---

## 快速开始 (Quick Start)

### 1. 独立构建与环境准备
```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0
source /opt/ros/lyrical/setup.bash

# 独立构建 tuinadex_recorder 功能包
colcon build --symlink-install --packages-select tuinadex_recorder
source install/setup.bash
```

### 2. 独立运行录制节点
```bash
# 启动录制节点 (默认监听 /hand 与 /arm 的状态及指令话题)
ros2 run tuinadex_recorder tuinadex_recorder_node
```

### 3. 通过 Trigger 服务控制录制生命周期
录制节点由标准的 ROS 2 `std_srvs/srv/Trigger` 服务进行状态机解耦控制，便于 Web UI Hub 或自动化脚本远程调用：

```bash
# 1. 触发开始录制 (自动在 bags/ 目录下创建以时间戳命名的存储空间)
ros2 service call /recorder/start std_srvs/srv/Trigger

# 终端返回示例: success=True, message="bags/session_20261102_164500"

# 2. 此时正常下发控制指令或进行遥操作示教...

# 3. 触发停止录制并落盘刷新 (Flush & Close)
ros2 service call /recorder/stop std_srvs/srv/Trigger

# 终端返回示例: success=True, message="stopped"
```

### 4. 检查生成的原始数据包 (rosbag2 info)
```bash
# 检查录制数据包的时长、消息数与采样率
ros2 bag info bags/session_20261102_164500
```

---

## 1. 节点与通信契约 (Node Interface Contract)

### 节点名称: `tuinadex_recorder`
- **执行体入口**: `ros2 run tuinadex_recorder tuinadex_recorder_node`
- **核心底层**: `rosbag2_py.SequentialWriter` (Storage ID: `sqlite3`)
- **高性能特性**: 订阅端开启 `raw=True` 捕获模式，直接收取并落盘 CDR 二进制字节，避免 ROS 2 进程内反序列化带来的 CPU 开销与时延抖动。

#### 服务接口 (Services)
| 服务名称 | 服务类型 | 请求/响应 | 功能描述 |
| :--- | :--- | :--- | :--- |
| `/recorder/start` | `std_srvs/srv/Trigger` | Client $\rightarrow$ Server | 动态创建时间戳目录，打开 `SequentialWriter` 并激活话题订阅开始记录。 |
| `/recorder/stop` | `std_srvs/srv/Trigger` | Client $\rightarrow$ Server | 销毁话题订阅，刷新底层缓存并安全关闭 bag 文件。 |

#### 参数配置 (Parameters)
| 参数名 | 类型 | 默认值 | 作用与配置说明 |
| :--- | :--- | :--- | :--- |
| `topics` | `string[]` | `["/hand/joint_states", "/hand/joint_command"]` | 需要录制的目标话题列表 (支持扩展 `/arm/joint_*`) |
| `topic_types`| `string[]` | `["sensor_msgs/msg/JointState", "sensor_msgs/msg/JointState"]` | 与 `topics` 严格一一对应的 ROS 2 消息类型列表 |
| `bag_root` | `string` | `"bags"` | 录制数据包输出的根目录路径 |

> **配置自定义录制话题示例**:
> ```bash
> ros2 run tuinadex_recorder tuinadex_recorder_node --ros-args \
>   -p topics:="['/hand/joint_states', '/hand/joint_command', '/arm/joint_states', '/arm/joint_command']" \
>   -p topic_types:="['sensor_msgs/msg/JointState', 'sensor_msgs/msg/JointState', 'sensor_msgs/msg/JointState', 'sensor_msgs/msg/JointState']" \
>   -p bag_root:="experiment_bags"
> ```

---

## 2. 目录结构

```
src/tuinadex_recorder/
├── package.xml                   # 功能包依赖声明 (rclpy, sensor_msgs, rosbag2_py)
├── resource/
│   └── tuinadex_recorder         # ament 包索引
├── setup.cfg                     # 安装配置
├── setup.py                      # 可执行脚本入口配置
└── tuinadex_recorder/
    ├── __init__.py
    └── recorder_node.py          # 基于 rosbag2_py 的轻量级服务控制录制节点
```

---

## 3. 黄金数据契约与下游流水线

```
┌────────────────────────────────────────────────────────┐
│ 运行态示教 / 遥操作 (50Hz 控制闭环)                     │
│ -> /hand/joint_states & /arm/joint_states              │
│ -> /hand/joint_command & /arm/joint_command            │
└───────────────────────────┬────────────────────────────┘
                            │ Raw CDR Bytes (raw=True)
┌───────────────────────────▼────────────────────────────┐
│ tuinadex_recorder (唯一事实源落盘)                      │
│ -> bags/session_YYYYmmdd_HHMMSS/ (sqlite3)             │
└───────────────────────────┬────────────────────────────┘
                            │ 离线无缝接入
┌───────────────────────────▼────────────────────────────┐
│ dataset/tuinadex_dataset (时空同步与格式转换)          │
│ -> 50Hz 网格插值 -> 关节限位校验 -> LeRobotDataset v3.0 │
└────────────────────────────────────────────────────────┘
```

1. **零丢帧设计**: 订阅队列采用 `depth=10` 并使用 CDR 二进制裸收，实测 50Hz 双模全量传输下 CPU 占用 $< 1\%$；
2. **时钟一致性**: 每条记录消息由 `node.get_clock().now().nanoseconds` 赋予统一时间戳，杜绝不同节点间的系统墙上时钟飘移；
3. **数据事实源契约 (Invariant 23)**: `rosbag2` 记录的原始数据严禁在线修改。任何重采样、单位对齐或滤波操作必须交由下游离线工具包 `tuinadex_dataset` 处理。

---

## 4. 验证与测试命令

```bash
# 激活环境并隔离 ROS_DOMAIN_ID
export ROS_DOMAIN_ID=99
source /opt/ros/lyrical/setup.bash

# 验证 recorder 功能包构建无误
colcon test --packages-select tuinadex_recorder
colcon test-result --all
```
