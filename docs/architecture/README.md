# TuinaDex v2.0 系统架构与代码模块全景规范全书

> **文档版本**: v2.1.0  
> **更新时间**: 2026-09-21  
> **归属工程**: `TuinaDex v2.0` (中医推拿机械臂-灵巧手具身智能系统)  
> **文档位置**: `docs/architecture/README.md`  
> **核心工具**: [Archify v2.17](https://github.com/tt-a1i/archify) 交互式架构图

---

## 一、系统架构全局全景与分层设计 (System Architecture Overview)

TuinaDex 是面向**中医推拿/理疗场景**构建的具身智能机器人系统。当前阶段专注于 **16-DOF LEAP Hand 灵巧手** 的高可靠闭环：
$$\text{视觉遥操作示教} \longrightarrow \text{ROS2 运行时控制} \longrightarrow \text{rosbag2 原始事实源} \longrightarrow \text{离线治理与对齐} \longrightarrow \text{LeRobot 策略学习} \longrightarrow \text{真机/仿真闭环部署}$$

```
+---------------------------------------------------------------------------------------------------+
|                                  TuinaDex v2.0 全系统分层拓扑架构                                    |
+---------------------------------------------------------------------------------------------------+
| [1. 感知与示教域]              操作员手势 -> RGB 相机 (30 FPS)                                        |
|   `tuinadex_teleop`            MediaPipe (21 3D Landmarks) -> 1€ Filter -> JointMapper (16-DOF 弧度)  |
+--------------------------------------------------+------------------------------------------------+
                                                   | 发布目标指令 (/hand/joint_command, 50Hz 16D rad)
                                                   v
+---------------------------------------------------------------------------------------------------+
| [2. 控制调度与安全护栏域]        指令校验门 (align_command) + 逐关节限位裁切 (JOINT_SPEC_LIMITS)          |
|   `tuinadex_core`              300ms 安全看门狗状态机 (WatchdogStateMachine: FOLLOWING / RELAXING)  |
+--------------------------------------------------+------------------------------------------------+
                                                   | 条件分支路由 (hand.launch.py use_sim:=true/false)
                         +-------------------------+-------------------------+
                         |                                                   |
                         v (use_sim:=false)                                  v (use_sim:=true)
+--------------------------------------------------+ +----------------------------------------------+
| [3A. 真实物理执行体]                              | | [3B. 数字孪生仿真执行体]                       |
|   `tuinadex_hand`                                | |   `tuinadex_sim`                             |
|   LeapHandDriver (位置-电流模式, 150mA 限流)       | |   SimulationCore + MujocoSimBridge           |
|   Dynamixel Protocol 2.0 (4Mbps USB-TTL)         | |   MuJoCo 500Hz 物理动力学 (dt=0.002s, 10步/tick) |
|   16-DOF LEAP Hand 真实物理舵机                    | |   推拿床与人体背部假人刚体接触场景 (`tuina_scene`) |
+--------------------------------------------------+ +----------------------------------------------+
                         |                                                   |
                         +-------------------------+-------------------------+
                                                   | 统一实测反馈 (/hand/joint_states, 50Hz 16D rad)
                                                   v
+---------------------------------------------------------------------------------------------------+
| [4. 示教采集与事实源域]        `tuinadex_recorder` (进程内 rosbag2_py SequentialWriter)             |
|                               服务控制 (/recorder/start, /recorder/stop) -> 不可变原始数据包 (MCAP/SQLite) |
+--------------------------------------------------+------------------------------------------------+
                                                   | 离线 CDR 数据反序列化
                                                   v
+---------------------------------------------------------------------------------------------------+
| [5. 具身数据治理与学习域]      `tuinadex_dataset` 离线治理管道                                       |
|                               TimeSynchronizer (50Hz 规则时间网格插值对齐)                           |
|                               DatasetValidator (NaN/Inf 校验, 规范逐关节限位, 0.5rad 跳变门禁)        |
|                               LeRobotDataset v3 (标准化 Parquet 特征表: observation.state + action)  |
+--------------------------------------------------+------------------------------------------------+
                                                   | 离线策略训练 (smolvla Conda 环境)
                                                   v
| [6. 自主策略推理域]            ACT / Diffusion Policy 具身策略模型 -> 回注 /hand/joint_command 自主闭环 |
+---------------------------------------------------------------------------------------------------+
```

### 核心架构原则 (Architectural Invariants)
1. **双域解耦 (Dual-Domain Decoupling)**:
   - **ROS2 运行时域 (Runtime Domain)**: 承载高频硬件驱动、实时通信、50Hz 闭环与 300ms 安全看门狗；
   - **LeRobot 学习域 (Learning Domain)**: 承载离线数据清洗、重采样网格对齐、策略训练与评测。
   - **红线**: LeRobot 绝对禁止跨过 ROS2 直连硬件驱动；ROS2 接口严禁为学习层 Schema 随意破坏机器人标准规范。
2. **唯一原始事实源 (Source of Truth)**:
   - 原始 `rosbag2` 记录唯一客观真实的物理实验时间基与全量传感器状态；
   - 下游 `LeRobotDataset` 是派生数据产品，可随时利用工具链重新转换。
3. **坐标系与国际单位全链路一致性**:
   - 全链路强制 **SI 国际标准单位：弧度 (rad)**；
   - 以物理标定全开平伸姿态 (`OPEN_POSE`) 为 **0 rad 基准零位**，各手指关节向内屈曲弯曲为正 ($q > 0$)。
4. **动作空间强制绝对位姿**:
   - 动作空间 (Action Space) 严格使用 **16 维 absolute joint position**，杜绝使用增量 delta 引入累积漂移。

---

## 二、代码功能包与模块清单职责详解 (Packages & Modules Ledger)

项目代码划分为 ROS2 功能包、离线算法工具包和通用纯逻辑核心层：

| 功能包 / 路径 | 核心定位 | 运行环境 | 主要职责 |
| :--- | :--- | :--- | :--- |
| `src/tuinadex_teleop` | 视觉手势遥操端 | ROS2 (Lyrical) | 捕获手势视频流，解算为 16 维目标绝对弧度，发布至指令话题 |
| `src/tuinadex_hand` | 物理灵巧手运行时 | ROS2 (Lyrical) | 驱动 16 个 Dynamixel 舵机，50Hz 定时回读发布实测状态，维持安全看门狗 |
| `src/tuinadex_sim` | MuJoCo 数字孪生 | ROS2 (Lyrical) | 500Hz 高保真物理接触动力学仿真，对等镜像真机话题与时序 |
| `src/tuinadex_recorder` | 示教数据采集端 | ROS2 (Lyrical) | 进程内原生录制双话题到 rosbag2，Trigger 服务控制起停 |
| `dataset/tuinadex_dataset`| 具身数据集适配器 | Conda (smolvla) / Python 3.10+ | 离线反序列化 bag，50Hz 时间网格插值对齐，生成 LeRobot Parquet 数据集 |
| `src/tuinadex_core` | 共享纯逻辑基础层 | 纯 Python (无 ROS 依赖) | 单源化 Canonical 关节命名、逐关节限位表、安全看门狗状态机 |

---

### 1. `src/tuinadex_teleop` (感知与手势示教端)

- **`teleop_gesture_node.py` (`TeleopGestureNode`)**:
  - **位置**: 感知示教上游入口节点。
  - **职责**: 管理摄像头视频流采集 (OpenCV 640x480 @ 30fps)；调用追踪算法解算手势；以 50Hz 定时发布 `/hand/joint_command`。支持离合手势暂停与姿态锁定。
  - **输入**: OpenCV 摄像头帧。
  - **输出**: ROS2 话题 `/hand/joint_command` (`sensor_msgs/msg/JointState`)。
- **`tracking/hand_tracker.py` (`HandTracker`)**:
  - **位置**: 视觉关键点检测模块。
  - **职责**: 封装 Google MediaPipe Hands 算法；检测图像中的手部骨架，输出 21 个 3D 归一化空间坐标点及置信度。
- **`tracking/joint_mapper.py` (`JointMapper`)**:
  - **位置**: 运动学投影映射模块。
  - **职责**: 将人手 21 个 3D 关键点通过向量夹角与空间几何投影，映射至 16-DOF LEAP Hand 的关节空间弧度 ($q \in \mathbb{R}^{16}$)，确保 0 rad 对应全开平伸。
- **`tracking/filter.py` (`OneEuroFilter`)**:
  - **位置**: 信号调理与平滑模块。
  - **职责**: 一阶自适应 1€ 滤波器；低速时提升平滑度消除人手自然微颤，高速时动态降低迟滞保证跟随性。
- **`tracking/teleop_logic.py`**:
  - **位置**: 追踪安全逻辑辅助模块。
  - **职责**: 检验左右手匹配度 (`handedness_ok`)；检测目标手短暂丢失后的重捕获 (`regained_after_gap`)，在重获时复位滤波器，杜绝跳变飞车。

---

### 2. `src/tuinadex_hand` (物理灵巧手运行时)

- **`leap_hand_node.py` (`LeapHandNode`)**:
  - **位置**: 物理真机主控节点。
  - **职责**: 维持 50Hz 单线程定时控制循环；订阅 `/hand/joint_command` 缓存目标；由 `WatchdogStateMachine` 裁决控制行为；调用底层驱动下发电机指令；回读实测状态并发布 `/hand/joint_states`；在节点析构时执行平稳归零安全退出。
- **`driver/leap_driver.py` (`LeapHandDriver`)**:
  - **位置**: 硬件抽象与坐标转换层。
  - **职责**: 
    1. 负责机器人关节空间弧度 $q$ 与舵机硬件绝对弧度 $\theta_{dxl}$ 的双向转换：
       $$\theta_{dxl} = \text{OPEN\_POSE} + \text{DEFAULT\_JOINT\_DIR} \odot q$$
       $$q = (\theta_{dxl} - \text{OPEN\_POSE}) \odot \text{DEFAULT\_JOINT\_DIR}$$
    2. 配置电机模式为位置-电流混合模式 (Mode 5)，锁定电流上限为 150mA；
    3. 实现 `relax_to_open_step(now)`：手势丢失时保持原位 hold 0.3s，随后 ramp 0.6s 阻尼收敛回 0 rad 全开位；
    4. 内置 Mock 模式支持离线轻量仿真。
- **`driver/dynamixel_client.py` (`DynamixelClient`)**:
  - **位置**: 串口总线协议层。
  - **职责**: 封装 Dynamixel SDK Protocol 2.0；通过 USB-TTL 串口 (`/dev/ttyUSB0`，波特率 4,000,000 bps) 执行高效的 `GroupSyncRead` (读取位置、速度、电流) 与 `GroupSyncWrite` (写入目标位置)。

---

### 3. `src/tuinadex_sim` (数字孪生动力学仿真)

- **`leap_hand_sim_node.py` (`LeapHandSimNode`)**:
  - **位置**: 仿真环境主控节点，与 `leap_hand_node` 100% 对等替换。
  - **职责**:
    1. 采用**单线程 50Hz ROS2 Timer**（Invariant 16）；
    2. 每周期在内部同步循环 10 次 0.002s 子步进（$10 \times 0.002\text{s} = 20\text{ms}$），完成 500Hz 物理刚体接触微分；
    3. 严格 50Hz 降采样发布实测位姿、速度与接触力，保持真机/仿真接口零差异。
- **`sim_bridge.py` (`MujocoSimBridge`)**:
  - **位置**: MuJoCo 物理引擎纯逻辑桥接器。
  - **职责**: 基于 Python `mujoco` 库加载 MJCF 模型文件；建立 Canonical Joint 与 Actuator ID 映射；提供 `set_joint_targets()`, `step(num_steps)`, `get_state()` 接口，脱离 ROS2 即可独立运行单测。
- **`sim_core.py` (`SimulationCore`)**:
  - **位置**: 纯逻辑仿真调度与平滑回退核心。
  - **职责**: 组合 `MujocoSimBridge` 与 `WatchdogStateMachine`，逐行镜像真机 `relax_to_open_step` 的 hold+ramp 阻尼曲线。
- **资产文件**:
  - `worlds/tuina_scene.xml`: 综合推拿场景模型，包含推拿床刚体模型与人体背部组织模型；
  - `urdf/tuina_bed.urdf`: 推拿床与背部假人 URDF 描述；
  - `meshes/`: 8 组高精度 STL 工业三角网格模型。

---

### 4. `src/tuinadex_recorder` (示教数据采集端)

- **`recorder_node.py` (`RecorderNode`)**:
  - **位置**: 进程内实验数据采集节点。
  - **职责**:
    1. 使用原生 `rosbag2_py.SequentialWriter`，避免 subprocess 外部命令调用引入的死锁或数据库损坏；
    2. 采用通用原始订阅 (`raw=True`) 直接落盘 CDR 序列化字节流，CPU 占用低于 1%；
    3. 提供 `/recorder/start` 与 `/recorder/stop` (`std_srvs/srv/Trigger`) 服务控制起停；
    4. 自动生成规范化存储路径 `bags/session_YYYYmmdd_HHMMSS`。

---

### 5. `dataset/tuinadex_dataset` (离线具身数据集治理与适配)

- **`bag_reader.py` (`read_joint_state_bag`)**:
  - **位置**: rosbag2 反序列化读取器。
  - **职责**: 使用 `rosbag2_py.SequentialReader` 遍历解析原始包，提取状态与动作的时间戳及 16 维弧度数据。
- **`synchronizer.py` (`TimeSynchronizer`)**:
  - **位置**: 规则时间网格重采样与插值同步器。
  - **职责**: 针对状态和动作两个独立异步话题，计算重叠时间区间 $[t_{start}, t_{end}]$，构建 50Hz (20ms) 等间隔时间网格，对 16 维状态与动作序列进行一阶线性插值对齐。
- **`validator.py` (`DatasetValidator`)**:
  - **位置**: 数据集质量与物理安全审查门禁。
  - **检验项**:
    1. 全量数据无 NaN / Inf；
    2. 时间戳严格单调递增；
    3. 16 维数值严格处于 `SPEC_JOINT_LIMITS` 逐关节规范限位内；
    4. 相邻两帧位置瞬时跳变 $\Delta q \le 0.50\text{ rad}$（剔除跳变飞车坏帧）。
- **`adapter.py` (`save_to_lerobot`, `convert_bag_to_episode`)**:
  - **位置**: Hugging Face LeRobot 数据集封装适配器。
  - **职责**: 构建符合 LeRobot 0.4.4+ 标准的 `LeRobotDataset` v3，写入 `observation.state` (16D) 与 `action` (16D) 特征表，导出高压缩比的 Parquet 格式数据。
- **`cli.py`**:
  - **位置**: 离线批量转换命令行入口工具。

---

### 6. `src/tuinadex_core` (共享纯逻辑基础层)

- **`names.py`**:
  - **职责**: `CANONICAL_JOINT_NAMES` 唯一单源定义，消除各包重复声明。
- **`command_processing.py`**:
  - **职责**: `align_command()` 报文合法性检查（拒绝非法维度、非有限值、未知关节名）与 `JOINT_SPEC_LIMITS` 夹取。
- **`watchdog.py`**:
  - **职责**: `WatchdogStateMachine` 状态机与 `WatchdogAction` 动作裁决。

---

## 三、系统数据接口契约矩阵 (Data Interfaces & Contract Matrix)

### 1. 核心 ROS2 话题 (Topic Contracts)

| 话题名称 | 消息类型 | 发布节点 | 订阅节点 | 发布频率 | 坐标/单位契约 | 语义说明 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `/hand/joint_command` | `sensor_msgs/msg/JointState` | `tuinadex_teleop` / 策略推理节点 | `tuinadex_hand` / `tuinadex_sim` / `tuinadex_recorder` | 50.0 Hz | SI 弧度 (rad), 0 为全开位, 弯曲 > 0 | 上层期望执行的目标绝对位置 (Action) |
| `/hand/joint_states` | `sensor_msgs/msg/JointState` | `tuinadex_hand` / `tuinadex_sim` | `tuinadex_recorder` / 上层监控 / 策略闭环 | 50.0 Hz | SI 弧度 (rad), 0 为全开位, 弯曲 > 0 | 底层物理或仿真测量得到的实际位置 (State) |

### 2. ROS2 服务 (Service Contracts)

| 服务名称 | 服务类型 | 服务端节点 | 调用客户端 | 语义说明 | 幂等保护 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/recorder/start` | `std_srvs/srv/Trigger` | `tuinadex_recorder` | 示教操作员 / 自动化脚本 | 启动进程内录制，创建 session 数据包 | 重复调用返回 `already recording` |
| `/recorder/stop` | `std_srvs/srv/Trigger` | `tuinadex_recorder` | 示教操作员 / 自动化脚本 | 停止录制，落盘并关闭 SequentialWriter | 未录制时调用返回 `not recording` |

### 3. Canonical 16-DOF 关节命名与物理限位基准表

全链路强制按照以下 16 维固定顺序排布（以实测 `OPEN_POSE` 全开平伸为 0 rad 基准）：

| 序号 | 关节名称 (Canonical Name) | 手指与部位 | 规范限位范围 `[min, max]` (rad) | 舵机旋转方向 | 典型动作语义 |
| :---: | :--- | :--- | :---: | :---: | :--- |
| **0** | `index_yaw` | 食指侧摆 (MCP 侧向) | `[-0.35, +0.35]` | -1.0 | 食指内外侧摆展开 |
| **1** | `index_mcp` | 食指掌指关节 (屈伸) | `[ 0.00, +2.00]` | -1.0 | 食指根部向内屈曲 |
| **2** | `index_pip` | 食指近端指间关节 | `[ 0.00, +1.85]` | -1.0 | 食指中段向内屈曲 |
| **3** | `index_dip` | 食指远端指间关节 | `[ 0.00, +1.80]` | -1.0 | 食指末端向内屈曲 |
| **4** | `middle_yaw` | 中指侧摆 (MCP 侧向) | `[-0.35, +0.35]` | -1.0 | 中指内外侧摆展开 |
| **5** | `middle_mcp` | 中指掌指关节 (屈伸) | `[ 0.00, +2.00]` | -1.0 | 中指根部向内屈曲 |
| **6** | `middle_pip` | 中指近端指间关节 | `[ 0.00, +1.85]` | -1.0 | 中指中段向内屈曲 |
| **7** | `middle_dip` | 中指远端指间关节 | `[ 0.00, +1.80]` | -1.0 | 中指末端向内屈曲 |
| **8** | `ring_yaw` | 无名指侧摆 (MCP 侧向) | `[-0.35, +0.35]` | -1.0 | 无名指内外侧摆展开 |
| **9** | `ring_mcp` | 无名指掌指关节 (屈伸) | `[ 0.00, +2.00]` | -1.0 | 无名指根部向内屈曲 |
| **10**| `ring_pip` | 无名指近端指间关节 | `[ 0.00, +1.85]` | -1.0 | 无名指中段向内屈曲 |
| **11**| `ring_dip` | 无名指远端指间关节 | `[ 0.00, +1.80]` | -1.0 | 无名指末端向内屈曲 |
| **12**| `thumb_yaw` | 拇指基底侧摆/回转 | `[-0.30, +1.80]` | +1.0 | 拇指外展与对指旋转 |
| **13**| `thumb_cmc` | 拇指腕掌关节 (屈伸) | `[ 0.00, +1.60]` | -1.0 | 拇指根部向内屈曲 |
| **14**| `thumb_mcp` | 拇指掌指关节 (屈伸) | `[ 0.00, +1.80]` | -1.0 | 拇指第二关节屈曲 |
| **15**| `thumb_ip` | 拇指指间关节 (屈伸) | `[ 0.00, +1.80]` | -1.0 | 拇指末端指尖屈曲 |

---

## 四、三大控制闭环时序与执行流 (Execution Loops & Timing Flows)

### 闭环 1: 视觉遥操作示教闭环 (Teleoperation Loop)
$$\text{RGB 相机 (30Hz)} \xrightarrow{\text{OpenCV}} \text{MediaPipe (21 pts)} \xrightarrow{\text{JointMapper}} q_{\text{raw}} \xrightarrow{\text{1€ Filter}} q_{\text{filtered}} \xrightarrow{50\text{Hz}} \text{/hand/joint\_command}$$
- **时钟与平滑**: 独立视频采样时钟与 50Hz ROS2 定时器解耦，利用 1€ Filter 平滑手势瞬时抖动。
- **异常保护**: 人手出画/遮挡时，断手检测保护，停止发布指令或保持锁定。

### 闭环 2: 真实/仿真双模统一执行闭环 (Dual-Mode Actuation Loop)
通过 `ros2 launch tuinadex_hand hand.launch.py use_sim:=true/false` 无缝切换：
- **真机执行流 (`use_sim:=false`)**:
  - `LeapHandNode` 50Hz 定时触发；
  - `WatchdogStateMachine` 检查最后指令时间戳 $\Delta t = t_{now} - t_{last}$；
  - $\Delta t \le 300\text{ms}$: 下发目标位置到 16 舵机；
  - $\Delta t > 300\text{ms}$: 触发 `relax_to_open_step` 平滑回退全开位；
  - SyncRead 回读电机物理弧度，发布至 `/hand/joint_states`。
- **仿真执行流 (`use_sim:=true`)**:
  - `LeapHandSimNode` 单线程 50Hz 定时触发；
  - 循环调用 10 次 `mujoco.mj_step()`（每次 0.002s，对应 500Hz 物理步进），确保接触力与数值积分绝对稳定；
  - 读取 MuJoCo $q_{pos}, q_{vel}, \tau$ 并封装为 `JointState`，发布至 `/hand/joint_states`。

### 闭环 3: 数据采集、治理与具身学习闭环 (Harvesting & Learning Loop)
$$\text{/hand/joint\_command} + \text{/hand/joint\_states} \xrightarrow{\text{tuinadex\_recorder}} \text{rosbag2 (MCAP/SQLite)}$$
$$\xrightarrow{\text{tuinadex\_dataset: synchronize}} 50\text{Hz 规则网格对齐} \xrightarrow{\text{validate}} \text{LeRobotDataset (Parquet)}$$
$$\xrightarrow{\text{LeRobot 训练}} \text{Diffusion / ACT Policy} \xrightarrow{\text{部署}} \text{/hand/joint\_command (自主闭环)}$$

---

## 五、硬件安全防御纵深体系 (Safety Guardrails)

系统构建了涵盖**空间、时间、电气与生命周期**的四重安全防护纵深：

```
                      +---------------------------------------+
                      |       安全防护第 1 道：空间限位门        |
                      |  align_command() 逐关节限位裁切 (Clip)  |
                      +-------------------+-------------------+
                                          |
                                          v
                      +---------------------------------------+
                      |       安全防护第 2 道：时间看门狗        |
                      |  WatchdogStateMachine 300ms 断流保护  |
                      |  hold(0.3s) + ramp(0.6s) 平滑回全开位  |
                      +-------------------+-------------------+
                                          |
                                          v
                      +---------------------------------------+
                      |       安全防护第 3 道：电气限流保护      |
                      |  Dynamixel 电流环硬件锁死 150mA 阈值    |
                      +-------------------+-------------------+
                                          |
                                          v
                      +---------------------------------------+
                      |       安全防护第 4 道：生命周期归零      |
                      |  节点退出序列强制 set_open() + dwell 驻留 |
                      +---------------------------------------+
```

1. **第 1 道（空间维度 · 关节限位门）**:
   - 指令入口通过 `command_processing.align_command()` 强制进行维度检查与有限值校验；
   - 越界弧度被强制截断到 `JOINT_SPEC_LIMITS` 边界，禁止任何超限指令进入电机驱动或物理引擎。
2. **第 2 道（时间维度 · 300ms 安全看门狗）**:
   - 当上位机手势丢失、网络中断或进程崩溃超过 300ms，状态机由 `FOLLOWING` 切换为 `RELAXING`；
   - 驱动层执行 `relax_to_open_step`：前 0.3s 保持原位避免骤停震荡，后 0.6s 线性平滑展开至全开平伸姿态（0 rad），消除暴力甩手。
3. **第 3 道（电气维度 · 150mA 硬件电流保护）**:
   - 舵机工作模式强制锁定在 Operating Mode = 5 (位置-电流混合模式)；
   - 寄存器 102 写入 `curr_lim = 150` (150mA)，发生机械卡阻或重度接触时自动退让，杜绝烧毁舵机或夹伤人体。
4. **第 4 道（生命周期 · 安全关停驻留序列）**:
   - `LeapHandNode.destroy_node()` 拦截 `SIGINT` / `KeyboardInterrupt`；
   - 关停定时器 $\to$ 下发全开指令 $\to$ 执行 `time.sleep(0.5)` 驻留等待手指物理走到全开位 $\to$ 安全断开串口。

---

## 六、Archify 架构可视化资产目录与交互指南 (Archify Visual Assets Catalog)

本目录由 [Archify](https://github.com/tt-a1i/archify) 引擎驱动，提供独立、可交互、出版级的自包含 HTML 架构资产：

| 资产文件 | 类型 | 核心内容 | 质量标准 | 查看与验证状态 |
| :--- | :--- | :--- | :--- | :--- |
| [`tuinadex_v2_architecture.html`](file:///home/bright/win_office/ubantu_files/project/TuinaDex_v2.0/docs/architecture/tuinadex_v2_architecture/tuinadex_v2_architecture.html) | `architecture` | 系统全局全景架构、双域解耦边界、端到端数据流向 | Showcase (9/9 pass) | 0 错误 0 警告，多分辨率全景回归通过 |
| [`tuinadex_sim_digital_twin.html`](file:///home/bright/win_office/ubantu_files/project/TuinaDex_v2.0/docs/architecture/tuinadex_sim_digital_twin/tuinadex_sim_digital_twin.html) | `architecture` | MuJoCo 数字孪生动力学仿真、真机/仿真统一切换与双速率时钟 | Showcase (9/9 pass) | 0 错误 0 警告，多分辨率全景回归通过 |

### 交互功能特性 (Interactive Features)
1. **暗黑 / 明亮主题无缝切换 (Theme Toggle)**:
   - 支持一键切换 Dark / Light 模式，自适应系统色调。
2. **导览视角切换 (Guided Views / Chapters)**:
   - 点击顶部导航卡片，可一键高亮聚焦指定数据流与子系统（如“视觉遥操与硬件控制”、“物理状态回读与示教捕获”、“具身学习与策略闭环”）。
3. **节点与路径高亮追踪 (Relationship Tracing)**:
   - 鼠标悬浮或点击任意组件节点，自动高亮其上下游调用链路与数据总线。
4. **平移与缩放 (Pan & Zoom)**:
   - 支持鼠标滚轮缩放、画布任意平移，支持一键复位居中。
5. **矢量高清导出 (Export)**:
   - 支持一键导出 SVG / PNG / WebP 高清图像。

### 查看与验证命令
```bash
# 1. 重新验证架构规范
node /home/bright/.gemini/config/skills/archify/bin/archify.mjs validate architecture docs/architecture/tuinadex_v2_architecture/tuinadex_v2_architecture.json --quality showcase --json
node /home/bright/.gemini/config/skills/archify/bin/archify.mjs validate architecture docs/architecture/tuinadex_sim_digital_twin/tuinadex_sim_digital_twin.json --quality showcase --json

# 2. 重新编译交付 HTML 资产
node /home/bright/.gemini/config/skills/archify/bin/archify.mjs deliver architecture docs/architecture/tuinadex_v2_architecture/tuinadex_v2_architecture.json docs/architecture/tuinadex_v2_architecture/tuinadex_v2_architecture.html --quality showcase --json
node /home/bright/.gemini/config/skills/archify/bin/archify.mjs deliver architecture docs/architecture/tuinadex_sim_digital_twin/tuinadex_sim_digital_twin.json docs/architecture/tuinadex_sim_digital_twin/tuinadex_sim_digital_twin.html --quality showcase --json

# 3. 收集浏览器自动化渲染实证
node /home/bright/.gemini/config/skills/archify/bin/archify.mjs visual-check docs/architecture/tuinadex_v2_architecture/tuinadex_v2_architecture.html --json
node /home/bright/.gemini/config/skills/archify/bin/archify.mjs visual-check docs/architecture/tuinadex_sim_digital_twin/tuinadex_sim_digital_twin.html --json
```

---

## 七、验证命令与开发工作流 (Verification & Workflow)

声称功能完成前必须执行充分的工程验证（已在 `AGENTS.md` 固化）：

1. **纯逻辑、遥操与驱动单元测试 (leap_hand 环境)**:
   ```bash
   ~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v
   ```
2. **ROS2 节点与仿真集成测试 (宿主机 Lyrical 环境)**:
   ```bash
   source /opt/ros/lyrical/setup.bash && python3 -m pytest tests/test_leap_hand_node_integration.py tests/test_recorder_node.py tests/test_sim_bridge.py -v
   ```
3. **LeRobot 数据集适配器测试 (smolvla 环境)**:
   ```bash
   ~/win_office/conda/envs/smolvla/bin/python -m pytest tests/test_dataset_adapter.py -v
   ```
4. **全工程 ROS2 功能包构建**:
   ```bash
   source /opt/ros/lyrical/setup.bash && colcon build --symlink-install --packages-select tuinadex_hand tuinadex_teleop tuinadex_recorder tuinadex_sim
   ```
5. **无硬件端到端闭环流水线自检**:
   ```bash
   bash scripts/e2e_mock_check.sh
   ```
