# tuinadex_teleop: 机械臂-灵巧手协同仿真与视-惯正交遥操作系统

`tuinadex_teleop` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **协同仿真与多模态遥操作系统**。

本包针对 **reBot-DevArm RS 机械臂 (6-DOF) + LEAP Hand 仿生灵巧手 (16-DOF)** 构成的 22 自由度复合机器人系统，提供视-惯正交融合传感（Intel RealSense 深度相机空间定位 + 腕部 6 轴 IMU 姿态解算）、Pink/Pinocchio QP-IK 机械臂实时逆解、16-DOF 灵巧手运动学几何重定向、4 级安全门限守护以及低延迟、高保真的 MuJoCo 数字孪生协同仿真环境。

---

## 快速开始 (Quick Start)

> [!TIP]
> 本包提供 **独立 3D 交互预览脚本** 与 **ROS 2 标准节点** 两种运行模式，均支持无物理硬件的 `--mock` 离线体验模式。

### 1. 环境准备与工作空间激活
```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0

# 激活遥操与控制专用环境 (Python 3.10)
source scripts/activate_tuinadex.sh control

# 或加载宿主机 ROS 2 环境 (Python 3.14)
source /opt/ros/lyrical/setup.bash && source install/setup.bash
```

### 2. 方式 A：独立 3D 交互预览（推荐：带 MuJoCo 视窗 + OpenCV 实时 HUD）
适合**实时观察遥操推拿动作、调试末端接触力与算法表现**：

```bash
# 1. 真实硬件启动 (连接 RealSense D455 与 6 轴 IMU)
python scripts/run_sim_teleop.py

# 2. 离线虚拟体验 (无需物理硬件，自动生成人手测试轨迹，带完整 3D 画面与仪表盘)
python scripts/run_sim_teleop.py --mock

# 3. 纯终端无头运行 (适用于 SSH 或自动化脚本测试)
python scripts/run_sim_teleop.py --mock --no-gui
```

### 3. 方式 B：ROS 2 复合节点与 Launch 启动
适合**接入 ROS 2 通信网络，供整个系统协同调度与数据录制**：

```bash
source /opt/ros/lyrical/setup.bash && source install/setup.bash

# 1. 连接物理硬件启动
ros2 launch tuinadex_teleop teleop_sim.launch.py use_sim:=true mock:=false

# 2. 纯仿真 Mock 启动 (自动合成人手与 IMU 信号)
ros2 launch tuinadex_teleop teleop_sim.launch.py use_sim:=true mock:=true
```

### 4. 方式 C：纯手部视觉手势遥操节点
若只需单独运行基于 RGB 单目/普通摄像头的纯手部手势遥操作：
```bash
ros2 run tuinadex_teleop teleop_gesture_node --ros-args -p camera_id:=0
```

### 5. 方式 D：一边仿真遥操一边数据采集 (多模态具身数据链路)
系统支持 **ROS 2 分布式解耦架构** 与 **轻量 Web UI Hub 直出** 两种边遥操边采集工作流，全模态对齐 Hugging Face LeRobotDataset v3.0 规范：

#### 方案 1：ROS 2 分布式协同采集模式（多终端解耦，标准具身生产架构）
仿真底座、遥操作系统与数据录制各自独立运行，通过 ROS 2 话题解耦协同：

```bash
# 终端 1：启动 MuJoCo 仿真机器人底座 (后台运行仿真引擎，发布 /arm/joint_states 与 /hand/joint_states)
source /opt/ros/lyrical/setup.bash && source install/setup.bash
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=true mock:=false

# 终端 2：启动视-惯遥操作节点 (连接 RealSense D455 与 IMU，发布 /arm/joint_command 与 /hand/joint_command)
source /opt/ros/lyrical/setup.bash && source install/setup.bash
ros2 launch tuinadex_teleop teleop_sim.launch.py use_sim:=true mock:=false

# 终端 3：启动数据采集节点 (两种途径任选其一)
# 途径 A: 录制原始 rosbag2 事实源 (通过 ros2 service call /recorder/start 触发录制)
ros2 run tuinadex_recorder recorder_node \
    --ros-args \
    -p topics:="['/arm/joint_states','/hand/joint_states','/arm/joint_command','/hand/joint_command']" \
    -p topic_types:="['sensor_msgs/msg/JointState','sensor_msgs/msg/JointState','sensor_msgs/msg/JointState','sensor_msgs/msg/JointState']"

# 途径 B: 启动 Web UI Hub 监听 ROS 2 话题并直出 LeRobotDataset (浏览器访问 http://127.0.0.1:8080)
source scripts/activate_tuinadex.sh control
python scripts/collect_teleop_data.py --env real --mode arm_hand --hub --repo-id data/tuina_real_dataset
```

#### 方案 2：内置仿真 Web UI Hub 多模态工作台（直通 3 机位视频与 Parquet）
无需启动 ROS 2 守护进程，直接使用采集系统内置的 MuJoCo 动力学引擎与 EGL 离屏渲染管线：

```bash
source scripts/activate_tuinadex.sh control

# 启动 22 维手-臂协同仿真与 Web UI Hub (默认绑定 127.0.0.1:8080)
python scripts/collect_teleop_data.py \
    --env sim \
    --mode arm_hand \
    --cameras overhead,wrist,side \
    --mock-teleop \
    --hub \
    --fps 50 \
    --ui-fps 25 \
    --repo-id data/tuina_hub_dataset
```
- 打开浏览器访问 👉 **`http://127.0.0.1:8080`**；
- 监视 3 视窗视频墙（俯视 60°、腕部 75°、侧向 45°）与 22 维实时关节/接触力曲线；
- 示教操作流程：按 **`[空格]`** 开始录制 $\rightarrow$ 推拿完成按 **`[S]`** 停止 $\rightarrow$ 满意按 **`[回车]`** 异步写入 LeRobotDataset v3.0（失误按 **`[退格]`** 舍弃）。

#### 方案 3：自动化 CI 门禁与离线转换
```bash
# 1. 一键端到端全链路 E2E 验证 (启动 Hub -> 离屏渲染 -> 交互 -> LeRobot v3.0 导出)
bash scripts/e2e_hub_collection_check.sh

# 2. rosbag2 离线格式转换工具
python -m tuinadex_dataset.cli convert --bag-path bags/session_xxx --output-dir data/tuina_lerobot
```

> 详细架构设计、REST/SSE 接口与全套操作指南请参阅 [数据采集层专项说明文档](tuinadex_teleop/collection/README.md)。

---

## 核心高频快捷键速查卡

在 `scripts/run_sim_teleop.py` 的 OpenCV 交互视窗激活时，可随时敲击以下快捷键实时交互：

| 按键 | 核心功能 | 详细行为说明 |
| :---: | :--- | :--- |
| **`[空格]`** / **`[C]`** | **离合器开关 (Clutch)** | **核心控制键**。在 `ENGAGED` (接管遥控) 与 `DISENGAGED` (挂起安全) 间切换；接管瞬间自动锚定当前人手位置与 IMU 零位，实现**无冲击平滑接管**。 |
| **`[S]`** | **安全防护门限切换** | **一键热切换**：在 `ACTIVE` (有安全门限守护，120N 力/防跳跃保护) 与 `BYPASS` (无安全门限自由模式) 之间随时切换。 |
| **`[Z]`** | **灵巧手平手归零复位** | **人手自然张开平放时按下**：动态固化当前平手姿态为零偏基准并保存，同时令仿真端 16 关节全部复位展平（0.00 rad），彻底消除残余弯曲。 |
| **`[I]`** | **IMU 姿态对齐归零** | 将手腕当前朝向锁定为机械臂末端的对应姿态基准。 |
| **`[M]`** | **机械臂姿态跟踪模式循环** | 循环切换：`TRACK_IMU` (全姿态跟随) $\rightarrow$ `FIXED_DOWNWARD` (垂直向下) $\rightarrow$ `TRACK_WRIST` (纯视觉) $\rightarrow$ `POSITION_ONLY` (仅控位置)。 |
| **`[A]`** | **位移映射轴向切换** | 实时轮换位置正交对齐（默认 `-y,+x,+z` 适配 90° 床侧基座旋转安装）。 |
| **`[O]`** | **IMU 姿态映射切换** | 实时轮换姿态映射（默认 `+x,+y,+z`，行列式严格保证 $\det=+1$）。 |
| **`[1]`/`[2]`/`[3]`** | **灵巧手解算模式切换** | 1: Hybrid 混合优化模式 (默认)；2: 纯几何映射模式；3: DexPilot 模式。 |
| **`[R]`** | **仿真一键重置** | 复位离合器、滤波器及 MuJoCo 仿真模型至 Ready 就绪姿态。 |
| **`[E]`** | **急停制动 (ESTOP)** | 触发急停，立即冻结机械臂末端位置与灵巧手输出。 |
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
                                                                    /arm/joint_command (6D)
                                                                    /hand/joint_command (16D)
                                                                          │
                                                                          ▼
                                                                 [MujocoSimAdapter (500Hz)]
                                                                          │
                                                                          ▼
                                                                    /arm/joint_states (6D)
                                                                    /hand/joint_states (16D)
```

1. **正交传感融合 (Orthogonal Sensing)**:
   - 位置信息由深度相机基于桌面 ArUco 物理平面提取，经 One-Euro 滤波消除抖动；
   - 姿态信息由高采样率 6 轴 IMU 提供，彻底解决手掌翻转时视觉严重遮挡与关键点翻转跳变缺陷。
2. **空间增量离合器 (ClutchController)**:
   - 支持踏板/按键离合，离合切断时人手可随意回位；重新使能瞬间建立相对锚点，机械臂与灵巧手绝不产生位置阶跃突变。
3. **4 级安全门限守护 (SafetySupervisor)**:
   - **Gate 1 (感知丢失检测)**: 手部丢失超 800ms 触发安全悬停；
   - **Gate 2 (位姿跳跃防护)**: 拦截帧间位移 $>0.18\text{m}$ 或角度 $>60^\circ$ 的奇异跳变；
   - **Gate 3 (末端接触力限制)**: 接触总力超过 $120\text{N}$（单指 $>70\text{N}$）触发柔顺卸力回退；
   - **Gate 4 (奇异点防御)**: 逆解雅可比奇异值低于门限时自动阻尼锁定。

---

## 2. 目录与配置资产结构

```text
src/tuinadex_teleop/
├── configs/
│   ├── mjcf/
│   │   ├── tuina_sim_scene.xml      # 22-DOF 一体化推拿全场景动力学模型
│   │   └── rebot_arm_leap_hand.xml  # 22-DOF 臂手一体纯机器人模型
│   ├── urdf/
│   │   ├── rebot_arm_rs.urdf        # 6-DOF 机械臂运动学模型 (供 Pink IK 求解)
│   │   └── leap_hand_right.urdf     # 16-DOF 灵巧手运动学模型
│   ├── meshes/                      # 机器人与推拿假人 STL 网格
│   ├── session_calibration.yaml     # 桌面、IMU 与灵巧手全集成标定文件
│   ├── table_calibration.yaml       # 桌面物理平面 ArUco 标定参数
│   ├── imu_calibration.yaml         # 腕部 IMU 姿态对齐标定参数
│   ├── teleop_config.yaml           # 遥操作全局安全阈值与时钟配置
│   └── leap_retargeting_config.yaml # 灵巧手关节优化权重与几何缩放配置
├── launch/
│   ├── teleop_sim.launch.py         # 22 维复合仿真遥操 ROS 2 启动入口
│   └── teleop.launch.py             # 纯手势遥操启动入口
├── tuinadex_teleop/
│   ├── adapters/                    # MujocoSimAdapter, MockSimAdapter
│   ├── core/                        # clutch.py, safety.py, types.py, names.py
│   ├── kinematics/                  # arm_ik_solver.py (Pink QP), hand_retargeter.py
│   ├── perception/                  # wrist_tracker.py, imu_stream.py, realsense_stream.py
│   ├── envs/                        # ros2_robot_env.py, mujoco_env.py (数据采集接口)
│   ├── collection/                  # lerobot_v3_exporter.py (多模态导出器)
│   ├── teleop_sim_composite_node.py # 22-DOF 复合仿真遥操 ROS 2 主节点
│   └── teleop_gesture_node.py       # 纯手部视觉遥操节点
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 3. 核心参数调参指南 (`configs/`)

| 配置文件 | 关键参数项 | 默认值 | 作用与调参说明 |
| :--- | :--- | :--- | :--- |
| **`teleop_config.yaml`** | `safety.max_total_force_n` | `120.0` | 推拿总按压力硬上限 (N)，超过触发自动卸力。 |
| | `safety.max_pos_jump_m` | `0.18` | 帧间位移最大允许跳变门限 (米)，防止视觉误识别冲撞。 |
| | `safety.max_rot_jump_deg`| `60.0` | 帧间姿态最大允许跳变角 (度)。 |
| **`session_calibration.yaml`** | `table.axis_mapping` | `-y,+x,+z` | 人手平移至机械臂基座映射 (适配床侧横置安装)。 |
| | `imu.imu_axis_mapping` | `+x,+y,+z` | IMU 轴向映射至机械臂末端姿态。 |
| | `hand.scaling_factor` | `1.60` | 人手掌长至 LEAP Hand 机械手的几何几何缩放比例。 |
| **`leap_retargeting_config.yaml`** | `thumb_weights` | 详见 YAML | 拇指对捏与对掌优化权重，调大可显著增强捏取贴合度。 |

---

## 4. 自动化测试与验证

本项目自带覆盖完整的单元与集成测试套件：

```bash
# 1. 运行遥操与算法核心单元测试 (81 项全绿，覆盖 IK、重定向、离合器与安全门)
~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/teleop/ -v

# 2. 运行 ROS 2 复合节点通信与物理闭环集成测试
source /opt/ros/lyrical/setup.bash && source install/setup.bash
python3 -m pytest tests/test_teleop_sim_composite_node.py -v

# 3. 运行全系统 22 维联合控制与启动端到端 E2E 自检
bash scripts/e2e_unified_robot_check.sh
```

