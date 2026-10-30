# tuinadex_sim: 机械臂-灵巧手 22-DOF 动力学数字孪生仿真包

`tuinadex_sim` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **22 维一体化 MuJoCo 物理动力学数字孪生功能包**。

本包在无需连接真实硬件的情况下，提供与物理真机完全对齐的 **6-DOF reBot-DevArm RS 机械臂 + 16-DOF LEAP Hand 灵巧手** 联合运行环境，包含中医推拿床、床侧底座支架与人体背部假人（成年男性躯干）的高保真刚体接触动力学模型，支持视觉手势遥操作、示教数据采集与具身策略学习离线回环。

---

## 快速开始 (Quick Start)

> [!TIP]
> 仿真节点与真机硬件驱动共享完全相同的 ROS 2 话题契约（机械臂 `/arm/joint_command` & `/arm/joint_states`，灵巧手 `/hand/joint_command` & `/hand/joint_states`），上层遥操作、控制算法与具身数据采集代码零修改无缝切换。

### 1. 环境准备与工作空间激活
```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0
source /opt/ros/lyrical/setup.bash

# 构建与刷新安装空间 (首次或资产修改后执行)
colcon build --symlink-install --packages-select tuinadex_sim
source install/setup.bash
```

### 2. 一键启动 22 维一体化 3D 仿真
```bash
# 启动 22 维一体化 3D 实时物理仿真视窗 (机械臂 + 灵巧手 + 推拿床 + 假人)
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=true gui:=true

# 或以纯后台无头模式运行 (适用于 CI、离线录制与自动化测试)
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=true gui:=false
```

### 3. 下发测试指令与观测状态反馈
在另一终端下发 22 维协同控制指令并实时回显物理响应：
```bash
source /opt/ros/lyrical/setup.bash && source install/setup.bash

# 下发 22 维联合测试指令 (臂 6 关节运动到 0.2 rad，手 16 关节弯曲至 0.4 rad，持续 3 秒)
python3 scripts/mock_command_publisher.py --mode arm_hand --arm-position 0.2 --position 0.4 --duration-sec 3.0

# 监听仿真 50Hz 机械臂与灵巧手真实物理状态反馈
ros2 topic echo /arm/joint_states
ros2 topic echo /hand/joint_states
```

### 4. 离线快速 3D 几何预览 (免 ROS 2 守护进程)
```bash
# 启动 MuJoCo 原生 3D 查看器 (展开右侧 Control 面板可手动拉动全部 22 个关节滑块)
python3 -m mujoco.viewer --mjcf=src/tuinadex_sim/worlds/tuina_scene.xml
```

### 5. 一键端到端闭环自检
```bash
# 自动拉起仿真并检验 22 维协同控制、500Hz 物理积分、50Hz 通信帧间隔以及 300ms 看门狗安全回退
bash scripts/e2e_unified_robot_check.sh
```

---

## 1. 核心特性与架构规范

1. **22-DOF 话题契约完全等价 (Invariants 2 & 13)**:
   - **机械臂 (6-DOF)**:
     - 订阅目标指令: `/arm/joint_command` (`sensor_msgs/JointState`, 6 维 `arm_joint_1`..`arm_joint_6`)
     - 发布反馈状态: `/arm/joint_states` (`sensor_msgs/JointState`, 6 维实际位置/速度/力矩)
   - **灵巧手 (16-DOF)**:
     - 订阅目标指令: `/hand/joint_command` (`sensor_msgs/JointState`, 16 维 `index_yaw`..`thumb_ip`)
     - 发布反馈状态: `/hand/joint_states` (`sensor_msgs/JointState`, 16 维实际位置/速度/力矩)
   - 关节命名、顺序与物理限位与真机硬件 100% 对齐。
2. **多速率时钟与动力学保真 (Invariants 14 & 16)**:
   - 内部物理动力学强制以 **500 Hz** (`dt = 0.002s`) 步进，确保高刚度刚体接触与数值积分稳定性；
   - ROS 2 节点采用单线程 50 Hz 定时器，每 tick 内锁步执行 10 次子步进 (`10 × 0.002s = 20ms`)，发布端严格降采样至 **50 Hz**，杜绝后台多线程竞态。
3. **双模看门狗一致性 (Invariants 15 & 18)**:
   - 内置 300ms Watchdog 超时检测机制；
   - 指令断联超过 300ms 时，灵巧手执行 `0.3s hold + 0.6s ramp` 阻尼曲线平滑回退至全开平伸姿态 (`Relax-to-OPEN`)，机械臂平滑保持安全推拿就绪姿态，杜绝机械冲击与冲量发散。
4. **全集成与单手双模自适应**:
   - 当加载包含机械臂的复合场景（`tuina_scene.xml`）时，自动激活 22 维臂手联合控制接口；
   - 当仅加载纯手部模型（`leap_hand_right.xml`）时，自动退化为纯 16-DOF 灵巧手模式，完美兼容既有测试。

---

## 2. 目录与资产结构

```text
src/tuinadex_sim/
├── config/
│   └── sim_params.yaml            # ROS 2 仿真节点运行参数 (频率、步长、看门狗、话题等)
├── launch/
│   └── sim.launch.py              # tuinadex_sim 独立启动脚本 (支持 gui:=true/false)
├── meshes/                        # 机械臂、灵巧手与假人完整 3D 网格模型
│   ├── shared/                    # 机械臂 base_link 到 link6 的 STL 碰撞与外观网格
│   ├── visual/                    # 成年男性推拿假人躯干 (male_torso.stl) 与装饰件
│   ├── leap_hand/                 # 16-DOF 灵巧手掌心、指节与指尖 STL 网格
│   └── mujoco_collision/         # 专用精细凸包碰撞网格
├── urdf/
│   ├── rebot_arm_rs.urdf          # 6-DOF reBot-DevArm RS 机械臂规范 URDF 模型
│   ├── leap_hand_right.urdf       # 16-DOF LEAP Hand 规范 Canonical URDF 模型
│   └── tuina_bed.urdf             # 中医推拿床与人体背部假人 URDF 模型
├── worlds/
│   ├── tuina_scene.xml            # 22-DOF 一体化全场景 (臂+手+推拿床+假人，默认使用)
│   ├── rebot_arm_leap_hand.xml    # 22-DOF 臂手一体纯机器人模型 (无床/无假人)
│   └── leap_hand_right.xml        # 16-DOF 纯灵巧手无约束动力学模型
├── tuinadex_sim/
│   ├── __init__.py
│   ├── sim_bridge.py              # MujocoSimBridge: 纯 Python 22-DOF MuJoCo 动力学步进与状态提取
│   ├── sim_core.py                # SimulationCore: 500Hz 锁步、双模状态机与平滑回退算法
│   └── leap_hand_sim_node.py      # LeapHandSimNode: ROS 2 双话题控制节点与 3D Passive Viewer
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 3. 详细仿真启动方式与高级选项

### 方式一：整机统一入口启动 (推荐)

项目统一通过 `tuinadex_bringup` 控制 22 维臂手真机与仿真切换：

```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0
source /opt/ros/lyrical/setup.bash && source install/setup.bash

# 1. 22 维臂手一体化仿真 (带 3D 实时可视化交互画面)
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=true gui:=true

# 2. 纯灵巧手 16 维仿真 (带 3D 可视化)
ros2 launch tuinadex_bringup robot.launch.py mode:=hand_only use_sim:=true gui:=true

# 3. 纯后台无头模式运行 (适用于 CI、自动录制与批量评测)
ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=true gui:=false
```

### 方式二：本包独立 launch 启动

直接启动 `tuinadex_sim` 仿真环境：

```bash
# 带 3D 渲染画面启动 22 维推拿场景
ros2 launch tuinadex_sim sim.launch.py gui:=true

# 无头后台模式启动
ros2 launch tuinadex_sim sim.launch.py gui:=false
```

### 方式三：ROS 2 单节点命令行启动

```bash
# 启动 22 维一体化推拿场景并开启 GUI
ros2 run tuinadex_sim leap_hand_sim_node --ros-args -p gui:=true

# 加载纯手部模型 (不加载推拿床、假人与机械臂)
ros2 run tuinadex_sim leap_hand_sim_node --ros-args -p use_scene_world:=false -p gui:=true
```

### 方式四：MuJoCo 原生独立 3D 预览器 (无需 ROS 2 通信)

用于快速检查模型几何形态、干涉碰撞与手动拖动滑块测试关节活动范围：

```bash
# 预览 22 维一体化推拿全场景 (机械臂 + 灵巧手 + 推拿床 + 假人)
python3 -m mujoco.viewer --mjcf=src/tuinadex_sim/worlds/tuina_scene.xml

# 预览 22 维纯机器人模型 (臂 + 手)
python3 -m mujoco.viewer --mjcf=src/tuinadex_sim/worlds/rebot_arm_leap_hand.xml

# 预览纯灵巧手模型
python3 -m mujoco.viewer --mjcf=src/tuinadex_sim/worlds/leap_hand_right.xml
```
- **快捷操作**: 鼠标左键旋转视角、右键缩放、滚轮平移；展开右侧 **Control** 面板可手动拉动 22 个关节滑块。

---

## 4. 各参数配置与调参指南

### 4.1 机械臂与推拿床相对基准布局 (`worlds/tuina_scene.xml`)

机械臂安装基座位于推拿床侧边，初始 Ready 姿态悬停在假人背部上方：

```xml
<!-- 机械臂基座安装位置与朝向 (安装在床侧支架上，Z 轴垂直向上) -->
<body name="base_link" pos="-0.15 0.52 0.64" quat="0.7071 0 0 -0.7071">
```

- **初始就绪关节姿态 (Ready Pose)**：
  `[0.0, 0.85, 0.95, -0.80, 0.0, 0.0]`（rad），机械臂自然弯曲前伸，掌心垂直向下正对推拿假人背部穴位区域。

---

### 4.2 执行器 KD 参数统一配置 (`worlds/tuina_scene.xml`)

本仿真采用 MuJoCo 原生的 `<default>` 属性继承与局部精细化重载结合的方式：

| 执行器分类 | 关节名称 | 默认 $K_p$ (N·m/rad) | 默认 $K_d$ (N·m·s/rad) | 物理特性与调参建议 |
| :--- | :--- | :--- | :--- | :--- |
| **机械臂大臂 (肩/肘)** | `joint1`, `joint2`, `joint3` | `60.0 ~ 80.0` | `6.0 ~ 8.0` | 承重大刚度驱动，确保机械臂下压推拿时不产生软塌超调。 |
| **机械臂手腕** | `joint4`, `joint5`, `joint6` | `25.0 ~ 40.0` | `3.0 ~ 4.0` | 灵活末端调姿，保证手腕滚转与俯仰响应平顺。 |
| **灵巧手基节 (MCP)** | `*_mcp`, `thumb_cmc` | `95.0` | `2.0` | 高按压力下压关节，提供推拿所需的指根部顶推力。 |
| **灵巧手指间 (PIP/DIP)** | `*_pip`, `*_dip` | `85.0` | `1.5 ~ 1.8` | 快速屈伸与捏持关节，兼顾敏捷度与接触稳定性。 |
| **灵巧手侧摆 (Yaw)** | `*_yaw` | `80.0` | `1.8` | 侧向张指关节，限位控制在 `±0.35 rad`。 |

---

### 4.3 物理接触动力学与相邻自碰撞过滤

为了避免相邻连杆之间的虚假内力发散，`tuina_scene.xml` 中内置了经过精确物理校核的自碰撞过滤白名单：
- **机械臂内部过滤**：`link1~link6` 串联相邻排除、`link3` 与 `link5` 弯曲折叠排除；
- **手腕与手掌过渡过滤**：`link6`、`link5`、`link4` 与 `leap_hand_palm`、`mcp_joint*` 根部连接排除；
- **灵巧手同指过滤**：各手指内部的大指节与远端指尖（`mcp_joint` 与 `dip` 等）排除，确保手指无论如何弯曲握拳均能平滑收敛。

---

### 4.4 ROS 2 节点层运行参数 (`config/sim_params.yaml`)

```yaml
leap_hand_sim_node:
  ros__parameters:
    # 物理步长与时钟
    physics_timestep: 0.002           # 500 Hz MuJoCo 动力学步长 (秒)
    publish_rate_hz: 50.0             # 50 Hz ROS2 通信降采样发布频率 (与真机 20ms 一致)
    physics_substeps: 10              # 每 20ms ROS tick 内执行 10 × 2ms = 20ms 锁步积分
    
    # 通信话题契约 (与真机 100% 相同)
    command_topic: "/hand/joint_command"
    state_topic: "/hand/joint_states"
    arm_command_topic: "/arm/joint_command"
    arm_state_topic: "/arm/joint_states"
    
    # 安全与看门狗
    watchdog_timeout_sec: 0.3         # 300ms 指令超时判定
    watchdog_hold_sec: 0.3            # 超时后前 300ms 保持当前实测姿态 (Hold 期)
    watchdog_ramp_sec: 0.6            # 随后的 600ms 线性收敛至全开平伸姿态 (Ramp 期)
    
    # 场景资产与可视化配置
    scene_file: "worlds/tuina_scene.xml"
    hand_only_file: "worlds/leap_hand_right.xml"
    use_scene_world: true             # true: 包含推拿床、假人与机械臂, false: 纯手部环境
    gui: false                        # true: 启动 3D 被动渲染窗口, false: 无头后台运行
```

---

## 5. 调参生效与开发验证工作流

```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0
source /opt/ros/lyrical/setup.bash

# 1. 快速更新安装空间 (采用符号链接安装，通常直接生效)
colcon build --symlink-install --packages-select tuinadex_sim
source install/setup.bash

# 2. 运行单测验证动力学桥接器与时钟未被破坏
~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_sim_bridge.py tests/test_sim_core.py -v

# 3. 运行 22 维联合控制与启动端到端 E2E 终验
bash scripts/e2e_unified_robot_check.sh
```
