# tuinadex_sim: MuJoCo 动力学数字孪生仿真包

`tuinadex_sim` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **MuJoCo 物理动力学数字孪生功能包**。

本包在无需连接真实硬件的情况下，提供与物理灵巧手完全一致的 16-DOF LEAP Hand 运行环境，包含中医推拿床与人体背部假人的刚体接触动力学模型，支持视觉手势遥操作、示教数据采集与策略部署离线回环。

---

## 1. 核心特性与架构规范

1. **话题契约完全等价 (Invariant 13)**:
   - 订阅目标指令: `/hand/joint_command` (`sensor_msgs/JointState`)
   - 发布反馈状态: `/hand/joint_states` (`sensor_msgs/JointState`)
   - 16 维 Canonical 关节命名与顺序与真机硬件 100% 对齐，上层算法零代码修改无缝切换。
2. **多速率时钟与动力学保真 (Invariants 14 & 16)**:
   - 内部物理动力学强制以 **500 Hz** (`dt = 0.002s`) 步进，确保刚体接触与数值积分稳定性；
   - ROS 2 节点采用单线程 50 Hz 定时器，每 tick 内锁步执行 10 次子步进 (`10 × 0.002s = 20ms`)，发布端严格降采样至 **50 Hz**，杜绝后台多线程竞态。
3. **双模看门狗一致性 (Invariants 15 & 18)**:
   - 内置 300ms Watchdog 超时机制；
   - 上层手势或指令断联超过 300ms 时，复刻真机驱动的 `0.3s hold + 0.6s ramp` 阻尼曲线平滑回退至全开平伸姿态 (`Relax-to-OPEN`)，杜绝机械冲击与仿真冲量发散。
4. **全局 KD 参数统一继承**:
   - 利用 MuJoCo `<default>` 机制，实现单处修改 `kp`/`kv` 即可同时作用于全部 16 个电机。

---

## 2. 目录与资产结构

```text
src/tuinadex_sim/
├── config/
│   └── sim_params.yaml            # ROS 2 仿真节点运行参数 (频率、步长、看门狗、话题等)
├── launch/
│   └── sim.launch.py              # tuinadex_sim 独立启动脚本 (支持 gui:=true/false)
├── meshes/                        # 8 组规范化的 LEAP Hand STL 网格模型
│   ├── palm_lower.stl             # 掌心基座外壳
│   ├── mcp_joint.stl              # 手指 MCP 关节
│   ├── pip.stl, dip.stl           # 手指 PIP / DIP 指节
│   ├── fingertip.stl              # 手指远端指尖
│   └── thumb_pip.stl ...          # 拇指专用指节与指尖
├── urdf/
│   ├── leap_hand_right_canonical.urdf  # 规范 16-DOF Canonical URDF 模型
│   └── tuina_bed_dummy.urdf       # 中医推拿床与人体背部假人 URDF 模型
├── worlds/
│   ├── tuina_scene.xml            # 全数字孪生世界 (推拿床 + 假人 + 灵巧手，默认使用)
│   └── leap_hand_right.xml        # 纯灵巧手无约束动力学模型
├── tuinadex_sim/
│   ├── __init__.py
│   ├── sim_bridge.py              # MujocoSimBridge: 纯 Python MuJoCo 动力学步进与状态提取
│   ├── sim_core.py                # SimulationCore: 500Hz 锁步、双模状态机与平滑回退算法
│   └── leap_hand_sim_node.py      # LeapHandSimNode: ROS 2 薄控制节点与 3D Passive Viewer
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 3. 仿真启动方式

### 方式一：系统统一入口启动 (推荐)

项目统一通过 `tuinadex_hand` 的入口控制真机与仿真切换：

```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0
source /opt/ros/lyrical/setup.bash
source install/setup.bash

# 1. 弹出 3D 实时可视化交互画面 (推荐手动联调与观察)
ros2 launch tuinadex_hand hand.launch.py use_sim:=true gui:=true

# 2. 纯后台无头模式运行 (适用于 CI、录制脚本与自动化测试)
ros2 launch tuinadex_hand hand.launch.py use_sim:=true
```

### 方式二：本包独立 launch 启动

直接启动 `tuinadex_sim` 仿真环境：

```bash
# 带 3D 渲染画面启动
ros2 launch tuinadex_sim sim.launch.py gui:=true

# 无头后台模式启动
ros2 launch tuinadex_sim sim.launch.py gui:=false
```

### 方式三：ROS 2 单节点命令行启动

```bash
# 启动并开启 GUI
ros2 run tuinadex_sim leap_hand_sim_node --ros-args -p gui:=true

# 加载纯手部模型 (不加载推拿床与假人)
ros2 run tuinadex_sim leap_hand_sim_node --ros-args -p use_scene_world:=false -p gui:=true
```

### 方式四：MuJoCo 原生独立 3D 预览器 (无需 ROS 2 通信)

用于快速检查模型几何形态、碰撞网格与手动拖动滑块测试关节活动范围：

```bash
# 预览推拿全场景 (推拿床 + 假人 + 灵巧手)
python3 -m mujoco.viewer --mjcf=src/tuinadex_sim/worlds/tuina_scene.xml

# 预览纯灵巧手模型
python3 -m mujoco.viewer --mjcf=src/tuinadex_sim/worlds/leap_hand_right.xml
```
- **快捷操作**: 鼠标左键旋转视角、右键缩放、滚轮平移；展开右侧 **Control** 面板可手动拉动 16 个关节滑块。

### 方式五：一键端到端自动化回环自检

```bash
bash scripts/e2e_sim_check.sh
```
自动拉起后台仿真并注入 50Hz 阶跃测试指令，检验物理收敛、50Hz 通信帧间隔以及 300ms 看门狗回退。

---

## 4. 各参数配置与调参指南

### 4.1 空间位置与位姿配置 (`worlds/tuina_scene.xml`)

灵巧手在推拿床与患者假人背部上方的初始位置和朝向定义在 `worlds/tuina_scene.xml` 中：

```xml
<!-- 灵巧手在仿真世界中的空间位置与姿态 -->
<body name="leap_hand" pos="0.20 0 0.95" euler="0 0 1.57079">
```

| 参数 | 默认值 | 物理含义 | 调参建议 |
| :--- | :--- | :--- | :--- |
| **`pos.x`** | `0.20` | 沿推拿床长轴坐标 (米) | `+x` 朝向假人头部，`-x` 朝向腿部。假人背部正中在 `x=0.20`。 |
| **`pos.y`** | `0.00` | 沿推拿床短轴坐标 (米) | `0.0` 为推拿床与脊柱正中线，`+y` 偏左，`-y` 偏右。 |
| **`pos.z`** | `0.95` | 离地垂直高度 (米) | 推拿床面高 `0.64m`，假人背部表面高 `0.86m`。当前 `z=0.95` 表示手掌悬浮在背部上方 9cm。<br>• **调低** (如 `0.91 ~ 0.92`): 手掌更贴近假人，指尖更容易深压进入皮肤；<br>• **调高** (如 `0.98`): 手指平伸时远离假人，避免微屈时误触碰撞。 |
| **`euler.roll`** | `0.0` | 翻滚角 (弧度) | **严格保持 `0.0`**，代表掌心垂直朝下 (-Z 方向，正对患者背部)。 |
| **`euler.pitch`** | `0.0` | 俯仰角 (弧度) | 手掌前后倾角。可微调 $\pm 0.1$ 模拟下压推拿的前倾倾角。 |
| **`euler.yaw`** | `1.57079` | 偏航角 (弧度) | `1.57079` ($\pi/2 = 90^\circ$): 掌心顺时针偏转 90°，手指横跨脊柱方向；若改为 `0.0` 则手指朝向患者头部纵向推拿。 |

---

### 4.2 执行器 KD 参数统一配置 (`worlds/tuina_scene.xml` & `worlds/leap_hand_right.xml`)

本仿真采用 MuJoCo 原生的 `<default>` 属性继承机制，**只需修改此处一处即可覆盖全部 16 个电机**：

```xml
<!-- 全局执行器 KD 参数统一配置 (单处修改覆盖全部 16 个关节) -->
<default>
  <position kp="35.0" kv="1.5"/>
</default>
```

| 参数 | 默认值 | 物理量与单位 | 调参效果说明 |
| :--- | :--- | :--- | :--- |
| **`kp`** | `35.0` | 比例刚度系数 (Stiffness, $\text{N}\cdot\text{m/rad}$) | • **调大** (如 `45.0 ~ 60.0`): 关节更硬，按压推拿更有力，但过大会产生高频抖动；<br>• **调小** (如 `15.0 ~ 25.0`): 关节更软，柔顺性更好，但按压深度不足易被弹起。 |
| **`kv`** | `1.5` | 微分速度阻尼 (Damping / $K_d$, $\text{N}\cdot\text{m}\cdot\text{s/rad}$) | • **调大** (如 `2.5 ~ 3.5`): 消除超调与震荡，手指运动更平稳沉着；<br>• **调小** (如 `0.5 ~ 1.0`): 响应更敏捷，但接触瞬间易出现抖动与回弹。 |

#### 单关节差异化微调能力 (可选)
如果需要让某个特定关节拥有不同的刚度或阻尼，直接在 `<actuator>` 的对应关节行显式写出 `kp` 或 `kv` 即可，局部参数会自动覆盖全局默认值：
```xml
<!-- 示例: 仅增强拇指根部的下压刚度，其余 15 个关节仍保持全局默认 -->
<position name="act_thumb_cmc" joint="thumb_cmc" kp="50.0" kv="2.5" ctrlrange="0.00 1.60"/>
```

---

### 4.3 力仿真与物理接触动力学参数

| 参数位置 | 标签与属性 | 默认值 | 作用与调参说明 |
| :--- | :--- | :--- | :--- |
| **各关节定义** | `<joint actuatorfrcrange="-0.95 0.95">` | `±0.95 N·m` | 执行器最大输出力矩限制。调大可提升推拿峰值按压力；调小可防止仿真中接触力过载穿透。 |
| **全局解算器** | `<option timestep="0.002">` | `0.002s` (500Hz) | 动力学积分步长。配合 ROS 2 的 10 步子步进，保证高刚度接触数值稳定。 |
| **数值积分器** | `<option integrator="implicitfast">` | `implicitfast` | 针对弹簧阻尼接触优化的隐式快速求解器。 |
| **自碰撞过滤** | `<contact><exclude .../></contact>` | 6 组手掌相邻对 | 过滤手掌基座与手指根部关节之间的相邻网格碰撞穿透，避免内力发散。 |

---

### 4.4 ROS 2 节点层运行参数 (`config/sim_params.yaml`)

ROS 2 运行时的时钟与话题配置位于 `config/sim_params.yaml`：

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
    
    # 安全与看门狗
    watchdog_timeout_sec: 0.3         # 300ms 指令超时判定
    watchdog_hold_sec: 0.3            # 超时后前 300ms 保持当前实测姿态 (Hold 期)
    watchdog_ramp_sec: 0.6            # 随后的 600ms 线性收敛至全开平伸姿态 (Ramp 期)
    
    # 场景资产与可视化配置
    scene_file: "worlds/tuina_scene.xml"
    hand_only_file: "worlds/leap_hand_right.xml"
    use_scene_world: true             # true: 包含推拿床与假人, false: 纯手部环境
    gui: false                        # true: 启动 3D 被动渲染窗口, false: 无头后台运行
```

---

## 5. 调参生效与开发验证工作流

由于 ROS 2 安装空间隔离机制，修改 `worlds/` 或 `config/` 中的参数后，建议按以下流程使改动生效：

```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0
source /opt/ros/lyrical/setup.bash

# 1. 快速更新安装空间 (采用符号链接安装，通常直接生效)
colcon build --symlink-install --packages-select tuinadex_sim
source install/setup.bash

# 2. 运行单元测试验证动力学与时钟未被破坏
~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_sim_bridge.py tests/test_sim_core.py -v

# 3. 启动带 3D 渲染的仿真节点验证实际效果
ros2 launch tuinadex_hand hand.launch.py use_sim:=true gui:=true
```
