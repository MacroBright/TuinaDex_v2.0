# tuinadex_core: 具身控制与安全看门狗纯逻辑内核

`tuinadex_core` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **核心控制与安全数学算法内核（Layer 3: Control & Safety Layer）**。

> [!IMPORTANT]
> **纯粹性架构契约 (Pure Logic Invariant)**:
> `tuinadex_core` 严禁引入任何 ROS 2 运行时依赖（如 `rclpy`、`sensor_msgs`），必须保证在脱机原生 Python 环境下被独立导入与单元测试，作为整个系统唯一的物理限位、关节字典与状态机裁决事实源。

---

## 快速开始 (Quick Start)

### 1. 环境准备与脱机导入
```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0

# 激活控制基座环境 (Python 3.10) 或使用任意安装了 numpy 的原生 Python
source scripts/activate_tuinadex.sh control
```

### 2. 独立纯 Python 测试与调用验证
```python
import numpy as np
from tuinadex_core.names import CANONICAL_JOINT_NAMES, CANONICAL_ARM_JOINT_NAMES
from tuinadex_core.command_processing import align_command, align_arm_command
from tuinadex_core.watchdog import WatchdogStateMachine, WatchdogAction

# 1. 校验与重排 16-DOF 灵巧手指令
raw_hand_q = [0.0] * 16
q_hand_safe, err = align_command(raw_hand_q, names=CANONICAL_JOINT_NAMES)
assert err == "", f"Hand command validation failed: {err}"

# 2. 校验与重排 6-DOF 机械臂指令 (自动 clamp 至物理安全限位)
raw_arm_q = [0.0, 0.85, 0.95, -0.80, 0.0, 0.0]
q_arm_safe, err = align_arm_command(raw_arm_q, names=CANONICAL_ARM_JOINT_NAMES)
assert err == "", f"Arm command validation failed: {err}"

# 3. 运行 300ms 安全看门狗状态机
fsm = WatchdogStateMachine(timeout_sec=0.30)
fsm.on_command(t_now=1.0)
action = fsm.tick(t_now=1.1)  # 耗时 100ms (< 300ms) -> 正常执行
assert action == WatchdogAction.WRITE

action_timeout = fsm.tick(t_now=1.4)  # 耗时 400ms (> 300ms) -> 触发回退
assert action_timeout == WatchdogAction.RELAX_STEP
print("tuinadex_core 核心算法校验通过！")
```

---

## 1. 核心架构与功能定位

`tuinadex_core` 在 TuinaDex 5 层架构中充当安全隔离墙，上承 ROS 2 中间件与遥操采集层，下启底层硬件驱动与动力学仿真桥：

```
┌────────────────────────────────────────────────────────┐
│ Layer 4: ROS2 Middleware Layer (Nodes, Hub, Teleop)    │
└───────────────────────────┬────────────────────────────┘
                            │ /joint_command (位置数组与名称)
┌───────────────────────────▼────────────────────────────┐
│ Layer 3: tuinadex_core (Pure Python Kernel)            │
│ ├── names.py              : 统一关节空间命名事实源     │
│ ├── command_processing.py : 脏值过滤、重排与安全限位门 │
│ └── watchdog.py           : 300ms 无 I/O 纯逻辑状态机 │
└───────────────────────────┬────────────────────────────┘
                            │ Verified Radian Array / Safe Action
┌───────────────────────────▼────────────────────────────┐
│ Layer 2: Driver & Bridge Layer (LeapDriver, SimBridge) │
└────────────────────────────────────────────────────────┘
```

---

## 2. 模块详解与接口契约

### 2.1 规范关节空间命名 (`names.py`)
统一整个软件栈（ROS2 节点、MuJoCo 仿真、Pinocchio IK 与 LeRobot 数据集）的关节序列字典：

- **`CANONICAL_JOINT_NAMES` (16-DOF LEAP Hand)**:
  - 食指 4-DOF: `index_yaw`, `index_mcp`, `index_pip`, `index_dip`
  - 中指 4-DOF: `middle_yaw`, `middle_mcp`, `middle_pip`, `middle_dip`
  - 无名指 4-DOF: `ring_yaw`, `ring_mcp`, `ring_pip`, `ring_dip`
  - 拇指 4-DOF: `thumb_yaw`, `thumb_cmc`, `thumb_mcp`, `thumb_ip`
- **`CANONICAL_ARM_JOINT_NAMES` (6-DOF reBot-DevArm RS)**:
  - 臂 6-DOF: `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`

### 2.2 指令报文校验与限位夹取 (`command_processing.py`)
秉持“安全优先，宁拒勿猜”原则，彻底拦截越界与污染数据直通硬件：

- **拦截并拒绝的异常指令**:
  - 维度不匹配（灵巧手必须严格 16 维，机械臂严格 6 维）；
  - 数值包含 `NaN` 或 `Inf`（杜绝模型发散脏值损坏舵机）；
  - `names` 字段包含未知关节名或数量不符（杜绝错位写入）。
- **逐关节安全限位门 (Spec §1)**:
  - `JOINT_SPEC_LIMITS` (16-DOF 弧度范围，以全开平伸姿态为 0 rad 基准)；
  - `ARM_SPEC_LIMITS` (6-DOF 机械臂关节物理限位)；
  - 合法但越界的绝对目标将被**硬饱和夹取到临界边缘 (np.clip)**，绝不向电机下发越界目标。

### 2.3 指令流看门狗安全状态机 (`watchdog.py`)
- **纯逻辑无 I/O 设计**：不直接调用系统时钟，由外部节点（ROS2 Node Clock 或仿真 Timer）注入时间戳 `t_now`；
- **状态迁移与动作裁决**:
  - `WatchdogState.FOLLOWING`: 正常指令流周期更新；
  - `WatchdogState.RELAXING`: 指令流中断超过 `timeout_sec`（默认 300ms），进入阻尼平滑回退；
  - `WatchdogAction.WRITE`: 指令流正常，向硬件或仿真下发目标；
  - `WatchdogAction.RELAX_STEP`: 超时触发，通知驱动侧执行一步平滑插值（`0.3s hold + 0.6s ramp` 回到基准零位）；
  - `WatchdogAction.NONE`: 从未收到过指令时保持原样。

---

## 3. 目录结构

```
src/tuinadex_core/
├── package.xml                   # ROS 2 包描述清单 (仅声明纯 Python 元信息)
├── setup.cfg                     # 安装配置文件
├── setup.py                      # Python 打包安装脚本
├── resource/
│   └── tuinadex_core             # ament 索引标记
└── tuinadex_core/
    ├── __init__.py               # 模块导出入口
    ├── names.py                  # 16D 手 / 6D 臂规范关节空间名称
    ├── command_processing.py     # 报文校验、Canonical 重排与限位饱和
    └── watchdog.py               # 指令超时状态机 (纯逻辑, 无 I/O)
```

---

## 4. 黄金架构不变量 (Invariants)

1. **零 ROS 依赖不变量**: `tuinadex_core` 任何子模块均不得出现 `import rclpy` 或导入 ROS 消息类型；
2. **SI 国际标准单位 (rad)**: 角度数值强制全链路采用弧度制，以全开平伸姿态为 0 rad；
3. **确定性重排**: 当外部调用方传入非 Canonical 顺序的 `names` 时，必须依据命名字典重排为规范索引顺序。

---

## 5. 单元测试与验证门禁

在原生 Python 或激活环境下一键运行脱机单元测试：

```bash
# 激活环境
source scripts/activate_tuinadex.sh control

# 运行核心算法与看门狗全量测试 (23 项单测)
pytest tests/test_command_processing.py tests/test_watchdog_fsm.py tests/test_tuinadex_core.py tests/test_tuinadex_arm_core.py -v
```

**预期输出指标**:
```
tests/test_command_processing.py::test_valid_canonical_command PASSED
tests/test_command_processing.py::test_dimension_mismatch PASSED
tests/test_command_processing.py::test_nan_inf_rejection PASSED
tests/test_watchdog_fsm.py::test_watchdog_following_to_relaxing PASSED
...
============================== 23 passed in 0.25s ==============================
```
