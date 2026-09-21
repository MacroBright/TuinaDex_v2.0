# ADR 0002: TuinaDex v2.0 MuJoCo 仿真数字孪生与真机统一切换架构决策记录

> **状态**: Accepted (已实施并通过端到端双模实证验证)  
> **日期**: 2026-09-21  
> **决策者**: Antigravity, Claude Code, Codex, MacroBright  
> **适用范围**: `tuinadex_core`, `tuinadex_sim`, `tuinadex_hand`, `tuinadex_teleop`, `tuinadex_recorder`

---

## 1. 背景与问题陈述 (Context & Problem)
在 TuinaDex v2.0 LEAP Hand MVP 闭环体系建立后，为在启动真实硬件（Dynamixel 4Mbps USB-TTL 串口舵机总线）前验证推拿触式控制与策略闭环，需引入基于 MuJoCo 的物理仿真数字孪生系统。

引入仿真模块面临以下核心挑战与架构权衡：
1. **通信等价性与零感知切换**: 上层遥操作 (`tuinadex_teleop`)、示教录制 (`tuinadex_recorder`) 及未来的具身策略学习模型，绝不能感知底层运行的是真实硬件还是 MuJoCo 仿真器；
2. **包依赖拓扑与循环依赖风险**: `tuinadex_hand` 中沉淀了成熟的 `command_processing.py`（指令限位门）与 `watchdog.py`（看门狗状态机）。若 `tuinadex_sim` 依赖 `tuinadex_hand`，而 `tuinadex_hand` 的启动入口又需依据 `use_sim` 条件拉起 `tuinadex_sim`，将导致包间双向循环依赖；
3. **多速率并发与时序确定性**: 真实物理舵机在硬件板载微控制器上以 kHz 级内环闭环，宿主通过 50Hz 周期轮询。在仿真中，MuJoCo 刚体接触需 500Hz (`dt=0.002s`) 步长以保证数值积分稳定，而 ROS 2 节点通信为 50Hz (`20ms`)。如何平衡物理步进与 ROS 2 定时器？
4. **失联回退动力学保真**: 真机驱动在看门狗超时时采用 `0.3s hold + 0.6s ramp` 平滑阻尼回退至 0 rad 全开姿态；仿真若直接重置目标到 0，会产生阶跃冲激甚至模型失稳。

---

## 2. 核心架构决策 (Decisions)

### 决策 1: 共享纯逻辑下沉 — 新建 `tuinadex_core` 功能包 (Invariant 17)
- **方案选择**: 创建底层无 ROS 依赖的纯逻辑功能包 `tuinadex_core`，将 16 维规范关节命名 (`CANONICAL_JOINT_NAMES`)、逐关节限位 (`JOINT_SPEC_LIMITS`)、指令校验门 (`align_command`) 与看门狗状态机 (`WatchdogStateMachine`) 单源化下沉至该包。
- **决策理由**:
  1. 彻底切断 `tuinadex_hand` 与 `tuinadex_sim` 的双向耦合，收敛为严格的单向无环图 (DAG)：`tuinadex_hand -> tuinadex_core`，`tuinadex_sim -> tuinadex_core`；
  2. `tuinadex_core` 零 ROS 2 依赖，可在任何轻量 Python 环境极速运行单测，保障核心不变量的一致性与单点维护。

### 决策 2: 单线程 50Hz Timer + 10x 0.002s 物理锁步积分 (Invariant 16)
- **方案选择**: 仿真节点 `leap_hand_sim_node` 采用**单线程 50Hz ROS2 Timer**。在每个 20ms 的 tick 内，连续调用 10 次 `mujoco.mj_step`（每次 $0.002\text{s}$，累计 $10 \times 0.002\text{s} = 20\text{ms}$），物理步进完成后单次提取状态并发布 `/hand/joint_states`。
- **决策理由**:
  1. 放弃并发后台物理线程模型，彻底根除跨线程数据竞态、锁竞争与时间抖动；
  2. 保障 500Hz 动力学微积分数值稳定性与 50Hz ROS 2 采样点的严格逐点对齐与确定性复现。

### 决策 3: 仿真端复刻 Hold-and-Ramp 看门狗平滑回退 (Invariant 18)
- **方案选择**: 在 `SimulationCore` 中实现逐点复刻真机 `driver.relax_to_open_step` 的回退逻辑：
  - **Hold 期 (0.3s)**: 指令超时进入 RELAXING 初始 0.3s 内，保持当前实测位姿不变；
  - **Ramp 期 (0.6s)**: 线性平滑插值插向 0 rad 全开位；
  - 收到新指令时调用 `reset_loss_state()` 清除回退轨迹。
- **决策理由**:
  杜绝断联瞬时目标归零引起的刚体机械冲击与 MuJoCo 求解器冲量发散，保证仿真动力学行为与真机高度一致。

### 决策 4: 统一入口双模切换 (`use_sim:=true/false`) 与话题完全等价 (Invariants 13 & 14)
- **方案选择**:
  - `src/tuinadex_hand/launch/hand.launch.py` 提供 `use_sim` 一键切换参数；
  - 当 `use_sim:=false` 时拉起真实物理驱动节点 `leap_hand_node`（支持 `mock:=true|false`）；
  - 当 `use_sim:=true` 时拉起数字孪生节点 `leap_hand_sim_node`；
  - 两者均对外严格暴露 `/hand/joint_command` 订阅与 `/hand/joint_states` 发布；
  - MuJoCo 求解器积分器固化为 `implicitfast`，配置 16 组带接触排除的 position 舵机执行器 ($k_p=35.0, k_v=1.5$)。

---

## 3. 结果与验证依据 (Consequences & Evidence)

1. **5 包全量 ROS 2 编译通过**:
   - 命令: `colcon build --symlink-install --packages-select tuinadex_core tuinadex_hand tuinadex_teleop tuinadex_recorder tuinadex_sim`
   - 结果: 5 packages finished cleanly (3.98s)。
2. **多矩阵自动化单测全绿**:
   - `leap_hand` conda env (Python 3.14): `pytest tests/` 输出 **47 passed, 6 skipped**；
   - `smolvla` conda env (Python 3.10): `pytest` 输出 **5 passed, 2 skipped**；
   - ROS 2 Lyrical 主机集成测试: `python3 -m pytest tests/test_leap_hand_node_integration.py tests/test_recorder_node.py tests/test_sim_node_integration.py -v` 输出 **4 passed** (4.24s)。
3. **端到端双模式闭环自动化通过**:
   - **MuJoCo 仿真闭环**: `bash scripts/e2e_sim_check.sh` 输出 `[E2E SIM] PASS` (131 帧状态，周期中位数 19.97ms，物理收敛并平滑回退 OPEN)；
   - **Mock 驱动闭环**: `bash scripts/e2e_mock_check.sh` 输出 `[E2E] PASS` (146 帧 rosbag2 录制与 LeRobot Parquet 转换通过)。
4. **启动流双分支实测验证**:
   - `ros2 launch tuinadex_hand hand.launch.py use_sim:=true` 成功拉起 `leap_hand_sim_node` (50.0Hz, 10 substeps)；
   - `ros2 launch tuinadex_hand hand.launch.py use_sim:=false mock:=true` 成功拉起 `leap_hand_node`。
