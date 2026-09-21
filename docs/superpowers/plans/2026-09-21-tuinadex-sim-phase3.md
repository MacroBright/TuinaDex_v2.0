# TuinaDex v2.0 仿真数字孪生 — Phase 3 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付 `tuinadex_sim` 数字孪生的可运行闭环：完整 LEAP Hand MuJoCo 仿真控制节点、真机/仿真统一 `use_sim` 启动流、ROS2 仿真回环测试与端到端验证。全程 TDD 可实证。

**Architecture:** 仿真节点 = 喂入纯逻辑仿真核心（`SimulationCore` 组合 `MujocoSimBridge` + `tuinadex_core.watchdog` + `tuinadex_core.command_processing` + 复刻真机的平滑 relax 插值）+ 薄 rclpy Node 驱动。**单线程 50Hz ROS2 Timer**，每 tick 内锁 10 次 `mj_step`（10×0.002s=20ms），严格 500Hz 物理积分、50Hz 采样发布。统一开关通过 `hand.launch.py use_sim:=true/false` 实现，共享纯逻辑下沉到新低层包 `tuinadex_core` 以消除 hand↔sim 循环依赖。

**Tech Stack:** ROS2 (Lyrical Luth Desktop，`/opt/ros/lyrical/setup.bash`) / Python 3 / numpy / mujoco 3.13.0 / pytest / launch_ros。

**Spec:** `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`（16-DOF 契约）。架构图：`docs/architecture/tuinadex_sim_digital_twin.html`。

## Global Constraints

- 全链路 SI 弧度 (rad)；OPEN_POSE 全开位 0 rad，屈曲为正。
- 通信契约与真机 100% 等价：订阅 `/hand/joint_command`、发布 `/hand/joint_states`，均为 `sensor_msgs/JointState`，16 维 Canonical 命名与限位完全对齐（Invariant 13）。
- 物理 500Hz 步进（`dt=0.002`）保接触稳定；ROS2 发布严格 50Hz 降采样（Invariant 14）。
- Watchdog 300ms 断流 → 平滑 Relax-to-OPEN；仿真端镜像真机 hold(0.3s)+ramp(0.6s) 曲线（Invariant 15）。
- **单线程时序**：50Hz Timer 回调内循环 10 次子步进完成物理积分，禁后台物理线程（Invariant 16）。
- **依赖 DAG**：纯逻辑（watchdog / command_processing / JOINT_SPEC_LIMITS / CANONICAL_JOINT_NAMES）下沉 `tuinadex_core`；依赖收敛为无环图，禁 hand↔sim 循环（Invariant 17）。
- 安全纯逻辑不依赖 rclpy，便于 pytest 快速单测。
- 本地单元测试：`~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`（mujoco 3.13.0 + numpy 2.5.1 已确认）。ROS2 依赖测试用 `pytest.importorskip("rclpy")` 守护。
- 每个任务一个 commit；现状基线：`73a55b8`（Phase 1 收官，sim 包 5/5 单测 + 4 包 colcon 编译通过）。

## 已裁决架构决策 (Invariants，来自 Phase 2 用户拍板)

| # | 裁决 | 内容 |
|---|------|------|
| 16 | **线程/时序模型** | 单线程 50Hz ROS2 Timer，每 tick 内锁步进 10 次 `mj_step(0.002)`=严格 500Hz 物理积分；50Hz 采样点发布。不用后台物理线程（零竞态、确定性、与真机 50Hz 逐点对齐）。 |
| 17 | **共享纯逻辑包 + 依赖 DAG** | 新建低层包 `tuinadex_core` 承接 `watchdog.py`/`command_processing.py`/`JOINT_SPEC_LIMITS`/`CANONICAL_JOINT_NAMES`；`tuinadex_hand` 与 `tuinadex_sim` 均依赖它。依赖收敛为无环 DAG：`hand→sim→core`、`hand→core`，消除 hand↔sim 循环。 |
| 18 | **仿真回退保真** | 仿真端 Relax-to-OPEN 逐点复刻真机 `driver.relax_to_open_step`：hold 期(0.3s)保持当前实测位姿 → ramp 期(0.6s)线性收敛 0 rad，基于实测关节位置、带 `_loss_t0`/`_relax_from_pos` 状态与 `reset_loss_state`。禁止瞬时跳变。 |

---

### Task 1: 共享纯逻辑包 `tuinadex_core` 建立 + Canonical 单源化

**Files:**
- Create: `src/tuinadex_core/package.xml`、`setup.py`、`setup.cfg`、`resource/tuinadex_core`、`tuinadex_core/__init__.py`
- Create: `src/tuinadex_core/tuinadex_core/names.py`（`CANONICAL_JOINT_NAMES` 唯一真源）
- Move (git mv 保留历史): `src/tuinadex_hand/tuinadex_hand/watchdog.py` → `src/tuinadex_core/tuinadex_core/watchdog.py`
- Move (git mv): `src/tuinadex_hand/tuinadex_hand/command_processing.py` → `src/tuinadex_core/tuinadex_core/command_processing.py`
- Modify: `driver/leap_driver.py`（`CANONICAL_JOINT_NAMES` 改从 core 导入并 `as CANONICAL_JOINT_NAMES` 保持 `from tuinadex_hand.driver.leap_driver import CANONICAL_JOINT_NAMES` 兼容）；`sim_bridge.py`（删本地复制，改从 core 导入）；`leap_hand_node.py`、`leap_hand_sim_node.py`（看门狗/指令路径改从 core 导入）
- Modify: `tuinadex_hand/package.xml` 加 `<depend>tuinadex_core</depend>`；`tuinadex_sim/package.xml` 把 `<depend>tuinadex_hand</depend>` 改为 `<depend>tuinadex_core</depend>`
- Modify: `tests/test_watchdog_fsm.py`、`tests/test_command_processing.py`（导入改 `from tuinadex_core...`）

**Interfaces:**
- Produces: 纯逻辑包 `tuinadex_core`（`watchdog.WatchdogStateMachine`, `command_processing.align_command`+`JOINT_SPEC_LIMITS`, `names.CANONICAL_JOINT_NAMES`），Task 2/3 依赖。`tuinadex_hand.driver.leap_driver.CANONICAL_JOINT_NAMES` 兼容性保留。

- [ ] **Step 1: 建包脚手架**（copy 现有 `tuinadex_sim` 的 package.xml/setup.py/setup.cfg，改 name=tuinadex_core，加 console 空 entry 或仅库）
- [ ] **Step 2: git mv 迁移 watchdog.py / command_processing.py 到 core**
- [ ] **Step 3: 建 `names.py` 持 `CANONICAL_JOINT_NAMES`；把命令处理与 sim_bridge 中的名单改为导入**
- [ ] **Step 4: 修 command_processing 的 `from .driver.leap_driver import CANONICAL_JOINT_NAMES` → `from .names import CANONICAL_JOINT_NAMES`；leap_driver.py 改为从 core.names 导入并 re-export**
- [ ] **Step 5: 修 tuinadex_hand/sim 相关引入：leap_hand_node.py / leap_hand_sim_node.py / sim_bridge.py**
- [ ] **Step 6: 更新两个 package.xml 依赖，消除 hand↔sim 循环**
- [ ] **Step 7: 更新 tests 两文件的 import**
- [ ] **Step 8: 全量回归与提交**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`
Expected: 全绿（含原 watchdog_fsm、command_processing、leap_driver、sim_bridge 用例，导入路径已切换）。

Run: `source /opt/ros/lyrical/setup.bash && colcon build --packages-select tuinadex_core tuinadex_hand tuinadex_sim`
Expected: 无依赖环路、4 包编译通过（顺序 tuinadex_core → tuinadex_hand/sim）。

```bash
git add -A && git commit -m "refactor(core): extract shared pure-logic tuinadex_core, single-source canonical joint names"
```

---

### Task 2: 纯逻辑仿真核心 `SimulationCore` + 复刻真机的平滑 relax 插值

**Files:**
- Create: `src/tuinadex_sim/tuinadex_sim/sim_core.py`
- Test: `tests/test_sim_core.py`

**Interfaces:**
- Produces: `SimulationCore(bridge, timeout_sec=0.30, hold_sec=0.30, ramp_sec=0.60, physics_substeps=10)`。`bridge` 为鸭子类型（`set_joint_targets(q16)` / `step(n)` / `get_state()->SimState`，测试注入 FakeBridge）。方法：
  - `apply_command(q_16, t_now) -> bool`：经 `align_command` 校验夹取；合法则 `watchdog.on_command(t_now)` 并缓存目标，返回 True；拒绝(NaN/维错/未知名)返回 False 不改状态。
  - `tick(t_now) -> SimState | None`：`watchdog.tick(t_now)` → `WRITE`: 用缓存目标；`RELAX_STEP`: 走内部 `_RelaxTrajectory`；`NONE`: 目标保持当前。设 `bridge.set_joint_targets` 后 `bridge.step(physics_substeps)`，返回 `bridge.get_state()`。
  - 内部 `_RelaxTrajectory`：忠实移植 `driver.relax_to_open_step`（else 分支即 `from_pos*(1-t)`），基于 `bridge.get_state().positions` 作 `_relax_from_pos`，带 `_loss_t0`，新指令经 `reset_loss_state()` 清挡。

- [ ] **Step 1: 写失败测试**（FakeBridge + 注入时钟，全走纯逻辑不涉 mujoco）：
  - 合法指令 → tick 步进 10 次且目标=夹取后指令。
  - 越界指令被夹到 `JOINT_SPEC_LIMITS` 边沿。
  - NaN/维错/未知名指令被拒，状态不变。
  - 300ms 内连续 tick → 持续 WRITE 跟随。
  - 超时进入 RELAX：前 hold(0.3s) 目标=from_pos 位姿不变；ramp(0.6s)+后 → 目标收敛≈0。
  - 新指令解除 relax（reset_loss_state 清挡）。
  - `physics_substeps` 被忠实调用（FakeBridge.step 计数==10）。
- [ ] **Step 2: 运行确认失败**
Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_sim_core.py -v`
Expected: FAIL，`ModuleNotFoundError: tuinadex_sim.sim_core`
- [ ] **Step 3: 最小实现 `sim_core.py`**（组合结构，relax 插值逐行对照 `leap_driver.py:224-246`）
- [ ] **Step 4: 全绿后提交**
Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`
```bash
git add -A && git commit -m "feat(sim): add pure SimulationCore with faithful hold+ramp relax-to-open, 500Hz substep"
```

---

### Task 3: 完整仿真控制节点 `leap_hand_sim_node.py`（薄节点 + 50Hz 单线程循环）

**Files:**
- Modify: `src/tuinadex_sim/tuinadex_sim/leap_hand_sim_node.py`
- Test: `tests/test_sim_node_integration.py`（`pytest.importorskip("rclpy")` 守护）

**Interfaces:**
- Produces: `LeapHandSimNode(Node)`：
  - 订阅 `/hand/joint_command` → `align_command` → `simulation_core.apply_command(q, t_now)`。
  - 50Hz `create_timer(1/rate, _control_loop)`：`simulation_core.tick(t_now)` → 构造 `sensor_msgs/JointState`（name=CANONICAL names，pos/vel/eff 取自 SimState）发布 `/hand/joint_states`。
  - 时钟统一 `self.get_clock().now()`（禁 wall-clock，与真机一致）。
  - 参数注入：`xml_path`（`scene_file`/`hand_only_file` 二选一 by `use_scene_world`）、`physics_timestep`、`publish_rate_hz`、`watchdog_timeout_sec`、`watchdog_hold_sec`、`watchdog_ramp_sec`、`physics_substeps`。
  - `__init__` 接受 `parameter_overrides` 便于直接构造测试。

- [ ] **Step 1: 设计注入接口**（节点可直接喂 FakeBridge 或真 MujocoSimBridge，验证不依赖硬件）
- [ ] **Step 2: 写失败集成测试**（importorskip rclpy）：
  - 构造节点(FakeBridge) + 发布 `/hand/joint_command` → 周期内 `/hand/joint_states` 收到并逐步收敛目标。
  - 停发指令 > 300ms → 状态平滑回退 0（hold+ramp 后）。
  - 发布频率≈50Hz（采样时间戳间隔≈20ms）。
- [ ] **Step 3: 实现节点**（复用 Task 2 SimulationCore，仅做 rclpy 薄接线；`main()` 与现有保持一致）
- [ ] **Step 4: 全绿后提交**
Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_sim_node_integration.py -v`
```bash
git add -A && git commit -m "feat(sim): implement LeapHandSimNode 50Hz single-thread loop, ROS2 equiv-invariant topics"
```

---

### Task 4: 统一 `use_sim` 条件切换 + end-to-end 回环验证

**Files:**
- Modify: `src/tuinadex_hand/launch/hand.launch.py`（加 `use_sim` 参数 + `IfCondition` 分支拉起 `leap_hand_sim_node`/`leap_hand_node`）
- Modify: `src/tuinadex_sim/launch/sim.launch.py`（参数与 sim_params.yaml 对齐，供 use_sim 内部复用）
- Script: 端到端回环验证命令录于 `AGENTS.md`（构建/测试命令必须落盘）

**Interfaces:**
- Produces: 命令 `ros2 launch tuinadex_hand hand.launch.py use_sim:=true|false` 一键无缝切换；上层（teleop）零感知。

- [ ] **Step 1: hand.launch.py 加 `use_sim` LaunchArgument + `launch.actions.IfCondition` 条件分支**（`use_sim:=true` → `tuinadex_sim` 的 `leap_hand_sim_node`；否则 `tuinadex_hand` 的 `leap_hand_node`）。`tuinadex_hand` manifest 补 `<depend>tuinadex_sim</depend>`（方向 hand→sim，无环，因 sim→core 已取代 sim→hand）。
- [ ] **Step 2: sim.launch.py 参数接线对齐**（`publish_rate_hz`、`use_scene_world`、`xml_path` 选择、watchdog 三件套），保证 use_sim 分支带入正确参数。
- [ ] **Step 3: 编译并验证两个模式启动**
Run: `source /opt/ros/lyrical/setup.bash && colcon build --packages-select tuinadex_core tuinadex_hand tuinadex_sim`
Run (sim 模式): `ros2 launch tuinadex_hand hand.launch.py use_sim:=true` + `ros2 topic echo /hand/joint_states` → 有 16 维状态流。
Run (real 模式, dry 检查): `ros2 launch tuinadex_hand hand.launch.py use_sim:=false` → 逻辑拉起 `leap_hand_node`（无硬件可 mock）。
- [ ] **Step 4: 端到端回环脚本**：发 `/hand/joint_command` → 断言 `/hand/joint_states` 收敛；停发 → 断言 300ms 后平滑回退 0；采样确认 50Hz。录为 `scripts/e2e_sim_check.sh`。
- [ ] **Step 5: 更新 AGENTS.md §2 验证命令 + 提交**

```bash
git add -A && git commit -m "feat(sim): unified use_sim launch switch, e2e sim loop check"
```

---

## 验收标准 (Completion Criteria)

- [ ] `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v` 全绿（新增 sim_core / sim_node_integration / 既有用例回归）。
- [ ] `colcon build --packages-select tuinadex_core tuinadex_hand tuinadex_sim` 无依赖环路通过。
- [ ] `ros2 launch tuinadex_hand hand.launch.py use_sim:=true` 拉起仿真，`/hand/joint_states` 16 维 50Hz 状态流。
- [ ] 端到端回环：指令收敛、300ms 断流平滑回退全开、发布频率≈50Hz 均有实证。
- [ ] 每任务一个 commit；`.agent/SESSION_STATE.md` 已更新为 Phase 3 完成、Phase 4 待交接。