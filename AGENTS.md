# AGENTS.md — TuinaDex v2.0 (中医推拿机械臂-灵巧手具身智能系统)

> 本文件是 TuinaDex v2.0 工程专属规则层。
> 全局通用规则位于 `~/.codex/AGENTS.md`，本项目专属的构建、测试、接口契约与硬件红线放这里。

---

## 1. 项目目的
TuinaDex 是面向中医推拿/按摩场景构建的具身智能机器人系统。
当前阶段（MVP）专注于 **16-DOF LEAP Hand 灵巧手** 的最小闭环系统：
`ROS2 运行时 -> 遥操作示教 -> rosbag2 数据记录 -> Dataset Adapter -> LeRobot 策略学习 -> ROS2 真机推理部署`。
后续再无缝平滑扩展至 6-DOF 机械臂与视觉感知系统。

### 五层分层架构（严禁跨层调用）
```
┌────────────────────────────────────────────────────────┐
│ 5. AI / Policy Layer (LeRobot / SmolVLA / ACT)         │  -> tuinadex-vla (CUDA 12.8)
└───────────────────────────┬────────────────────────────┘
                            │ Observation / Action (16D rad)
┌───────────────────────────▼────────────────────────────┐
│ 4. ROS2 Middleware Layer (Nodes, rosbag2, Teleop)      │  -> tuinadex-control
└───────────────────────────┬────────────────────────────┘
                            │ /hand/joint_command & /hand/joint_states
┌───────────────────────────▼────────────────────────────┐
│ 3. Control & Safety Layer (Watchdog FSM, Clamping)     │  -> tuinadex_core
└───────────────────────────┬────────────────────────────┘
                            │ Verified Radian Array / Safe Hold+Ramp
┌───────────────────────────▼────────────────────────────┐
│ 2. Driver & Bridge Layer (LeapDriver, MujocoSimBridge) │  -> tuinadex_hand / tuinadex_sim
└───────────────────────────┬────────────────────────────┘
                            │ Dynamixel Protocol 2.0 / 500Hz Substep
┌───────────────────────────▼────────────────────────────┐
│ 1. Hardware / Simulation Layer (LEAP Hand / MuJoCo)    │  -> U2D2 USB-TTL / tuina_scene.xml
└────────────────────────────────────────────────────────┘
```

---

## 2. 技术栈与运行环境
- **中间件与框架**: ROS2 Lyrical Luth Desktop / Python 3.14+ / Hugging Face LeRobot
  - 当前宿主机为 Ubuntu 26.04 (Resolute)，已通过官方 ROS apt 源安装 `ros-lyrical-desktop` 和 `ros-dev-tools`。
  - `~/.bashrc` 已固化 `source /opt/ros/lyrical/setup.bash`。
  - `scripts/ros2_docker.sh` 仅作为官方 `ros:lyrical` 镜像的隔离回退环境，不再作为默认运行入口。
  - 仿真/真机统一入口：`ros2 launch tuinadex_hand hand.launch.py use_sim:=true|false`。
- **硬件通信协议**: 
  - LEAP Hand: Dynamixel Protocol 2.0 (USB-TTL, 波特率 4,000,000 bps, 位置-电流模式)
  - 机械臂 (后续): Linux SocketCAN (500 kbps)
- **构建系统**: `colcon build --symlink-install`
- **单测框架**: `pytest tests/`

---

## 3. 核心接口与架构不变量 (Invariants)
1. **坐标与单位契约**:
   - ROS2 接口与 LeRobot 数据集全链路强制 **SI 国际标准单位：弧度 (rad)**；
   - 以实测标定后的 `OPEN_POSE`（全开平伸）为 **0 rad 基准零位**，手指各关节向内弯曲屈曲为正 ($q > 0$)。
2. **状态与动作严格分离**:
   - 反馈状态: `/hand/joint_states` (`sensor_msgs/JointState`)，反映真实物理关节位置；
   - 目标指令: `/hand/joint_command` (`sensor_msgs/JointState`)，反映上层目标位置期望；
   - 动作空间 (Action Space): 严格使用 **16 维 absolute joint position**，绝对禁止使用增量 delta。
3. **双域架构解耦**:
   - **ROS2** 负责机器人运行时（Runtime）：通信、控制循环、硬件抽象、安全看门狗；
   - **LeRobot** 负责策略学习（Learning）：数据集处理、离线训练、策略评测；
   - 严禁 LeRobot 绕过 ROS2 直接读写硬件驱动。
4. **数据事实源契约**:
   - `rosbag2` 是实验原始数据的唯一事实源 (Source of Truth)；
   - `LeRobotDataset` 是下游学习数据产品，由独立离线工具 `tuinadex_dataset` 转换生成。

---

## 4. 关键路径约束与硬件安全红线 (Safety Guardrails)
- **高危操作防范**:
  - 严禁无物理限位保护向电机发送越界绝对弧度；
  - 串口断开或主端手势丢失超过 300ms 时，灵巧手必须触发安全看门狗，平滑阻尼回退至全开平伸姿态 (`Relax-to-OPEN`)；
  - 电机工作电流上限锁定为 `150mA`，杜绝堵转过热或暴力夹捏；
  - 关键控制循环内部禁止执行重型阻塞 I/O 或高耗时复杂计算。
- **硬件需人工确认的操作清单**:
  - 重新烧录/刷写舵机 ID 或固件；
  - 覆盖修改物理绝对零位标定文件 (`poses.json`)；
  - 机械臂上电使能或解除急停。

---

## 5. 完成标准与验证 (Completion Criteria)
声称功能完成前必须提供充分实证：
- ROS2 节点能成功通过 `colcon build` 编译；
- 单元测试覆盖率充足且全绿 (`pytest tests/`)；
- 无物理硬件连接时，提供完整的虚拟仿真/Mock 回环测试或离线转换自检。

### 验证命令 (固化)
- 宿主机控制、仿真与遥操单测 (tuinadex-control)：
  `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/ -v`
- ROS2 集成测试（宿主机 Lyrical Desktop）：
  `source /opt/ros/lyrical/setup.bash && source install/setup.bash && python3 -m pytest tests/test_leap_hand_node_integration.py tests/test_arm_node_integration.py tests/test_sim_node_integration.py tests/test_tuinadex_bringup_launch.py tests/test_arm_hand_unified_control.py tests/test_recorder_node.py tests/test_ros2_robot_env.py tests/test_teleop_sim_composite_node.py -v`
- 遥操与仿真算法单测 (tuinadex-control, 81 项全量)：
  `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/teleop/ -v`
- 22 维复合仿真遥操节点 ROS2 启动入口：
  `ros2 launch tuinadex_teleop teleop_sim.launch.py use_sim:=true mock:=false`
- 22 维复合仿真遥操独立预览交互（带 OpenCV HUD & MuJoCo 3D 窗口）：
  `python scripts/run_sim_teleop.py` (加 `--mock --no-gui` 支持离线 CI)
- LeRobot 适配器与学习测试 (tuinadex-vla)：
  `~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_dataset_adapter.py tests/test_lerobot_v3_exporter.py -v`
- 构建（8 包全量）：
  `source /opt/ros/lyrical/setup.bash && colcon build --symlink-install`
- 臂手 22 维联合控制与双模式启动端到端闭环 E2E：
  `bash scripts/e2e_unified_robot_check.sh`
- 整机统一启动入口：
  `ros2 launch tuinadex_bringup robot.launch.py mode:=arm_hand use_sim:=true|false mock:=true|false`
- 无真机灵巧手端到端闭环：
  `bash scripts/e2e_mock_check.sh`
- MuJoCo 仿真节点集成测试：
  `source /opt/ros/lyrical/setup.bash && python3 -m pytest tests/test_sim_node_integration.py -v`
- MuJoCo 统一入口 E2E：
  `bash scripts/e2e_sim_check.sh`
- 多模态数据采集与 LeRobot v3.0 导出测试 (tuinadex-vla)：
  `~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_lerobot_v3_exporter.py -v`
- 多模态数据采集与 LeRobot v3.0 端到端闭环 E2E：
  `bash scripts/e2e_collection_check.sh`
- 三机位采集 Hub 端到端 E2E（真实 EGL 渲染 + H.264 导出）：
  `bash scripts/e2e_hub_collection_check.sh`
- 三机位相机与采集契约单测：
  `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_camera_rig.py tests/test_episode_buffer.py tests/test_data_channel.py tests/test_data_hub_controller.py tests/test_collection_hub_api.py -v`
- 数据采集运行入口：
  `python scripts/collect_teleop_data.py --env sim|real --mode hand_only|arm_hand --cameras head,wrist`

---

## 6. 环境管理与依赖变更门禁 (Environment & Dependency Change Gate)
任何涉及项目运行环境、Conda 环境、Docker 容器、系统依赖、pip 包或通信协议配置的改动，**必须严格遵循《多端异构环境管理架构设计规范》(`docs/specs/ENVIRONMENT_MANAGEMENT_SPEC.md`)，并执行以下准入检查后方可放行**：

1. **环境归属唯一性校验 (Environment Alignment)**:
   - 严禁随意创建未在规范中备案的零散 Conda 环境或在宿主机随意 `pip install`；
   - 6-DOF 机械臂/16-DOF 灵巧手控制、MuJoCo 动力学仿真、手势/视觉遥操严格归入 `tuinadex-control` (Python 3.10.20)；
   - 具身策略学习、LeRobot 轨迹转换、SmolVLA/ACT 训练评估严格归入 `tuinadex-vla` (Python 3.10.20 + CUDA 12.8)；
   - 跨版本/异构计算任务使用容器化隔离 (`TD_v2.0-ENV-DATA`, `TD_v2.0-ENV-CLOUD`)。
2. **零污染与终端局部作用域准则 (Anti-Pollution & Terminal Scope)**:
   - 禁止在宿主机 `~/.bashrc` 中直接写入特定 Conda 环境的自动激活；
   - 统一使用 `source scripts/activate_tuinadex.sh [control|vla]` 仅在当前终端按需切换，杜绝与外部项目（如 hamer 等）产生依赖污染。
3. **可复现性规格同步 (Specification Reproducibility)**:
   - 一旦向 `tuinadex-control` 或 `tuinadex-vla` 添加、升级或移除依赖包，**必须同步更新并导出锁定文件** (`scripts/env/tuinadex-control.yml` / `scripts/env/tuinadex-vla.yml`)。
4. **强行放行前置门禁 (Pre-Commit / Pre-Release Gate)**:
   - 任何涉及环境、依赖、脚本改动的代码提交前，必须运行全套自动化环境验证：
     `bash scripts/verify_all_envs.sh`
   - 只有当环境自动化测试（`test_env_management.py`）全绿且系统诊断全部为 `[PASS]` 时，才允许放行与合入主分支。

---

## 7. 多 Agent 协作与对抗验证门禁 (Multi-Agent Collab & Verification Protocol)
项目全面遵循全局多智能体协作与长程接力协议（`~/.codex/AGENTS.md` §14）：

1. **工位协同矩阵 (Collab Matrix)**:
   - **Antigravity** (Seat 1: Recon & Review · 看得广): 负责全局架构分析、MuJoCo 仿真与环境建模、长程知识图谱沉淀 (`~/.agent-memory/knowledge-graph.jsonl`)、端到端全链路终验。
   - **Claude Code** (Seat 2: Lead & Arch · 想得深): 负责需求纵深拆解、Invariants 质量与硬件安全裁决、并发状态机死锁检测、签发 `implementation_plan.md`。
   - **Codex** (Seat 3: Coder · 改得准): 负责高密度代码落地、TDD 单元测试套件开发、协议编解码与驱动实现。
   - **协作拓扑双模态 (Topology Modes)**:
     - **Hierarchical (分层流水线, 默认)**: Phase 1~4 全链路严谨递进，适合大型重构、系统级演进与跨模块交付；
     - **Pair/Mesh (结对双工攻坚)**: 面向算法边界突破与高并发死锁，Claude Code (Navigator 领航员 · 把关 Invariants 与逻辑) 与 Codex (Driver 操作手 · 极速 TDD) 点对点双工直连互审，免去全套文档流转，直达终验。
2. **对抗性验证门禁体系 (Adversarial Verification Tier)**:
   - **Critic (代码规范审查员)**: 审查 PEP 8 / Google 规范、接口契约与代码内聚度。
   - **Challenger (对抗性压力测试员)**: 在 `agents/challenger.md` 定义，针对系统边界构造极端输入、最差复杂度、丢帧与超时故障，跑红用例方为及格。
   - **Auditor (真实性审计员)**: 在 `agents/auditor.md` 定义，审查终端命令输出，彻底拦截空断言、测试打桩 (Anti-Mock) 与跳过测试 (`skip`)。
   - **Success Auditor (终验守门员)**: 跨模块全量构建与 E2E 双模回环验证全绿放行。
3. **排他性文件领空原则与租约看门狗 (Exclusive File Ownership & Lease Locks)**:
   - 子任务原子认领必须绑定独占文件与时间戳租约：`- [LOCKED: Codex @ HH:MM (TTL: 30m)] [FILES: src/.../leap_driver.py] <任务描述>`；
   - 同一时刻一个文件严禁被两个 Agent 同时修改，杜绝 Git 冲突与脏写；
   - 超期未刷新租约（超过 TTL 或默认 2 小时无更新）视为死锁陈旧锁，接班 Agent 或人类可强制标记 `[RELEASED]` 接管释放。
4. **长程记忆分治与认知反思闭环 (Memory Namespaces & Reflect)**:
   - `invariants` (永久固化): 架构与硬件核心约束、通信协议，永久沉淀至 `~/.agent-memory/knowledge-graph.jsonl`；
   - `patterns` (经验升华): 踩坑反思法则（`{Trigger → Gotcha → Crystallized Invariant}`），有预算召回（$\le 800$ tokens）；
   - `collaboration` (协作备忘): 黑板中的阶段任务状态与跨 Agent 方案草案；
   - `ephemeral` (暂态握手): 跨 Agent 临时握手信号与中间缓存，赋予 TTL（单轮任务结束后自动清理，绝不污染长期图谱）。
5. **PreCompact 抗失忆钩子与现场快照**:
   - 联动 Claude Code 的 `PreCompact` 钩子（工具 `collab-precompact`），在长会话压缩前夕将分支、脏文件与快照自动持久化写入黑板；
   - 通过 `collab-tmux` 一键分配后台工位分屏，支持现场快照与状态恢复。
6. **项目就绪度自检 (`agentctl audit`)**:
   - 支持通过 `agentctl audit .` 快速审计项目规则规范、黑板状态机、任务死锁锁态与防爆安全带，输出 0~100 综合就绪度评分。

---

## 8. 命令输出防爆安全带 (Output Explosion Safeguards)
为防止全量测试或构建的长日志淹没上下文或触发截断，所有命令执行严格遵守防爆安全带与管道截断规则：
- **测试与验证输出截断**:
  - 全量 pytest 执行必须使用管道截断或关键行过滤：
    `pytest tests/ -v | head -n 30` 或 `pytest tests/ -v | grep -E "FAIL|ERROR|Summary|passed"`
  - 宿主机构建命令长输出截断：
    `colcon build --symlink-install | tail -n 30`
- **Shell 脚本严谨性**:
  - 所有 Bash 脚本顶部强制声明 `set -euo pipefail`，防止管道静默故障；
  - 变量强制包裹双引号 `"$VAR"`，函数局部变量必须显式声明为 `local`。
- **计算下沉与代码化思考 (Think in Code)**:
  - 严禁使用原生读取工具硬灌数十万字节长日志或原始数据；优先编写本地微脚本（Python/Bash）过滤后将提炼统计传回上下文。


