# Session State & Multi-Agent Blackboard

> 项目多 Agent 协同接力黑板：用于 Antigravity、Claude Code、Codex 跨会话与跨工具协作。
> 放置在项目根目录 `.agent/SESSION_STATE.md`。

## 1. 当前任务目标 (Active Goal)
- **任务主题**: TuinaDex v2.0 引入 MuJoCo 仿真模块 (`tuinadex_sim`) 与真机/仿真统一控制架构
- **关联需求 / Issue**: `docs/superpowers/plans/2026-09-21-tuinadex-sim-phase3.md` & `docs/specs/ENVIRONMENT_MANAGEMENT_SPEC.md`
- **发起时间**: 2026-09-21 (2026-10-28 协作协议升级)
- **工程完整性模式 (Integrity Mode)**: `development` (允许成熟框架与标准库，严禁测试伪造打桩与空壳，严格硬件安全红线)
- **协同迭代轮次**: 1 / 5 (熔断阈值: 5 轮，超限将挂起并升级至人类裁决)
- **核心原则**:
  1. 真机与仿真 100% 共享 ROS 2 通信接口 (`/hand/joint_command` & `/hand/joint_states`)；
  2. 统一切换架构：支持 `ros2 launch tuinadex_hand hand.launch.py use_sim:=true/false` 一键无缝切换；
  3. 动力学保真：500Hz 物理循环 (dt=0.002s) + 50Hz ROS2 降采样，集成推拿床与人体背部假人刚体接触模型。

## 2. 流水线阶段与工单状态机 (Pipeline & Task Locks)
### 阶段分工与推荐工具
- [x] **Phase 1: 广域侦察与资产规范** (Seat 1: Antigravity 主导 · 看得广 | 专属 Skills: `diagram-design`, `research`, `domain-modeling`, `documentation-and-adrs`, `verification-before-completion`)
  - 完成 MuJoCo 3.13 宿主环境验证、8 组 STL 网格规范化、16-DOF Canonical URDF 与推拿床/假人 URDF 建模、MuJoCo MJCF 动力学世界构建、MujocoSimBridge 纯逻辑开发与单元测试 5/5 全绿、全工程 4 功能包 colcon 编译通过、知识图谱沉淀、出版级双模系统架构图交付。
- [x] **Phase 2: 深度规划与质量裁决** (Seat 2: Claude Code 主导 · 想得深 | 专属 Skills: `writing-plans`, `constraint-driven-development`, `subagent-driven-development`, `two-axis-review`, `robotics-hardware-safety`)
  - 审视 sim/real 统一切换 launch 架构、看门狗在仿真下的超时保护机制、动力学接触阻尼参数、编写详细 implementation_plan.md。三项架构裁决拍板并固化为 Invariant 16/17/18。
- [x] **Phase 3: 手术级执行与交叉审查** (Seat 3: Codex 主导执行 · 改得准 + Claude Code 监督 | 专属 Skills: `test-driven-development`, `incremental-implementation`, `code-simplification`, `systematic-debugging`, `doubt-driven-development`)
  - 极速落地 `leap_hand_sim_node` 完整控制循环、`tuinadex_core` 下沉、`hand.launch.py` 统一切换、ROS2 仿真话题集成测试与端到端闭环验证。
- [x] **Phase 4: 全局复核与对抗门禁** (Seat 1: Antigravity 主导 · Critic 规范评审 / Challenger 压力反例 / Auditor 真实性审计 | 专属 Skills: `verification-before-completion`, `documentation-and-adrs`, `diagram-design`)
  - 仿真/真机双模式全回归构建、多矩阵测试覆盖 (47 passed + 6 skipped / 4 passed)、更新项目 README 与架构文档闭环、ADR 0002 沉淀、长程知识图谱同步。

### 细分工单原子锁与排他文件领空 (Atomic Task Checkout & Exclusive File Ownership)
> 规则：任务原子认领必须绑定独占文件领空 `[FILES: ...]`, 任一文件同一时刻严禁被两个 Agent 同时修改。
- `[COMPLETED] [FILES: src/tuinadex_sim/]` <子任务 1: MuJoCo 动力学仿真功能包与数字孪生场景构建> (Antigravity 已归档)
- `[COMPLETED] [FILES: docs/superpowers/plans/]` <子任务 2: 统一架构规划、架构裁决与 Phase 3 方案设计> (Claude Code 已归档)
- `[COMPLETED] [FILES: src/tuinadex_core/, src/tuinadex_sim/tuinadex_sim/leap_hand_sim_node.py]` <子任务 3: 核心逻辑下沉与 50Hz 仿真节点落地> (Codex 已归档)
- `[COMPLETED] [FILES: tests/, scripts/verify_all_envs.sh, scripts/activate_tuinadex.sh]` <子任务 4: 多端异构环境管理与双基座收敛门禁> (Antigravity 已归档)
- `[PENDING] [FILES: src/tuinadex_hand/]` <子任务 5: LEAP Hand 实体真机接驳联调与物理看门狗测试> (待认领)

**当前进度**: Phase 1 ~ Phase 4 完整协同接力流水线圆满收敛。5 个功能包 colcon build 编译全过，多环境单元与集成测试矩阵全绿 (47 passed + 6 skipped / 5 passed + 2 skipped / 4 passed)，MuJoCo 仿真与 Mock 双模式端到端脚本 (`e2e_sim_check.sh` / `e2e_mock_check.sh`) 自动化验证通过，双分支 launch 验证无误，ADR 0002 与 README 已完备，长程图谱与接力黑板完成归档。多 Agent 协作协议与排他领空机制已全面同步。

## 3. 协作推进记录与实证 (Completed Work & Evidence)
| 时间 | 执行 Agent | 完成内容 | 验证依据 (测试命令 / 构建结果) | Git Commit |
| :--- | :--- | :--- | :--- | :--- |
| 2026-09-18 | Antigravity | 完成旧代码调研与 16-DOF 接口完全冻结，建立脚手架与规范，8项单元测试全绿 | 规范文档 `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md` | `d2a9767` |
| 2026-09-18 | Claude Code | Phase 2 深度审计 + P0 遥操崩溃修复 + 四项架构裁决 + Phase 3 实施计划落盘 | `docs/superpowers/plans/2026-09-18-tuinadex-mvp-phase3.md` | `77cb9ba` |
| 2026-09-18 | Codex | 完成 Task 0~8 极速 TDD 落地与三环境矩阵，端到端 mock 闭环 147 帧 PASS | `pytest tests/` 全绿；`bash scripts/e2e_mock_check.sh` PASS | `bc62a13` ~ `360a305` |
| 2026-09-18 | Antigravity | Phase 4 全局复核：全工程 `colcon build` 编译 3 包全过，沉淀 ADR 0001、README 与长程图谱 | `colcon build` 3包通过；测试矩阵全绿 | `71e92af`, `b17caa6` |
| 2026-09-21 | Antigravity | **Phase 1: 广域侦察与资产规范**: 建立 `src/tuinadex_sim/` 功能包骨架；迁移 8 组 STL 网格；生成规范 16-DOF Canonical URDF；建立推拿床与人体假人 URDF；生成 `tuina_scene.xml` 与 `leap_hand_right.xml` 动力学场景；开发 `MujocoSimBridge` 纯逻辑模块并通过 5/5 单元测试；通过 4 功能包 `colcon build`；生成出版级架构图 `docs/architecture/tuinadex_sim_digital_twin.html`；沉淀知识图谱 | `python3 -m pytest tests/test_sim_bridge.py -v` (5 passed); `colcon build --packages-select tuinadex_hand tuinadex_teleop tuinadex_recorder tuinadex_sim` (4 passed); `archify validate/deliver` (9/9 passed) | `73a55b8` |
| 2026-09-21 | Claude Code | **Phase 2: 深度规划与质量裁决**: 侦察 Phase 1 全部资产与 `tuinadex_hand` 可复用模块；识别 hand↔sim 循环依赖冲突并交用户拍板；三项架构裁决固化为 Invariant 16/17/18；落盘详细 Phase 3 实施计划 `docs/superpowers/plans/2026-09-21-tuinadex-sim-phase3.md`（4 Task：core 提取 / SimulationCore / 薄节点 / use_sim 切换+E2E） | 裁决经 AskUserQuestion 用户逐项拍板（3/3 采纳推荐）；计划文档落盘；黑板更新 | `a9a62b1` |
| 2026-09-21 | Codex | Phase 3 Task 1~4: 新建 `tuinadex_core` 单源化共享逻辑；实现 `SimulationCore` hold+ramp 回退与 500Hz 子步；实现 `LeapHandSimNode` 50Hz 单线程循环；增加 `use_sim` 统一入口和 `e2e_sim_check.sh`；将 MuJoCo 积分器固化为 `implicitfast` 修复模型失稳 | `pytest tests/` 45 passed + 6 skipped；ROS 集成 4 passed；5 包 `colcon build` 通过；`bash scripts/e2e_sim_check.sh` PASS（131 帧，median 19.98ms，收敛并回 OPEN） | `64d6cd7`, `ee30a7e`, `e7a9beb`, `bb8bcf0` |
| 2026-09-21 | Claude Code | **Phase 3 双轴交叉审查 + 修复合入**: 并行双 sub-agent 审查（Standards 轴: 0 硬违规 + 3 判断题 smell + 1 文档缺口；Spec 轴: 0 行为错误，8 项重点核验全过）。经用户拍板"修复后合入"，完成 3 处轻微修复（tick 单次取状态 / `physics_timestep` 接入 bridge + 2 条新测试 / AGENTS.md build 行补新包），`phase3-sim-execution` 已合入 master | 修复分支 `72e3473`；合并 `1a5eab8`；合并后 `pytest tests/` **47 passed + 6 skipped** | `72e3473`, `1a5eab8` |
| 2026-09-21 | Antigravity | **Phase 4: 全局复核与文档闭环**: 5 包全量 colcon build 回归通过；多环境单元与集成测试矩阵全绿（47 passed / 5 passed / 4 passed）；双模式端到端自动化脚本闭环验证（131 帧 sim / 146 帧 mock 全 PASS）；双模 launch 成功拉起验证；沉淀 ADR 0002、README.md、长程图谱实体与关系；全流水线闭环归档 | 5 包 colcon build 3.98s 通过；pytest tests/ 全绿；bash scripts/e2e_sim_check.sh PASS；bash scripts/e2e_mock_check.sh PASS；ros2 launch 双模式验证通过 | `bcb3a64` |
| 2026-10-14 | Antigravity | **多端异构环境管理架构落地**: 制定 `ENVIRONMENT_MANAGEMENT_SPEC`；交付 Udev 硬件规则、CycloneDDS 跨端基线 (ROS_DOMAIN_ID=42)、香橙派 CAN systemd 服务、数据清洗与云端 VLA Dockerfile 及 `verify_all_envs.sh` 全景自检工具；全流程自动化单测通过 | `pytest tests/test_env_management.py` 8 passed; `bash scripts/verify_all_envs.sh` 全检通过 | `a9db14b`, `5859520` |
| 2026-10-22 | Antigravity | **宿主机双基座环境收敛与准入门禁**: 废弃碎片环境，全面对齐 `tuinadex-control` 与 `tuinadex-vla` 双基座；交付一键激活脚本与导出锁定文件；建立环境依赖变更放行门禁 (Invariant 24) | `pytest tests/test_env_management.py` 9 passed; `bash scripts/verify_all_envs.sh` 全检通过; 双模 E2E PASS | `9ef320f` |
| 2026-10-29 | Antigravity | **多模态数据采集框架构建与 LeRobot v3.0 闭环**: 落地 `RobotConfig` 自适应 16D/22D 维度、`BaseTuinaEnv` 抽象、`ROS2RobotEnv` 话题桥接、`MuJoCoEnv` / `RealRobotEnv` 特化、`EpisodeBuffer` 试次缓冲与 `TuinaCollector` 状态机；交付 `LeRobotDatasetWriter` 直通 LeRobotDataset v3.0 视频分块 (MP4 + Parquet)；交付 `collect_teleop_data.py` 入口脚本与 `e2e_collection_check.sh` 自动化端到端回环 | 单元测试全绿 (9 passed / 2 passed / 6 passed); 5 包 colcon build 通过; `bash scripts/e2e_collection_check.sh` PASS (30 帧双目视频 + Parquet 验证通过) | `9288c30` |


## 4. 技术决策与认知反思沉淀 (Decisions & Reflections)
### 4.1 核心技术决策与不变量 (Invariants)
- **不变量 1 (坐标与单位)**: ROS2 与 LeRobot 全链路强制 SI 单位弧度 (rad)。以实测校准后的 `OPEN_POSE`（全开平伸）为 0 rad 基准零位，手指屈伸弯曲为正 ($q > 0$)。
- **不变量 2 (双域解耦)**: ROS2 负责机器人运行时（Runtime）；LeRobot 负责策略学习（Learning）。LeRobot 绝对禁止跨过 ROS2 直连硬件；ROS2 Topic 绝对不为 LeRobot Schema 强行妥协。
- **不变量 3 (数据源契约)**: `rosbag2` 为唯一实验原始事实源 (Source of Truth)；`LeRobotDataset` 为衍生训练数据产品。解耦通过离线 `tuinadex_dataset` 工具完成。
- **不变量 4 (语义分离)**: State = 物理实测位置 (`/hand/joint_states`)；Action = 目标控制位置 (`/hand/joint_command`)，两者严格区分。
- **不变量 5 (动作空间)**: Action 采用 16 维 absolute joint position，绝对禁止使用增量 delta。
- **不变量 6 (控制模式与周期)**: 手部节点采用 50Hz (20ms) 循环，电机模式锁定为成熟验证过的位置-电流混合模式 (Position-Current Mode = 5)，限流 150mA。
- **不变量 7 (节点生命周期)**: 普通 rclpy Node + 显式 `WatchdogStateMachine` 纯类 FSM（FOLLOWING/RELAXING），不使用 LifecycleNode；安全逻辑必须抽成不依赖 rclpy 的纯模块。
- **不变量 8 (采集器实现)**: `tuinadex_recorder` 采用 rosbag2_py 进程内节点 + `std_srvs/Trigger` 起停服务，严禁 subprocess 包装 `ros2 bag record`。
- **不变量 9 (bag 反序列化)**: 依赖 ROS2 环境（本机安装 ROS2，Ubuntu 26.04 对应发行版），不写纯 Python CDR 解析器。
- **不变量 10 (测试环境矩阵, 2026-10-22 更新)**: 本机环境彻底收敛为双规范环境：日常控制基座 = `tuinadex-control` (`~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/ -v`，涵盖臂手驱动、MuJoCo 物理动力学仿真与 Co_Teleop 视觉遥操)；具身学习与模型训练 = `tuinadex-vla` (`~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_dataset_adapter.py -v`，涵盖 LeRobot 数据集转换与 SmolVLA/ACT 训练)；ROS2 依赖测试用 `pytest.importorskip("rclpy")` 守护。
- **不变量 23 (本机统一双基座环境收敛, 2026-10-22 追加)**: 废弃原先散落的 `arm_robot`、`leap_hand`、`smolvla` 临时环境，本机环境严格收敛为 `tuinadex-control` (Python 3.10.20，负责 90% 驱动控制仿真遥操) 与 `tuinadex-vla` (Python 3.10.20 + CUDA 12.8，负责数据录制与模型训练)，彻底避免跨项目污染。
- **不变量 11 (ROS2 运行环境)**: ROS2 发行版为 Lyrical Luth Desktop，默认使用宿主机 `/opt/ros/lyrical/setup.bash`。
- **不变量 12 (电机路径限位门)**: `/hand/joint_command` 必须先通过 `align_command()` 校验，并用 `JOINT_SPEC_LIMITS` 夹取到 Spec §1 逐关节限位；拒绝维度/非有限值/未知关节名，禁止越界绝对弧度进入驱动。
- **不变量 13 (仿真话题完全等价性, Phase 1 追加)**: 仿真主控 `leap_hand_sim_node` 对外必须严格使用 `/hand/joint_states` 与 `/hand/joint_command`，消息格式均为 `sensor_msgs/JointState`，16 维 Canonical Joint 命名与限位完全对齐，上层遥操、采集与学习算法零感知切换。
- **不变量 14 (多速率时钟解耦, Phase 1 追加)**: MuJoCo 内部物理动力学强制以 500Hz (`dt = 0.002s`) 步进以保证刚体接触与数值积分稳定；ROS 2 通信发布端必须通过定时器严格降采样至 50Hz (`20ms`)，与真实硬件时钟频率完全一致。
- **不变量 15 (双模看门狗一致性, Phase 1 追加)**: 仿真节点与真机节点同样受 300ms Watchdog 保护。当上层手势或指令断联超过 300ms，仿真执行器必须自动平滑回退至全开姿态 (`Relax-to-OPEN`，各关节目标位 0 rad)。
- **不变量 16 (仿真单线程时序, Phase 2 追加)**: 仿真节点采用**单线程 50Hz ROS2 Timer**，每 tick 内锁步进 `mj_step` 共 10 次 (`10 × 0.002s = 20ms`)，即严格 500Hz 物理积分、50Hz 采样点发布。禁止后台物理线程（零竞态、确定性、与真机 50Hz 逐点对齐）。
- **不变量 17 (共享纯逻辑包 + 依赖 DAG, Phase 2 追加)**: 纯逻辑 `watchdog.py` / `command_processing.py` / `JOINT_SPEC_LIMITS` / `CANONICAL_JOINT_NAMES` 下沉到新低层包 `tuinadex_core`（`CANONICAL_JOINT_NAMES` 在此单源化，删除 `sim_bridge.py` 与 `driver.leap_driver.py` 的复制）。`tuinadex_hand` 与 `tuinadex_sim` 均依赖 `tuinadex_core`。依赖收敛为无环 DAG：`hand→sim→core`、`hand→core`，**禁止 hand↔sim 循环依赖**。
- **不变量 18 (仿真回退保真, Phase 2 追加)**: 仿真端 Relax-to-OPEN 逐点复刻真机 `driver.relax_to_open_step`（`leap_driver.py:224-246`）：hold 期 (0.3s) 保持当前实测位姿 → ramp 期 (0.6s) 线性收敛 0 rad，基于实测关节位置、带 `_loss_t0`/`_relax_from_pos` 状态并由 `reset_loss_state()` 清挡。禁止瞬时跳变。
- **不变量 19 (跨端 CycloneDDS 基线, 2026-10-14 追加)**: 跨设备通信（本地 PC 26.04 Lyrical 与香橙派 22.04 Humble）全网强制锁定 `Eclipse CycloneDDS` (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)，统一 `ROS_DOMAIN_ID=42`，网络仅允许传输 ROS 2 标准消息类型 (`sensor_msgs`, `geometry_msgs`, `std_msgs`)，严禁跨版本传输非标自定义消息。
- **不变量 20 (三端异构职责自治, 2026-10-14 追加)**: 架构严格分为三端：云端 GPU (VLA 推理推流 + 策略训练)；香橙派 AI Pro 20T (30fps NPU 穴位感知 + SocketCAN 臂/USB 手控制中枢 + 300ms 本地看门狗)；本地 PC (专家示教采集 + MuJoCo 仿真 + 自动化测试)。
- **不变量 21 (Action Chunking 动作推流与超时熔断, 2026-10-14 追加)**: 云端至香橙派采用 gRPC/Zenoh Action Chunking 协议，香橙派内置 100Hz 局部样条插值。若新动作 Chunk 超过 300ms 未达，边缘端就地触发熔断，阻尼平滑回退至全开姿态 (`Relax-to-OPEN`)。
- **不变量 22 (AI 数据转换环境容器隔离, 2026-10-14 追加)**: `rosbag2 -> LeRobotDataset` 的离线转换与云端训练运行在专属 Docker 容器中，避免本地 Ubuntu 26.04 (Python 3.14) 对 PyTorch/LeRobot 依赖生态的破坏。
- **不变量 24 (环境依赖准入与变更门禁规则, 2026-10-22 追加)**: 任何涉及项目运行环境、Conda 环境、Docker 容器、系统依赖、pip 包或通信协议配置的改动，必须严格遵循《多端异构环境管理架构设计规范》(`docs/specs/ENVIRONMENT_MANAGEMENT_SPEC.md`)。改动前须核验环境归属唯一性与防污染原则，改动后必须运行 `bash scripts/verify_all_envs.sh` 全检通过并同步更新导出的 Conda 锁定文件（`scripts/env/*.yml`），方可放行与合并。

### 4.2 结构化认知反思 (Reflections · 经验升华)
| 触发场景 (Trigger) | 踩坑诱因 (Gotcha) | 升华法则 (Crystallized Invariant) |
| :--- | :--- | :--- |
| MuJoCo 动力学仿真失稳发散 | 默认 Euler 积分器在刚体接触碰撞时高频数值震荡 | 物理仿真强制锁定 `implicitfast` 隐式快速积分器 (Invariant 16) |
| hand 与 sim 相互 import 环路 | 驱动和仿真均需复用限位与命令处理逻辑 | 提取纯逻辑下沉包 `tuinadex_core`，消除循环依赖收敛为 DAG (Invariant 17) |
| 仿真端断联回退发生动力学跳变 | 仿真节点单步直接重置为 0 rad 造成机构冲击 | 仿真 Relax-to-OPEN 逐点复刻真机 hold (0.3s) + ramp (0.6s) 平滑回退 (Invariant 18) |
| 宿主机多版本 Python/Conda 冲突 | Ubuntu 26.04 (Python 3.14) 破坏 PyTorch/LeRobot 生态 | 废弃散落环境，收敛为 `tuinadex-control` 与 `tuinadex-vla` 双基座 (Invariant 23/24) |
| 跨端 ROS2 通信数据丢包与错位 | 本地 PC 26.04 Lyrical 与香橙派 22.04 Humble 混用非标消息 | 全网锁定 Eclipse CycloneDDS，统一 ROS_DOMAIN_ID=42，仅传标准消息 (Invariant 19) |

## 5. 接力指令 (Handoff to Next Agent)
- **状态**: **多端异构环境管理架构规范冻结 + 双基座收敛门禁已落地 + 多 Agent 协作协议全面升级**
- **接手 Agent 启动动作**:
  1. 运行环境准入门禁：`bash scripts/verify_all_envs.sh`
  2. 运行双基座单测：`source scripts/activate_tuinadex.sh control && pytest tests/ -v`
- **未决疑问与风险提示**:
  - 物理舵机真机连接（`use_sim:=false mock:=false`）需在接驳真实物理供电与串口、且有急停预案的前提下由人工审核后执行。
  - 机械臂需确保在 70% 臂展内安全运动。

## 6. 本轮协作闭环总结汇报 (Collaboration Debriefing)
- **任务目标与完成度**: 100% 完成，LEAP Hand 16-DOF 仿真与控制闭环、双基座环境收敛与多 Agent 协议升级全部交付。
- **各 Agent 贡献清单**:
  - `Antigravity`: 完成 Phase 1 广域侦察、URDF/MJCF 动力学建模、双基座环境管理架构 (SPEC-ENV) 与 Phase 4 全景回归验证。
  - `Claude Code`: 完成 Phase 2 深度规划与质量裁决 (Invariant 16/17/18)、双轴交叉审查 (`two-axis-review`)。
  - `Codex`: 完成 Phase 3 手术级执行与 TDD 测试套件落地，交付 `tuinadex_core` 与 `leap_hand_sim_node`。
- **验证实证**:
  - 5 个功能包 `colcon build` 编译全过；
  - 多环境单元与集成测试矩阵全绿 (`tuinadex-control` 47 passed + 6 skipped / `tuinadex-vla` 4 passed)；
  - 自动化环境检验 `bash scripts/verify_all_envs.sh` 100% [PASS]；
  - 双模式端到端自动化脚本闭环验证 (`e2e_sim_check.sh` / `e2e_mock_check.sh`) 全绿 PASS。
- **后续建议 / 未决项**:
  - 下一里程碑推荐：推进物理 LEAP Hand 真机接驳联调（带急停）或推进 6-DOF 机械臂与视觉感知节点集成。




<!-- PRECOMPACT_SNAPSHOT_START -->
### 🔄 PreCompact 压缩前夕现场快照 (2026-10-28 15:21:47)
> [!NOTE]
> 本区块由 PreCompact 钩子自动生成。会话压缩后，Agent 应首先依据此快照对齐工作现场。
- **分支状态**: `master` | **最近提交**: `9b2c7e0 feat(gate): establish environment change gate and invariant 24`
- **工作区脏文件 (Top 15)**:
```text

```
<!-- PRECOMPACT_SNAPSHOT_END -->
