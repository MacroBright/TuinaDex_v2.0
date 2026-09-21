# Session State & Multi-Agent Blackboard

> 项目多 Agent 协同接力黑板：用于 Antigravity、Claude Code、Codex 跨会话与跨工具协作。
> 放置在项目根目录 `.agent/SESSION_STATE.md`。

## 1. 当前任务目标 (Active Goal)
- **任务主题**: TuinaDex v2.0 引入 MuJoCo 仿真模块 (`tuinadex_sim`) 与真机/仿真统一控制架构
- **核心原则**:
  1. 真机与仿真 100% 共享 ROS 2 通信接口 (`/hand/joint_command` & `/hand/joint_states`)；
  2. 统一切换架构：支持 `ros2 launch tuinadex_hand hand.launch.py use_sim:=true/false` 一键无缝切换；
  3. 动力学保真：500Hz 物理循环 (dt=0.002s) + 50Hz ROS2 降采样，集成推拿床与人体背部假人刚体接触模型。
- **发起时间**: 2026-09-21

## 2. 流水线阶段与 Agent 职责分工 (4-Phase Collaboration Pipeline)
- [x] **Phase 1: 广域侦察与资产规范** (Antigravity 主导 · 看得广 - 完成 MuJoCo 3.13 宿主环境验证、8 组 STL 网格规范化、16-DOF Canonical URDF 与推拿床/假人 URDF 建模、MuJoCo MJCF 动力学世界构建、MujocoSimBridge 纯逻辑开发与单元测试 5/5 全绿、全工程 4 功能包 colcon 编译通过、知识图谱沉淀、出版级双模系统架构图交付)
- [ ] **Phase 2: 深度规划与质量裁决** (Claude Code 主导 · 想得深 - 审视 sim/real 统一切换 launch 架构、看门狗在仿真下的超时保护机制、动力学接触阻尼参数、编写详细 implementation_plan.md)
- [ ] **Phase 3: 手术级执行与交叉审查** (Codex 主导执行 · 改得准 + Claude Code 监督 - 极速落地 `leap_hand_sim_node` 完整控制循环、`hand.launch.py` 统一切换、ROS2 仿真话题集成测试与端到端闭环验证)
- [ ] **Phase 4: 全局复核与文档闭环** (Antigravity 主导 · 看得广 - 仿真/真机双模式全回归构建、多矩阵测试覆盖、更新项目 README 与架构文档闭环)

**当前进度**: Phase 1 顺利收官，已建立 `src/tuinadex_sim` 功能包骨架与物理资产，准备交接 Phase 2！

## 3. 协作推进记录与实证 (Completed Work & Evidence)
| 时间 | 执行 Agent | 完成内容 | 验证依据 (测试命令 / 构建结果) | Git Commit |
| :--- | :--- | :--- | :--- | :--- |
| 2026-09-18 | Antigravity | 完成旧代码调研与 16-DOF 接口完全冻结，建立脚手架与规范，8项单元测试全绿 | 规范文档 `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md` | `d2a9767` |
| 2026-09-18 | Claude Code | Phase 2 深度审计 + P0 遥操崩溃修复 + 四项架构裁决 + Phase 3 实施计划落盘 | `docs/superpowers/plans/2026-09-18-tuinadex-mvp-phase3.md` | `77cb9ba` |
| 2026-09-18 | Codex | 完成 Task 0~8 极速 TDD 落地与三环境矩阵，端到端 mock 闭环 147 帧 PASS | `pytest tests/` 全绿；`bash scripts/e2e_mock_check.sh` PASS | `bc62a13` ~ `360a305` |
| 2026-09-18 | Antigravity | Phase 4 全局复核：全工程 `colcon build` 编译 3 包全过，沉淀 ADR 0001、README 与长程图谱 | `colcon build` 3包通过；测试矩阵全绿 | `71e92af`, `b17caa6` |
| 2026-09-21 | Antigravity | **Phase 1: 广域侦察与资产规范**: 建立 `src/tuinadex_sim/` 功能包骨架；迁移 8 组 STL 网格；生成规范 16-DOF Canonical URDF；建立推拿床与人体假人 URDF；生成 `tuina_scene.xml` 与 `leap_hand_right.xml` 动力学场景；开发 `MujocoSimBridge` 纯逻辑模块并通过 5/5 单元测试；通过 4 功能包 `colcon build`；生成出版级架构图 `docs/architecture/tuinadex_sim_digital_twin.html`；沉淀知识图谱 | `python3 -m pytest tests/test_sim_bridge.py -v` (5 passed); `colcon build --packages-select tuinadex_hand tuinadex_teleop tuinadex_recorder tuinadex_sim` (4 passed); `archify validate/deliver` (9/9 passed) | `73a55b8` |

## 4. 核心技术决策与不变量 (Decisions & Invariants)
- **不变量 1 (坐标与单位)**: ROS2 与 LeRobot 全链路强制 SI 单位弧度 (rad)。以实测校准后的 `OPEN_POSE`（全开平伸）为 0 rad 基准零位，手指屈伸弯曲为正 ($q > 0$)。
- **不变量 2 (双域解耦)**: ROS2 负责机器人运行时（Runtime）；LeRobot 负责策略学习（Learning）。LeRobot 绝对禁止跨过 ROS2 直连硬件；ROS2 Topic 绝对不为 LeRobot Schema 强行妥协。
- **不变量 3 (数据源契约)**: `rosbag2` 为唯一实验原始事实源 (Source of Truth)；`LeRobotDataset` 为衍生训练数据产品。解耦通过离线 `tuinadex_dataset` 工具完成。
- **不变量 4 (语义分离)**: State = 物理实测位置 (`/hand/joint_states`)；Action = 目标控制位置 (`/hand/joint_command`)，两者严格区分。
- **不变量 5 (动作空间)**: Action 采用 16 维 absolute joint position，绝对禁止使用增量 delta。
- **不变量 6 (控制模式与周期)**: 手部节点采用 50Hz (20ms) 循环，电机模式锁定为成熟验证过的位置-电流混合模式 (Position-Current Mode = 5)，限流 150mA。
- **不变量 7 (节点生命周期)**: 普通 rclpy Node + 显式 `WatchdogStateMachine` 纯类 FSM（FOLLOWING/RELAXING），不使用 LifecycleNode；安全逻辑必须抽成不依赖 rclpy 的纯模块。
- **不变量 8 (采集器实现)**: `tuinadex_recorder` 采用 rosbag2_py 进程内节点 + `std_srvs/Trigger` 起停服务，严禁 subprocess 包装 `ros2 bag record`。
- **不变量 9 (bag 反序列化)**: 依赖 ROS2 环境（本机安装 ROS2，Ubuntu 26.04 对应发行版），不写纯 Python CDR 解析器。
- **不变量 10 (测试环境矩阵)**: 本地单元测试 = `leap_hand` conda env (`~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`)；LeRobot 适配器测试 = `smolvla` env (lerobot 0.4.4)；ROS2 依赖测试用 `pytest.importorskip("rclpy")` 守护。
- **不变量 11 (ROS2 运行环境)**: ROS2 发行版为 Lyrical Luth Desktop，默认使用宿主机 `/opt/ros/lyrical/setup.bash`。
- **不变量 12 (电机路径限位门)**: `/hand/joint_command` 必须先通过 `align_command()` 校验，并用 `JOINT_SPEC_LIMITS` 夹取到 Spec §1 逐关节限位；拒绝维度/非有限值/未知关节名，禁止越界绝对弧度进入驱动。
- **不变量 13 (仿真话题完全等价性, Phase 1 追加)**: 仿真主控 `leap_hand_sim_node` 对外必须严格使用 `/hand/joint_states` 与 `/hand/joint_command`，消息格式均为 `sensor_msgs/JointState`，16 维 Canonical Joint 命名与限位完全对齐，上层遥操、采集与学习算法零感知切换。
- **不变量 14 (多速率时钟解耦, Phase 1 追加)**: MuJoCo 内部物理动力学强制以 500Hz (`dt = 0.002s`) 步进以保证刚体接触与数值积分稳定；ROS 2 通信发布端必须通过定时器严格降采样至 50Hz (`20ms`)，与真实硬件时钟频率完全一致。
- **不变量 15 (双模看门狗一致性, Phase 1 追加)**: 仿真节点与真机节点同样受 300ms Watchdog 保护。当上层手势或指令断联超过 300ms，仿真执行器必须自动平滑回退至全开姿态 (`Relax-to-OPEN`，各关节目标位 0 rad)。

## 5. 接力指令 (Handoff to Next Agent)
- **移交对象**: **Claude Code** (`想得深 · 深度规划与质量裁决`)
- **下一阶段**: **Phase 2: 深度规划与质量裁决**
- **建议主责行动清单**:
  1. **架构与启动流裁决**: 审视 `hand.launch.py`，设计支持 `use_sim:=true/false` 的条件分支拉起逻辑；
  2. **仿真节点逻辑拆解**: 规划 `leap_hand_sim_node.py` 的实现方案，确定其如何复用 `tuinadex_hand.watchdog` 与 `tuinadex_hand.command_processing`；
  3. **时钟与线程安全设计**: 明确 500Hz MuJoCo 步进与 50Hz ROS2 定时发布在 rclpy 节点中的线程模型（Timer vs Background Thread）；
  4. **产出 Phase 3 实施规划文档**: 拆解为可独立 TDD 验证的 Task 清单（例如 Task 1: 仿真节点看门狗与指令对齐接入；Task 2: launch 条件切换；Task 3: ROS 2 仿真回环单测与端到端回环），写入 `docs/superpowers/plans/` 并交接给 Codex 执行。
