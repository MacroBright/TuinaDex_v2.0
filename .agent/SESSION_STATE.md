# Session State & Multi-Agent Blackboard

> 项目多 Agent 协同接力黑板：用于 Antigravity、Claude Code、Codex 跨会话与跨工具协作。
> 放置在项目根目录 `.agent/SESSION_STATE.md`。

## 1. 当前任务目标 (Active Goal)
- **任务主题**: TuinaDex v2.0 框架重构 — 建立 LEAP Hand MVP 闭环体系 (ROS2 Runtime + Teleop + Data Collection + LeRobot Adapter)
- **关联需求 / 规范文档**: `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`
- **发起时间**: 2026-09-18

## 2. 流水线阶段与 Agent 职责分工 (4-Phase Collaboration Pipeline)
- [x] **Phase 1: 广域侦察与影响分析** (Antigravity 主导 · 看得广 - 完成旧工程源码研读、架构解耦分析、16-DOF 接口契约冻结、脚手架与基础驱动搭建)
- [x] **Phase 2: 深度规划与质量裁决** (Claude Code 主导 · 想得深 - 全量代码审计、P0 缺陷修复、四项架构裁决落盘、Phase 3 实施计划完成)
- [x] **Phase 3: 手术级执行与交叉审查** (Codex 主导执行 · 改得准 + Claude Code 监督 - 按计划逐任务 TDD 落地 Task 0~8，双轴审查修正全部闭环)
- [x] **Phase 4: 全局复核与文档闭环** (Antigravity 主导 · 看得广 - 全工程 colcon 构建、跨环境依赖解耦修复、三矩阵测试全绿、E2E mock 回环验证、ADR 与项目 README 归档)

**当前进度**: Phase 1~4 全流程顺利闭环！MVP 闭环系统与三环境矩阵 100% 验证通过。

## 3. 协作推进记录与实证 (Completed Work & Evidence)
| 时间 | 执行 Agent | 完成内容 | 验证依据 (测试命令 / 构建结果) | Git Commit |
| :--- | :--- | :--- | :--- | :--- |
| 2026-09-18 | Antigravity | 完成旧代码调研与 16-DOF 接口完全冻结，建立脚手架与规范，8项单元测试全绿 | 规范文档 `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md` | `d2a9767` |
| 2026-09-18 | Claude Code | Phase 2 深度审计 + P0 遥操崩溃修复（`mapper.map()` → `map_keypoints_to_leap()`，新增 API 守卫测试）+ 四项架构裁决 + Phase 3 实施计划落盘 | 计划 `docs/superpowers/plans/2026-09-18-tuinadex-mvp-phase3.md`；`pytest tests/` 9/9 全绿（`leap_hand` conda env） | `77cb9ba` |
| 2026-09-18 | Claude Code | Phase 3 交接前双轴交叉审查（`two-axis-review`）→ 按用户指令 retrofit 4 处缺陷进规划（Task 2 电机关节空间限位门 / Task 6 订阅参数顺序 / Task 3 ghost 表达式 / Task 7 lerobot 整集批量 + 去 timestamp 键） | 审查报告双轴分离；计划含「交叉审查 retrofit 记录」；Codex 落地即干净首版 | `8b74e04` |
| 2026-09-18 | Codex | Task 1: WatchdogStateMachine 纯类 FSM，TDD 红绿通过 | `pytest tests/test_watchdog_fsm.py -v` 5/5；全量 14/14 | `26cde3e` |
| 2026-09-18 | Codex | Task 2: /hand/joint_command 严格校验与 Canonical 重排，TDD 红绿通过 | `pytest tests/test_command_processing.py -v` 7/7；全量 21/21 | `e5c24af` |
| 2026-09-18 | Codex | Task 3: 手节点接入 FSM/节点时钟/安全关停；ROS2 集成测试待 Task 0 | 本机全量 21 passed + 1 skipped | `91196d1` |
| 2026-09-18 | Codex | Task 4: 遥操利手过滤、重捕获滤波 reset、安全退出与资源释放 | 本机全量 24 passed + 1 skipped | `34b5a3a` |
| 2026-09-18 | Codex | Task 5: 数据集校验器切换到规范逐关节限位 | `pytest tests/test_dataset_sync_validator.py -v` 4/4 | `2efeac2` |
| 2026-09-18 | Codex | Task 6: tuinadex_recorder 进程内 rosbag2_py 采集包骨架与实现 | 本机全量 26 passed + 2 skipped；bag 落盘测试待 Task 0 | `3272b0b` |
| 2026-09-18 | Codex | Task 7: rosbag2 → LeRobotDataset 离线适配器与 CLI；实测 LeRobot 0.4.4 使用 `lerobot.datasets` + `add_frame/save_episode` 而非计划中的 `add_episode` | 本机全量 26 passed + 3 skipped；全链测试待 Task 0 | `063b0c2` |
| 2026-09-18 | Codex | Task 0: 按用户指定教程安装 ROS2 Lyrical Luth Desktop；安装官方 `ros2-apt-source 1.3.0~resolute`、`ros-lyrical-desktop 0.13.0`、`ros-dev-tools 1.0.3`，并固化 `~/.bashrc` source | `ros2`、`rclpy`、`rosbag2_py`、`colcon` 验证通过；talker/listener 实际收到 4 条消息 | `bc62a13` |
| 2026-09-18 | Codex | ROS2 Lyrical API 兼容修复：`logger.warning`、`TopicMetadata(id=...)`、`create_subscription(..., raw=True)` | 容器内 `test_leap_hand_node_integration.py` 与 `test_recorder_node.py` 2/2 全绿 | `23720c8` |
| 2026-09-18 | Codex | Task 8: 端到端 mock 闭环；新增 mock 指令发布器与 E2E 脚本，修复 `tuinadex_hand` 缺失 `__init__.py`，适配 LeRobot 0.4.4 `add_frame` 与 v3.0 数据文件布局 | `bash scripts/e2e_mock_check.sh` 输出 PASS；mock bag 转换 147 帧，LeRobot parquet 生成 | `3139e75` |
| 2026-09-18 | Codex | 接手 Claude 双轴审查：Task 6 订阅 API、Task 3 参数注入、Task 7 LeRobot API 均已在实现层落地；新增 Task 2 `JOINT_SPEC_LIMITS` + `np.clip` 电机路径限位门，并同步修正越界测试输入 | `pytest tests/test_command_processing.py -v` 9/9；全量 28 passed + 3 skipped；ROS 集成 2/2；E2E 146 帧 PASS | `360a305` |
| 2026-09-18 | Antigravity | Phase 4 全局复核：全工程 `colcon build` 编译 3 包全过；修复跨环境（Python 3.10/3.14，MediaPipe/ROS2/LeRobot）解耦与 synthetic_bag 逐关节限位；全矩阵测试全绿；E2E Mock 回环通过；沉淀 ADR 0001、README 与长程图谱 | `colcon build` 3包通过；`leap_hand` env 28 passed；`smolvla` env 5 passed；ROS2 host 4 passed；`bash scripts/e2e_mock_check.sh` 147 帧 PASS | `71e92af`, `b17caa6`

## 4. 核心技术决策与不变量 (Decisions & Invariants)
- **不变量 1 (坐标与单位)**: ROS2 与 LeRobot 全链路强制 SI 单位弧度 (rad)。以实测校准后的 `OPEN_POSE`（全开平伸）为 0 rad 基准零位，手指屈伸弯曲为正 ($q > 0$)。
- **不变量 2 (双域解耦)**: ROS2 负责机器人运行时（Runtime）；LeRobot 负责策略学习（Learning）。LeRobot 绝对禁止跨过 ROS2 直连硬件；ROS2 Topic 绝对不为 LeRobot Schema 强行妥协。
- **不变量 3 (数据源契约)**: `rosbag2` 为唯一实验原始事实源 (Source of Truth)；`LeRobotDataset` 为衍生训练数据产品。解耦通过离线 `tuinadex_dataset` 工具完成。
- **不变量 4 (语义分离)**: State = 物理实测位置 (`/hand/joint_states`)；Action = 目标控制位置 (`/hand/joint_command`)，两者严格区分。
- **不变量 5 (动作空间)**: Action 采用 16 维 absolute joint position，绝对禁止使用增量 delta。
- **不变量 6 (控制模式与周期)**: 手部节点采用 50Hz (20ms) 循环，电机模式锁定为成熟验证过的位置-电流混合模式 (Position-Current Mode = 5)，限流 150mA。
- **不变量 7 (节点生命周期, Phase 2 用户拍板)**: 普通 rclpy Node + 显式 `WatchdogStateMachine` 纯类 FSM（FOLLOWING/RELAXING），不使用 LifecycleNode；安全逻辑必须抽成不依赖 rclpy 的纯模块。
- **不变量 8 (采集器实现, Phase 2 用户拍板)**: `tuinadex_recorder` 采用 rosbag2_py 进程内节点 + `std_srvs/Trigger` 起停服务，严禁 subprocess 包装 `ros2 bag record`。
- **不变量 9 (bag 反序列化, Phase 2 用户拍板)**: 依赖 ROS2 环境（本机安装 ROS2，Ubuntu 26.04 对应发行版），不写纯 Python CDR 解析器。Task 0 为系统级安装，执行前必须人工确认。
- **不变量 10 (测试环境矩阵)**: 本地单元测试 = `leap_hand` conda env (`~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`)；LeRobot 适配器测试 = `smolvla` env (lerobot 0.4.4)；ROS2 依赖测试用 `pytest.importorskip("rclpy")` 守护。
- **不变量 11 (ROS2 运行环境, Task 0 用户确认)**: ROS2 发行版为 Lyrical Luth Desktop，默认使用宿主机 `/opt/ros/lyrical/setup.bash`。不要混用不同发行版路径；`scripts/ros2_docker.sh` 仅作为隔离回退。
- **不变量 12 (电机路径限位门, Claude 双轴审查)**: `/hand/joint_command` 必须先通过 `align_command()` 校验，并用 `JOINT_SPEC_LIMITS` 夹取到 Spec §1 逐关节限位；拒绝维度/非有限值/未知关节名，禁止越界绝对弧度进入驱动。

## 5. 接力指令 (Handoff to Next Agent)
- **阶段状态**: 全部 Phase 1~4 协同已圆满完成闭环。
- **后续动作**: 本轮多 Agent 任务收尾归档。下一步可由用户安排真机联调（插上 `/dev/ttyUSB0` 与上电）或进一步扩展机械臂 SocketCAN 节点。

## 6. 本轮协作闭环总结汇报 (Collaboration Debriefing)
- **本轮任务目标 (Mission Objective)**:
  TuinaDex v2.0 框架重构：彻底解决 v1.0 驱动耦合深、单位不一、遥操易崩溃、缺少看门狗保护、录制机制脆弱等技术债务；冻结 16-DOF LEAP Hand 接口契约与数据规范，建立从 ROS2 运行时、遥操作、示教数据采集、离线重采样校验到 LeRobot 格式转换的全链路 MVP 闭环体系。
- **完成度与实证 (Completion & Evidence)**:
  1. **全工程 ROS2 编译**: `colcon build --symlink-install` 成功编译 `tuinadex_hand`、`tuinadex_recorder`、`tuinadex_teleop` 3 个核心功能包，0 编译错误。
  2. **多环境测试矩阵全绿**:
     - `leap_hand` conda env (Python 3.14): `pytest tests/ -v` 输出 **28 passed, 5 skipped** (0 failures)；
     - `smolvla` conda env (Python 3.10 + LeRobot 0.4.4): `pytest tests/test_dataset_sync_validator.py tests/test_dataset_adapter.py -v` 输出 **5 passed, 2 skipped** (0 failures)；
     - ROS2 Host Lyrical env (Python 3.14.4 + rclpy): `pytest` 输出 **4 passed, 1 skipped** (0 failures)。
  3. **端到端 Mock 回环验证**: 执行 `bash scripts/e2e_mock_check.sh`，50Hz 控制节点 + 示教录制节点 + 模拟指令发布 + rosbag2 落盘 (.db3) + 147 帧 50Hz 对齐校验 + LeRobot Parquet 数据集落地，完整输出 `[E2E] PASS: mock closed loop completed`。
- **各 Agent 贡献明细 (Agent Contributions)**:
  - **Antigravity (Phase 1 & Phase 4)**:
    - Phase 1: 广域侦察与影响分析，梳理旧仓库架构，输出冻结的 16-DOF 接口与通信契约 (`docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`)，搭建基础脚手架；
    - Phase 4: 全局复核与文档闭环，排查跨环境依赖兼容缺陷并手术修复，沉淀 ADR 0001 与项目 README，长程知识图谱归档。
  - **Claude Code (Phase 2 & Phase 3 监督)**:
    - Phase 2: 全量代码审计，定位并修复 P0 级遥操崩溃 Bug (`mapper.map()` 修复)，组织并落盘四项核心架构裁决，产出结构化 Phase 3 实施规划；
    - 阶段交叉审查: 执行双轴审查 (`two-axis-review`)，识别电机限位门、参数注入、LeRobot 批量 API 等 4 处隐患并指导修复。
  - **Codex (Phase 3 执行)**:
    - 手术级 TDD 极速落地 Task 0~8：WatchdogStateMachine 纯类 FSM、指令校验重排、手节点安全时钟改造、遥操利手与重捕获重置、rosbag2_py 进程内采集器、LeRobot 适配器与 CLI、端到端 mock 自动化脚本与测试用例。
- **沉淀资产与不变量 (Assets & Invariants)**:
  - 沉淀 ADR 0001（4 项重大架构决策）；
  - 沉淀至共享长程图谱 (`~/.agent-memory/knowledge-graph.jsonl`)：`TuinaDex_Watchdog_FSM`、`TuinaDex_Recorder_Node`、`TuinaDex_Dataset_Adapter`、`ROS2_Lyrical_Luth_Desktop` 实体与拓扑关系；
  - 冻结 12 项架构不变量。
- **后续建议 / 未决项 (Next Steps & Open Issues)**:
  - 后续若接入真实 LEAP Hand 硬件，需插入 USB-TTL 转接板并检查 `/dev/ttyUSB0` 权限；
  - 实机上电须严格遵循安全五步梯，电机电流上限锁定 150mA；
  - 后续阶段可平滑引入 6-DOF 机械臂 SocketCAN 接口。
