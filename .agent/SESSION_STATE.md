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
- [ ] **Phase 4: 全局复核与文档闭环** (Antigravity 主导 · 看得广 - 全工程 colcon 构建、仿真/回环录制回放验证、文档与长程图谱归档)

**当前进度**: Phase 3 完成并已接手 Claude 双轴审查修正；待 Phase 4 全局复核

## 3. 协作推进记录与实证 (Completed Work & Evidence)
| 时间 | 执行 Agent | 完成内容 | 验证依据 (测试命令 / 构建结果) | Git Commit |
| :--- | :--- | :--- | :--- | :--- |
| 2026-09-18 | Antigravity | 完成旧代码调研与 16-DOF 接口完全冻结，建立脚手架与规范，8项单元测试全绿 | 规范文档 `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md` | `d2a9767` |
| 2026-09-18 | Claude Code | Phase 2 深度审计 + P0 遥操崩溃修复（`mapper.map()` → `map_keypoints_to_leap()`，新增 API 守卫测试）+ 四项架构裁决 + Phase 3 实施计划落盘 | 计划 `docs/superpowers/plans/2026-09-18-tuinadex-mvp-phase3.md`；`pytest tests/` 9/9 全绿（`leap_hand` conda env） | `77cb9ba` |
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
- **接手 Agent: Codex (Phase 3 手术级执行)**
- **接手 Agent 启动动作**:

  1. 运行自感知流程确认当前状态（本文件）；
  2. 通读 `docs/superpowers/plans/2026-09-18-tuinadex-mvp-phase3.md`（唯一执行依据，含完整代码与 TDD 步骤），并查阅 `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`；
  3. 按 Task 0 → 8 顺序逐任务执行，每任务一个 commit；Task 1/2/4/5 可立即在本机 TDD（测试环境见不变量 10），Task 0 为系统级 ROS2 安装须**先向用户报告再执行**；
  4. 每完成一个任务，在本文件第 3 节追加一行实证记录。
- **当前交接指令**: 移交 Antigravity 执行 Phase 4 全局复核：完整跑固化测试矩阵、colcon 构建、E2E mock 闭环、检查文档与规范一致性；无真机硬件不得上电机。完成后归档黑板并转入多 Agent 闭环总结。
- **工作树与分支**: `/home/bright/win_office/ubantu_files/project/TuinaDex_v2.0/.worktrees/phase3-mvp-execution`，分支 `phase3-mvp-execution`；Task 1~7 提交已完成。
- **未决疑问与风险提示**:
  - Ubuntu 26.04 对应的 ROS2 发行版名称需实证核实（apt 源 / docs.ros.org），禁止凭记忆写死；若无官方支持则回退 Docker 方案并报告用户；
  - 实机测试前需确保 `/dev/ttyUSB0` 拥有 dialout 读写权限（当前用户已在 dialout 组，设备未插时无该节点）；
  - 遥操作启动前需保证 MediaPipe 与 OpenCV 基础环境就绪（`leap_hand` env 已装 mediapipe 1.0.0）；
  - lerobot 0.4.4 的 `create`/`add_episode` API 签名需按计划 Task 7 Step 1 实测微调。

## 6. 本轮协作闭环总结汇报 (Collaboration Debriefing)
*(待一轮多 Agent 任务全部完成后由收尾 Agent 填写)*
