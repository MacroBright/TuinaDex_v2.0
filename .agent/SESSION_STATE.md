# Session State & Multi-Agent Blackboard

> 项目多 Agent 协同接力黑板：用于 Antigravity、Claude Code、Codex 跨会话与跨工具协作。
> 放置在项目根目录 `.agent/SESSION_STATE.md`。

## 1. 当前任务目标 (Active Goal)
- **任务主题**: TuinaDex v2.0 框架重构 — 建立 LEAP Hand MVP 闭环体系 (ROS2 Runtime + Teleop + Data Collection + LeRobot Adapter)
- **关联需求 / 规范文档**: `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`
- **发起时间**: 2026-09-18

## 2. 流水线阶段与 Agent 职责分工 (4-Phase Collaboration Pipeline)
- [x] **Phase 1: 广域侦察与影响分析** (Antigravity 主导 · 看得广 - 完成旧工程源码研读、架构解耦分析、16-DOF 接口契约冻结、脚手架与基础驱动搭建)
- [ ] **Phase 2: 深度规划与质量裁决** (Claude Code 主导 · 想得深 - 节点生命周期、看门狗安全状态机、MediaPipe 遥操解耦审核、Implementation Plan 细化)
- [ ] **Phase 3: 手术级执行与交叉审查** (Codex 主导执行 · 改得准 + Claude Code 监督 - 落地 `tuinadex_hand` 节点、`tuinadex_teleop` 手势节点、`tuinadex_dataset` 离线转换工具与 TDD 测试)
- [ ] **Phase 4: 全局复核与文档闭环** (Antigravity 主导 · 看得广 - 全工程 colcon 构建、仿真/回环录制回放验证、文档与长程图谱归档)

**当前进度**: Phase 1 进行中 (Antigravity 已冻结 16-DOF 规范，已初始化工程脚手架与基础驱动库)

## 3. 协作推进记录与实证 (Completed Work & Evidence)
| 时间 | 执行 Agent | 完成内容 | 验证依据 (测试命令 / 构建结果) | Git Commit |
| :--- | :--- | :--- | :--- | :--- |
| 2026-09-18 | Antigravity | 完成旧代码调研与 16-DOF 接口完全冻结，建立脚手架与规范，8项单元测试全绿 | 规范文档 `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md` | `d2a9767` |

## 4. 核心技术决策与不变量 (Decisions & Invariants)
- **不变量 1 (坐标与单位)**: ROS2 与 LeRobot 全链路强制 SI 单位弧度 (rad)。以实测校准后的 `OPEN_POSE`（全开平伸）为 0 rad 基准零位，手指屈伸弯曲为正 ($q > 0$)。
- **不变量 2 (双域解耦)**: ROS2 负责机器人运行时（Runtime）；LeRobot 负责策略学习（Learning）。LeRobot 绝对禁止跨过 ROS2 直连硬件；ROS2 Topic 绝对不为 LeRobot Schema 强行妥协。
- **不变量 3 (数据源契约)**: `rosbag2` 为唯一实验原始事实源 (Source of Truth)；`LeRobotDataset` 为衍生训练数据产品。解耦通过离线 `tuinadex_dataset` 工具完成。
- **不变量 4 (语义分离)**: State = 物理实测位置 (`/hand/joint_states`)；Action = 目标控制位置 (`/hand/joint_command`)，两者严格区分。
- **不变量 5 (动作空间)**: Action 采用 16 维 absolute joint position，绝对禁止使用增量 delta。
- **不变量 6 (控制模式与周期)**: 手部节点采用 50Hz (20ms) 循环，电机模式锁定为成熟验证过的位置-电流混合模式 (Position-Current Mode = 5)，限流 150mA。

## 5. 接力指令 (Handoff to Next Agent)
- **接手 Agent 启动动作**: 
  1. 运行自感知流程确认当前状态；
  2. 查阅 `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md` 确认关节空间到舵机空间的数学转换关系；
  3. 推进 Phase 2 规划或 Phase 3 节点与测试的高吞吐编写。
- **未决疑问与风险提示**: 
  - 实机测试前需确保 `/dev/ttyUSB0` 拥有 dialout 读写权限；
  - 遥操作启动前需保证 MediaPipe 与 OpenCV 基础环境就绪。

## 6. 本轮协作闭环总结汇报 (Collaboration Debriefing)
*(待一轮多 Agent 任务全部完成后由收尾 Agent 填写)*
