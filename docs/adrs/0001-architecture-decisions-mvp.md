# ADR 0001: TuinaDex v2.0 LEAP Hand MVP 核心架构决策记录

> **状态**: Accepted (已全部实施并通过端到端实证验证)  
> **日期**: 2026-09-18  
> **决策者**: Antigravity, Claude Code, Codex, MacroBright  
> **适用范围**: `tuinadex_hand`, `tuinadex_teleop`, `tuinadex_recorder`, `tuinadex_dataset`

---

## 1. 背景与问题陈述 (Context & Problem)
TuinaDex v1.0 存在旧代码与灵巧手驱动耦合过深、单位与坐标系不统一（部分角度、部分弧度、部分舵机 raw 刻度）、遥操作存在调用未定义接口的崩溃风险、缺少看门狗安全保护、数据录制依赖脆弱的 subprocess 命令行包装等严重问题。

在 v2.0 框架重构中，需要面向中医推拿机械臂-灵巧手具身智能系统建立最小闭环（MVP），要求明确解耦运行时与策略学习域，并在缺少实体硬件时具备 100% 仿真/Mock 闭环自检能力。

---

## 2. 核心架构决策 (Decisions)

### 决策 1: 节点生命周期模型 — 普通 rclpy Node + 独立纯逻辑 FSM
- **方案选择**: 采用普通 `rclpy.node.Node` 搭配纯 Python 类的显式状态机 `WatchdogStateMachine`（FOLLOWING / RELAXING），而非 ROS2 复杂的 `LifecycleNode`。
- **决策理由**:
  1. MVP 阶段状态机语义极其集中（只有两态：主端指令有效时跟随 FOLLOWING；超时 >300ms 或关停时平滑阻尼回全开平伸姿态 RELAXING）；
  2. 纯类状态机完全剥离 ROS2 依赖，可在任何轻量 Python 环境进行 100% 确定性单测与边界测试；
  3. 大幅降低 ROS2 Humble / Lyrical 间 Lifecycle API 细微变更带来的兼容风险。

### 决策 2: 示教采集器架构 — 进程内 `rosbag2_py` SequentialWriter + Trigger 服务
- **方案选择**: 采用原生 `rosbag2_py` 进程内节点，通过 `/recorder/start` 和 `/recorder/stop` (`std_srvs/srv/Trigger`) 服务控制起停，严禁使用 subprocess 包装 `ros2 bag record` 命令行。
- **决策理由**:
  1. 规避了外部进程信号（SIGINT/SIGTERM）处理不当导致的 `.db3-wal` 数据库损坏或数据落盘丢帧；
  2. 进程内可精确获取首尾时间戳与写入指标；
  3. 服务起停调用具备明确的同步返回响应与幂等性防护（防止重复启动或重复停止）。

### 决策 3: 双域解耦与离线适配器管道 (Dual-Domain Decoupling & Offline Adapter)
- **方案选择**: 
  - **ROS2 运行时域**: 专注于机器人高频控制 (50Hz)、硬件抽象驱动、安全看门狗、实测状态发布与原始 rosbag2 记录；
  - **LeRobot 学习域**: 专注于数据集 Schema 校验、离线重采样对齐与策略训练；
  - **解耦管道**: `rosbag2` 是原始数据的唯一事实源 (Source of Truth)。离线工具 `tuinadex_dataset` 负责两阶段转换：
    1. ROS2 环境 (`python3`): 反序列化 rosbag2，50Hz 时间网格插值对齐，执行 `SPEC_JOINT_LIMITS` 严格校验，导出中间数据；
    2. 学习环境 (`smolvla` conda env / lerobot 0.4.4): 基于 `LeRobotDataset` API 生成标准 Parquet 格式数据集。
- **决策理由**:
  1. 避免 LeRobot 强依赖 ROS2 C++ 动态库与 Python 运行时（Python 3.10 vs Python 3.14 依赖隔离）；
  2. 严禁学习层直接读写机器人硬件，保障物理安全。

### 决策 4: 双重关节空间限位护栏 (Double Limit Gate)
- **方案选择**: 
  - **第一道护栏 (电机入口)**: 控制节点在 `command_processing.align_command()` 处强制进行 16 关节空间限位门检查，使用 `JOINT_SPEC_LIMITS` 逐关节截断 (`np.clip`)，杜绝越界弧度进入舵机；
  - **第二道护栏 (数据集入口)**: 数据集校验器 `DatasetValidator` 强制使用同源 `SPEC_JOINT_LIMITS`（食指/中指/无名指侧摆 `[-0.35, 0.35]`，拇指回转 `[-0.30, 1.80]`，指关节屈曲 `[0.0, 1.6~2.0]`），拒收任何 NaN/Inf/越界/跳变帧。
- **决策理由**: 形成软硬件防御纵深，防止仿真或模型输出的异常大动作打坏机械手指。

---

## 3. 结果与验证 (Consequences & Evidence)
- **全工程编译**: `colcon build --symlink-install` 编译 3 个 ROS2 包（`tuinadex_hand`, `tuinadex_recorder`, `tuinadex_teleop`）全部成功。
- **测试矩阵全绿**:
  - `leap_hand` conda env: 28 passed, 5 skipped (0 failures)
  - `smolvla` conda env: 5 passed, 2 skipped (0 failures)
  - ROS2 Lyrical host env: 4 passed, 1 skipped (0 failures)
- **端到端闭环验证**: `bash scripts/e2e_mock_check.sh` 完整跑通 Mock 控制循环 -> rosbag2 写入 -> 148 帧数据对齐校验 -> LeRobot Parquet 数据集落地，输出 `[E2E] PASS`。
