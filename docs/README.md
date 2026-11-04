# docs: 系统架构、技术规范与决策记录 (ADRs)

`docs/` 目录沉淀了 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **顶层架构设计、硬件接口规范、环境管理准则与关键架构决策记录 (ADR)**。

---

## 目录索引与导航

```
docs/
├── architecture/                 # 系统架构图与交互时序图
│   └── README.md                 # 架构图示索引与 Mermaid 全景视图
│
├── specs/                        # 核心工程与接口规范 (Specifications)
│   ├── LEAP_HAND_MVP_INTERFACE_SPEC.md   # 16-DOF LEAP Hand 硬件接口与关节空间规范 (Spec §1)
│   └── ENVIRONMENT_MANAGEMENT_SPEC.md    # 多端异构双基座环境管理与隔离规范 (Spec §2)
│
├── adrs/                         # 架构决策记录 (Architecture Decision Records)
│   ├── 0001-architecture-decisions-mvp.md            # ADR-001: 核心五层分层架构与纯 Python 内核
│   ├── 0002-architecture-decisions-sim.md            # ADR-002: 500Hz MuJoCo 动力学与 50Hz 定时器解耦
│   └── 0003-data-collection-hub-and-lerobot-pipeline.md # ADR-003: 零依赖 Web UI Hub 与 LeRobotDataset v3.0 管道
│
└── superpowers/plans/            # 专项技术演进与设计实施计划
```

---

## 核心技术规范速查

### 1. 关节空间与接口契约规范 (`LEAP_HAND_MVP_INTERFACE_SPEC.md`)
- **全链路弧度制契约 (SI Radian)**: 0 rad 全开平伸基准零位，弯曲屈曲为正；
- **规范 16-DOF 排序**: 食指 4 维 $\rightarrow$ 中指 4 维 $\rightarrow$ 无名指 4 维 $\rightarrow$ 拇指 4 维；
- **看门狗安全兜底**: 300ms 指令断流自动触发 `0.3s hold + 0.6s ramp` 阻尼平滑回退至全开平伸姿态 (`Relax-to-OPEN`)；
- **硬件电流硬锁**: 舵机限制在 $\le 150\text{ mA}$。

### 2. 双基座环境隔离规范 (`ENVIRONMENT_MANAGEMENT_SPEC.md`)
- **`tuinadex-control` (Python 3.10.20)**: 控制算法、ROS 2 运行时、MuJoCo 仿真、Pinocchio 动力学、Web UI 采集 Hub；
- **`tuinadex-vla` (Python 3.10.20 + CUDA 12.8)**: 具身策略学习、Hugging Face LeRobot、PyTorch、SmolVLA / ACT 模型训练；
- **环境准入门禁**: 任何依赖调整均须导出 YAML 锁定并跑通 `bash scripts/verify_all_envs.sh`。

### 3. 架构决策记录 (ADRs)
- **ADR-001**: 确立五层分层架构，`tuinadex_core` 严禁引入 ROS 2 依赖；
- **ADR-002**: 仿真引擎以 500Hz 子步进稳定接触求解，单线程锁步降采样至 50Hz 发布状态，杜绝后台线程竞态；
- **ADR-003**: 示教数据事实源为 `rosbag2`，采集 Hub 采用轻量原生 HTTP+SSE 前端，流式转码为 `LeRobotDataset v3.0`。

