# ADR 0003: TuinaDex v2.0 手-臂协同多模态数据采集前端 UI Hub 与 LeRobot v3.0 数据链路决策记录

> **状态**: Accepted (已实施并通过真实 EGL+H.264 全链路自动化 E2E 终验)  
> **日期**: 2026-10-30  
> **决策者**: Antigravity, Claude Code, Codex, MacroBright  
> **适用范围**: `tuinadex_teleop`, `tuinadex_sim`, `tuinadex_dataset`, `tuinadex_bringup`

---

## 1. 背景与问题陈述 (Context & Problem)
在 TuinaDex v2.0 完成 6-DOF 机械臂与 16-DOF LEAP Hand 22 维整机统一控制与动力学孪生后，系统进入具身示教数据采集与策略学习闭环阶段。

构建工业级推拿多模态数据采集系统面临以下核心技术挑战与权衡：
1. **多视角视觉空间与分辨率标准化**: 中医推拿要求全局观察人体经络（俯视视角）、腕部灵巧手精准穴位接触（腕部第一视角随动）与指尖下压接触深度（侧向水平视角）。多机位如何规范空间位姿与视场角？
2. **高频渲染与实时控制隔离**: 实测表明 MuJoCo 在宿主机 EGL 后端渲染 3 路 640x480 图像耗时约 11.2ms (p95 11.7ms)。若在 50Hz 控制主循环中同步阻塞渲染，将吃满 20ms 时间窗口导致控制严重抖动；
3. **图像缓存内存与吞吐压力**: 原始 RGB 图像 10 秒 50fps 产生 1.38GB 内存占用，极易引发 OOM；
4. **前端工作台零依赖与低延迟交互**: 采集人员需要实时监控 3 路视频流、22 维关节与指尖力矩曲线、状态机按键与快捷键、阶段打标（如“按压”、“揉法”），但系统严禁引入臃肿的 Node.js/npm 前端构建链；
5. **具身策略学习格式闭环**: 采集数据必须无缝直通 Hugging Face LeRobotDataset v3.0 规范，包含多流 H.264 视频分块与 Parquet 元数据；
6. **话题调度与多节点双发竞争**: 仿真环境中 `LeapHandSimNode` 与 `ArmNode` 同时接入时，避免对 `/arm/joint_states` 产生双发调度冲突。

---

## 2. 核心架构决策 (Decisions)

### 决策 1: 标准三机位光学与空间拓扑 (Invariants 25 & 36)
- **方案选择**: 建立固定且规范的三机位配置字典 (`STANDARD_CAMERA_SPECS`)：
  - `overhead` (MJCF `overhead_cam`, Eye-to-Hand, 640x480, fovy=60°): 工作台正上方偏后，俯视全景推拿场景；
  - `wrist` (MJCF `wrist_cam`, Eye-in-Hand, 640x480, fovy=75°): 绑定 `leap_hand_palm`，随动聚焦掌心与指尖交互区；
  - `side` (MJCF `side_cam`, Eye-to-Hand, 640x480, fovy=45°): 工作台侧面水平机位，垂直于主要下压方向。
- **决策理由**: 覆盖中医推拿所需的大局观、接触局部与垂直下压形变，规范相机命名并锁定两份场景 XML 逐字节完全一致（Invariant 34）。

### 决策 2: 独立 EGL 渲染线程与控制回路彻底解耦 (Invariant 28)
- **方案选择**: 控制主线程严格运行在 50Hz，仅更新物理状态；渲染由专属 `CameraProducerThread` 异步驱动，独占专用 `mujoco.Renderer` 与私有 `MjData` 副本。
- **决策理由**: 彻底切断渲染耗时对控制周期的影响。设立相机超时与丢帧熔断机制（连续 3 拍丢帧立即中止试次），坚决杜绝测试与运行时补齐或伪造时间戳。

### 决策 3: 内存 JPEG 缓存与快速所有权转移 (Invariant 29)
- **方案选择**: 渲染线程单拍内直接完成 OpenCV JPEG 压缩（实测耗时仅 1.49ms），在内存中仅保留 JPEG 字节串。试次结束时将 `EpisodeBuffer` 所有权瞬间移交后台异步落盘线程，主采集回路立即重置进入空闲就绪。
- **决策理由**: 30 秒 50fps 试次内存占用从 4.14GB 压缩至约 135MB（降低 96.7%），兼顾高吞吐与内存安全性。

### 决策 4: 零依赖轻量 Web UI Hub (Invariants 27, 32, 33)
- **方案选择**: 基于 Python 原生 `http.server` 构建微型 Web 服务，前端采用单文件原生 HTML5/CSS/JS：
  - 遥测数据流采用 SSE (`/api/stream/events`, 25Hz)；
  - 视频流采用 MJPEG (`/api/stream/video/<camera>`)，复用缓存中的已编码 JPEG 字节串，每帧仅编码一次；
  - 默认绑定 `127.0.0.1` 本地回环，禁止任何直接写电机的端点。
- **决策理由**: 无需 node/npm/vite 构建链，开箱即用，秒级冷启动，严格保障网络边界安全性。

### 决策 5: LeRobotDataset v3.0 多流流式导出 (Invariants 30, 35)
- **方案选择**: 在 `tuinadex_dataset.lerobot_exporter` 中集成 LeRobot 0.4.4+ 的 `streaming_encoding` 特性，输出规范的 22 维 `observation.state`、22 维 `observation.effort`、4 指尖法向力 `observation.contact_forces`、22 维 `action`、`phase_index` 与三路 MP4 视频流。
- **决策理由**: 严格遵循 LeRobotDataset 官方规范，导出结束时强制清空 `latest_episode` 避免 Parquet 截断覆盖。

### 决策 6: ROS 2 `/arm/joint_states` 双发布竞争裁决
- **方案选择**: `LeapHandSimNode` 默认设置 `publish_arm_states=False`。机械臂话题 `/arm/joint_command` 与 `/arm/joint_states` 由 `ArmNode` 独占主导；`LeapHandSimNode` 仅订阅 `/arm/joint_states` 用于驱动 MuJoCo 动力学模型中的机械臂姿态对齐。
- **决策理由**: 严格符合模块化单一事实源（Single Source of Truth），彻底消除 ROS 2 话题双发调度竞争与看门狗状态抖动。

---

## 3. 验证与实证 (Verification Evidence)
1. **真实三机位 EGL + H.264 端到端闭环**: `bash scripts/e2e_hub_collection_check.sh` 100% 通过（60 帧 1 episode，三路视频与 22D 动作特征完整导出）；
2. **22 维臂手联合控制双模式终验**: `bash scripts/e2e_unified_robot_check.sh` ALL PASS；
3. **单元与集成测试矩阵**: 30 项 Hub 单元测试全绿，13 项 ROS 2 节点集成单测全绿，81 项遥操单测全绿；
4. **8 功能包全量编译**: `colcon build --symlink-install` 8 包全部构建通过。
