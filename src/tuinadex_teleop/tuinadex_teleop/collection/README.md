# TuinaDex v2.0 多模态数据采集与前端 UI Hub 系统 (Data Collection Subsystem)

本目录为 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **多模态数据采集层 (Data Collection Layer)**。

系统面向 **reBot-DevArm RS 机械臂 (6-DOF) + LEAP Hand 仿生灵巧手 (16-DOF)** 构成的 22 自由度协同系统，集成 **标准三机位视觉系统 (640×480)**、**22 维本体感知与动力学反馈**、**4 指尖法向接触力反馈**、**22 维动作标签** 以及 **轻量 Web UI Hub 实时交互工作台**，并直通 **Hugging Face LeRobotDataset v3.0** 格式导出。

---

## 快速开始 (Quick Start)

> [!TIP]
> 采集系统提供 **轻量 Web UI Hub 交互工作台（推荐）**、**纯终端/无头自动化模式**、**真机 ROS 2 话题模式** 以及 **一键端到端 E2E 自检脚本**。

### 1. 运行环境与依赖激活

TuinaDex 采用双 Conda 环境分治架构：
- 采集控制、仿真渲染与 Web Hub 运行于 `tuinadex-control` (Python 3.10)；
- LeRobotDataset v3.0 导出与 PyTorch 模型转换运行于 `tuinadex-vla` (Python 3.10 + CUDA 12.8)。

```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0

# 激活采集与控制主环境
source scripts/activate_tuinadex.sh control
```

---

### 2. 方式 A：启动轻量 Web UI Hub 交互工作台（推荐）

适合**实时观察 3 机位视频墙、监视 22 维关节曲线与接触力柱状图、交互式示教录制推拿轨迹**：

```bash
# 启动 22 维手-臂协同仿真与 Web UI Hub (默认绑定 127.0.0.1:8080)
python scripts/collect_teleop_data.py \
    --env sim \
    --mode arm_hand \
    --cameras overhead,wrist,side \
    --hub \
    --fps 50 \
    --ui-fps 25 \
    --repo-id data/tuina_hub_dataset
```

启动后控制台将输出 `HUB_READY port=8080`。用浏览器打开：
👉 **`http://127.0.0.1:8080`**

#### 浏览器工作台操作指引：
1. **任务配置**: 在顶部输入当前推拿手法与穴位（例如语言指令：`按压大椎穴`，阶段标签：`approach` / `press` / `hold` / `release`）；
2. **开始示教**: 点击 **Start Recording**（快捷键 **`[空格]`**），系统状态切换至 `RECORDING`，开始以 50Hz 严格时间网格记录 3 机位画面与 22 维运动学/动力学数据；
3. **结束试次**: 推拿动作完成后点击 **Stop Recording**（快捷键 **`[S]`**），状态进入 `REVIEWING` 评审确认；
4. **决策保存/舍弃**: 
   - 示教满意：点击 **Save Episode**（快捷键 **`[回车]`**），后台异步线程自动将其分块写入 LeRobotDataset v3.0；
   - 示教失误：点击 **Discard**（快捷键 **`[退格]`**），清空缓冲区直接进入下一试次。

---

### 3. 方式 B：自动化生成示教轨迹与自检 (无需人工介入)

适合**算法快速回归、无浏览器环境或 CI/CD 流水线自检**：

```bash
# 自动生成 3 个 Episode，每个 50 帧，采用平滑正弦示教动作并自动保存
python scripts/collect_teleop_data.py \
    --env sim \
    --mode arm_hand \
    --mock-teleop \
    --episodes 3 \
    --frames-per-episode 50 \
    --fps 50 \
    --repo-id data/tuina_mock_demo
```

---

### 4. 方式 C：切换至真实物理机器人采集模式

当系统切换至真实硬件环境时，无需改动任何上层代码，只需将 `--env` 参数切换为 `real`：

```bash
# 接入 ROS 2 话题与真实硬件驱动 (需先启动 robot.launch.py use_sim:=false)
python scripts/collect_teleop_data.py \
    --env real \
    --mode arm_hand \
    --hub \
    --cameras overhead,wrist,side \
    --repo-id data/tuina_real_dataset
```

---

### 5. 方式 D：一键全链路 E2E 自动化回归验证 (CI 门禁)

执行全栈自动化测试（启动 Hub 服务 $\rightarrow$ 模拟 REST 控制调用 $\rightarrow$ 真实 EGL 渲染 3 机位图像 $\rightarrow$ 异步落盘 $\rightarrow$ 验证 LeRobot v3.0 视频分块与 Parquet 元数据合规）：

```bash
bash scripts/e2e_hub_collection_check.sh
```

终端输出 `[E2E] PASS` 即证明采集层所有链路完全健康。

---

## 目录结构与模块定位

数据采集层核心代码位于 `src/tuinadex_teleop/tuinadex_teleop/collection/`，并与相机模块、Web Hub 模块及导出模块形成高内聚、低耦合的协作体系：

```
TuinaDex_v2.0/
├── src/tuinadex_teleop/tuinadex_teleop/
│   ├── collection/                         # 【核心数据采集层】
│   │   ├── __init__.py
│   │   ├── collector.py                    # 状态机编排器 (IDLE/RECORDING/PAUSED/REVIEWING/SAVING)
│   │   ├── episode_buffer.py               # 多模态试次内存环形缓冲 (含帧完整性校验与 JPEG 原生内存存储)
│   │   ├── data_channel.py                 # 双通道数据源抽象 (SimDataChannel 与 RealDataChannel 对等切换)
│   │   ├── data_hub_controller.py          # 线程安全 Hub 控制器 (命令队列、50Hz 步进锁步与异步落盘调度)
│   │   └── README.md                       # 本说明文档
│   │
│   ├── cameras/                            # 【多相机阵列与渲染管线】
│   │   ├── camera_specs.py                 # 标准 3 机位光学契约定义 (overhead, wrist, side)
│   │   └── sim_camera_rig.py               # MuJoCo 离屏渲染器 (SimCameraRig 与独立 CameraProducerThread)
│   │
│   └── hub/                                # 【轻量 Web UI Hub 服务端与前端】
│       ├── server.py                       # 原生 Python http.server (REST API + SSE 遥测 + MJPEG 视频流)
│       └── static/                         # 零外部依赖前端工作台
│           ├── index.html                  # 响应式布局 (3 视窗视频墙 + 仪表盘 + 控制台)
│           ├── style.css                   # 现代深色模式 UI 样式
│           └── app.js                      # 状态同步、实时曲线绘制与键盘快捷键响应
│
├── dataset/tuinadex_dataset/               # 【数据转换与学习导出层】
│   └── lerobot_exporter.py                 # LeRobotDataset v3.0 导出器 (H.264 视频流编码与 Parquet 写入)
│
└── scripts/                                # 【统一运行与测试脚本】
    ├── collect_teleop_data.py              # 统一数据采集 CLI 终端入口
    └── e2e_hub_collection_check.sh         # 端到端 E2E 自动化回归自检脚本
```

---

## 核心设计与契约约束 (Invariants)

1. **标准三机位视觉拓扑 (Eye-to-Hand + Eye-in-Hand)**:
   - `overhead` (Eye-to-Hand, 640×480, `fovy="60"`): 架设于工作台/受试者背部正上方偏后侧，下倾 50° 俯视作业全景与机械臂基座；
   - `wrist` (Eye-in-Hand, 640×480, `fovy="75"` 广角): 绑定于机械臂法兰盘 `link6` 或灵巧手手背支架 `palm_lower`，镜头朝向指尖正前方偏掌心侧；
   - `side` (Eye-to-Hand, 640×480, `fovy="45"`): 架设于推拿床侧面，镜头平行于接触表面（背部切面）横向水平拍摄。
2. **22-DOF 协同本体与动作特征字典**:
   - 坐标与单位全链路强制遵守 **SI 国际标准单位（弧度 rad）**；
   - 严格区分 **实际物理状态 (State)** 与 **上层示教动作 (Action)**；
   - 包含机械臂 6 维 + 灵巧手 16 维关节角度、22 维关节力矩/电流反馈、4 指（食指、中指、无名指、拇指）指尖法向接触力 (N)。
3. **仿真/真机双通道对等抽象 (Channel Parity)**:
   - [`SimDataChannel`](file:///home/bright/win_office/ubantu_files/project/TuinaDex_v2.0/src/tuinadex_teleop/tuinadex_teleop/collection/data_channel.py) 从 MuJoCo 物理引擎与离屏渲染线程读取；
   - [`RealDataChannel`](file:///home/bright/win_office/ubantu_files/project/TuinaDex_v2.0/src/tuinadex_teleop/tuinadex_teleop/collection/data_channel.py) 从 ROS 2 话题 `/hand/joint_states`、`/arm/joint_states` 与物理相机驱动读取；
   - 两者实现完全相同的 `BaseDataChannel` 契约，上层采集状态机与下游 LeRobot 导出器 100% 透明无感。
4. **轻量零依赖 Web UI Hub**:
   - 彻底规避 Node.js/npm 外部庞大构建工具链，采用 Python 原生多线程 `ThreadingHTTPServer`；
   - MJPEG 多流分发与 SSE 遥测推流将 UI 端到端监控延迟控制在 $< 50\text{ms}$。
5. **硬件安全与丢帧熔断**:
   - 独立后台渲染线程 (`CameraProducerThread`)，渲染耗时绝不阻塞 50Hz 核心控制循环；
   - 若图像流连续丢帧或超时超过 $60\text{ms}$（3 帧以上），控制器自动熔断当前试次，严禁向训练集灌入静止帧伪造数据。

---

## 数据特征字典与元数据 Schema (LeRobotDataset v3.0 对齐)

采集保存的每条 Episode 均严格符合 Hugging Face LeRobot Codebase v3.0 规范：

| 键名 (Key) | 数据类型 (dtype) | 维度 (Shape) | 说明 |
| :--- | :--- | :--- | :--- |
| `observation.state` | `float32` | `[22]` | 机械臂 6 维 + 灵巧手 16 维物理关节角度 (`rad`) |
| `observation.effort` | `float32` | `[22]` | 机械臂 6 维关节力矩 (`Nm`) + 灵巧手 16 维电流/力矩 (`mA`) |
| `observation.contact_forces` | `float32` | `[4]` | 食指、中指、无名指、拇指指尖法向接触力 (`N`) |
| `action` | `float32` | `[22]` | 遥操作下发的 22 维目标关节位置期望 (`rad`) |
| `observation.images.overhead` | `video` / `image` | `(480, 640, 3)` | 全局俯视固定视角彩色画面 |
| `observation.images.wrist` | `video` / `image` | `(480, 640, 3)` | 腕部第一视角随动彩色画面 |
| `observation.images.side` | `video` / `image` | `(480, 640, 3)` | 侧向水平固定视角彩色画面 |
| `timestamp` | `float32` | `[1]` | 试次内相对时间戳（以秒为单位，如 0.00, 0.02, 0.04...） |
| `frame_index` | `int64` | `[1]` | 当前帧在 Episode 内的自增索引（从 0 开始） |
| `episode_index` | `int64` | `[1]` | 轨迹试次编号（从 0 开始） |
| `next.done` | `bool` | `[1]` | 是否为该次推拿轨迹的最后一帧 |
| `language_instruction` | `string` | 标量 | 推拿自然语言任务指令（如"以 5N 力度按压大椎穴 3 次"） |
| `phase_label` | `string` | 标量 | 推拿动作阶段标签（`approach` / `press` / `hold` / `release`） |

---

## 前端 UI Hub 接口与通信契约

Web UI Hub 通过 REST API、Server-Sent Events (SSE) 与 MJPEG 视频流三路并发通信：

### 1. REST 控制接口
- `GET /api/status`: 查询当前采集状态机、缓冲帧数、已保存试次数、丢帧统计等；
- `POST /api/action/start`: 开始录制（状态转为 `RECORDING`）；
- `POST /api/action/stop`: 停止录制（状态转为 `REVIEWING`）；
- `POST /api/action/pause`: 暂停录制；
- `POST /api/action/resume`: 恢复录制；
- `POST /api/action/save`: 确认保存当前试次；
- `POST /api/action/discard`: 舍弃当前试次；
- `POST /api/action/label`: 更新当前推拿任务的 `language_instruction` 与 `phase_label`。

### 2. 流式传输接口
- `GET /events/telemetry`: SSE 实时遥测流（25Hz），向前端推送 22D 关节位置、接触力、时间戳及状态变化；
- `GET /video/overhead`: 俯视相机 MJPEG 视频流（`multipart/x-mixed-replace`）；
- `GET /video/wrist`: 腕部随动相机 MJPEG 视频流；
- `GET /video/side`: 侧向相机 MJPEG 视频流。

### 3. 工作台键盘快捷键速查

| 按键 | 对应动作 | 行为说明 |
| :---: | :--- | :--- |
| **`[空格]`** | **Start / Pause / Resume** | 录制开/关热键 |
| **`[S]`** | **Stop Recording** | 停止录制并进入试次评审 |
| **`[回车]`** | **Save Episode** | 确认保存当前轨迹至磁盘 |
| **`[退格]`** | **Discard Episode** | 舍弃当前试次缓冲 |
| **`[1] ~ [4]`** | **Phase Shortcut** | 快速切换推拿阶段 (1: approach, 2: press, 3: hold, 4: release) |

---

## 单元测试与验证命令

数据采集层具备完备的单元测试矩阵，全部通过 pytest 运行：

```bash
# 激活环境
source scripts/activate_tuinadex.sh control

# 运行相机阵列、缓冲队列、双通道路由器、控制器与 HTTP API 全套单测 (27 项)
python -m pytest \
    tests/test_camera_rig.py \
    tests/test_episode_buffer.py \
    tests/test_data_channel.py \
    tests/test_data_hub_controller.py \
    tests/test_collection_hub_api.py -v

# 运行 LeRobotDataset v3.0 多视频流切片与 Parquet 导出测试 (需 tuinadex-vla 环境)
~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest \
    tests/test_lerobot_v3_exporter.py -v

# 运行全链路端到端集成测试 (启动 Hub -> 渲染 -> 交互 -> 导出 -> Parquet与视频校验)
bash scripts/e2e_hub_collection_check.sh
```
