# tuinadex_dataset: 具身智能 LeRobotDataset v3.0 离线转换与流式导出工具

`tuinadex_dataset` 是 TuinaDex v2.0 中医推拿机械臂-灵巧手具身智能系统的 **离线数据清洗、时空对齐与 LeRobotDataset v3.0 格式流式转换工具包（Layer 5: AI Policy Preparation Layer）**。

本包负责将运行态采集的原生 `rosbag2` 示教数据包无损解析，执行严格的高频时网格重采样与限位合法性校验，并多路流式并行编码为包含 3 路 H.264 视频流（俯视、腕部、侧向）与 22 维数值特征（状态、力矩、指尖接触力）的标准 Hugging Face `LeRobotDataset v3.0`，直接存储至工程根目录的 `data/` 目录，供给下游模仿学习策略（如 SmolVLA、ACT、Diffusion Policy）进行分布式训练。

---

## 快速开始 (Quick Start)

根据《多端异构环境管理规范》，本模块跨越 ROS 2 数据解析与 PyTorch/LeRobot 策略模型两套隔离基座：

### 1. 命令行一键转换 (CLI Usage)
在准备好 `rosbag2` 录制目录后，执行离线转换：

```bash
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0

# 激活具身策略环境 (Python 3.10 + CUDA + LeRobot 0.4.4)
source scripts/activate_tuinadex.sh vla

# 将指定 rosbag2 目录转换为标准 LeRobotDataset 格式并存入 data/
python3 -m tuinadex_dataset.cli \
  --bag bags/session_20261102_164500 \
  --out data/tuina_demo \
  --fps 50.0
```

> **参数说明**:
> - `--bag`: 待转换的原始 rosbag2 目录路径；
> - `--out`: 目标 LeRobotDataset 本地根路径（推荐输出至 `data/<dataset_name>`）；
> - `--fps`: 目标重采样帧率（默认 50.0 Hz）；
> - `--allow-invalid`: 遇到超出物理限位的数据时仅警告而不阻断导出（默认严格拒绝，确保训练数据纯净）。

---

## 1. 核心架构与数据流拓扑

```mermaid
flowchart TD
    RawBag["原始事实源: bags/session_* (rosbag2 / sqlite3)"] --> Reader["bag_reader.py\n(反序列化双话题: /joint_states 与 /joint_command)"]
    Reader --> Sync["TimeSynchronizer (synchronizer.py)\n(50Hz 统一时间网格线性/三次样条插值)"]
    Sync --> Validator{"DatasetValidator (validator.py)\n(维度校验, NaN/Inf 拦截, 关节物理限位检查)"}

    Validator -->|校验失败 (Reject)| Abort["终止导出并输出错误诊断报告"]
    Validator -->|校验通过 (Pass)| Exporter["LeRobotDatasetWriter (lerobot_exporter.py)"]

    subgraph Modalities ["多模态全特征流式对齐"]
        State["observation.state (22D rad)"]
        Effort["observation.effort (22D Nm)"]
        Forces["observation.contact_forces (4指法向力 N)"]
        Action["action (22D 目标弧度)"]
        Video["3 路 640x480 H.264 MP4 视频\n(overhead / wrist / side)"]
    end

    Exporter --> Modalities
    Modalities --> Dataset[("LeRobotDataset v3.0\n落地存储于: data/tuina_demo")]
```

---

## 2. 工程数据目录布局 (`data/` 与 `dataset/`)

TuinaDex 遵循数据产品与代码工具严格分离的原则：

| 根级目录 | 定位与性质 | 核心内容 |
| :--- | :--- | :--- |
| **`dataset/`** | **代码与算法层** | `dataset/tuinadex_dataset/` 代码包。包含 `adapter.py`、`synchronizer.py`、`validator.py`、`lerobot_exporter.py` 等转换核心逻辑。 |
| **`data/`** | **训练就绪数据集产品层** | 存放已清洗、对齐并流式转码完成的 Hugging Face `LeRobotDataset v3.0` 数据集。 |
| **`bags/`** | **实验原始事实源层** | 存放由 `tuinadex_recorder` 录制的未修改原生 `rosbag2` 数据库。 |

---

## 3. 实测基准数据集 (`data/tuina_demo`)

工程目录 `data/tuina_demo` 现已沉淀为标准的具身推拿基准示教数据集：

- **基本规模**: **52** 个完整 Episodes，共计 **50,654** 帧（50Hz 采样，连续有效示教时长约 17 分钟）；
- **数据格式**: 严格遵循 Hugging Face LeRobotDataset `v3.0` 规范，采用分块 Parquet 与分块 H.264 MP4 视频；
- **文件体积**: 数据集总大小约 300MB（Parquet 列式数据约 100MB，视频分块约 200MB）。

### 3.1 目录资产拓扑 (`data/tuina_demo/`)
```text
data/tuina_demo/
├── data/
│   └── chunk-000/
│       └── file-000.parquet      # 包含 22D state, effort, forces, action 的列式特征表
├── videos/
│   ├── observation.images.overhead/
│   │   └── chunk-000/episode_000000.mp4  # 640x480 俯视全局视窗 H.264 视频流
│   ├── observation.images.wrist/
│   │   └── chunk-000/episode_000000.mp4  # 640x480 腕部第一人称接触视窗 H.264 视频流
│   └── observation.images.side/
│       └── chunk-000/episode_000000.mp4  # 640x480 侧向水平观察视窗 H.264 视频流
└── meta/
    ├── info.json                 # 数据集元数据 (codebase_version: v3.0, fps: 50, 特征字典)
    ├── stats.json                # 全量 22 关节的统计指标 (min, max, mean, std, 分位数)
    ├── tasks.parquet             # 推拿任务描述词表 (如 "tuina acupoint massage")
    ├── phase_labels.json         # 推拿各阶段语义标注
    └── episodes/                 # 试次切分索引
```

---

## 4. 全模态特征契约规范 (Dataset Feature Schema)

导出的数据集严格遵守 Hugging Face LeRobotDataset v3.0 特征字典契约：

| 特征键名 (Feature Key) | 数据类型 (dtype) | 形状 (Shape) | 物理含义与通道约定 |
| :--- | :--- | :--- | :--- |
| `observation.state` | `float32` | `(22,)` | 机械臂 6 关节 (`joint1`~`joint6`) + 灵巧手 16 关节 (`index_yaw`~`thumb_ip`) 实测角度 (rad) |
| `observation.effort` | `float32` | `(22,)` | 22 维实测关节力矩或电流 (N·m / mA) |
| `observation.contact_forces` | `float32` | `(4,)` | 食指、中指、无名指、拇指指尖法向接触力 (N) |
| `action` | `float32` | `(22,)` | 22 维绝对目标关节位置 (rad, 严禁 delta 增量) |
| `phase_index` | `int64` | `(1,)` | 中医推拿阶段枚举序号（如寻穴、点按、揉捏、回撤）|
| `observation.images.overhead`| `video` (H.264)| `(3, 480, 640)` | 俯视工作台全局视角彩色视频流 |
| `observation.images.wrist` | `video` (H.264)| `(3, 480, 640)` | 灵巧手腕部随动第一人称推拿视角视频流 |
| `observation.images.side` | `video` (H.264)| `(3, 480, 640)` | 侧向平行于推拿接触表面水平视频流 |
| `timestamp` | `float32` | `(1,)` | 对齐后的 50Hz 相对时间戳 (秒) |
| `frame_index` | `int64` | `(1,)` | 本试次内帧索引序号 ($0, 1, 2, \dots$) |

---

## 5. 核心子模块详解

### 5.1 时钟对齐器 (`synchronizer.py` · `TimeSynchronizer`)
- **时间网格对齐**: 以目标帧率（默认 50Hz, $\Delta t = 20\text{ ms}$）生成绝对单调递增的时间网格；
- **自适应插值**: 对观测状态与动作序列分别执行时间序列对齐，消除异步传感器话题到达时延抖动。

### 5.2 离线质量验证器 (`validator.py` · `DatasetValidator`)
- **维度完整性**: 确保每个时间戳的数据维度严格对齐 22 维（或 16 维）；
- **脏数据拦截**: 扫描全量序列，彻底拦截 `NaN`、`Inf` 及丢失帧；
- **物理限位校验**: 逐帧校验每个关节是否超出 `SPEC_JOINT_LIMITS`。若发现越界则精确报告违例关节名称、越界极值与对应帧数。

### 5.3 多模态流式导出器 (`lerobot_exporter.py` · `LeRobotDatasetWriter`)
- **零拷贝 JPEG 缓存转码**: 内存中保持高效 JPEG 流，仅在导出时由专用生产线程多路并行压缩为 H.264 MP4 视频，避免内存 OOM；
- **Parquet 分块序列化**: 数值特征使用高效列式存储，支持训练时高并发读取与快速 DataLoader 预加载。

---

## 6. 跨环境双重单元测试与验证

根据双环境管理准则，分别在 ROS 2 基座与 VLA 基座下执行闭环测试：

### 阶段 1：在 ROS 2 环境中测试 rosbag2 数据解析与时序对齐 (6 项通过)
```bash
source /opt/ros/lyrical/setup.bash
python3 -m pytest tests/test_dataset_adapter.py tests/test_dataset_sync_validator.py -v
```

### 阶段 2：在 VLA 策略环境中测试 LeRobot v3.0 数据集生成与三机位视频导出 (7 项通过)
```bash
# 激活具身策略环境
eval "$(~/miniconda3/bin/conda shell.bash hook)"
conda activate tuinadex-vla

# 运行数据校验与 LeRobotDataset v3.0 视频多模态导出单测
pytest tests/test_dataset_sync_validator.py tests/test_lerobot_v3_exporter.py -v
```

**预期输出指标**:
```
tests/test_dataset_sync_validator.py::test_time_synchronizer PASSED
tests/test_dataset_sync_validator.py::test_dataset_validator PASSED
tests/test_lerobot_v3_exporter.py::test_lerobot_v3_multi_modal_export PASSED
tests/test_lerobot_v3_exporter.py::test_multi_modal_v3_export_with_three_cameras PASSED
================== 7 passed, 2 skipped, 39 warnings in 7.32s ===================
```
