# 数据采集 UI Hub 与多模态链路 — Phase 2 裁决版实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> 本文件为执行唯一事实源。上游 Phase 1 文档 `docs/superpowers/plans/2026-10-30-data-collection-ui-hub-and-pipeline.md` 已归档为只读参考。

**Goal:** 在 `tuinadex_teleop` 内交付三机位（overhead / wrist / side）多模态数据采集系统：50Hz 控制回路 + 50Hz 相机采集（独立渲染线程）、22D 状态/力矩/动作 + 4 指尖法向力、零依赖 Web UI Hub（SSE + MJPEG）、直通 LeRobotDataset v3.0 三路视频流导出。

**Architecture:** 相机采集从控制回路剥离到独立 `CameraProducerThread`（独占 EGL `mujoco.Renderer` + 私有 `MjData`），控制回路每拍只取"最新 JPEG 帧 + 其真实采集时刻"。图像在内存中以 JPEG 字节串存储，导出时解码由 LeRobot 流式编码器落盘。Hub 为 `ThreadingHTTPServer`，HTTP 线程只读共享槽位，绝不渲染或编码。所有动作经线程安全命令队列在控制拍边界生效。

**Tech Stack:** Python 3.10（`tuinadex-control`）、MuJoCo 3.x（EGL 离屏）、NumPy、OpenCV、ROS 2 Lyrical、LeRobot 0.4.4（CODEBASE_VERSION=`v3.0`）、PyTorch、HTML5/SSE/MJPEG。

**Spec:** 本文件第 3 节（新增不变量 I28–I36）；上游 `docs/superpowers/plans/2026-10-30-data-collection-ui-hub-and-pipeline.md`；`docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`；`docs/adrs/0002-architecture-decisions-sim.md`。

---

## 1. Phase 2 裁决摘要（对 Antigravity 原案的修改）

| # | 原案 | 裁决 | 依据（本机实测 / 源码核查） |
|---|---|---|---|
| 1 | 三相机 `~0.5ms/frame`，50Hz 同拍渲染 | 保留 50Hz，但改由独立渲染线程承担；控制回路不渲染 | 实测 EGL 640×480：单相机 3.7ms，三相机 **11.2ms 中位 / 11.7ms p95**。320×240 仍为 10.5ms → 瓶颈是固定开销，降分辨率无效 |
| 2 | `EpisodeBuffer` 存原始 RGB | 存 JPEG 字节串（q90），`add_frame` 同时接受 ndarray 与 bytes | 原始 RGB 2.76MB/帧；10s@50fps=**1.38GB**，30s=**4.1GB** |
| 3 | 新建 `MultiModalEpisodeBuffer` / `MultiModalLeRobotWriter` / `DataHubController` | 扩展既有 `EpisodeBuffer` / `LeRobotDatasetWriter` / `TuinaCollector`；仅新增 Hub 薄控制器 | 三套实现已存在且功能重叠（`collection/episode_buffer.py`、`dataset/tuinadex_dataset/lerobot_exporter.py`、`collection/collector.py`） |
| 4 | 显式声明 `timestamp`/`frame_index`/`episode_index`/`next.done` 为 dataset feature | 前三者由 LeRobot 自动管理，**不得重复声明**；`next.done` 仅存于缓存，导出时由 episode 边界派生 | `lerobot.datasets.utils.DEFAULT_FEATURES` 已含 `timestamp/frame_index/episode_index/index/task_index`；`next.done` 非标准字段（仅 `lerobot/rl/buffer.py` 内部使用） |
| 5 | Task 4 测试 import `DataHubController`，但 Files 清单无该文件 | 补入 `collection/data_hub_controller.py` 领空 | 排他文件领空缺项，Codex 会无授权创建 |
| 6 | Task 1 测试用 `pytest.skip` 跳过缺失场景 | 改为硬失败 | 违反 AGENTS.md §7 Auditor 反跳过门禁 |
| 7 | 场景相机名 `overhead_cam/wrist_cam/side_cam` 与规范名 `overhead/wrist/side` 混用 | 建立唯一映射表 `CameraSpec.mjcf_name` | 现场景已有 `overview`/`side_view`/`realsense_sim_cam`，`RobotConfig.camera_names` 默认 `("head",)` 与场景完全不对应 |
| 8 | Task 5 新建 writer，未提流式编码 | 在既有 writer 上启用 `streaming_encoding=True` | 已验证：3 路视频 + `observation.effort` + `observation.contact_forces` + `phase_index` 全部通过，60 帧导出成功 |
| 9 | Task 6 新建 `scripts/run_collection_hub.py` | 改为扩展 `scripts/collect_teleop_data.py`（`--hub`） | 该 CLI 已具备 `--env/--mode/--cameras/--repo-id/--mock-teleop`，平行入口属重复建设 |
| 10 | Hub 未定义绑定地址与写入权限 | 默认绑定 `127.0.0.1`；无任何直接关节指令端点 | 硬件安全红线：Hub 只能操作录制 FSM |
| 11 | `TuinaCollector.review_decision` 同步清空 buffer | 保存时**所有权转移**给后台保存线程 | 现有实现 `on_save(buffer)` 后立即 `clear()`，异步保存将读到空缓存 |

### 1.1 复现实测证据的命令

```bash
MUJOCO_GL=egl ~/win_office/conda/envs/tuinadex-control/bin/python - <<'PY'
import mujoco, time, statistics
xml = "src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml"
m = mujoco.MjModel.from_xml_path(xml); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
cams = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(m.ncam)]
r = mujoco.Renderer(m, height=480, width=640); ts = []
for _ in range(50):
    t = time.perf_counter()
    for c in cams[:3]:
        r.update_scene(d, camera=c); r.render()
    ts.append((time.perf_counter() - t) * 1000)
print("render3 median %.2fms p95 %.2fms" % (statistics.median(ts), sorted(ts)[47]))
PY
```

---

## 2. Global Constraints

- **Invariant 1/4**：全链路 SI 弧度；0 rad = 实测 `OPEN_POSE`；屈曲为正。state=真实位置，action=目标位置，动作空间恒为 absolute joint position。
- **Invariant 2/13**：仿真与真机共享同一 `BaseDataChannel` 契约，上层与导出层不感知后端。
- **Invariant 12**：任何下发电机位置必须经 `align_command()` 与安全限位钳制。
- **Invariant 23/24**：控制/仿真/遥操在 `tuinadex-control`（Python 3.10.20）；LeRobot 导出与 PyTorch 校验在 `tuinadex-vla`（Python 3.10.20 + CUDA 12.8）。
- **Invariant 27**：Web UI Hub 只用 Python 标准库 + 原生 HTML/CSS/JS，禁止 npm/node 构建链。
- **排他文件领空**：每个任务只改其 `Files:` 列出的文件；未列出的文件一律不碰。

---

## 3. 新增不变量（Phase 2 签发）

**I28 — 采集与控制解耦**
- 50Hz 控制回路内**不得**执行 `mujoco.Renderer.render()`。
- `CameraProducerThread` 独占 `mujoco.Renderer` 与私有 `MjData`；创建与 `close()` 必须在同一线程（EGL 上下文亲和）。
- 线程在锁内仅拷贝 `qpos/qvel`，锁外执行 `mj_forward` + 渲染 + JPEG 编码。
- 控制拍取"最新帧 + 其真实产出时刻"；`age > max_staleness_s`（默认 0.06s）计为丢帧，**禁止伪造时间戳或补齐不存在的图像**。
- 连续 3 拍丢帧（或单试次丢帧率 > 10%）→ 中止试次、清空缓存、状态回 `IDLE` 并记录 `last_abort_reason`。

**I29 — 图像存储契约**
- 内存中图像一律为 JPEG 字节串；`EpisodeBuffer.add_frame` 的 `images` 接受 `np.ndarray | bytes`，bytes 直存零拷贝。
- 单试次上限：`max_episode_bytes = 512MB` 或 `max_episode_frames = 3000`（50Hz 下 60s），先到先触发 `BufferCapacityError`。
- RGB→JPEG 必须显式 `COLOR_RGB2BGR`，解码必须显式 `COLOR_BGR2RGB`，并在测试中以纯色像素校验无通道交换。

**I30 — 特征字典契约**
- 数据集 feature：`observation.state`(22)、`observation.effort`(22)、`observation.contact_forces`(4)、`action`(22)、`phase_index`(int64[1])、`observation.images.{overhead,wrist,side}`(video)。
- `language_instruction` 落 LeRobot `task`（自动 `task_index` + `tasks.jsonl`）；`phase_index` 的字符串映射写 `meta/phase_labels.json`。
- `timestamp/frame_index/episode_index/index/task_index` **不得**作为自定义 feature 声明。
- `next.done` 仅存在于 `EpisodeBuffer`；导出依赖 LeRobot 的 episode 边界，测试断言每个 episode 末帧 `next_done=True`。

**I31 — 复用优先**
- 不得新建与 `EpisodeBuffer` / `LeRobotDatasetWriter` / `TuinaCollector` 职责重叠的类。
- 允许新增：`cameras/`、`hub/`、`collection/data_channel.py`、`collection/data_hub_controller.py`。

**I32 — Hub 安全边界**
- 默认绑定 `127.0.0.1`；绑定 `0.0.0.0` 必须显式 `--host 0.0.0.0` 且打印 WARNING。
- Hub 不提供任何关节目标下发端点；`POST /api/action/*` 只能驱动录制 FSM。
- HTTP 线程只调用 `submit()`/`status()`，不得直接修改缓冲区或 FSM；命令在控制拍边界由 `tick()` 串行消费。
- `save` 后的落盘在独立保存线程执行，绝不阻塞控制回路。

**I33 — 推流契约**
- MJPEG/SSE 只读取共享槽位；每帧只编码一次，供所有客户端复用。
- SSE 推送频率 `ui_fps = 25Hz`（控制 50Hz 的整数抽帧）；MJPEG 跟随相机产出频率。
- 流式响应必须 `Connection: close`，REST 响应必须带 `Content-Length`。

**I34 — 场景单一事实源**
- `src/tuinadex_sim/worlds/tuina_scene.xml` 与 `src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml` 必须逐字节一致，由测试守护。
- 新增相机必须同时写入两份；既有 `overview/side_view/realsense_sim_cam` 保留不动。

**I35 — 测试真实性**
- 禁止 `pytest.skip` 跳过缺失场景/依赖；禁止 mock 渲染结果。
- E2E 必须真实走 MuJoCo EGL 离屏渲染 + 真实 H.264 编码。

**I36 — 相机命名契约**
- 规范名（对外）`overhead` / `wrist` / `side` ↔ MJCF 名 `overhead_cam` / `wrist_cam` / `side_cam`，映射唯一定义在 `STANDARD_CAMERA_SPECS`。
- `RobotConfig.camera_names` 默认值改为 `("overhead", "wrist", "side")`。

---

## 4. 文件结构与原子领空

| 工单 | 独占文件 |
|---|---|
| W1 | `src/tuinadex_teleop/tuinadex_teleop/cameras/{__init__,camera_specs,sim_camera_rig}.py`、`src/tuinadex_sim/worlds/tuina_scene.xml`、`src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml`、`src/tuinadex_teleop/tuinadex_teleop/envs/config.py`、`tests/test_camera_rig.py` |
| W2 | `src/tuinadex_teleop/tuinadex_teleop/collection/episode_buffer.py`、`collection/__init__.py`、`tests/test_episode_buffer.py` |
| W3 | `src/tuinadex_teleop/tuinadex_teleop/collection/data_channel.py`、`tests/test_data_channel.py` |
| W4 | `src/tuinadex_teleop/tuinadex_teleop/collection/collector.py`、`collection/data_hub_controller.py`、`tests/test_collector_fsm.py`、`tests/test_data_hub_controller.py` |
| W5 | `src/tuinadex_teleop/tuinadex_teleop/hub/{__init__,server}.py`、`hub/static/{index.html,app.js,style.css}`、`src/tuinadex_teleop/setup.py`、`tests/test_collection_hub_api.py` |
| W6 | `dataset/tuinadex_dataset/lerobot_exporter.py`、`tests/test_lerobot_v3_exporter.py` |
| W7 | `scripts/collect_teleop_data.py`、`scripts/e2e_hub_collection_check.sh`、`AGENTS.md`、`README.md` |

依赖顺序：W1 → W2 → W3 → W4 → W5 → W6 → W7。W6 仅依赖 W2，可与 W4/W5 并行（若 Codex 支持双工位）。

---

## 5. Task 1（W1）：三机位 MJCF + 相机规格 + 独立渲染线程

**Files:**
- Modify: `src/tuinadex_sim/worlds/tuina_scene.xml`（`<worldbody>` 内新增固定机位；`<body name="leap_hand_palm">` 内新增腕部机位）
- Modify: `src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml`（与上者逐字节一致）
- Modify: `src/tuinadex_teleop/tuinadex_teleop/envs/config.py:18,23,30`
- Create: `src/tuinadex_teleop/tuinadex_teleop/cameras/__init__.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/cameras/camera_specs.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/cameras/sim_camera_rig.py`
- Test: `tests/test_camera_rig.py`

**Interfaces:**
- Produces:
  - `CameraMountType(Enum)`: `EYE_TO_HAND`, `EYE_IN_HAND`
  - `CameraSpec(name, mjcf_name, mount_type, width=640, height=480, fovy)`（frozen dataclass）
  - `STANDARD_CAMERA_SPECS: dict[str, CameraSpec]`，键为 `overhead` / `wrist` / `side`
  - `SimCameraRig(model, data, specs=STANDARD_CAMERA_SPECS, jpeg_quality=90)`：`render_camera(name) -> (480,640,3) uint8 RGB`、`render_all() -> dict[str, np.ndarray]`、`encode_all() -> dict[str, bytes]`、`close()`
  - `encode_rgb_to_jpeg(image, quality=90) -> bytes`、`decode_jpeg_to_rgb(payload) -> np.ndarray`
  - `CameraProducerThread(model, data, data_lock, specs=STANDARD_CAMERA_SPECS, target_fps=50.0, jpeg_quality=90)`：`start()`、`wait_ready(timeout) -> bool`、`latest() -> dict[str, tuple[bytes, float]]`、`stats() -> dict`、`stop()`

- [ ] **Step 1: 写失败测试**

```python
# tests/test_camera_rig.py
"""三机位规格、MJCF 相机与独立渲染线程测试 (需 EGL 离屏渲染)."""
import os
import threading
import time
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("MUJOCO_GL", "egl")
mujoco = pytest.importorskip("mujoco")

from tuinadex_teleop.cameras.camera_specs import (
    CameraMountType,
    STANDARD_CAMERA_SPECS,
)
from tuinadex_teleop.cameras.sim_camera_rig import CameraProducerThread, SimCameraRig

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml"
SCENE_COPY = ROOT / "src/tuinadex_sim/worlds/tuina_scene.xml"


def _load_scene():
    assert SCENE.is_file(), f"场景缺失: {SCENE}"  # I35: 硬失败，不 skip
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_camera_specs_are_standard():
    assert set(STANDARD_CAMERA_SPECS) == {"overhead", "wrist", "side"}
    expected = {
        "overhead": ("overhead_cam", CameraMountType.EYE_TO_HAND, 60.0),
        "wrist": ("wrist_cam", CameraMountType.EYE_IN_HAND, 75.0),
        "side": ("side_cam", CameraMountType.EYE_TO_HAND, 45.0),
    }
    for name, (mjcf_name, mount, fovy) in expected.items():
        spec = STANDARD_CAMERA_SPECS[name]
        assert spec.mjcf_name == mjcf_name
        assert spec.mount_type is mount
        assert spec.fovy == fovy
        assert (spec.width, spec.height) == (640, 480)


def test_scene_contains_standard_cameras_with_matching_fovy():
    model, _ = _load_scene()
    for spec in STANDARD_CAMERA_SPECS.values():
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, spec.mjcf_name)
        assert cid >= 0, f"MJCF 缺少相机 {spec.mjcf_name}"
        assert model.cam_fovy[cid] == pytest.approx(spec.fovy, abs=1.0)


def test_scene_copies_are_byte_identical():
    assert SCENE.read_bytes() == SCENE_COPY.read_bytes(), "两份场景文件必须逐字节一致 (I34)"


def test_sim_camera_rig_renders_three_non_blank_views():
    model, data = _load_scene()
    rig = SimCameraRig(model, data)
    try:
        frames = rig.render_all()
        assert set(frames) == {"overhead", "wrist", "side"}
        for cam, img in frames.items():
            assert img.shape == (480, 640, 3), cam
            assert img.dtype == np.uint8
            assert img.mean() > 5.0, f"{cam} 渲染为空白"
        encoded = rig.encode_all()
        assert set(encoded) == {"overhead", "wrist", "side"}
        for cam, payload in encoded.items():
            assert isinstance(payload, bytes) and len(payload) > 1000, cam
    finally:
        rig.close()


def test_camera_producer_thread_runs_off_control_loop():
    model, data = _load_scene()
    lock = threading.Lock()
    producer = CameraProducerThread(model, data, lock, target_fps=25.0)
    producer.start()
    try:
        assert producer.wait_ready(timeout=30.0), "渲染线程未能建立 EGL 上下文"
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with lock:
                mujoco.mj_step(model, data)
            time.sleep(0.02)
        latest = producer.latest()
        assert set(latest) == {"overhead", "wrist", "side"}
        for cam, (payload, ts) in latest.items():
            assert len(payload) > 1000, cam
            assert ts <= time.monotonic()
        assert producer.stats()["produced"] >= 3
    finally:
        producer.stop()
    assert not producer.is_alive()
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_camera_rig.py -v`
Expected: FAIL，`ModuleNotFoundError: No module named 'tuinadex_teleop.cameras'`

- [ ] **Step 3: 写入三处相机定义（两份场景 XML 保持一致）**

在 `<worldbody>` 内、既有 `<camera>` 三行之后追加：

```xml
    <camera name="overhead_cam" pos="0.20 -0.85 1.45" xyaxes="1 0 0 0 0.766 0.643" fovy="60"/>
    <camera name="side_cam" pos="0.20 -1.35 0.78" xyaxes="1 0 0 0 0 1" fovy="45"/>
```

在 `<body name="leap_hand_palm">` 的首个子元素位置插入腕部机位（局部坐标，镜头朝指尖方向）：

```xml
      <camera name="wrist_cam" pos="0 -0.03 0.04" xyaxes="1 0 0 0 0.7071 0.7071" fovy="75"/>
```

同时消除两份文件之间唯一的差异（第 264 行附近一处空行），保证 `cmp` 返回 0。

> 腕部机位姿态为经验初值。若 Step 6 渲染朝错，**只允许调整该 `<camera>` 的 `pos`/`xyaxes`**，禁止改动任何几何或关节。

- [ ] **Step 4: 实现 `camera_specs.py`**

```python
"""camera_specs.py — 标准三机位光学与挂载规范 (唯一映射事实源)."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class CameraMountType(Enum):
    """相机挂载拓扑类型."""
    EYE_TO_HAND = auto()
    EYE_IN_HAND = auto()


@dataclass(frozen=True)
class CameraSpec:
    """单机位光学规格与 MJCF 名称映射."""
    name: str
    mjcf_name: str
    mount_type: CameraMountType
    width: int = 640
    height: int = 480
    fovy: float = 60.0


STANDARD_CAMERA_SPECS: dict[str, CameraSpec] = {
    "overhead": CameraSpec(
        name="overhead", mjcf_name="overhead_cam",
        mount_type=CameraMountType.EYE_TO_HAND, fovy=60.0,
    ),
    "wrist": CameraSpec(
        name="wrist", mjcf_name="wrist_cam",
        mount_type=CameraMountType.EYE_IN_HAND, fovy=75.0,
    ),
    "side": CameraSpec(
        name="side", mjcf_name="side_cam",
        mount_type=CameraMountType.EYE_TO_HAND, fovy=45.0,
    ),
}
```

- [ ] **Step 5: 实现 `sim_camera_rig.py`**

```python
"""sim_camera_rig.py — MuJoCo 离屏三机位渲染与独立采集线程."""
from __future__ import annotations

import logging
import threading
import time
from typing import Any

import cv2
import mujoco
import numpy as np

from .camera_specs import STANDARD_CAMERA_SPECS, CameraSpec

logger = logging.getLogger(__name__)


def encode_rgb_to_jpeg(image: np.ndarray, quality: int = 90) -> bytes:
    """RGB uint8 (H,W,3) -> JPEG bytes (显式 RGB->BGR 转换)."""
    arr = np.asarray(image)
    if arr.dtype != np.uint8 or arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f"expect uint8 (H,W,3), got {arr.dtype} {arr.shape}")
    ok, buf = cv2.imencode(
        ".jpg", cv2.cvtColor(arr, cv2.COLOR_RGB2BGR),
        [cv2.IMWRITE_JPEG_QUALITY, int(quality)],
    )
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return buf.tobytes()


def decode_jpeg_to_rgb(payload: bytes) -> np.ndarray:
    """JPEG bytes -> RGB uint8 (H,W,3)."""
    bgr = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("corrupted JPEG payload")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


class SimCameraRig:
    """同步离屏渲染器：一个 EGL Renderer 服务全部机位.

    创建与 close() 必须在同一线程；请勿在控制回路中直接使用。
    """

    def __init__(
        self,
        model: Any,
        data: Any,
        specs: dict[str, CameraSpec] | None = None,
        jpeg_quality: int = 90,
    ):
        self.model = model
        self.data = data
        self.specs = dict(specs or STANDARD_CAMERA_SPECS)
        self.jpeg_quality = int(jpeg_quality)
        sizes = {(s.width, s.height) for s in self.specs.values()}
        if len(sizes) != 1:
            raise ValueError(f"all camera specs must share one resolution, got {sizes}")
        self.width, self.height = sizes.pop()
        self._renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        self._closed = False

    def render_camera(self, name: str) -> np.ndarray:
        if self._closed:
            raise RuntimeError("SimCameraRig already closed")
        spec = self.specs[name]
        self._renderer.update_scene(self.data, camera=spec.mjcf_name)
        return self._renderer.render()

    def render_all(self) -> dict[str, np.ndarray]:
        return {name: self.render_camera(name) for name in self.specs}

    def encode_all(self) -> dict[str, bytes]:
        return {
            name: encode_rgb_to_jpeg(self.render_camera(name), self.jpeg_quality)
            for name in self.specs
        }

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._renderer.close()


class CameraProducerThread(threading.Thread):
    """独立相机采集线程：独占 Renderer + 私有 MjData (I28)."""

    def __init__(
        self,
        model: Any,
        data: Any,
        data_lock: threading.Lock,
        specs: dict[str, CameraSpec] | None = None,
        target_fps: float = 50.0,
        jpeg_quality: int = 90,
    ):
        super().__init__(name="camera-producer", daemon=True)
        if target_fps <= 0:
            raise ValueError("target_fps must be > 0")
        self.model = model
        self.data = data
        self._data_lock = data_lock
        self.specs = dict(specs or STANDARD_CAMERA_SPECS)
        self.target_fps = float(target_fps)
        self.jpeg_quality = int(jpeg_quality)
        self._render_data = mujoco.MjData(model)
        self._ready = threading.Event()
        self._stop_evt = threading.Event()
        self._cond = threading.Condition()
        self._latest: dict[str, tuple[bytes, float]] = {}
        self._stats = {
            "produced": 0,
            "errors": 0,
            "achieved_fps": 0.0,
            "last_error": None,
        }

    def wait_ready(self, timeout: float = 30.0) -> bool:
        return self._ready.wait(timeout)

    def latest(self) -> dict[str, tuple[bytes, float]]:
        with self._cond:
            return dict(self._latest)

    def stats(self) -> dict:
        with self._cond:
            return dict(self._stats)

    def run(self) -> None:
        rig = None
        try:
            rig = SimCameraRig(self.model, self._render_data, self.specs, self.jpeg_quality)
            self._ready.set()
            period = 1.0 / self.target_fps
            t_start = time.monotonic()
            n = 0
            while not self._stop_evt.is_set():
                n += 1
                next_t = t_start + n * period
                with self._data_lock:
                    np.copyto(self._render_data.qpos, self.data.qpos)
                    np.copyto(self._render_data.qvel, self.data.qvel)
                mujoco.mj_forward(self.model, self._render_data)
                payload = rig.encode_all()
                now = time.monotonic()
                with self._cond:
                    self._latest = {cam: (buf, now) for cam, buf in payload.items()}
                    self._stats["produced"] += 1
                    elapsed = now - t_start
                    self._stats["achieved_fps"] = (
                        round(self._stats["produced"] / elapsed, 2) if elapsed > 0 else 0.0
                    )
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0:
                    self._stop_evt.wait(sleep_s)
        except Exception as exc:  # noqa: BLE001 - 线程内异常必须外显
            with self._cond:
                self._stats["errors"] += 1
                self._stats["last_error"] = f"{type(exc).__name__}: {exc}"
            logger.exception("CameraProducerThread failed")
        finally:
            if rig is not None:
                rig.close()
            self._ready.set()  # 让等待方解除阻塞

    def stop(self, timeout: float = 5.0) -> None:
        self._stop_evt.set()
        with self._cond:
            self._cond.notify_all()
        self.join(timeout)
        if self.is_alive():
            logger.error("CameraProducerThread did not stop within %.1fs", timeout)
```

- [ ] **Step 6: 创建 `cameras/__init__.py` 并跑通**

```python
"""tuinadex_teleop.cameras — 标准三机位规格与仿真采集."""
from .camera_specs import STANDARD_CAMERA_SPECS, CameraMountType, CameraSpec
from .sim_camera_rig import (
    CameraProducerThread,
    SimCameraRig,
    decode_jpeg_to_rgb,
    encode_rgb_to_jpeg,
)

__all__ = [
    "CameraSpec",
    "CameraMountType",
    "STANDARD_CAMERA_SPECS",
    "SimCameraRig",
    "CameraProducerThread",
    "encode_rgb_to_jpeg",
    "decode_jpeg_to_rgb",
]
```

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_camera_rig.py -v`
Expected: PASS (5/5)

**人工视觉门禁（Phase 4 前必须完成）**：将三路首帧存盘供人工确认腕部机位朝向。

```bash
MUJOCO_GL=egl ~/win_office/conda/envs/tuinadex-control/bin/python - <<'PY'
import mujoco, cv2
from tuinadex_teleop.cameras.sim_camera_rig import SimCameraRig
m = mujoco.MjModel.from_xml_path("src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml")
d = mujoco.MjData(m); mujoco.mj_forward(m, d)
rig = SimCameraRig(m, d)
for k, v in rig.render_all().items():
    cv2.imwrite(f"/tmp/preview_{k}.png", cv2.cvtColor(v, cv2.COLOR_RGB2BGR))
rig.close()
PY
```

- [ ] **Step 7: 修改 `RobotConfig` 默认相机名**

`src/tuinadex_teleop/tuinadex_teleop/envs/config.py` 中三处 `("head",)` 全部替换为 `("overhead", "wrist", "side")`（第 18 行 docstring、第 23 行字段默认值、第 30 行 `__init__` 默认参数）。

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_env_factory.py tests/test_robot_env_spec.py tests/test_camera_rig.py -v`
Expected: PASS（既有用例均显式传入 `camera_names`，不受默认值变更影响）

- [ ] **Step 8: 提交**

```bash
git add src/tuinadex_sim/worlds/tuina_scene.xml \
        src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml \
        src/tuinadex_teleop/tuinadex_teleop/cameras \
        src/tuinadex_teleop/tuinadex_teleop/envs/config.py \
        tests/test_camera_rig.py
git commit -m "feat(cameras): add standard 3-camera rig with dedicated EGL render thread"
```

---

## 6. Task 2（W2）：EpisodeBuffer 扩展（22D 力矩/4 指尖力/元数据 + JPEG 存储）

**Files:**
- Modify（整体重写，保留 `to_dict()` 旧键）: `src/tuinadex_teleop/tuinadex_teleop/collection/episode_buffer.py`
- Modify: `src/tuinadex_teleop/tuinadex_teleop/collection/__init__.py`
- Test（追加，不改既有断言）: `tests/test_episode_buffer.py`

**Interfaces:**
- Consumes: `encode_rgb_to_jpeg` / `decode_jpeg_to_rgb`（Task 1）
- Produces:
  - `BufferCapacityError(RuntimeError)`
  - `FrameData(state, action, effort, contact_forces, images_jpeg, image_timestamps, timestamp, frame_index, episode_index, next_done, phase_label, language_instruction)`，方法 `decode_image(cam) -> np.ndarray`
  - `EpisodeBuffer(episode_index=0, language_instruction="", max_episode_bytes=512*1024*1024, max_episode_frames=3000)`
  - `add_frame(state, action, images, timestamp, *, effort=None, contact_forces=None, image_timestamps=None, phase_label="", language_instruction=None)`
  - `seal_episode()`、`validate_continuity(max_gap_s=0.1) -> list[str]`、`to_dict()`、`clear()`、`iter_frames()`、`image_bytes` 属性

- [ ] **Step 1: 追加失败测试**

```python
# tests/test_episode_buffer.py 追加
import numpy as np
import pytest

from tuinadex_teleop.collection.episode_buffer import (
    BufferCapacityError,
    EpisodeBuffer,
    FrameData,
)
from tuinadex_teleop.cameras.sim_camera_rig import encode_rgb_to_jpeg


def _frame_kwargs(i: int, cam="overhead"):
    return dict(
        state=np.full(22, i * 0.01, dtype=np.float32),
        action=np.full(22, i * 0.02, dtype=np.float32),
        effort=np.full(22, i * 0.1, dtype=np.float32),
        contact_forces=np.array([i, i + 1, i + 2, i + 3], dtype=np.float32),
        images={cam: np.full((48, 64, 3), i, dtype=np.uint8)},
        timestamp=float(i) * 0.02,
        phase_label="press" if i < 3 else "release",
    )


def test_multi_modal_buffer_metadata_and_derived_done():
    buf = EpisodeBuffer(episode_index=7, language_instruction="按压大椎穴")
    for i in range(5):
        buf.add_frame(**_frame_kwargs(i))
    buf.seal_episode()

    data = buf.to_dict()
    assert data["state"].shape == (5, 22)
    assert data["effort"].shape == (5, 22)
    assert data["contact_forces"].shape == (5, 4)
    assert data["frame_index"].tolist() == [0, 1, 2, 3, 4]
    assert (data["episode_index"] == 7).all()
    assert data["next_done"].tolist() == [False, False, False, False, True]
    assert data["phase_label"].tolist() == ["press", "press", "press", "release", "release"]
    assert data["language_instruction"] == "按压大椎穴"
    # 旧键向后兼容
    assert data["action"].shape == (5, 22)
    assert data["images"]["overhead"].shape == (5, 48, 64, 3)


def test_images_stored_as_jpeg_and_roundtrip_rgb():
    buf = EpisodeBuffer()
    red_rgb = np.zeros((16, 16, 3), dtype=np.uint8)
    red_rgb[..., 0] = 255
    buf.add_frame(
        state=np.zeros(22, np.float32), action=np.zeros(22, np.float32),
        images={"overhead": red_rgb}, timestamp=0.0,
    )
    frame = buf.iter_frames().__next__()
    assert isinstance(frame.images_jpeg["overhead"], bytes)
    decoded = frame.decode_image("overhead")
    assert decoded.shape == (16, 16, 3)
    assert decoded[..., 0].mean() > 240 and decoded[..., 1].mean() < 20  # 无通道交换


def test_add_frame_accepts_preencoded_bytes_without_reencoding():
    payload = encode_rgb_to_jpeg(np.full((16, 16, 3), 7, dtype=np.uint8))
    buf = EpisodeBuffer()
    buf.add_frame(
        state=np.zeros(22, np.float32), action=np.zeros(22, np.float32),
        images={"overhead": payload}, timestamp=0.0,
    )
    assert buf.iter_frames().__next__().images_jpeg["overhead"] == payload


def test_buffer_capacity_guard():
    buf = EpisodeBuffer(max_episode_frames=2, max_episode_bytes=10)
    buf.add_frame(**_frame_kwargs(0))
    buf.add_frame(**_frame_kwargs(1))
    with pytest.raises(BufferCapacityError, match="frame limit"):
        buf.add_frame(**_frame_kwargs(2))


def test_validate_continuity_reports_gaps():
    buf = EpisodeBuffer()
    for i in range(3):
        buf.add_frame(**_frame_kwargs(i))
    assert buf.validate_continuity(max_gap_s=0.1) == []
    buf._frames[2] = FrameData(**{**buf._frames[2].__dict__, "timestamp": 5.0})
    assert any("gap" in msg for msg in buf.validate_continuity(max_gap_s=0.1))


def test_dimension_mismatch_raises():
    buf = EpisodeBuffer()
    with pytest.raises(ValueError, match="effort must match"):
        buf.add_frame(
            state=np.zeros(22, np.float32), action=np.zeros(22, np.float32),
            effort=np.zeros(16, np.float32), images={}, timestamp=0.0,
        )
    with pytest.raises(ValueError, match="contact_forces must be"):
        buf.add_frame(
            state=np.zeros(22, np.float32), action=np.zeros(22, np.float32),
            contact_forces=np.zeros(3, np.float32), images={}, timestamp=0.0,
        )
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_episode_buffer.py -v`
Expected: FAIL，`ImportError: cannot import name 'BufferCapacityError'`

- [ ] **Step 3: 重写 `episode_buffer.py`**

```python
"""episode_buffer.py — 多模态试次内存暂存缓存 (JPEG 压缩存储)."""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Iterator

import numpy as np

from ..cameras.sim_camera_rig import decode_jpeg_to_rgb, encode_rgb_to_jpeg

DEFAULT_MAX_EPISODE_BYTES = 512 * 1024 * 1024
DEFAULT_MAX_EPISODE_FRAMES = 3000


class BufferCapacityError(RuntimeError):
    """单试次缓存超过内存上限 (I29)."""


@dataclass
class FrameData:
    """单个控制周期的多模态采样帧."""
    state: np.ndarray
    action: np.ndarray
    effort: np.ndarray
    contact_forces: np.ndarray
    images_jpeg: dict[str, bytes]
    image_timestamps: dict[str, float]
    timestamp: float
    frame_index: int
    episode_index: int
    next_done: bool
    phase_label: str
    language_instruction: str

    def decode_image(self, cam: str) -> np.ndarray:
        """将指定机位的 JPEG 负载解码为 RGB (H,W,3) uint8."""
        return decode_jpeg_to_rgb(self.images_jpeg[cam])


class EpisodeBuffer:
    """单个推拿试次的多模态内存缓存.

    图像以 JPEG 字节串存储；`add_frame` 同时接受 ndarray 与已编码 bytes。
    """

    def __init__(
        self,
        episode_index: int = 0,
        language_instruction: str = "",
        max_episode_bytes: int = DEFAULT_MAX_EPISODE_BYTES,
        max_episode_frames: int = DEFAULT_MAX_EPISODE_FRAMES,
    ):
        self.episode_index = int(episode_index)
        self.language_instruction = str(language_instruction)
        self.max_episode_bytes = int(max_episode_bytes)
        self.max_episode_frames = int(max_episode_frames)
        self._frames: list[FrameData] = []
        self._image_bytes = 0

    def __len__(self) -> int:
        return len(self._frames)

    @property
    def image_bytes(self) -> int:
        return self._image_bytes

    @property
    def frames(self) -> list[FrameData]:
        return self._frames

    def iter_frames(self) -> Iterator[FrameData]:
        return iter(self._frames)

    def add_frame(
        self,
        state: np.ndarray | list[float],
        action: np.ndarray | list[float],
        images: dict[str, np.ndarray | bytes],
        timestamp: float,
        *,
        effort: np.ndarray | list[float] | None = None,
        contact_forces: np.ndarray | list[float] | None = None,
        image_timestamps: dict[str, float] | None = None,
        phase_label: str = "",
        language_instruction: str | None = None,
    ) -> None:
        s = np.asarray(state, dtype=np.float32).ravel()
        a = np.asarray(action, dtype=np.float32).ravel()
        if s.shape != a.shape:
            raise ValueError(
                f"state and action must have same dimension, got state={s.shape}, action={a.shape}"
            )
        e = (
            np.zeros_like(s, dtype=np.float32)
            if effort is None
            else np.asarray(effort, dtype=np.float32).ravel()
        )
        if e.shape != s.shape:
            raise ValueError(f"effort must match state dim {s.shape}, got {e.shape}")
        f = (
            np.zeros(4, dtype=np.float32)
            if contact_forces is None
            else np.asarray(contact_forces, dtype=np.float32).ravel()
        )
        if f.shape != (4,):
            raise ValueError(f"contact_forces must be (4,), got {f.shape}")

        jpeg: dict[str, bytes] = {}
        for cam, payload in images.items():
            if isinstance(payload, (bytes, bytearray)):
                jpeg[cam] = bytes(payload)
            else:
                jpeg[cam] = encode_rgb_to_jpeg(payload)

        new_bytes = sum(len(v) for v in jpeg.values())
        if len(self._frames) >= self.max_episode_frames:
            raise BufferCapacityError(f"frame limit reached: {self.max_episode_frames} frames")
        if self._image_bytes + new_bytes > self.max_episode_bytes:
            raise BufferCapacityError(
                f"byte limit reached: {self._image_bytes + new_bytes} > {self.max_episode_bytes}"
            )

        ts0 = float(timestamp)
        ts_map = {cam: ts0 for cam in jpeg}
        if image_timestamps:
            ts_map.update({k: float(v) for k, v in image_timestamps.items() if k in jpeg})

        self._frames.append(
            FrameData(
                state=s,
                action=a,
                effort=e,
                contact_forces=f,
                images_jpeg=jpeg,
                image_timestamps=ts_map,
                timestamp=ts0,
                frame_index=len(self._frames),
                episode_index=self.episode_index,
                next_done=False,
                phase_label=str(phase_label),
                language_instruction=(
                    self.language_instruction
                    if language_instruction is None
                    else str(language_instruction)
                ),
            )
        )
        self._image_bytes += new_bytes

    def seal_episode(self) -> None:
        """标记末帧 next_done=True (I30)."""
        if not self._frames:
            raise ValueError("Cannot seal an empty episode")
        self._frames[-1] = replace(self._frames[-1], next_done=True)

    def validate_continuity(self, max_gap_s: float = 0.1) -> list[str]:
        """返回时间戳/索引/数值连续性缺陷列表；空列表表示通过."""
        issues: list[str] = []
        for i, frame in enumerate(self._frames):
            if frame.frame_index != i:
                issues.append(f"frame_index {frame.frame_index} != position {i}")
            if not np.isfinite(frame.state).all():
                issues.append(f"non-finite state at frame {i}")
            if i > 0:
                prev = self._frames[i - 1].timestamp
                if frame.timestamp <= prev:
                    issues.append(
                        f"non-monotonic timestamp at frame {i}: {frame.timestamp} <= {prev}"
                    )
                elif frame.timestamp - prev > max_gap_s:
                    issues.append(
                        f"gap {frame.timestamp - prev:.3f}s > {max_gap_s}s at frame {i}"
                    )
        return issues

    def clear(self) -> None:
        self._frames.clear()
        self._image_bytes = 0

    def to_dict(self) -> dict[str, Any]:
        """批量聚合为 NumPy 数组字典 (含旧版兼容键)."""
        if not self._frames:
            return {
                "state": np.empty((0, 0), dtype=np.float32),
                "action": np.empty((0, 0), dtype=np.float32),
                "effort": np.empty((0, 0), dtype=np.float32),
                "contact_forces": np.empty((0, 4), dtype=np.float32),
                "timestamp": np.empty((0,), dtype=np.float32),
                "frame_index": np.empty((0,), dtype=np.int64),
                "episode_index": np.empty((0,), dtype=np.int64),
                "next_done": np.empty((0,), dtype=bool),
                "phase_label": [],
                "language_instruction": self.language_instruction,
                "images": {},
            }

        cams = list(self._frames[0].images_jpeg)
        return {
            "state": np.stack([f.state for f in self._frames]),
            "action": np.stack([f.action for f in self._frames]),
            "effort": np.stack([f.effort for f in self._frames]),
            "contact_forces": np.stack([f.contact_forces for f in self._frames]),
            "timestamp": np.array([f.timestamp for f in self._frames], dtype=np.float32),
            "frame_index": np.array([f.frame_index for f in self._frames], dtype=np.int64),
            "episode_index": np.array([f.episode_index for f in self._frames], dtype=np.int64),
            "next_done": np.array([f.next_done for f in self._frames], dtype=bool),
            "phase_label": [f.phase_label for f in self._frames],
            "language_instruction": self.language_instruction,
            "images": {
                cam: np.stack([f.decode_image(cam) for f in self._frames]) for cam in cams
            },
        }
```

> `to_dict()` 会一次性解码全部图像，仅用于调试与小样本测试；导出必须走 `iter_frames()` 逐帧解码。

- [ ] **Step 4: 更新 `collection/__init__.py`**

```python
"""tuinadex_teleop.collection — 数据采集会话编排与试次缓存."""
from .episode_buffer import BufferCapacityError, EpisodeBuffer, FrameData
from .collector import TuinaCollector, CollectionState

__all__ = [
    "EpisodeBuffer",
    "FrameData",
    "BufferCapacityError",
    "TuinaCollector",
    "CollectionState",
]
```

- [ ] **Step 5: 跑通**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_episode_buffer.py tests/test_collector_fsm.py -v`
Expected: PASS (8/8)

- [ ] **Step 6: 提交**

```bash
git add src/tuinadex_teleop/tuinadex_teleop/collection/episode_buffer.py \
        src/tuinadex_teleop/tuinadex_teleop/collection/__init__.py \
        tests/test_episode_buffer.py
git commit -m "feat(collection): extend EpisodeBuffer with effort, contact forces, metadata and JPEG storage"
```

---

## 7. Task 3（W3）：双通道数据源抽象（仿真 / 真机零成本对齐）

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/collection/data_channel.py`
- Modify: `src/tuinadex_teleop/tuinadex_teleop/envs/ros2_robot_env.py`（仅补 effort 缓存）
- Test: `tests/test_data_channel.py`

**Interfaces:**
- Consumes: `MujocoSimAdapter.read_observation()/get_contact_forces()`、`CameraProducerThread.latest()`、`ROS2RobotEnv.get_observation()`
- Produces:
  - `CameraPacket(jpeg: dict[str, bytes], timestamps: dict[str, float])`，方法 `ages(now=None) -> dict[str, float]`、`worst_age(now=None) -> float`
  - `BaseDataChannel(ABC)`：`camera_names`、`get_camera_packets()`、`get_proprioception() -> (q, effort, forces)`、`get_action_label()`、`get_camera_frames()`、`close()`
  - `SimDataChannel(adapter, producer, action_supplier, camera_names)`
  - `RealDataChannel(env, camera_names, action_supplier)`
  - `MockDataChannel(camera_names, shape=(480, 640, 3), dof=22)`

- [ ] **Step 1: 写失败测试**

```python
# tests/test_data_channel.py
"""双通道数据源契约测试."""
import time

import numpy as np
import pytest

from tuinadex_teleop.collection.data_channel import (
    BaseDataChannel,
    CameraPacket,
    MockDataChannel,
)


def test_mock_channel_contract():
    channel = MockDataChannel()
    try:
        packets = channel.get_camera_packets()
        assert set(packets.jpeg) == {"overhead", "wrist", "side"}
        assert all(isinstance(v, bytes) and len(v) > 100 for v in packets.jpeg.values())
        assert packets.worst_age() < 1.0

        frames = channel.get_camera_frames()
        assert frames["overhead"].shape == (480, 640, 3)
        assert frames["overhead"].dtype == np.uint8

        q, effort, forces = channel.get_proprioception()
        assert q.shape == (22,) and effort.shape == (22,) and forces.shape == (4,)

        assert channel.get_action_label().shape == (22,)
    finally:
        channel.close()


def test_camera_packet_age_semantics():
    now = time.monotonic()
    packet = CameraPacket(
        jpeg={"overhead": b"x", "side": b"y"},
        timestamps={"overhead": now - 0.02, "side": now - 0.5},
    )
    ages = packet.ages(now)
    assert ages["overhead"] == pytest.approx(0.02, abs=1e-6)
    assert packet.worst_age(now) == pytest.approx(0.5, abs=1e-6)
    assert CameraPacket(jpeg={}, timestamps={}).worst_age(now) == float("inf")


def test_base_channel_is_abstract():
    with pytest.raises(TypeError):
        BaseDataChannel()


def test_sim_channel_reads_adapter_and_producer():
    """真实 MujocoSimAdapter + 真实渲染线程，验证 22D/4 力/3 图契约 (不 mock)."""
    import threading
    from pathlib import Path
    from tuinadex_teleop.adapters.mujoco_sim_adapter import MujocoSimAdapter
    from tuinadex_teleop.cameras.sim_camera_rig import CameraProducerThread
    from tuinadex_teleop.collection.data_channel import SimDataChannel

    root = Path(__file__).resolve().parents[1]
    model_path = root / "src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml"
    assert model_path.is_file()
    adapter = MujocoSimAdapter(model_path=model_path, headless=True)
    assert adapter.connect()
    lock = threading.Lock()
    producer = CameraProducerThread(adapter.model, adapter.data, lock, target_fps=20.0)
    producer.start()
    try:
        assert producer.wait_ready(30.0)
        for _ in range(5):
            with lock:
                adapter.step(1)
            time.sleep(0.03)
        channel = SimDataChannel(
            adapter, producer,
            action_supplier=lambda: np.zeros(22, dtype=np.float32),
        )
        packets = channel.get_camera_packets()
        assert set(packets.jpeg) == {"overhead", "wrist", "side"}
        q, effort, forces = channel.get_proprioception()
        assert q.shape == (22,) and effort.shape == (22,) and forces.shape == (4,)
        assert np.all(np.isfinite(q))
        assert channel.get_action_label().shape == (22,)
        channel.close()
    finally:
        producer.stop()
        adapter.disconnect()
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_data_channel.py -v`
Expected: FAIL，`ModuleNotFoundError: No module named 'tuinadex_teleop.collection.data_channel'`

- [ ] **Step 3: 实现 `data_channel.py`**

```python
"""data_channel.py — 仿真/真机统一数据源抽象 (I28/I29 契约)."""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable

import numpy as np

from ..cameras.sim_camera_rig import decode_jpeg_to_rgb, encode_rgb_to_jpeg

DEFAULT_CAMERAS = ("overhead", "wrist", "side")


@dataclass(frozen=True)
class CameraPacket:
    """一次相机采集快照：JPEG 负载 + 本地单调时钟首次观测时刻."""
    jpeg: dict[str, bytes]
    timestamps: dict[str, float]

    def ages(self, now: float | None = None) -> dict[str, float]:
        t = time.monotonic() if now is None else now
        return {cam: t - ts for cam, ts in self.timestamps.items()}

    def worst_age(self, now: float | None = None) -> float:
        ages = self.ages(now)
        return max(ages.values()) if ages else float("inf")


class BaseDataChannel(ABC):
    """数据源抽象：上层采集逻辑不得感知仿真/真机差异."""

    camera_names: tuple[str, ...] = DEFAULT_CAMERAS

    @abstractmethod
    def get_camera_packets(self) -> CameraPacket:
        """返回最新一路已编码图像与采集时刻."""

    @abstractmethod
    def get_proprioception(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回 (q[22] rad, effort[22], contact_forces[4] N)."""

    @abstractmethod
    def get_action_label(self) -> np.ndarray:
        """返回 22 维绝对目标位置标签 (rad)."""

    def get_camera_frames(self) -> dict[str, np.ndarray]:
        """解码为 RGB 图像字典（调试/预览用，导出走 packets）."""
        return {
            cam: decode_jpeg_to_rgb(buf)
            for cam, buf in self.get_camera_packets().jpeg.items()
        }

    def close(self) -> None:
        return None


class SimDataChannel(BaseDataChannel):
    """仿真通道：图像取自独立渲染线程，本体感知取自 MujocoSimAdapter."""

    def __init__(
        self,
        adapter,
        producer,
        action_supplier: Callable[[], np.ndarray],
        camera_names: tuple[str, ...] = DEFAULT_CAMERAS,
    ):
        self._adapter = adapter
        self._producer = producer
        self._action_supplier = action_supplier
        self.camera_names = tuple(camera_names)

    def get_camera_packets(self) -> CameraPacket:
        latest = self._producer.latest()
        selected = {c: v for c, v in latest.items() if c in self.camera_names}
        return CameraPacket(
            jpeg={c: payload for c, (payload, _ts) in selected.items()},
            timestamps={c: ts for c, (_payload, ts) in selected.items()},
        )

    def get_proprioception(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        obs = self._adapter.read_observation()
        forces = self._adapter.get_contact_forces()
        q = np.asarray(obs.composite_state, dtype=np.float32)
        effort = np.concatenate(
            [np.asarray(obs.arm_effort, dtype=np.float32),
             np.asarray(obs.hand_current, dtype=np.float32)]
        )
        f4 = np.array(
            [forces.get("index", 0.0), forces.get("middle", 0.0),
             forces.get("ring", 0.0), forces.get("thumb", 0.0)],
            dtype=np.float32,
        )
        return q, effort, f4

    def get_action_label(self) -> np.ndarray:
        return np.asarray(self._action_supplier(), dtype=np.float32).ravel()


class RealDataChannel(BaseDataChannel):
    """真机通道：ROS 2 话题经 ROS2RobotEnv 缓存后读取.

    图像仅在底层数组对象变化时编码一次（相机速率 << 控制速率）。
    """

    def __init__(
        self,
        env,
        action_supplier: Callable[[], np.ndarray],
        camera_names: tuple[str, ...] = DEFAULT_CAMERAS,
        jpeg_quality: int = 90,
    ):
        self._env = env
        self._action_supplier = action_supplier
        self._jpeg_quality = int(jpeg_quality)
        self.camera_names = tuple(camera_names)
        self._cache: dict[str, tuple[np.ndarray, bytes]] = {}
        self._stamps: dict[str, float] = {}

    def get_camera_packets(self) -> CameraPacket:
        obs = self._env.get_observation()
        images = obs.get("images", {})
        now = time.monotonic()
        jpeg: dict[str, bytes] = {}
        stamps: dict[str, float] = {}
        for cam in self.camera_names:
            arr = images.get(cam)
            if arr is None:
                continue
            cached = self._cache.get(cam)
            if cached is not None and cached[0] is arr:
                jpeg[cam] = cached[1]
                stamps[cam] = self._stamps[cam]
            else:
                payload = encode_rgb_to_jpeg(arr, self._jpeg_quality)
                self._cache[cam] = (arr, payload)
                self._stamps[cam] = now
                jpeg[cam] = payload
                stamps[cam] = now
        return CameraPacket(jpeg=jpeg, timestamps=stamps)

    def get_proprioception(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        obs = self._env.get_observation()
        q = np.asarray(obs["state"], dtype=np.float32)
        effort = np.asarray(obs.get("effort", np.zeros_like(q)), dtype=np.float32)
        forces = np.asarray(
            obs.get("contact_forces", np.zeros(4, dtype=np.float32)), dtype=np.float32
        )
        return q, effort, forces

    def get_action_label(self) -> np.ndarray:
        return np.asarray(self._action_supplier(), dtype=np.float32).ravel()

    def close(self) -> None:
        self._cache.clear()
        self._stamps.clear()


class MockDataChannel(BaseDataChannel):
    """确定性合成通道：仅供 Hub API/前端测试，不得用于数据集 E2E."""

    def __init__(
        self,
        camera_names: tuple[str, ...] = DEFAULT_CAMERAS,
        shape: tuple[int, int, int] = (480, 640, 3),
        dof: int = 22,
    ):
        self.camera_names = tuple(camera_names)
        self._shape = shape
        self._dof = int(dof)
        self._frames = {
            cam: encode_rgb_to_jpeg(
                np.full(shape, 40 + 60 * i, dtype=np.uint8)
            )
            for i, cam in enumerate(self.camera_names)
        }
        self._t0 = time.monotonic()

    def get_camera_packets(self) -> CameraPacket:
        now = time.monotonic()
        return CameraPacket(
            jpeg=dict(self._frames),
            timestamps={cam: now for cam in self._frames},
        )

    def get_proprioception(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        phase = (time.monotonic() - self._t0) * 2.0
        q = (np.sin(phase) * 0.2).astype(np.float32) * np.ones(self._dof, np.float32)
        effort = (np.cos(phase) * 0.05).astype(np.float32) * np.ones(self._dof, np.float32)
        forces = np.full(4, max(0.0, np.sin(phase)) * 2.0, dtype=np.float32)
        return q, effort, forces

    def get_action_label(self) -> np.ndarray:
        return self.get_proprioception()[0].copy()
```

- [ ] **Step 4: 为 `ROS2RobotEnv` 补 effort 缓存（真机 22D 力矩来源）**

在 `_on_hand_state` / `_on_arm_state` 中按 `sensor_msgs/JointState.effort` 缓存，缺失时回退零向量：

```python
# __init__ 中新增
self._latest_hand_effort = np.zeros(16, dtype=np.float32)
self._latest_arm_effort = np.zeros(6, dtype=np.float32)

# _on_hand_state 内（pos 校验之后）
eff = np.asarray(msg.effort, dtype=np.float32)
with self._lock:
    if eff.shape[0] >= 16:
        self._latest_hand_effort = eff[:16].copy()
    if not self._latest_arm_state.any() and self.config.mode == "hand_only":
        pass

# _on_arm_state 内
eff = np.asarray(msg.effort, dtype=np.float32)
with self._lock:
    if eff.shape[0] >= 6:
        self._latest_arm_effort = eff[:6].copy()

# get_observation 返回体中补充
"effort": np.concatenate([self._latest_arm_effort, self._latest_hand_effort]),
"contact_forces": np.zeros(4, dtype=np.float32),
```

> 真机 LEAP Hand 无指尖 FSR，`contact_forces` 恒为零向量，属已声明限制（见 §11 风险 R4），**不得伪造估算值**。

- [ ] **Step 5: 跑通**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_data_channel.py tests/test_ros2_robot_env.py -v`
Expected: PASS

- [ ] **Step 6: 提交**

```bash
git add src/tuinadex_teleop/tuinadex_teleop/collection/data_channel.py \
        src/tuinadex_teleop/tuinadex_teleop/envs/ros2_robot_env.py \
        tests/test_data_channel.py
git commit -m "feat(collection): add sim/real data channel abstraction with effort and force contracts"
```

---

## 8. Task 4（W4）：TuinaCollector 扩展 + 线程安全 Hub 控制器

**Files:**
- Modify: `src/tuinadex_teleop/tuinadex_teleop/collection/collector.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/collection/data_hub_controller.py`
- Test: `tests/test_collector_fsm.py`（追加）、`tests/test_data_hub_controller.py`

**Interfaces:**
- Consumes: `BaseDataChannel`、`EpisodeBuffer`、`TuinaCollector`
- Produces:
  - `CollectionState` 新增 `SAVING`
  - `TuinaCollector.review_decision(save, *, defer_completion=False)`、`notify_save_complete()`
  - `HubAction(str, Enum)`
  - `DataHubController(channel, collector, control_fps=50.0, ui_fps=25.0, max_staleness_s=0.06, max_consecutive_stale=3)`：`submit(action, payload=None) -> dict`、`tick() -> None`、`status() -> dict`、`on_save` 回调、`shutdown()`

- [ ] **Step 1: 追加失败测试**

```python
# tests/test_collector_fsm.py 追加
def test_review_save_transfers_ownership_and_defers_idle():
    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    collector = TuinaCollector(MockEnv(cfg))
    collector.start_recording()
    for _ in range(4):
        collector.step_once(np.zeros(16, dtype=np.float32))
    collector.stop_recording()

    captured = []
    collector.on_save = captured.append
    assert collector.review_decision(save=True, defer_completion=True) is True
    assert collector.state == CollectionState.SAVING
    assert len(captured) == 1 and len(captured[0]) == 4   # 所有权已转移
    assert len(collector.buffer) == 0                      # 新空缓存
    collector.notify_save_complete()
    assert collector.state == CollectionState.IDLE
```

```python
# tests/test_data_hub_controller.py
"""Hub 控制器：线程安全命令队列、丢帧熔断、异步保存."""
import threading
import time

import numpy as np
import pytest

from tuinadex_teleop.collection.collector import CollectionState, TuinaCollector
from tuinadex_teleop.collection.data_channel import MockDataChannel
from tuinadex_teleop.collection.data_hub_controller import DataHubController, HubAction


def _make(fps=50.0, **kwargs):
    channel = MockDataChannel(shape=(48, 64, 3), dof=22)
    collector = TuinaCollector(env=None)
    ctrl = DataHubController(channel, collector, control_fps=fps, **kwargs)
    return channel, collector, ctrl


def test_commands_apply_on_tick_boundary():
    channel, collector, ctrl = _make()
    try:
        assert ctrl.status()["state"] == "IDLE"
        assert ctrl.submit(HubAction.START.value)["ok"] is True
        assert ctrl.status()["state"] == "IDLE"       # 尚未 tick
        ctrl.tick()
        assert ctrl.status()["state"] == "RECORDING"
        for _ in range(5):
            ctrl.tick()
        assert ctrl.status()["frames_recorded"] == 5

        ctrl.submit(HubAction.STOP.value); ctrl.tick()
        assert ctrl.status()["state"] == "REVIEWING"
        ctrl.submit(HubAction.DISCARD.value); ctrl.tick()
        assert ctrl.status()["state"] == "IDLE"
        assert ctrl.status()["discarded"] == 1
    finally:
        ctrl.shutdown()


def test_async_save_does_not_block_tick():
    channel, collector, ctrl = _make()
    saved = []
    ctrl.on_save = lambda episode: (time.sleep(0.2), saved.append(len(episode)))
    try:
        ctrl.submit(HubAction.START.value); ctrl.tick()
        for _ in range(3):
            ctrl.tick()
        ctrl.submit(HubAction.STOP.value); ctrl.tick()
        t0 = time.monotonic()
        ctrl.submit(HubAction.SAVE.value); ctrl.tick()
        assert time.monotonic() - t0 < 0.1           # 未阻塞控制拍
        assert ctrl.status()["state"] == "SAVING"
        deadline = time.monotonic() + 3.0
        while ctrl.status()["state"] != "IDLE" and time.monotonic() < deadline:
            time.sleep(0.01)
        assert ctrl.status()["state"] == "IDLE"
        assert saved == [3]
    finally:
        ctrl.shutdown()


def test_stale_camera_frames_abort_episode_instead_of_duplicating():
    channel, collector, ctrl = _make()
    ctrl.max_staleness_s = -1.0   # 强制所有帧判定为过期
    try:
        ctrl.submit(HubAction.START.value); ctrl.tick()
        for _ in range(3):
            ctrl.tick()
        status = ctrl.status()
        assert status["state"] == "IDLE"
        assert status["frames_recorded"] == 0
        assert "stale_camera" in status["invalid_reason"]
    finally:
        ctrl.shutdown()


def test_submit_rejects_unknown_action():
    channel, collector, ctrl = _make()
    try:
        assert ctrl.submit("explode")["ok"] is False
    finally:
        ctrl.shutdown()
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_data_hub_controller.py -v`
Expected: FAIL，`ModuleNotFoundError: No module named 'tuinadex_teleop.collection.data_hub_controller'`

- [ ] **Step 3: 修改 `collector.py`**

```python
class CollectionState(Enum):
    """数据采集交互状态机枚举."""
    IDLE = auto()
    RECORDING = auto()
    PAUSED = auto()
    REVIEWING = auto()
    SAVING = auto()      # 后台落盘中，禁止开始新试次


class TuinaCollector:
    def __init__(self, env: BaseTuinaEnv | None = None,
                 on_save: Optional[Callable[[EpisodeBuffer], None]] = None):
        self.env = env
        self.buffer = EpisodeBuffer()
        self.state = CollectionState.IDLE
        self.on_save = on_save
        self._next_episode_index = 1
        self.total_episodes_saved = 0
        self.total_episodes_discarded = 0

    def start_recording(self) -> None:
        if self.state in (CollectionState.IDLE, CollectionState.PAUSED):
            self.state = CollectionState.RECORDING
        elif self.state == CollectionState.SAVING:
            logger.warning("Cannot start recording while an episode is being saved.")
        elif self.state == CollectionState.REVIEWING:
            logger.warning("Cannot start recording in REVIEWING state; confirm or discard first.")

    def review_decision(self, save: bool, *, defer_completion: bool = False) -> bool:
        """评审当前试次。保存时把缓存所有权移交给 on_save (I32)."""
        if self.state != CollectionState.REVIEWING:
            logger.warning(
                f"review_decision called in state {self.state}, expected REVIEWING."
            )
            return False

        episode = self.buffer
        self.buffer = EpisodeBuffer(
            episode_index=self._next_episode_index,
            language_instruction=episode.language_instruction,
        )
        self._next_episode_index += 1

        if save and len(episode) > 0:
            self.state = (
                CollectionState.SAVING if defer_completion else CollectionState.IDLE
            )
            self.total_episodes_saved += 1
            if self.on_save is not None:
                self.on_save(episode)          # 所有权转移，接收方负责落盘
            return True

        self.total_episodes_discarded += 1
        self.state = CollectionState.IDLE
        return False

    def notify_save_complete(self) -> None:
        """后台保存线程完成后回收 SAVING 状态."""
        if self.state == CollectionState.SAVING:
            self.state = CollectionState.IDLE

    def restart_episode(self) -> None:
        self.buffer = EpisodeBuffer(
            episode_index=self._next_episode_index,
            language_instruction=self.buffer.language_instruction,
        )
        self._next_episode_index += 1
        self.state = CollectionState.RECORDING

    def step_once(self, action: np.ndarray) -> tuple[dict[str, Any], dict[str, Any]]:
        if self.env is None:
            raise RuntimeError("TuinaCollector has no env; use DataHubController.tick() instead.")
        obs, info = self.env.step(action)
        if self.state == CollectionState.RECORDING:
            self.buffer.add_frame(
                state=obs["state"],
                action=action,
                images=obs.get("images", {}),
                timestamp=obs.get("timestamp", 0.0),
                effort=obs.get("effort"),
                contact_forces=obs.get("contact_forces"),
                phase_label=obs.get("phase_label", ""),
            )
        return obs, info
```

- [ ] **Step 4: 实现 `data_hub_controller.py`**

```python
"""data_hub_controller.py — Hub 线程安全命令队列与采集编排 (I32)."""
from __future__ import annotations

import logging
import queue
import threading
import time
from enum import Enum
from typing import Callable

import numpy as np

from .collector import CollectionState, TuinaCollector
from .data_channel import BaseDataChannel
from .episode_buffer import BufferCapacityError, EpisodeBuffer

logger = logging.getLogger(__name__)


class HubAction(str, Enum):
    START = "start"
    PAUSE = "pause"
    RESUME = "resume"
    STOP = "stop"
    SAVE = "save"
    DISCARD = "discard"
    LABEL = "label"


class DataHubController:
    """控制回路侧唯一写入者；HTTP 线程只允许 submit()/status()."""

    def __init__(
        self,
        channel: BaseDataChannel,
        collector: TuinaCollector,
        control_fps: float = 50.0,
        ui_fps: float = 25.0,
        max_staleness_s: float = 0.06,
        max_consecutive_stale: int = 3,
    ):
        self.channel = channel
        self.collector = collector
        self.control_fps = float(control_fps)
        self.ui_fps = float(ui_fps)
        self.max_staleness_s = float(max_staleness_s)
        self.max_consecutive_stale = int(max_consecutive_stale)

        self._lock = threading.RLock()
        self._commands: queue.Queue = queue.Queue(maxsize=32)
        self._save_q: queue.Queue = queue.Queue(maxsize=4)
        self._saver = threading.Thread(
            target=self._save_worker, name="episode-saver", daemon=True
        )
        self._saver.start()

        self._ui_every = max(1, int(round(self.control_fps / self.ui_fps)))
        self._tick_no = 0
        self._episode_index = 0
        self._episode_t0 = 0.0
        self._phase_label = "idle"
        self._instruction = ""
        self._dropped = 0
        self._stale_streak = 0
        self._save_errors = 0
        self._last_abort_reason: str | None = None
        self._last_telemetry: dict = {}

        self.on_save: Callable[[EpisodeBuffer], None] | None = None
        self.collector.on_save = self._enqueue_save

    # ---------------- HTTP 侧 (任意线程) ----------------
    def submit(self, action: str, payload: dict | None = None) -> dict:
        try:
            act = HubAction(action)
        except ValueError:
            return {"ok": False, "error": f"unknown action '{action}'"}
        try:
            self._commands.put_nowait((act, payload or {}))
        except queue.Full:
            return {"ok": False, "error": "command queue full"}
        return {"ok": True, "accepted": act.value}

    def status(self) -> dict:
        with self._lock:
            buffer = self.collector.buffer
            return {
                "state": self.collector.state.name,
                "episode_index": getattr(buffer, "episode_index", 0),
                "frames_recorded": len(buffer),
                "image_bytes": getattr(buffer, "image_bytes", 0),
                "dropped_frames": self._dropped,
                "invalid_reason": self._last_abort_reason,
                "phase_label": self._phase_label,
                "language_instruction": self._instruction,
                "cameras": list(self.channel.camera_names),
                "saved": self.collector.total_episodes_saved,
                "discarded": self.collector.total_episodes_discarded,
                "save_errors": self._save_errors,
                "telemetry": self._last_telemetry,
            }

    # ---------------- 控制回路侧 ----------------
    def tick(self) -> None:
        now = time.monotonic()
        self._drain_commands()
        self._tick_no += 1
        if self.collector.state is CollectionState.RECORDING:
            self._record_tick(now)
        if self._tick_no % self._ui_every == 0:
            self._publish(now)

    def shutdown(self, timeout: float = 5.0) -> None:
        self._commands.put_nowait((HubAction.STOP, {}))
        self._save_q.put(None)
        self._saver.join(timeout)

    # ---------------- 内部 ----------------
    def _drain_commands(self) -> None:
        while True:
            try:
                act, payload = self._commands.get_nowait()
            except queue.Empty:
                return
            self._apply(act, payload)

    def _apply(self, act: HubAction, payload: dict) -> None:
        collector = self.collector
        if act is HubAction.START:
            if collector.state is CollectionState.IDLE:
                self._begin_episode()
            collector.start_recording()
        elif act is HubAction.PAUSE:
            collector.pause_recording()
        elif act is HubAction.RESUME:
            collector.start_recording()
        elif act is HubAction.STOP:
            collector.stop_recording()
        elif act is HubAction.SAVE:
            self._finish_episode(save=True)
        elif act is HubAction.DISCARD:
            self._finish_episode(save=False)
        elif act is HubAction.LABEL:
            if "phase_label" in payload:
                self._phase_label = str(payload["phase_label"])
            if "language_instruction" in payload:
                self._instruction = str(payload["language_instruction"])
                self.collector.buffer.language_instruction = self._instruction

    def _begin_episode(self) -> None:
        with self._lock:
            self.collector.buffer = EpisodeBuffer(
                episode_index=self._episode_index,
                language_instruction=self._instruction,
            )
            self._episode_index += 1
            self._episode_t0 = time.monotonic()
            self._dropped = 0
            self._stale_streak = 0
            self._last_abort_reason = None

    def _finish_episode(self, save: bool) -> None:
        if self.collector.state is not CollectionState.REVIEWING:
            return
        buffer = self.collector.buffer
        if save and len(buffer) > 0:
            issues = buffer.validate_continuity(max_gap_s=1.5 / self.control_fps)
            if issues:
                logger.warning("Episode continuity rejected: %s", issues[:5])
                self._last_abort_reason = f"continuity: {issues[0]}"
                self.collector.review_decision(save=False)
                return
            buffer.seal_episode()
        self.collector.review_decision(save=save, defer_completion=save)

    def _record_tick(self, now: float) -> None:
        packets = self.channel.get_camera_packets()
        if not packets.jpeg:
            self._abort("no_camera_frames")
            return
        worst = packets.worst_age(now)
        if worst > self.max_staleness_s:
            self._dropped += 1
            self._stale_streak += 1
            if self._stale_streak >= self.max_consecutive_stale:
                self._abort(f"stale_camera_{worst * 1000:.0f}ms")
            return
        self._stale_streak = 0

        q, effort, forces = self.channel.get_proprioception()
        action = self.channel.get_action_label()
        t_rel = now - self._episode_t0
        try:
            self.collector.buffer.add_frame(
                state=q,
                action=action,
                effort=effort,
                contact_forces=forces,
                images=packets.jpeg,
                timestamp=t_rel,
                image_timestamps={cam: t_rel for cam in packets.timestamps},
                phase_label=self._phase_label,
            )
        except BufferCapacityError as exc:
            self._abort(str(exc))
            return

    def _abort(self, reason: str) -> None:
        logger.warning("Episode aborted: %s", reason)
        with self._lock:
            self._last_abort_reason = reason
            self.collector.buffer.clear()
            self.collector.state = CollectionState.IDLE
            self._dropped = 0
            self._stale_streak = 0

    def _publish(self, now: float) -> None:
        try:
            q, effort, forces = self.channel.get_proprioception()
        except Exception:  # noqa: BLE001 - 遥测失败不得打断采集
            logger.exception("Telemetry read failed")
            return
        self._last_telemetry = {
            "timestamp": now - self._episode_t0
            if self.collector.state is CollectionState.RECORDING
            else 0.0,
            "state": [float(v) for v in q],
            "effort": [float(v) for v in effort],
            "contact_forces": [float(v) for v in forces],
        }

    def _enqueue_save(self, episode: EpisodeBuffer) -> None:
        try:
            self._save_q.put_nowait(episode)
        except queue.Full:
            logger.error("Save queue full; episode dropped without writing")
            self.collector.notify_save_complete()

    def _save_worker(self) -> None:
        while True:
            episode = self._save_q.get()
            if episode is None:
                return
            try:
                if self.on_save is not None:
                    self.on_save(episode)
            except Exception:  # noqa: BLE001 - 保存失败必须记录且不崩溃控制回路
                logger.exception("Episode save failed")
                with self._lock:
                    self._save_errors += 1
            finally:
                self.collector.notify_save_complete()
```

> `review_decision` 必须**先**置 `SAVING` 再调用 `on_save`：否则保存队列满时同步回调 `notify_save_complete()` 会落在 `REVIEWING` 上而失效，状态永久卡死。

- [ ] **Step 5: 跑通**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_data_hub_controller.py tests/test_collector_fsm.py -v`
Expected: PASS (7/7)

- [ ] **Step 6: 提交**

```bash
git add src/tuinadex_teleop/tuinadex_teleop/collection/collector.py \
        src/tuinadex_teleop/tuinadex_teleop/collection/data_hub_controller.py \
        tests/test_collector_fsm.py tests/test_data_hub_controller.py
git commit -m "feat(collection): add thread-safe hub controller with abort policy and async save"
```

---

## 9. Task 5（W5）：零依赖 Web UI Hub（REST + SSE + MJPEG）

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/__init__.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/server.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/static/index.html`
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/static/app.js`
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/static/style.css`
- Modify: `src/tuinadex_teleop/setup.py`（`package_data` 增加 `static/*`）
- Test: `tests/test_collection_hub_api.py`

**Interfaces:**
- Consumes: `DataHubController.submit()/status()`、`CameraPacket`、`StreamHub.publish()`
- Produces:
  - `StreamHub`：`publish_frames(packets)`、`publish_telemetry(status)`、`publish(packets, telemetry)`（便捷）、`wait_frame(cam, after_ts, timeout)`、`wait_telemetry(after_seq, timeout)`、`stop()`
  - `CollectionHubServer(controller, host="127.0.0.1", port=8080)`：`start() -> int`（返回实际端口）、`stop()`、`port`
  - HTTP：`GET /`、`GET /app.js`、`GET /style.css`、`GET /api/status`、`POST /api/action/{start|pause|resume|stop|save|discard|label}`、`GET /video/{cam}`、`GET /events/telemetry`

- [ ] **Step 1: 写失败测试**

```python
# tests/test_collection_hub_api.py
"""Hub HTTP/REST/MJPEG 端到端接口测试 (动态端口, 无固定端口冲突)."""
import json
import time
import urllib.request

import numpy as np
import pytest

from tuinadex_teleop.collection.collector import TuinaCollector
from tuinadex_teleop.collection.data_channel import MockDataChannel
from tuinadex_teleop.collection.data_hub_controller import DataHubController
from tuinadex_teleop.hub.server import CollectionHubServer


def _post(url, payload=None):
    body = json.dumps(payload or {}).encode()
    req = urllib.request.Request(
        url, data=body, headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        return resp.status, json.loads(resp.read().decode())


def _get_json(url):
    with urllib.request.urlopen(url, timeout=5) as resp:
        return resp.status, json.loads(resp.read().decode())


@pytest.fixture
def hub():
    channel = MockDataChannel(shape=(48, 64, 3), dof=22)
    collector = TuinaCollector(env=None)
    controller = DataHubController(channel, collector, control_fps=50.0, ui_fps=25.0)
    server = CollectionHubServer(controller, host="127.0.0.1", port=0)
    port = server.start()
    base = f"http://127.0.0.1:{port}"
    try:
        yield base, controller, server
    finally:
        server.stop()
        controller.shutdown()


def test_status_and_control_api(hub):
    base, controller, _ = hub
    status, data = _get_json(f"{base}/api/status")
    assert status == 200
    assert data["state"] == "IDLE"
    assert data["cameras"] == ["overhead", "wrist", "side"]

    status, data = _post(f"{base}/api/action/start")
    assert status == 200 and data["ok"] is True
    controller.tick()                                  # 命令在拍边界生效
    assert _get_json(f"{base}/api/status")[1]["state"] == "RECORDING"

    for _ in range(4):
        controller.tick()
    assert _get_json(f"{base}/api/status")[1]["frames_recorded"] == 4

    _post(f"{base}/api/action/stop")
    controller.tick()
    assert _get_json(f"{base}/api/status")[1]["state"] == "REVIEWING"

    _post(f"{base}/api/action/discard")
    controller.tick()
    assert _get_json(f"{base}/api/status")[1]["state"] == "IDLE"


def test_unknown_action_returns_ok_false(hub):
    base, _, _ = hub
    status, data = _post(f"{base}/api/action/launch_missiles")
    assert status == 400 and data["ok"] is False


def test_index_and_static_assets(hub):
    base, _, _ = hub
    with urllib.request.urlopen(f"{base}/", timeout=5) as resp:
        assert resp.status == 200
        assert "TuinaDex" in resp.read().decode()
    for path in ("/app.js", "/style.css"):
        with urllib.request.urlopen(f"{base}{path}", timeout=5) as resp:
            assert resp.status == 200


def test_mjpeg_stream_yields_jpeg_frames(hub):
    base, controller, server = hub
    import threading

    stop = threading.Event()

    def _pump():
        while not stop.is_set():
            controller.tick()
            server.stream.publish(controller.channel.get_camera_packets(), controller.status())
            time.sleep(0.01)

    t = threading.Thread(target=_pump, daemon=True)
    t.start()
    try:
        with urllib.request.urlopen(f"{base}/video/overhead", timeout=5) as resp:
            assert resp.headers["Content-Type"].startswith("multipart/x-mixed-replace")
            chunk = resp.read(2048)
            assert b"--frame" in chunk or b"Content-Type: image/jpeg" in chunk
    finally:
        stop.set()
        t.join(2.0)


def test_sse_stream_emits_telemetry(hub):
    base, controller, server = hub
    import threading

    stop = threading.Event()

    def _pump():
        while not stop.is_set():
            controller.tick()
            server.stream.publish(controller.channel.get_camera_packets(), controller.status())
            time.sleep(0.01)

    t = threading.Thread(target=_pump, daemon=True)
    t.start()
    try:
        with urllib.request.urlopen(f"{base}/events/telemetry", timeout=5) as resp:
            assert resp.headers["Content-Type"].startswith("text/event-stream")
            payload = resp.read(512).decode(errors="ignore")
            assert "data:" in payload
    finally:
        stop.set()
        t.join(2.0)
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_collection_hub_api.py -v`
Expected: FAIL，`ModuleNotFoundError: No module named 'tuinadex_teleop.hub'`

- [ ] **Step 3: 实现 `hub/server.py`**

```python
"""server.py — 零依赖 Web UI Hub (REST + SSE + MJPEG)."""
from __future__ import annotations

import json
import logging
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from ..collection.data_channel import CameraPacket

logger = logging.getLogger(__name__)
STATIC_DIR = Path(__file__).resolve().parent / "static"


class StreamHub:
    """单生产者 / 多消费者共享槽位：每帧只编码一次 (I33)."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._frames: dict[str, tuple[bytes, float]] = {}
        self._telemetry: dict[str, Any] = {}
        self._telem_seq = 0
        self._stopped = False

    @property
    def stopped(self) -> bool:
        return self._stopped

    def publish_frames(self, packets: CameraPacket) -> None:
        if not packets.jpeg:
            return
        with self._cond:
            self._frames = {cam: (buf, ts) for cam, ts in packets.jpeg.items()}
            self._cond.notify_all()

    def publish_telemetry(self, telemetry: dict[str, Any]) -> None:
        with self._cond:
            self._telemetry = telemetry
            self._telem_seq += 1
            self._cond.notify_all()

    def publish(self, packets: CameraPacket, telemetry: dict[str, Any] | None = None) -> None:
        """便捷方法：同时推送图像与遥测 (测试用)."""
        self.publish_frames(packets)
        if telemetry is not None:
            self.publish_telemetry(telemetry)

    def wait_frame(
        self, cam: str, after_ts: float, timeout: float
    ) -> tuple[bytes | None, float]:
        with self._cond:
            self._cond.wait_for(
                lambda: self._stopped
                or (cam in self._frames and self._frames[cam][1] > after_ts),
                timeout=timeout,
            )
            item = self._frames.get(cam)
            if self._stopped or item is None or item[1] <= after_ts:
                return None, after_ts
            return item

    def wait_telemetry(
        self, after_seq: int, timeout: float
    ) -> tuple[dict[str, Any] | None, int]:
        with self._cond:
            self._cond.wait_for(
                lambda: self._stopped or self._telem_seq > after_seq, timeout=timeout
            )
            if self._stopped or self._telem_seq <= after_seq:
                return None, after_seq
            return self._telemetry, self._telem_seq

    def stop(self) -> None:
        with self._cond:
            self._stopped = True
            self._cond.notify_all()


class _HubHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class CollectionHubServer:
    """后台 HTTP/SSE/MJPEG 服务器；默认只绑定回环地址 (I32)."""

    def __init__(self, controller, host: str = "127.0.0.1", port: int = 8080):
        self.controller = controller
        self.stream = StreamHub()
        self._httpd = _HubHTTPServer((host, int(port)), _HubHandler)
        self._httpd.controller = controller
        self._httpd.stream = self.stream
        self._thread: threading.Thread | None = None
        if host not in ("127.0.0.1", "localhost"):
            logger.warning("Hub bound to non-loopback host %s; ensure network isolation.", host)

    @property
    def port(self) -> int:
        return int(self._httpd.server_address[1])

    def start(self) -> int:
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name="hub-http", daemon=True
        )
        self._thread.start()
        return self.port

    def stop(self) -> None:
        self.stream.stop()
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(5.0)


class _HubHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "TuinaHub/1.0"

    def log_message(self, fmt: str, *args) -> None:
        logging.getLogger("tuinadex.hub.http").debug(fmt, *args)

    # ---------------- GET ----------------
    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler 约定
        path = self.path.split("?", 1)[0]
        try:
            if path == "/":
                self._static("index.html", "text/html; charset=utf-8")
            elif path == "/app.js":
                self._static("app.js", "application/javascript; charset=utf-8")
            elif path == "/style.css":
                self._static("style.css", "text/css; charset=utf-8")
            elif path == "/api/status":
                self._json(HTTPStatus.OK, self.server.controller.status())
            elif path.startswith("/video/"):
                self._mjpeg(path.rsplit("/", 1)[-1])
            elif path == "/events/telemetry":
                self._sse()
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
        except (BrokenPipeError, ConnectionResetError):
            return

    # ---------------- POST ----------------
    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if not path.startswith("/api/action/"):
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        action = path.rsplit("/", 1)[-1]
        result = self.server.controller.submit(action, self._read_json())
        status = HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
        self._json(status, result)

    # ---------------- helpers ----------------
    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            value = json.loads(raw.decode("utf-8") or "{}")
        except json.JSONDecodeError:
            return {}
        return value if isinstance(value, dict) else {}

    def _json(self, status: HTTPStatus, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(int(status))
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _static(self, name: str, content_type: str) -> None:
        path = STATIC_DIR / name
        if not path.is_file():
            self._json(HTTPStatus.NOT_FOUND, {"error": f"missing static asset {name}"})
            return
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _mjpeg(self, cam: str) -> None:
        if cam not in self.server.controller.channel.camera_names:
            self._json(HTTPStatus.NOT_FOUND, {"error": f"unknown camera {cam}"})
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        last_ts = 0.0
        while not self.server.stream.stopped:
            payload, last_ts = self.server.stream.wait_frame(cam, last_ts, timeout=1.0)
            if payload is None:
                continue
            header = (
                f"--frame\r\nContent-Type: image/jpeg\r\n"
                f"Content-Length: {len(payload)}\r\n\r\n"
            ).encode("ascii")
            self.wfile.write(header)
            self.wfile.write(payload)
            self.wfile.write(b"\r\n")
            self.wfile.flush()

    def _sse(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(b"retry: 1000\r\n\r\n")
        self.wfile.flush()
        seq = 0
        while not self.server.stream.stopped:
            payload, seq = self.server.stream.wait_telemetry(seq, timeout=1.0)
            if payload is None:
                continue
            data = json.dumps(payload, ensure_ascii=False)
            self.wfile.write(f"data: {data}\r\n\r\n".encode("utf-8"))
            self.wfile.flush()
```

- [ ] **Step 4: 写前端三件套**

`hub/static/index.html`：

```html
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>TuinaDex 数据采集 Hub</title>
  <link rel="stylesheet" href="/style.css"/>
</head>
<body>
<header>
  <h1>TuinaDex 采集工作台</h1>
  <span id="state" class="pill">IDLE</span>
  <span id="stats" class="stats"></span>
</header>
<main>
  <section class="videos">
    <figure><img id="v-overhead" src="/video/overhead" alt="overhead"/><figcaption>全局俯视 · 60°</figcaption></figure>
    <figure><img id="v-wrist" src="/video/wrist" alt="wrist"/><figcaption>腕部第一视角 · 75°</figcaption></figure>
    <figure><img id="v-side" src="/video/side" alt="side"/><figcaption>侧向机位 · 45°</figcaption></figure>
  </section>
  <section class="panels">
    <div class="panel"><h2>22 维关节 (rad)</h2><div id="joints" class="bars"></div></div>
    <div class="panel"><h2>指尖法向力 (N)</h2><div id="forces" class="bars"></div></div>
    <div class="panel">
      <h2>任务标签</h2>
      <input id="instruction" placeholder="语言指令，如：按压大椎穴"/>
      <input id="phase" placeholder="阶段标签，如：press"/>
      <button data-act="label">写入标签</button>
    </div>
    <div class="panel controls">
      <button data-act="start">开始 · Space</button>
      <button data-act="pause">暂停 · P</button>
      <button data-act="resume">恢复 · R</button>
      <button data-act="stop">停止 · T</button>
      <button data-act="save">保存 · S</button>
      <button data-act="discard">丢弃 · D</button>
    </div>
  </section>
</main>
<script src="/app.js"></script>
</body>
</html>
```

`hub/static/app.js`：

```javascript
const stateEl = document.getElementById('state');
const statsEl = document.getElementById('stats');
const jointsEl = document.getElementById('joints');
const forcesEl = document.getElementById('forces');
const FORCE_NAMES = ['index', 'middle', 'ring', 'thumb'];

function makeRow(label) {
  const row = document.createElement('div');
  row.className = 'bar-row';
  row.innerHTML = '<span class="bar-label">' + label + '</span>' +
    '<span class="bar-track"><span class="bar-fill"></span></span>' +
    '<span class="bar-value">0.00</span>';
  return row;
}

for (let i = 0; i < 22; i++) jointsEl.appendChild(makeRow('J' + i));
FORCE_NAMES.forEach((n) => forcesEl.appendChild(makeRow(n)));

function updateBars(container, values, lo, hi) {
  const rows = container.children;
  for (let i = 0; i < rows.length && i < values.length; i++) {
    const v = Number(values[i]);
    const pct = Math.max(0, Math.min(1, (v - lo) / (hi - lo))) * 100;
    rows[i].querySelector('.bar-fill').style.width = pct.toFixed(1) + '%';
    rows[i].querySelector('.bar-value').textContent = v.toFixed(2);
  }
}

const es = new EventSource('/events/telemetry');
es.onmessage = (ev) => {
  const status = JSON.parse(ev.data);
  stateEl.textContent = status.state;
  stateEl.dataset.state = status.state;
  statsEl.textContent = '帧 ' + status.frames_recorded + ' · 已存 ' + status.saved +
    ' · 丢弃 ' + status.discarded + ' · 丢帧 ' + status.dropped_frames +
    (status.invalid_reason ? ' · 中止: ' + status.invalid_reason : '');
  const tele = status.telemetry || {};
  updateBars(jointsEl, tele.state || [], -0.3, 2.0);
  updateBars(forcesEl, tele.contact_forces || [], 0, 10);
};
es.onerror = () => { stateEl.textContent = 'DISCONNECTED'; };

async function act(name) {
  const body = {};
  if (name === 'label') {
    body.phase_label = document.getElementById('phase').value;
    body.language_instruction = document.getElementById('instruction').value;
  }
  const resp = await fetch('/api/action/' + name, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  if (!resp.ok) {
    statsEl.textContent = '操作被拒绝: ' + name;
  }
}

document.querySelectorAll('[data-act]').forEach((btn) => {
  btn.addEventListener('click', () => act(btn.dataset.act));
});

const KEYS = { ' ': 'start', p: 'pause', r: 'resume', t: 'stop', s: 'save', d: 'discard' };
document.addEventListener('keydown', (ev) => {
  if (ev.target.tagName === 'INPUT') return;
  const name = KEYS[ev.key.toLowerCase()];
  if (name) {
    ev.preventDefault();
    act(name);
  }
});
```

`hub/static/style.css`：

```css
:root {
  --bg: #10141a;
  --panel: #1a2029;
  --line: #2b3442;
  --text: #e6edf3;
  --muted: #8b9bb0;
  --accent: #4cc2ff;
  --ok: #3ddc97;
  --warn: #ffb020;
  --bad: #ff5c5c;
}
* { box-sizing: border-box; }
body {
  margin: 0; padding: 16px; background: var(--bg); color: var(--text);
  font: 14px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif;
}
header { display: flex; align-items: center; gap: 12px; margin-bottom: 12px; }
h1 { font-size: 18px; margin: 0; }
.pill {
  padding: 2px 10px; border-radius: 999px; background: var(--line);
  font-size: 12px; letter-spacing: .04em;
}
.pill[data-state="RECORDING"] { background: var(--bad); }
.pill[data-state="REVIEWING"] { background: var(--warn); }
.pill[data-state="SAVING"] { background: var(--accent); }
.stats { color: var(--muted); font-size: 12px; }
main { display: grid; gap: 16px; }
.videos { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
figure { margin: 0; background: var(--panel); border: 1px solid var(--line); border-radius: 8px; overflow: hidden; }
figure img { display: block; width: 100%; aspect-ratio: 4 / 3; object-fit: cover; background: #000; }
figcaption { padding: 6px 10px; color: var(--muted); font-size: 12px; }
.panels { display: grid; grid-template-columns: 2fr 1fr; gap: 12px; }
.panel { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 12px; }
.panel h2 { margin: 0 0 8px; font-size: 13px; color: var(--muted); font-weight: 600; }
.bars { display: grid; gap: 3px; }
.bar-row { display: grid; grid-template-columns: 64px 1fr 52px; align-items: center; gap: 8px; }
.bar-label { color: var(--muted); font-size: 11px; }
.bar-track { height: 8px; background: #0b0f14; border-radius: 4px; overflow: hidden; }
.bar-fill { display: block; height: 100%; width: 0; background: var(--accent); transition: width .08s linear; }
.bar-value { font-variant-numeric: tabular-nums; font-size: 11px; color: var(--muted); text-align: right; }
.controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; align-content: start; }
button, input {
  background: #0b0f14; color: var(--text); border: 1px solid var(--line);
  border-radius: 6px; padding: 8px 10px; font: inherit;
}
button { cursor: pointer; }
button:hover { border-color: var(--accent); }
button[data-act="start"] { border-color: var(--ok); }
button[data-act="discard"] { border-color: var(--bad); }
@media (max-width: 900px) {
  .videos, .panels { grid-template-columns: 1fr; }
}
```

`hub/__init__.py`：

```python
"""tuinadex_teleop.hub — 零依赖 Web 采集工作台."""
from .server import CollectionHubServer, StreamHub

__all__ = ["CollectionHubServer", "StreamHub"]
```

- [ ] **Step 5: `setup.py` 增加静态资源打包**

```python
    package_data={
        package_name + '.tracking': ['models/*.task'],
        package_name + '.perception': ['models/*.task'],
        package_name + '.hub': ['static/*'],
    },
```

- [ ] **Step 6: 跑通**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_collection_hub_api.py -v`
Expected: PASS (5/5)

- [ ] **Step 7: 提交**

```bash
git add src/tuinadex_teleop/tuinadex_teleop/hub src/tuinadex_teleop/setup.py tests/test_collection_hub_api.py
git commit -m "feat(hub): add zero-dependency web hub with REST, SSE and MJPEG streaming"
```

---

## 10. Task 6（W6）：LeRobotDataset v3.0 多模态导出器扩展

**Files:**
- Modify: `dataset/tuinadex_dataset/lerobot_exporter.py`
- Test: `tests/test_lerobot_v3_exporter.py`（保留既有用例，追加）

**Interfaces:**
- Consumes: `EpisodeBuffer.iter_frames()`、`FrameData.decode_image(cam)`
- Produces:
  - `LeRobotDatasetWriter(repo_id, config, fps=50.0, use_videos=True, vcodec="h264", streaming_encoding=True)`
  - feature：`observation.state`、`observation.effort`、`observation.contact_forces`、`action`、`phase_index`、`observation.images.{cam}`（video）
  - 产物：`meta/info.json`、`meta/tasks.jsonl`、`meta/phase_labels.json`、`data/**/*.parquet`、`videos/observation.images.{cam}/**/*.mp4`

- [ ] **Step 1: 追加失败测试**

```python
# tests/test_lerobot_v3_exporter.py 追加
def test_multi_modal_v3_export_with_three_cameras(tmp_path):
    import numpy as np
    from tuinadex_teleop.envs.config import RobotConfig
    from tuinadex_teleop.collection.episode_buffer import EpisodeBuffer
    from tuinadex_dataset.lerobot_exporter import LeRobotDatasetWriter
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    cfg = RobotConfig(mode="arm_hand", camera_names=["overhead", "wrist", "side"])
    writer = LeRobotDatasetWriter(repo_id=str(tmp_path / "ds3cam"), config=cfg, fps=50.0)
    for ep in range(2):
        buf = EpisodeBuffer(episode_index=ep, language_instruction="按压肺俞穴")
        for i in range(12):
            img = np.full((96, 128, 3), 40 + i, dtype=np.uint8)
            buf.add_frame(
                state=np.full(22, i * 0.01, np.float32),
                action=np.full(22, i * 0.02, np.float32),
                effort=np.full(22, i * 0.1, np.float32),
                contact_forces=np.array([1.0, 2.0, 3.0, 4.0], np.float32),
                images={"overhead": img, "wrist": img, "side": img},
                timestamp=i * 0.02,
                phase_label="press" if i < 6 else "release",
            )
        buf.seal_episode()
        assert buf.frames[-1].next_done is True          # I30: 缓存层保留
        writer.save_episode(buf, task="tuina_massage")

    info = json.loads((tmp_path / "ds3cam/meta/info.json").read_text())
    assert info["codebase_version"] == "v3.0"
    assert info["total_episodes"] == 2
    assert info["total_frames"] == 24
    features = info["features"]
    for key in (
        "observation.state",
        "observation.effort",
        "observation.contact_forces",
        "action",
        "phase_index",
        "observation.images.overhead",
        "observation.images.wrist",
        "observation.images.side",
    ):
        assert key in features, key
    assert features["observation.images.wrist"]["dtype"] == "video"

    for cam in ("overhead", "wrist", "side"):
        assert list((tmp_path / f"ds3cam/videos/observation.images.{cam}").rglob("*.mp4"))

    phase_map = json.loads((tmp_path / "ds3cam/meta/phase_labels.json").read_text())
    assert sorted(phase_map.values()) == ["press", "release"]

    ds = LeRobotDataset("ds3cam", root=tmp_path / "ds3cam")
    assert len(ds) == 24
    frame = ds[0]
    assert frame["observation.state"].shape == (22,)
    assert frame["observation.effort"].shape == (22,)
    assert frame["observation.contact_forces"].shape == (4,)
    assert "observation.images.side" in frame
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_lerobot_v3_exporter.py::test_multi_modal_v3_export_with_three_cameras -v`
Expected: FAIL（`observation.effort` 未在 features 中；`phase_labels.json` 缺失）

- [ ] **Step 3: 修改 `lerobot_exporter.py`**

```python
"""lerobot_exporter.py — LeRobotDataset v3.0 多模态轨迹导出器."""
from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

CONTACT_FORCE_NAMES = ["index", "middle", "ring", "thumb"]


class LeRobotDatasetWriter:
    """将 EpisodeBuffer 导出为原生 LeRobotDataset v3.0 (含 Parquet 与 MP4 视频分块)."""

    def __init__(
        self,
        repo_id: str,
        config: Any,
        fps: float = 50.0,
        use_videos: bool = True,
        vcodec: str = "h264",
        streaming_encoding: bool = True,
    ):
        self.repo_id = repo_id
        self.config = config
        self.fps = float(fps)
        self.use_videos = use_videos
        self.vcodec = vcodec
        self.streaming_encoding = bool(streaming_encoding)
        self.output_root = Path(repo_id).resolve()
        self._dataset = None
        self._phase_index: dict[str, int] = {}

    def _init_dataset_if_needed(self, first_frame: Any) -> Any:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        info_path = self.output_root / "meta" / "info.json"
        if info_path.exists():
            return LeRobotDataset(self.output_root.name, root=self.output_root)

        dim = int(self.config.action_dim)
        joint_names = list(self.config.joint_names)
        features: dict[str, Any] = {
            "observation.state": {
                "dtype": "float32", "shape": (dim,), "names": {"joints": joint_names},
            },
            "observation.effort": {
                "dtype": "float32", "shape": (dim,), "names": {"joints": joint_names},
            },
            "observation.contact_forces": {
                "dtype": "float32", "shape": (4,),
                "names": {"fingers": CONTACT_FORCE_NAMES},
            },
            "action": {
                "dtype": "float32", "shape": (dim,), "names": {"joints": joint_names},
            },
            "phase_index": {"dtype": "int64", "shape": (1,), "names": None},
        }

        for cam in self.config.camera_names:
            key = f"observation.images.{cam}"
            if cam in first_frame.images_jpeg:
                h, w = first_frame.decode_image(cam).shape[:2]
            else:
                raise ValueError(
                    f"camera '{cam}' missing from first frame; refusing to fabricate a shape"
                )
            features[key] = {
                "dtype": "video" if self.use_videos else "image",
                "shape": (3, h, w),
                "names": ["channels", "height", "width"],
            }

        logger.info("Creating LeRobotDataset v3.0 at %s (videos=%s)", self.output_root, self.use_videos)
        return LeRobotDataset.create(
            repo_id=self.output_root.name,
            fps=int(self.fps),
            features=features,
            root=self.output_root,
            use_videos=self.use_videos,
            vcodec=self.vcodec,
            metadata_buffer_size=1,
            streaming_encoding=self.streaming_encoding,
            encoder_queue_maxsize=30,
        )

    def _phase_id(self, label: str) -> int:
        key = label or "unspecified"
        if key not in self._phase_index:
            self._phase_index[key] = len(self._phase_index)
        return self._phase_index[key]

    def _write_phase_labels(self) -> None:
        path = self.output_root / "meta" / "phase_labels.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {str(idx): name for name, idx in self._phase_index.items()}
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def flush(self) -> None:
        """刷新并关闭底层写入器，保证 Parquet 与视频数据完整落盘."""
        if self._dataset is not None:
            if hasattr(self._dataset, "_close_writer"):
                self._dataset._close_writer()
            if hasattr(self._dataset, "meta"):
                if hasattr(self._dataset.meta, "_flush_metadata_buffer"):
                    self._dataset.meta._flush_metadata_buffer()
                if hasattr(self._dataset.meta, "_close_writer"):
                    self._dataset.meta._close_writer()

    def save_episode(self, buffer: Any, task: str = "tuina_massage") -> str:
        """将一个完整 EpisodeBuffer 写入并落盘 (逐帧解码，不驻留全量图像)."""
        import torch

        if len(buffer) == 0:
            raise ValueError("Cannot export empty EpisodeBuffer.")

        if self._dataset is None:
            self._dataset = self._init_dataset_if_needed(buffer.frames[0])

        for frame in buffer.iter_frames():
            frame_dict: dict[str, Any] = {
                "observation.state": torch.from_numpy(frame.state.astype(np.float32)),
                "observation.effort": torch.from_numpy(frame.effort.astype(np.float32)),
                "observation.contact_forces": torch.from_numpy(
                    frame.contact_forces.astype(np.float32)
                ),
                "action": torch.from_numpy(frame.action.astype(np.float32)),
                "phase_index": torch.tensor(
                    [self._phase_id(frame.phase_label)], dtype=torch.int64
                ),
                "task": str(frame.language_instruction or task),
            }
            for cam in self.config.camera_names:
                key = f"observation.images.{cam}"
                if cam in frame.images_jpeg:
                    img = frame.decode_image(cam)
                    frame_dict[key] = torch.from_numpy(
                        np.transpose(img, (2, 0, 1)).astype(np.uint8)
                    )
            self._dataset.add_frame(frame=frame_dict)

        self._dataset.save_episode(parallel_encoding=False)
        self._write_phase_labels()
        self.flush()
        logger.info("Saved episode with %d frames to %s", len(buffer), self.output_root)
        return str(self.output_root)

    def close(self) -> None:
        """会话结束时的最终落盘."""
        self._write_phase_labels()
        self.flush()
```

> `save_episode` 内调用 `flush()` 是既有行为（实测两个连续 episode 均能正确落盘，`total_episodes=2`），本次保持不变以避免扩大改动面。

- [ ] **Step 4: 跑通（含既有用例）**

Run: `~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_lerobot_v3_exporter.py -v`
Expected: PASS (2/2)

- [ ] **Step 5: 提交**

```bash
git add dataset/tuinadex_dataset/lerobot_exporter.py tests/test_lerobot_v3_exporter.py
git commit -m "feat(dataset): extend v3.0 exporter with effort, contact forces, phase labels and streaming encode"
```

---

## 11. Task 7（W7）：统一 CLI 与端到端自动化验证

**Files:**
- Modify: `scripts/collect_teleop_data.py`
- Create: `scripts/e2e_hub_collection_check.sh`
- Modify: `AGENTS.md`（验证命令区追加 Hub 入口）
- Modify: `README.md`

**Interfaces:**
- Produces：
  - `python scripts/collect_teleop_data.py --env sim --mode arm_hand --mock-teleop --hub --headless --port 0 --repo-id data/hub_run`
  - 启动后 stdout 打印 `HUB_READY port=<n>`
  - `bash scripts/e2e_hub_collection_check.sh`（退出码 0 表示通过）

- [ ] **Step 1: 扩展 `collect_teleop_data.py`**

新增参数（在 `parse_args()` 内追加）：

```python
    parser.add_argument("--hub", action="store_true", help="Launch the web UI hub instead of terminal collection")
    parser.add_argument("--host", default="127.0.0.1", help="Hub bind address (default loopback only)")
    parser.add_argument("--port", type=int, default=8080, help="Hub port (0 picks a free port)")
    parser.add_argument("--headless", action="store_true", help="Do not open a MuJoCo viewer")
    parser.add_argument("--ui-fps", type=float, default=25.0, help="SSE telemetry push rate")
```

新增 Hub 会话函数（`main()` 之前）：

```python
def _mock_action(config: "RobotConfig", k: int) -> np.ndarray:
    """与 run_mock_collection 一致的平滑屈伸轨迹."""
    if config.mode == "hand_only":
        return (0.3 + 0.2 * np.sin(0.1 * k + np.linspace(0, np.pi, 16))).astype(np.float32)
    arm_act = (0.2 * np.sin(0.05 * k + np.arange(6))).astype(np.float32)
    hand_act = (0.3 + 0.2 * np.sin(0.1 * k + np.linspace(0, np.pi, 16))).astype(np.float32)
    return np.concatenate([arm_act, hand_act]).astype(np.float32)


def run_hub_session(args, config):
    """Hub 模式：独立渲染线程 + 线程安全控制器 + Web 服务."""
    import os
    import threading

    os.environ.setdefault("MUJOCO_GL", "egl")

    from tuinadex_teleop.adapters.mujoco_sim_adapter import MujocoSimAdapter
    from tuinadex_teleop.cameras.sim_camera_rig import CameraProducerThread
    from tuinadex_teleop.collection.data_channel import RealDataChannel, SimDataChannel
    from tuinadex_teleop.collection.data_hub_controller import DataHubController
    from tuinadex_teleop.core.types import (
        ClutchState, TeleopArmCommand, TeleopCommand, TeleopHandCommand,
    )
    from tuinadex_teleop.hub.server import CollectionHubServer

    if config.mode != "arm_hand":
        raise SystemExit("--hub requires --mode arm_hand (22-DOF contract, invariant 26)")

    writer = LeRobotDatasetWriter(
        repo_id=args.repo_id, config=config, fps=args.fps, use_videos=True
    )
    collector = TuinaCollector(env=None)
    commanded = {"q": np.zeros(config.action_dim, dtype=np.float32)}

    adapter = None
    producer = None
    env = None
    lock = threading.Lock()

    if args.env == "sim":
        adapter = MujocoSimAdapter(headless=True)
        if not adapter.connect():
            raise RuntimeError("MujocoSimAdapter connection failed")
        producer = CameraProducerThread(
            adapter.model, adapter.data, lock,
            target_fps=args.fps, jpeg_quality=90,
        )
        producer.start()
        if not producer.wait_ready(30.0):
            raise RuntimeError(f"Camera producer failed to start: {producer.stats()['last_error']}")
        channel = SimDataChannel(adapter, producer, action_supplier=lambda: commanded["q"])
    else:
        env = make_tuina_env(env_type="real", config=config)
        channel = RealDataChannel(env, action_supplier=lambda: commanded["q"])

    controller = DataHubController(
        channel, collector, control_fps=args.fps, ui_fps=args.ui_fps
    )
    controller.on_save = lambda episode: writer.save_episode(episode, task=args.task)

    server = CollectionHubServer(controller, host=args.host, port=args.port)
    port = server.start()
    print(f"HUB_READY port={port}", flush=True)

    period = 1.0 / args.fps
    substeps = max(1, int(round(period / 0.002)))
    ui_every = max(1, int(round(args.fps / args.ui_fps)))
    tick = 0
    next_t = time.monotonic()
    try:
        while True:
            if args.env == "sim":
                if args.mock_teleop:
                    commanded["q"] = _mock_action(config, tick)
                tick += 1
                q = commanded["q"]
                now = time.time()
                cmd = TeleopCommand(
                    timestamp=now,
                    arm_command=TeleopArmCommand(
                        timestamp=now, target_pose=np.eye(4), target_q=q[:6].copy()
                    ),
                    hand_command=TeleopHandCommand(timestamp=now, target_q=q[6:].copy()),
                    clutch_state=ClutchState.ENGAGED,
                )
                with lock:
                    adapter.send_command(cmd)
                    adapter.step(substeps)
            else:
                env.spin_once()

            controller.tick()
            server.stream.publish_frames(channel.get_camera_packets())
            if tick % ui_every == 0:
                server.stream.publish_telemetry(controller.status())

            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        logger.info("Hub session interrupted by user.")
    finally:
        server.stop()
        controller.shutdown()
        writer.close()
        if producer is not None:
            producer.stop()
        if adapter is not None:
            adapter.disconnect()
        if env is not None:
            env.close()
```

`main()` 分支（在实例化 env 之前）：

```python
    if args.hub:
        run_hub_session(args, config)
        return
```

> `ClutchState.ENGAGED` 已确认存在于 `core/types.py:20-25`，不得新增枚举值。

- [ ] **Step 2: 创建 `scripts/e2e_hub_collection_check.sh`**

```bash
#!/usr/bin/env bash
# e2e_hub_collection_check.sh — Hub 全链路端到端验证（真实 EGL 渲染 + 真实 H.264）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${TUINADEX_CONTROL_PYTHON:-$HOME/win_office/conda/envs/tuinadex-control/bin/python}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"

WORK="$(mktemp -d)"
LOG="$WORK/hub.log"
PID=""

cleanup() {
  if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
    kill -INT "$PID" 2>/dev/null || true
    wait "$PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cd "$ROOT"
"$PY" scripts/collect_teleop_data.py \
  --env sim --mode arm_hand --mock-teleop --hub --headless \
  --host 127.0.0.1 --port 0 --fps 50 --ui-fps 25 \
  --repo-id "$WORK/repo" > "$LOG" 2>&1 &
PID=$!

PORT=""
for _ in $(seq 1 60); do
  PORT="$(grep -m1 -o 'HUB_READY port=[0-9]*' "$LOG" 2>/dev/null | cut -d= -f2 || true)"
  [[ -n "$PORT" ]] && break
  sleep 0.5
done
if [[ -z "$PORT" ]]; then
  echo "[E2E] FAIL: hub did not report HUB_READY"
  tail -n 40 "$LOG"
  exit 1
fi
echo "[E2E] hub listening on $PORT"

BASE="http://127.0.0.1:$PORT"
curl -fsS -X POST "$BASE/api/action/start" -H 'Content-Type: application/json' -d '{}' > /dev/null
sleep 1.2
curl -fsS -X POST "$BASE/api/action/stop" -H 'Content-Type: application/json' -d '{}' > /dev/null
curl -fsS -X POST "$BASE/api/action/save" -H 'Content-Type: application/json' -d '{}' > /dev/null

STATE=""
for _ in $(seq 1 120); do
  STATE="$(curl -fsS "$BASE/api/status" | "$PY" -c 'import sys,json;print(json.load(sys.stdin)["state"])')"
  [[ "$STATE" == "IDLE" ]] && break
  sleep 0.5
done
[[ "$STATE" == "IDLE" ]] || { echo "[E2E] FAIL: state stuck at $STATE"; tail -n 40 "$LOG"; exit 1; }

kill -INT "$PID" 2>/dev/null || true
wait "$PID" 2>/dev/null || true
PID=""

"$PY" - "$WORK/repo" <<'PY'
import json
import sys
from pathlib import Path

repo = Path(sys.argv[1])
info = json.loads((repo / "meta" / "info.json").read_text())
features = info["features"]

required = [
    "observation.state", "observation.effort", "observation.contact_forces",
    "action", "phase_index",
    "observation.images.overhead", "observation.images.wrist", "observation.images.side",
]
missing = [k for k in required if k not in features]
assert not missing, f"missing features: {missing}"
assert info["fps"] == 50, info["fps"]
assert info["total_frames"] >= 20, info["total_frames"]
assert features["observation.state"]["shape"] == [22], features["observation.state"]
assert features["observation.contact_forces"]["shape"] == [4]

for cam in ("overhead", "wrist", "side"):
    vids = list((repo / f"videos/observation.images.{cam}").rglob("*.mp4"))
    assert vids, f"no mp4 for {cam}"

assert list((repo / "data").rglob("*.parquet")), "no parquet output"
assert (repo / "meta" / "phase_labels.json").is_file(), "missing phase labels"
print(f"[E2E] dataset OK: frames={info['total_frames']} episodes={info['total_episodes']}")
PY

echo "[E2E] PASS"
```

- [ ] **Step 3: 运行 E2E**

Run: `bash scripts/e2e_hub_collection_check.sh`
Expected: `[E2E] dataset OK ...` 与 `[E2E] PASS`，退出码 0

- [ ] **Step 4: 追加 AGENTS.md 固化命令并提交**

在 `AGENTS.md` §5「验证命令」追加：

```markdown
- 三机位采集 Hub 端到端 E2E（真实 EGL 渲染 + H.264 导出）：
  `bash scripts/e2e_hub_collection_check.sh`
- 三机位相机与采集契约单测：
  `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_camera_rig.py tests/test_episode_buffer.py tests/test_data_channel.py tests/test_data_hub_controller.py tests/test_collection_hub_api.py -v`
```

```bash
git add scripts/collect_teleop_data.py scripts/e2e_hub_collection_check.sh AGENTS.md README.md
git commit -m "feat(hub): add unified hub CLI mode and end-to-end collection verification"
```

---

## 12. 验证矩阵（Phase 4 终验必查）

| 层 | 命令 | 通过判据 |
|---|---|---|
| 相机 | `pytest tests/test_camera_rig.py -v` | 5 passed，含两份场景逐字节一致 |
| 缓存 | `pytest tests/test_episode_buffer.py -v` | 8 passed，含 RGB/JPEG 通道一致性 |
| 通道 | `pytest tests/test_data_channel.py -v` | 4 passed，含真实 `MujocoSimAdapter` + 真实渲染线程 |
| 编排 | `pytest tests/test_data_hub_controller.py tests/test_collector_fsm.py -v` | 7 passed，含丢帧中止与异步保存 |
| Hub | `pytest tests/test_collection_hub_api.py -v` | 5 passed，动态端口 |
| 导出 | `~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_lerobot_v3_exporter.py -v` | 2 passed，2 episode |
| E2E | `bash scripts/e2e_hub_collection_check.sh` | `[E2E] PASS`，退出码 0 |
| 回归 | `pytest tests/ -v`（既有全量） | 既有用例零退化 |
| 构建 | `colcon build --symlink-install` | 8 包全绿 |

---

## 13. 风险登记（Phase 3/4 必须跟踪）

| ID | 风险 | 影响 | 缓解 |
|---|---|---|---|
| R1 | 50Hz 三机位渲染单核占用约 78%（11.2ms 渲染 + 4.5ms JPEG / 20ms 周期） | 低配机器上 `achieved_fps` 低于 50，出现周期性重复帧 | Hub 状态暴露 `camera_fps`；低于 0.9×target 时 WARN；提供 `--camera-fps` 降档开关 |
| R2 | MuJoCo Python 绑定在 `render()` 期间持有 GIL，最长 3.7ms | 控制回路抖动，仿真时间步进不平滑 | 仿真下可接受（物理步数确定）；真机侧禁止在同进程渲染，改为独立进程或专用采集机 |
| R3 | 腕部机位姿态为经验初值 | 可能拍到手掌背面而非指尖 | Task 1 Step 6 人工视觉门禁；只调 `<camera>` 属性 |
| R4 | LEAP Hand 无指尖 FSR | 真机 `contact_forces` 恒为 0 | 已在 I30/通道层显式声明，禁止伪造估算 |
| R5 | 三路 MJPEG + SSE 占用浏览器 4 条长连接 | 同域并发上限 6，留 2 条给 REST | 已在 I33 记录；若需多标签页，改 WebSocket 或降低 MJPEG 流数 |
| R6 | 单试次 512MB 上限 | 长时推拿（>60s）被强制中止 | 上限可配；界面显示 `image_bytes` 供操作员预判 |
| R7 | `pytest.importorskip("lerobot")` 仍存在 | 若环境缺 lerobot 会静默跳过 | 在 `verify_all_envs.sh` 中断言 lerobot/torch/av 已安装，把跳过变成环境失败 |

---

## 14. 移交 Phase 3（Codex）

- **执行顺序**：W1 → W2 → W3 → W4 → W5 → W6 → W7，严禁跨工单改文件。
- **每个工单完成判据**：对应测试文件全绿 + `git commit` 已生成 + 黑板 Section 2 工单状态由 `[LOCKED]` 改为 `[COMPLETED]`。
- **熔断**：任一工单在同一失败点重试超过 3 次，立即在 `.agent/SESSION_STATE.md` 标记 `[SUSPENDED: ESCALATED]` 并回报。
- **禁止事项**：新增与既有三大件职责重叠的类；为通过测试而 `skip`/空断言/打桩渲染；在控制回路内调用渲染或落盘。
