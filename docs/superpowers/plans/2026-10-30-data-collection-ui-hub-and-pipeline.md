# Multi-Modal Data Collection UI Hub and Backend Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **STATUS: SUPERSEDED（2026-10-30 Phase 2 裁决）。执行请使用 `docs/superpowers/plans/2026-10-30-data-collection-hub-PHASE2-ADJUDICATED.md`。本文件仅作 Phase 1 侦察与原始方案归档。**

**Goal:** Build a robust, real-time multi-modal data collection system with a responsive Frontend UI Hub and high-throughput Backend Data Pipeline in `tuinadex_teleop`, featuring a standardized 3-camera rig (overhead, wrist, side), 22-DOF proprioception/effort/contact-force logging, action labels, rich metadata (`timestamp`, `frame_index`, `episode_index`, `next.done`, `language_instruction`, `phase_label`), zero-cost sim/real channel parity, and native LeRobotDataset v3.0 export.

**Architecture:**
1. **Camera Layer (`tuinadex_teleop.cameras`)**: Encapsulates a standardized 3-camera specification:
   - `overhead_cam` (Eye-to-Hand, 640x480, fovy=60°, angled 45°~60° down over patient dummy and table)
   - `wrist_cam` (Eye-in-Hand, 640x480, fovy=75° wide-angle, attached to arm flange / hand palm looking forward-palm)
   - `side_cam` (Eye-to-Hand, 640x480, fovy=45°, parallel to bed contact surface)
   - Features `SimCameraRig` using `mujoco.Renderer` offscreen rendering (~0.5ms/frame) and `RealCameraRig` subscribing to ROS 2 Image topics or RealSense/V4L2 devices.
2. **Proprioception & Action Data Pipeline (`tuinadex_teleop.collection`)**:
   - Captures 22-DOF composite joint states (`6D arm` + `16D hand`), 22-DOF joint efforts/currents, 4-finger fingertip normal contact forces (N), and 22-DOF target joint position commands from ArUco+IMU+IK teleoperation.
   - Embeds standardized metadata: relative `timestamp (float32[1])`, `frame_index (int64[1])`, `episode_index (int64[1])`, `next.done (bool[1])`, `language_instruction (str)`, `phase_label (str)`.
3. **Frontend UI Hub (`tuinadex_teleop.hub`)**:
   - Zero-external-dependency, high-performance web dashboard served via Python stdlib `ThreadingHTTPServer` + Server-Sent Events (SSE) telemetry + multi-stream MJPEG video feeds at `http://localhost:8080`.
   - Live 3-camera video wall, real-time 22-DOF joint bar charts, contact force gauges, clutch indicators, and interactive collection controls (Start, Pause, Resume, Stop, Save, Discard, Re-record) with keyboard hotkeys.
   - Dual-mode fallback: Automatic headless/OpenCV HUD mode when running without a web browser.
4. **Dataset Export Layer (`dataset.tuinadex_dataset`)**:
   - Extends `LeRobotDatasetWriter` to chunk 3 camera video streams into separate H.264 MP4 videos while packaging 22D states, 22D efforts, 4D forces, 22D actions, and metadata into Apache Parquet tables complying with Hugging Face LeRobot Codebase v3.0.

**Tech Stack:**
- Python 3.10 / 3.14, MuJoCo 3.13 (`mujoco.Renderer`), NumPy, OpenCV (`cv2`)
- ROS 2 Lyrical Luth Desktop (`rclpy`, `sensor_msgs`)
- LeRobot 0.4.4 (Codebase v3.0), PyAV / FFmpeg, PyArrow / Parquet, PyTorch
- HTML5, Canvas, Modern CSS, Fetch API, Server-Sent Events (SSE)

**Spec:**
- `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`
- `docs/adrs/0002-architecture-decisions-sim.md`
- User Work Order Requirements: 3-camera rig (overhead 60°, wrist 75°, side 45°), 22-DOF proprioception/effort, 22-DOF action label, metadata schema, Frontend UI Hub + Backend data pipeline with sim/real parity.

## Global Constraints

- **Invariant 1 & 4**: All joint coordinates strictly use SI standard radians (rad), 0 rad open pose reference, positive flexion ($q > 0$). State = actual physical joint positions, Action = commanded target positions.
- **Invariant 2 & 13**: Double-domain decoupling. Sim and real share identical topic/data dictionary interfaces. Upstream teleoperation and downstream dataset export remain 100% agnostic to whether the backend is simulation or hardware.
- **Invariant 12**: Joint commands must be validated through `align_command()` and clamped within safety limits before execution.
- **Invariant 23 & 24**: Runtime execution within `tuinadex-control` (Python 3.10.20) for control, simulation, and teleop; `tuinadex-vla` (Python 3.10.20 + CUDA 12.8) for LeRobot dataset export and PyTorch model verification.
- **Exclusive File Ownership**: Each task strictly modifies only its designated locked files to prevent cross-agent collisions.

---

### Task 1: 3-Camera Rig Specification and MuJoCo Simulation Scene Upgrades

**Files:**
- Modify: `src/tuinadex_sim/worlds/tuina_scene.xml:70-85`
- Modify: `src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml:70-85`
- Create: `src/tuinadex_teleop/tuinadex_teleop/cameras/__init__.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/cameras/camera_specs.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/cameras/sim_camera_rig.py`
- Test: `tests/test_camera_rig.py`

**Interfaces:**
- Consumes: `src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml`
- Produces:
  - `CameraSpec`: Dataclass defining name, camera_type (`EYE_TO_HAND` / `EYE_IN_HAND`), width=640, height=480, fovy.
  - `STANDARD_CAMERA_SPECS`: Dict with `"overhead"` (fovy=60), `"wrist"` (fovy=75), `"side"` (fovy=45).
  - `SimCameraRig(model, data)`: `render_camera(name: str) -> np.ndarray (480, 640, 3)`, `render_all() -> dict[str, np.ndarray]`.

- [ ] **Step 1: Write failing test for CameraSpec and SimCameraRig**

```python
# tests/test_camera_rig.py
import pytest
import numpy as np
from pathlib import Path

from tuinadex_teleop.cameras.camera_specs import (
    CameraSpec,
    CameraMountType,
    STANDARD_CAMERA_SPECS,
)
from tuinadex_teleop.cameras.sim_camera_rig import SimCameraRig

def test_camera_spec_definitions():
    assert "overhead" in STANDARD_CAMERA_SPECS
    assert "wrist" in STANDARD_CAMERA_SPECS
    assert "side" in STANDARD_CAMERA_SPECS

    overhead = STANDARD_CAMERA_SPECS["overhead"]
    assert overhead.width == 640
    assert overhead.height == 480
    assert overhead.fovy == 60.0
    assert overhead.mount_type == CameraMountType.EYE_TO_HAND

    wrist = STANDARD_CAMERA_SPECS["wrist"]
    assert wrist.width == 640
    assert wrist.height == 480
    assert wrist.fovy == 75.0
    assert wrist.mount_type == CameraMountType.EYE_IN_HAND

    side = STANDARD_CAMERA_SPECS["side"]
    assert side.width == 640
    assert side.height == 480
    assert side.fovy == 45.0
    assert side.mount_type == CameraMountType.EYE_TO_HAND

def test_sim_camera_rig_rendering():
    import mujoco
    scene_xml = Path(__file__).resolve().parent.parent / "src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml"
    if not scene_xml.exists():
        pytest.skip("Simulation scene not found")
    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    rig = SimCameraRig(model, data)
    frames = rig.render_all()

    assert set(frames.keys()) == {"overhead", "wrist", "side"}
    for cam_name, img in frames.items():
        assert img.shape == (480, 640, 3)
        assert img.dtype == np.uint8
        # Ensure rendered image is non-trivial (not all black)
        assert img.mean() > 5.0, f"Camera {cam_name} image is blank"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_camera_rig.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tuinadex_teleop.cameras'`

- [ ] **Step 3: Update MJCF scenes with overhead, wrist, and side camera models**

In `src/tuinadex_sim/worlds/tuina_scene.xml` and `src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml`:
1. Add `overhead_cam` at `pos="0.20 -0.80 1.70"` pitched down ~50° overlooking the bed and arm base (`fovy="60"`).
2. Attach `wrist_cam` directly to `link6` / `leap_hand_palm` facing forward towards fingertips (`fovy="75"`).
3. Add `side_cam` at `pos="0.20 -1.35 0.78"` parallel to the mattress contact surface (`fovy="45"`).

- [ ] **Step 4: Implement CameraSpec, SimCameraRig and export in cameras/__init__.py**

Implement `CameraSpec`, `CameraMountType`, `STANDARD_CAMERA_SPECS` in `camera_specs.py`.
Implement `SimCameraRig` in `sim_camera_rig.py` using `mujoco.Renderer` with caching to prevent OpenGL memory leaks.

- [ ] **Step 5: Run tests and ensure all pass**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_camera_rig.py -v`
Expected: PASS (2/2)

- [ ] **Step 6: Commit Task 1 changes**

Run: `git add src/tuinadex_sim/worlds/tuina_scene.xml src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml src/tuinadex_teleop/tuinadex_teleop/cameras/ tests/test_camera_rig.py && git commit -m "feat(cameras): implement standard 3-camera rig and MJCF sim views"`

---

### Task 2: Multi-Modal Proprioception, Effort, Contact Forces and Metadata EpisodeBuffer

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/collection/multi_modal_buffer.py`
- Modify: `src/tuinadex_teleop/tuinadex_teleop/collection/__init__.py`
- Test: `tests/test_multi_modal_buffer.py`

**Interfaces:**
- Consumes: `CameraSpec`, `RobotObservation`, `TeleopCommand`
- Produces:
  - `MultiModalFrame`: Dataclass encapsulating:
    - `images`: `dict[str, np.ndarray]` (`overhead`, `wrist`, `side` each `(480, 640, 3)`)
    - `state`: `np.ndarray` (float32[22]: 6 arm + 16 hand)
    - `effort`: `np.ndarray` (float32[22]: 6 arm torque Nm + 16 hand current/effort)
    - `contact_forces`: `np.ndarray` (float32[4]: index, middle, ring, thumb normal force N)
    - `action`: `np.ndarray` (float32[22]: target positions)
    - `timestamp`: float (relative seconds)
    - `frame_index`: int
    - `episode_index`: int
    - `next_done`: bool
    - `language_instruction`: str
    - `phase_label`: str
  - `MultiModalEpisodeBuffer`: Buffer with `append()`, `finish_episode()`, `as_dict()`, `clear()`, `validate_continuity()`.

- [ ] **Step 1: Write failing test for MultiModalEpisodeBuffer**

```python
# tests/test_multi_modal_buffer.py
import pytest
import numpy as np

from tuinadex_teleop.collection.multi_modal_buffer import (
    MultiModalFrame,
    MultiModalEpisodeBuffer,
)

def test_multi_modal_frame_validation():
    frame = MultiModalFrame(
        images={
            "overhead": np.zeros((480, 640, 3), dtype=np.uint8),
            "wrist": np.zeros((480, 640, 3), dtype=np.uint8),
            "side": np.zeros((480, 640, 3), dtype=np.uint8),
        },
        state=np.zeros(22, dtype=np.float32),
        effort=np.zeros(22, dtype=np.float32),
        contact_forces=np.zeros(4, dtype=np.float32),
        action=np.zeros(22, dtype=np.float32),
        timestamp=0.04,
        frame_index=2,
        episode_index=0,
        next_done=False,
        language_instruction="按压大椎穴",
        phase_label="press",
    )
    assert frame.state.shape == (22,)
    assert frame.effort.shape == (22,)
    assert frame.contact_forces.shape == (4,)
    assert frame.action.shape == (22,)
    assert len(frame.images) == 3

def test_multi_modal_episode_buffer_lifecycle():
    buf = MultiModalEpisodeBuffer(episode_index=1, language_instruction="推拿天宗穴")
    assert len(buf) == 0

    for i in range(10):
        buf.add_frame(
            images={
                "overhead": np.full((480, 640, 3), i, dtype=np.uint8),
                "wrist": np.full((480, 640, 3), i + 1, dtype=np.uint8),
                "side": np.full((480, 640, 3), i + 2, dtype=np.uint8),
            },
            state=np.ones(22, dtype=np.float32) * (i * 0.01),
            effort=np.ones(22, dtype=np.float32) * (i * 0.1),
            contact_forces=np.array([i, i, i, i], dtype=np.float32) * 0.5,
            action=np.ones(22, dtype=np.float32) * (i * 0.02),
            timestamp=i * 0.02,
            phase_label="press" if i < 8 else "release",
        )

    assert len(buf) == 10
    buf.seal_episode()
    data = buf.as_dict()

    assert data["observation.state"].shape == (10, 22)
    assert data["observation.effort"].shape == (10, 22)
    assert data["observation.contact_forces"].shape == (10, 4)
    assert data["action"].shape == (10, 22)
    assert data["timestamp"].shape == (10, 1)
    assert data["frame_index"].shape == (10, 1)
    assert data["episode_index"].shape == (10, 1)
    assert data["next.done"].shape == (10, 1)
    assert data["next.done"][-1, 0] is True
    assert data["next.done"][0, 0] is False
    assert len(data["images"]["overhead"]) == 10
```

- [ ] **Step 2: Run test to verify it fails**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_multi_modal_buffer.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tuinadex_teleop.collection.multi_modal_buffer'`

- [ ] **Step 3: Implement MultiModalFrame and MultiModalEpisodeBuffer**

In `src/tuinadex_teleop/tuinadex_teleop/collection/multi_modal_buffer.py`:
Implement dataclasses, numpy conversions, validation guards, and sealing logic ensuring `next.done` is automatically tagged on the final frame.

- [ ] **Step 4: Run tests and ensure all pass**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_multi_modal_buffer.py -v`
Expected: PASS (2/2)

- [ ] **Step 5: Commit Task 2 changes**

Run: `git add src/tuinadex_teleop/tuinadex_teleop/collection/ tests/test_multi_modal_buffer.py && git commit -m "feat(collection): implement multi-modal episode buffer with 22D state, 3-camera, and metadata"`

---

### Task 3: Dual-Channel Teleoperation Data Stream Adapter (Sim & Real Channel Parity)

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/collection/data_channel.py`
- Modify: `src/tuinadex_teleop/tuinadex_teleop/collection/__init__.py`
- Test: `tests/test_data_channel.py`

**Interfaces:**
- Consumes: `SimCameraRig`, `MujocoSimAdapter`, `ROS2RobotEnv`
- Produces:
  - `BaseDataChannel(ABC)`:
    - `get_camera_frames() -> dict[str, np.ndarray]`
    - `get_proprioception() -> tuple[np.ndarray, np.ndarray, np.ndarray]` (q[22], effort[22], contact_forces[4])
    - `get_action_label() -> np.ndarray` (target_q[22])
  - `SimDataChannel(sim_adapter, camera_rig, latest_teleop_cmd)`
  - `RealDataChannel(ros2_env, real_camera_rig)`

- [ ] **Step 1: Write failing test for SimDataChannel and RealDataChannel**

```python
# tests/test_data_channel.py
import pytest
import numpy as np

from tuinadex_teleop.collection.data_channel import (
    BaseDataChannel,
    SimDataChannel,
    MockDataChannel,
)

def test_mock_data_channel_contract():
    channel = MockDataChannel()
    imgs = channel.get_camera_frames()
    assert set(imgs.keys()) == {"overhead", "wrist", "side"}
    assert imgs["overhead"].shape == (480, 640, 3)

    q, effort, forces = channel.get_proprioception()
    assert q.shape == (22,)
    assert effort.shape == (22,)
    assert forces.shape == (4,)

    act = channel.get_action_label()
    assert act.shape == (22,)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_data_channel.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tuinadex_teleop.collection.data_channel'`

- [ ] **Step 3: Implement BaseDataChannel, SimDataChannel, RealDataChannel, and MockDataChannel**

In `src/tuinadex_teleop/tuinadex_teleop/collection/data_channel.py`:
Implement channel abstractions isolating backend specifics. `SimDataChannel` extracts state and forces from `MujocoSimAdapter` and images from `SimCameraRig`. `RealDataChannel` bridges to `/arm/joint_states`, `/hand/joint_states`, and ROS 2 camera topics.

- [ ] **Step 4: Run tests and ensure all pass**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_data_channel.py -v`
Expected: PASS

- [ ] **Step 5: Commit Task 3 changes**

Run: `git add src/tuinadex_teleop/tuinadex_teleop/collection/data_channel.py tests/test_data_channel.py && git commit -m "feat(collection): implement dual-channel sim and real data stream adapters"`

---

### Task 4: Lightweight High-Performance Web UI Hub & REST/SSE Server

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/__init__.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/server.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/static/index.html`
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/static/app.js`
- Create: `src/tuinadex_teleop/tuinadex_teleop/hub/static/style.css`
- Test: `tests/test_collection_hub_api.py`

**Interfaces:**
- Consumes: `DataHubController`, `MultiModalEpisodeBuffer`
- Produces:
  - `CollectionHubServer(host, port, controller)`: Background HTTP/SSE server.
  - Endpoints:
    - `GET /`: Serves web dashboard HTML5 UI.
    - `GET /api/status`: JSON session status, FPS, episode stats, safety level.
    - `POST /api/action/start`: Trigger recording.
    - `POST /api/action/pause`: Pause recording.
    - `POST /api/action/resume`: Resume recording.
    - `POST /api/action/stop`: Stop recording and enter review.
    - `POST /api/action/review`: Submit decision (`save=True/False`, label, instruction).
    - `GET /video/{camera_name}`: Low-latency MJPEG video stream (30-50 FPS).
    - `GET /events/telemetry`: Server-Sent Events (SSE) streaming real-time 22D angles and forces.

- [ ] **Step 1: Write failing test for CollectionHubServer REST and MJPEG endpoints**

```python
# tests/test_collection_hub_api.py
import pytest
import urllib.request
import json
import time

from tuinadex_teleop.collection.data_channel import MockDataChannel
from tuinadex_teleop.collection.data_hub_controller import DataHubController
from tuinadex_teleop.hub.server import CollectionHubServer

@pytest.fixture
def hub_server():
    channel = MockDataChannel()
    controller = DataHubController(data_channel=channel, repo_id="/tmp/tuina_hub_test")
    server = CollectionHubServer(host="127.0.0.1", port=18899, controller=controller)
    server.start()
    time.sleep(0.2)
    yield server, controller
    server.stop()

def test_hub_status_and_control_api(hub_server):
    server, ctrl = hub_server
    url_base = "http://127.0.0.1:18899"

    # 1. Status API
    with urllib.request.urlopen(f"{url_base}/api/status") as resp:
        assert resp.status == 200
        data = json.loads(resp.read().decode())
        assert data["state"] == "IDLE"
        assert "cameras" in data

    # 2. Start Recording
    req = urllib.request.Request(f"{url_base}/api/action/start", data=b"{}", headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req) as resp:
        assert resp.status == 200
        data = json.loads(resp.read().decode())
        assert data["state"] == "RECORDING"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_collection_hub_api.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tuinadex_teleop.hub'`

- [ ] **Step 3: Implement CollectionHubServer and DataHubController**

1. Implement `DataHubController` managing FSM (`IDLE`, `RECORDING`, `PAUSED`, `REVIEWING`), trial buffering, and data saving.
2. Implement `CollectionHubServer` using Python stdlib `ThreadingHTTPServer` with non-blocking handlers for REST, SSE, and MJPEG streaming (`cv2.imencode('.jpg')`).
3. Create `index.html`, `style.css`, and `app.js` providing:
   - 3-column live camera monitor (`overhead`, `wrist`, `side`).
   - Telemetry gauges (22D joint sliders/bars, 4 fingertip force meters).
   - Touch/click buttons and keyboard event listeners (`Space`=start/stop, `S`=save, `D`=discard).
   - Task label and massage phase tag input controls.

- [ ] **Step 4: Run tests and ensure all pass**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_collection_hub_api.py -v`
Expected: PASS

- [ ] **Step 5: Commit Task 4 changes**

Run: `git add src/tuinadex_teleop/tuinadex_teleop/hub/ tests/test_collection_hub_api.py && git commit -m "feat(hub): implement lightweight Web UI hub server, REST API, SSE, and MJPEG streams"`

---

### Task 5: 3-Camera Video Chunking & LeRobotDataset v3.0 Multi-Modal Exporter

**Files:**
- Create: `dataset/tuinadex_dataset/multi_modal_v3_writer.py`
- Modify: `dataset/tuinadex_dataset/__init__.py`
- Test: `tests/test_multi_modal_v3_exporter.py`

**Interfaces:**
- Consumes: `MultiModalEpisodeBuffer`
- Produces:
  - `MultiModalLeRobotWriter(repo_id, fps=50.0)`: Encodes `overhead`, `wrist`, `side` image sequences into separate MP4 video files (`videos/observation.images.overhead/`, etc.) and states/actions/metadata into Apache Parquet chunks complying with `features` schema.

- [ ] **Step 1: Write failing test for MultiModalLeRobotWriter**

```python
# tests/test_multi_modal_v3_exporter.py
import pytest
import numpy as np
from pathlib import Path

from tuinadex_teleop.collection.multi_modal_buffer import MultiModalEpisodeBuffer
from tuinadex_dataset.multi_modal_v3_writer import MultiModalLeRobotWriter

def test_multi_modal_lerobot_export(tmp_path):
    repo_dir = tmp_path / "tuina_v3_3cam"
    writer = MultiModalLeRobotWriter(repo_dir=repo_dir, fps=50.0)

    buf = MultiModalEpisodeBuffer(episode_index=0, language_instruction="按压肺俞穴")
    for i in range(20):
        buf.add_frame(
            images={
                "overhead": np.full((480, 640, 3), 100 + i, dtype=np.uint8),
                "wrist": np.full((480, 640, 3), 120 + i, dtype=np.uint8),
                "side": np.full((480, 640, 3), 140 + i, dtype=np.uint8),
            },
            state=np.ones(22, dtype=np.float32) * (i * 0.01),
            effort=np.ones(22, dtype=np.float32) * (i * 0.1),
            contact_forces=np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float32),
            action=np.ones(22, dtype=np.float32) * (i * 0.02),
            timestamp=i * 0.02,
            phase_label="kneading",
        )
    buf.seal_episode()
    writer.write_episode(buf)
    writer.close()

    assert (repo_dir / "meta/info.json").exists()
    assert (repo_dir / "videos/observation.images.overhead").exists()
    assert (repo_dir / "videos/observation.images.wrist").exists()
    assert (repo_dir / "videos/observation.images.side").exists()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_multi_modal_v3_exporter.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tuinadex_dataset.multi_modal_v3_writer'`

- [ ] **Step 3: Implement MultiModalLeRobotWriter in tuinadex_dataset**

Implement PyAV video encoding for the 3 distinct camera keys and metadata dictionary matching LeRobot v3.0 standard.

- [ ] **Step 4: Run tests and ensure all pass**

Run: `~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_multi_modal_v3_exporter.py -v`
Expected: PASS

- [ ] **Step 5: Commit Task 5 changes**

Run: `git add dataset/tuinadex_dataset/ tests/test_multi_modal_v3_exporter.py && git commit -m "feat(dataset): implement 3-camera video chunking and LeRobot v3.0 multi-modal exporter"`

---

### Task 6: Unified Data Collection CLI and End-to-End Automated Regression

**Files:**
- Create: `scripts/run_collection_hub.py`
- Create: `scripts/e2e_hub_collection_check.sh`
- Modify: `AGENTS.md`
- Modify: `README.md`

**Interfaces:**
- Produces:
  - Command: `python scripts/run_collection_hub.py --env sim|real --port 8080 --cameras overhead,wrist,side --mode arm_hand`
  - Automated check: `bash scripts/e2e_hub_collection_check.sh`

- [ ] **Step 1: Implement scripts/run_collection_hub.py**

Tie together `DataHubController`, `SimDataChannel` / `RealDataChannel`, `CollectionHubServer`, and the live teleoperation loop. Add `--headless` and `--no-web` options for CI testing.

- [ ] **Step 2: Implement scripts/e2e_hub_collection_check.sh**

Create automated test script that:
1. Spawns `run_collection_hub.py --mock --headless --port 18990` in background.
2. Triggers `start`, simulates 30 frames, triggers `stop`, and submits `save` via REST API.
3. Validates the generated LeRobotDataset v3.0 directory (3 MP4 video streams, 22D state, 22D action, metadata).
4. Asserts all exit codes are 0.

- [ ] **Step 3: Execute e2e_hub_collection_check.sh and verify PASS**

Run: `bash scripts/e2e_hub_collection_check.sh`
Expected: PASS

- [ ] **Step 4: Update AGENTS.md, README.md and Commit**

Run: `git add scripts/run_collection_hub.py scripts/e2e_hub_collection_check.sh AGENTS.md README.md && git commit -m "feat(hub): add unified collection hub CLI and automated E2E verification script"`

