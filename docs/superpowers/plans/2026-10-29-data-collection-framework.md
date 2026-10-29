# Multi-Modal Data Collection Framework with LeRobotDataset v3.0 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Build a layered, multi-modal robot data collection framework in `tuinadex_teleop` supporting seamless switching between MuJoCo simulation and real robot via ROS 2 topic interfaces, dynamic 16D/22D action-state adaptation, and automated multi-camera LeRobotDataset v3.0 export.

**Architecture:**
A 4-tier decoupled pipeline:
1. `BaseTuinaEnv` provides a Gym-style interface (`reset`, `step`, `get_obs`, `close`) with standard multi-modal observation dictionaries `{"state": (N,), "images": {"head": (H,W,3), "wrist": ...}}`. `ROS2RobotEnv` connects to ROS 2 topics (`/hand/joint_command`, `/arm/joint_command`, `/hand/joint_states`, `/arm/joint_states`, `/camera/...`) with thread-safe cached queues. `MuJoCoEnv` and `RealRobotEnv` inherit from `ROS2RobotEnv` (with a fast direct-rendering fallback in `MuJoCoEnv` for standalone testing).
2. `RobotConfig` dynamically governs action/state dimensions (`mode="hand_only"` 16D vs `mode="arm_hand"` 22D) and camera streams.
3. `TuinaCollector` & `EpisodeBuffer` manage interactive collection sessions with an FSM (`IDLE`, `RECORDING`, `PAUSED`, `REVIEWING`) supporting keyboard-triggered trial review (save/discard/restart).
4. `LeRobotDatasetWriter` encodes multi-modal episode buffers directly into native `LeRobotDataset v3.0` formats (MP4 video chunking for cameras + Parquet for states/actions).

**Tech Stack:**
- Python 3.10 / 3.14, ROS 2 Lyrical Luth, rclpy, sensor_msgs, cv_bridge / OpenCV
- Hugging Face LeRobot 0.4.4 (Codebase v3.0), PyTorch, PyAV / FFmpeg
- NumPy, pytest

**Spec:**
- `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`
- `docs/adrs/0002-architecture-decisions-sim.md`
- Discussion decisions recorded in session: ROS 2 topic bridge, adaptive dimensions (16D/22D), RGB multi-camera inclusion (`head`/`wrist`), native LeRobotDataset v3.0 auto-conversion.

## Global Constraints

- **Invariants 1 & 4**: SI rad units, 0 rad open pose, positive flexion ($q > 0$). State = `/hand/joint_states`, Action = `/hand/joint_command`.
- **Invariants 2 & 13**: Double-domain decoupling. ROS 2 is the runtime interface. Sim and real share identical ROS 2 topic contracts.
- **Invariant 12**: Commands must pass `align_command()` and `JOINT_SPEC_LIMITS` before reaching actuators.
- **Invariant 23 & 24**: Execution environments: `tuinadex-control` for ROS 2 / teleop / simulation; `tuinadex-vla` for LeRobot dataset export and PyTorch tests.
- **Exclusive File Ownership**: Each task strictly touches only its designated files.

---

### Task 1: RobotConfig & BaseTuinaEnv Abstraction with Adaptive Dimensions

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/envs/__init__.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/envs/config.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/envs/base_env.py`
- Modify: `src/tuinadex_teleop/setup.py` (add `tuinadex_teleop.envs`)
- Test: `tests/test_robot_env_spec.py`

**Interfaces:**
- Consumes: `tuinadex_core.names.CANONICAL_JOINT_NAMES`, `tuinadex_core.names.CANONICAL_ARM_JOINT_NAMES`
- Produces:
  - `RobotConfig(mode="hand_only"|"arm_hand", camera_names=("head", "wrist"), fps=50.0)`
  - `BaseTuinaEnv(ABC)`: `reset() -> dict`, `step(action: np.ndarray) -> tuple[dict, dict]`, `get_observation() -> dict`, `close() -> None`
  - Observation format: `{"state": np.ndarray (float32), "images": dict[str, np.ndarray], "timestamp": float}`

- [x] **Step 1: Write failing test for RobotConfig and BaseTuinaEnv**

```python
# tests/test_robot_env_spec.py
import pytest
import numpy as np
from tuinadex_teleop.envs.config import RobotConfig
from tuinadex_teleop.envs.base_env import BaseTuinaEnv

def test_robot_config_adaptive_dimensions():
    cfg_hand = RobotConfig(mode="hand_only")
    assert cfg_hand.action_dim == 16
    assert len(cfg_hand.joint_names) == 16

    cfg_arm_hand = RobotConfig(mode="arm_hand")
    assert cfg_arm_hand.action_dim == 22
    assert len(cfg_arm_hand.joint_names) == 22
    assert cfg_arm_hand.joint_names[:6] == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

def test_base_tuina_env_contract():
    class DummyEnv(BaseTuinaEnv):
        def reset(self):
            return self.get_observation()
        def step(self, action):
            assert len(action) == self.config.action_dim
            return self.get_observation(), {}
        def get_observation(self):
            return {
                "state": np.zeros(self.config.action_dim, dtype=np.float32),
                "images": {"head": np.zeros((480, 640, 3), dtype=np.uint8)},
                "timestamp": 1000.0,
            }
        def close(self):
            pass

    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    env = DummyEnv(cfg)
    obs = env.reset()
    assert obs["state"].shape == (16,)
    assert "head" in obs["images"]
    obs2, info = env.step(np.zeros(16, dtype=np.float32))
    assert obs2["state"].shape == (16,)
```

- [x] **Step 2: Run test to verify it fails**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_robot_env_spec.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tuinadex_teleop.envs'`

- [x] **Step 3: Implement RobotConfig, BaseTuinaEnv, and update setup.py**

Create `src/tuinadex_teleop/tuinadex_teleop/envs/config.py`:
- `mode`: `"hand_only"` | `"arm_hand"`
- Computes `joint_names`, `action_dim`
- Validates cameras list

Create `src/tuinadex_teleop/tuinadex_teleop/envs/base_env.py`:
- Abstract class with standard typing, validation helper `validate_action(action: np.ndarray)`.

Update `src/tuinadex_teleop/setup.py` packages list to include `tuinadex_teleop.envs`.

- [x] **Step 4: Run test to verify it passes**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_robot_env_spec.py -v`
Expected: PASS (2 passed)

- [x] **Step 5: Commit changes**

```bash
git add src/tuinadex_teleop/ tests/test_robot_env_spec.py
git commit -m "feat(teleop): implement RobotConfig and BaseTuinaEnv abstraction"
```

---

### Task 2: ROS2RobotEnv with Topic Sync, Image Buffers & Action Gates

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/envs/ros2_robot_env.py`
- Test: `tests/test_ros2_robot_env.py`

**Interfaces:**
- Consumes:
  - `BaseTuinaEnv`, `RobotConfig`
  - `tuinadex_core.command_processing.align_command`
  - `rclpy.node.Node`, `sensor_msgs.msg.JointState`, `sensor_msgs.msg.Image`
- Produces:
  - `ROS2RobotEnv(BaseTuinaEnv)`: manages ROS 2 pub/sub, thread-safe observation synchronization with `cv_bridge` or direct buffer unpacking, and gated action dispatching.

- [x] **Step 1: Write failing test for ROS2RobotEnv with mock ROS node callbacks**

```python
# tests/test_ros2_robot_env.py
import pytest
import numpy as np

rclpy = pytest.importorskip("rclpy")

def test_ros2_robot_env_observation_sync():
    from tuinadex_teleop.envs.config import RobotConfig
    from tuinadex_teleop.envs.ros2_robot_env import ROS2RobotEnv
    
    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    env = ROS2RobotEnv(cfg, node_name="test_collector_env")
    
    # Simulate receiving joint states and image
    env._on_hand_state(mock_joint_state_msg(16))
    env._on_image("head", mock_image_msg(480, 640))
    
    obs = env.get_observation()
    assert obs["state"].shape == (16,)
    assert "head" in obs["images"]
    assert obs["images"]["head"].shape == (480, 640, 3)
    
    # Step with action validation
    obs2, info = env.step(np.zeros(16, dtype=np.float32))
    assert obs2["state"].shape == (16,)
    env.close()
```

- [x] **Step 2: Run test to verify it fails**

Run: `source /opt/ros/lyrical/setup.bash && python3 -m pytest tests/test_ros2_robot_env.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tuinadex_teleop.envs.ros2_robot_env'`

- [x] **Step 3: Implement ROS2RobotEnv**

In `src/tuinadex_teleop/tuinadex_teleop/envs/ros2_robot_env.py`:
- Initialize node (or accept external node).
- Publishers: `/hand/joint_command` (`JointState`), `/arm/joint_command` (if `mode="arm_hand"`).
- Subscribers: `/hand/joint_states`, `/arm/joint_states`, `/camera/{name}/image_raw`.
- Convert `sensor_msgs/Image` to NumPy array (supporting `rgb8`, `bgr8`).
- Thread-safe latest-value lock.
- In `step(action)`: validate and clamp action via `align_command()` / `JOINT_SPEC_LIMITS`, publish command, and wait for next tick or return latest observation.

- [x] **Step 4: Run test to verify it passes**

Run: `source /opt/ros/lyrical/setup.bash && python3 -m pytest tests/test_ros2_robot_env.py -v`
Expected: PASS

- [x] **Step 5: Commit changes**

```bash
git add src/tuinadex_teleop/tuinadex_teleop/envs/ros2_robot_env.py tests/test_ros2_robot_env.py
git commit -m "feat(teleop): implement ROS2RobotEnv topic bridge and observation sync"
```

---

### Task 3: Specialized MuJoCoEnv and RealRobotEnv + Factory Pattern

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/envs/mujoco_env.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/envs/real_env.py`
- Modify: `src/tuinadex_teleop/tuinadex_teleop/envs/__init__.py`
- Test: `tests/test_env_factory.py`

**Interfaces:**
- Consumes: `ROS2RobotEnv`, `RobotConfig`
- Produces:
  - `MuJoCoEnv(ROS2RobotEnv)`: simulation specialization, with optional direct in-process bridge fallback for fast unit testing.
  - `RealRobotEnv(ROS2RobotEnv)`: physical hardware specialization, checks node connectivity and safety guards.
  - `make_tuina_env(env_type: str, config: RobotConfig, **kwargs) -> BaseTuinaEnv`

- [x] **Step 1: Write failing test for env factory and specialization**

```python
# tests/test_env_factory.py
import pytest
from tuinadex_teleop.envs.config import RobotConfig
from tuinadex_teleop.envs import make_tuina_env, BaseTuinaEnv

def test_make_tuina_env_factory():
    cfg = RobotConfig(mode="hand_only")
    env_sim = make_tuina_env("sim", cfg, standalone=True)
    assert isinstance(env_sim, BaseTuinaEnv)
    obs = env_sim.reset()
    assert obs["state"].shape == (16,)
    env_sim.close()
```

- [x] **Step 2: Run test to verify it fails**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_env_factory.py -v`
Expected: FAIL with `cannot import name 'make_tuina_env'`

- [x] **Step 3: Implement MuJoCoEnv, RealRobotEnv, and make_tuina_env**

- `MuJoCoEnv`: inherits `ROS2RobotEnv`; when `standalone=True`, initializes `MujocoSimBridge` directly to allow offline/headless simulation testing without running ROS2 daemons.
- `RealRobotEnv`: inherits `ROS2RobotEnv`; adds startup health-check (pings topics).
- `make_tuina_env`: factory function resolving `"sim"` $\rightarrow$ `MuJoCoEnv`, `"real"` $\rightarrow$ `RealRobotEnv`.

- [x] **Step 4: Run test to verify it passes**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_env_factory.py -v`
Expected: PASS

- [x] **Step 5: Commit changes**

```bash
git add src/tuinadex_teleop/tuinadex_teleop/envs/ tests/test_env_factory.py
git commit -m "feat(teleop): implement MuJoCoEnv, RealRobotEnv, and make_tuina_env factory"
```

---

### Task 4: Multi-Modal EpisodeBuffer and TuinaCollector FSM

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/collection/__init__.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/collection/episode_buffer.py`
- Create: `src/tuinadex_teleop/tuinadex_teleop/collection/collector.py`
- Modify: `src/tuinadex_teleop/setup.py` (add `tuinadex_teleop.collection`)
- Test: `tests/test_episode_buffer.py`
- Test: `tests/test_collector_fsm.py`

**Interfaces:**
- Consumes: `BaseTuinaEnv`, `RobotConfig`
- Produces:
  - `EpisodeBuffer`: `add_frame(state, action, images, timestamp)`, `clear()`, `__len__()`, `to_dict()`
  - `TuinaCollector`: `start_episode()`, `pause_episode()`, `stop_episode()`, `review_decision(save: bool)`

- [x] **Step 1: Write failing tests for EpisodeBuffer and Collector FSM**

```python
# tests/test_episode_buffer.py
import numpy as np
from tuinadex_teleop.collection.episode_buffer import EpisodeBuffer

def test_episode_buffer_lifecycle():
    buf = EpisodeBuffer()
    assert len(buf) == 0
    for i in range(10):
        buf.add_frame(
            state=np.zeros(16, dtype=np.float32),
            action=np.ones(16, dtype=np.float32),
            images={"head": np.zeros((100, 100, 3), dtype=np.uint8)},
            timestamp=float(i) * 0.02,
        )
    assert len(buf) == 10
    data = buf.to_dict()
    assert data["state"].shape == (10, 16)
    assert data["action"].shape == (10, 16)
    assert len(data["images"]["head"]) == 10
    buf.clear()
    assert len(buf) == 0
```

- [x] **Step 2: Run test to verify it fails**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_episode_buffer.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tuinadex_teleop.collection'`

- [x] **Step 3: Implement EpisodeBuffer and TuinaCollector FSM**

In `src/tuinadex_teleop/tuinadex_teleop/collection/episode_buffer.py`:
- Dataclass/List-backed buffer holding frames.
- Vectorized array conversion in `to_dict()`.

In `src/tuinadex_teleop/tuinadex_teleop/collection/collector.py`:
- `CollectionState` Enum: `IDLE`, `RECORDING`, `PAUSED`, `REVIEWING`.
- FSM transition methods.
- Periodic 50Hz step loop taking actions from a teleoperator callback and writing to `EpisodeBuffer`.

- [x] **Step 4: Run test to verify it passes**

Run: `~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_episode_buffer.py tests/test_collector_fsm.py -v`
Expected: PASS

- [x] **Step 5: Commit changes**

```bash
git add src/tuinadex_teleop/tuinadex_teleop/collection/ tests/test_episode_buffer.py tests/test_collector_fsm.py
git commit -m "feat(teleop): implement EpisodeBuffer and TuinaCollector FSM"
```

---

### Task 5: Multi-Modal LeRobotDataset v3.0 Exporter (Video Chunking + Parquet)

**Files:**
- Create: `dataset/tuinadex_dataset/lerobot_exporter.py`
- Modify: `dataset/tuinadex_dataset/adapter.py` (integrate v3 exporter)
- Test: `tests/test_lerobot_v3_exporter.py`

**Interfaces:**
- Consumes:
  - `EpisodeBuffer`
  - `RobotConfig`
  - `lerobot.datasets.lerobot_dataset.LeRobotDataset` (v3.0)
- Produces:
  - `LeRobotDatasetWriter(repo_id: str, config: RobotConfig, fps: float = 50.0)`:
    - `save_episode(buffer: EpisodeBuffer, task: str = "tuina_massage") -> str`
    - Encodes RGB cameras to MP4 video chunks (`use_videos=True`)
    - Validates `meta/info.json` codebase version (`v3.0`)

- [x] **Step 1: Write failing test for LeRobotDatasetWriter with synthetic multi-camera episode**

```python
# tests/test_lerobot_v3_exporter.py
import pytest
import numpy as np
from pathlib import Path
import json

pytest.importorskip("lerobot")

def test_lerobot_v3_multi_modal_export(tmp_path):
    from tuinadex_teleop.envs.config import RobotConfig
    from tuinadex_teleop.collection.episode_buffer import EpisodeBuffer
    from tuinadex_dataset.lerobot_exporter import LeRobotDatasetWriter
    
    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    buf = EpisodeBuffer()
    # 20 frames of synthetic RGB data (120x160)
    for i in range(20):
        img = np.full((120, 160, 3), i * 10, dtype=np.uint8)
        buf.add_frame(
            state=np.zeros(16, dtype=np.float32),
            action=np.zeros(16, dtype=np.float32),
            images={"head": img},
            timestamp=float(i) * 0.02,
        )
    
    output_dir = tmp_path / "test_v3_dataset"
    writer = LeRobotDatasetWriter(repo_id=str(output_dir), config=cfg, fps=50)
    res_path = writer.save_episode(buf, task="tuina_demo")
    
    # Verify LeRobotDataset v3.0 metadata and files
    info_path = Path(res_path) / "meta" / "info.json"
    assert info_path.exists()
    with open(info_path) as f:
        meta = json.load(f)
    assert meta["codebase_version"] == "v3.0"
    assert meta["total_frames"] == 20
    assert "observation.images.head" in meta["features"]
    assert meta["features"]["observation.images.head"]["dtype"] == "video"
```

- [x] **Step 2: Run test to verify it fails**

Run: `~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_lerobot_v3_exporter.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tuinadex_dataset.lerobot_exporter'`

- [x] **Step 3: Implement LeRobotDatasetWriter**

In `dataset/tuinadex_dataset/lerobot_exporter.py`:
- Configure features dictionary:
  - `observation.state`: `dtype="float32"`, shape `(config.action_dim,)`, names `config.joint_names`.
  - `action`: `dtype="float32"`, shape `(config.action_dim,)`, names `config.joint_names`.
  - For each camera in `config.camera_names`:
    `observation.images.{cam}`: `dtype="video"`, `shape=(3, H, W)`, `names=["channels", "height", "width"]`.
- Initialize `LeRobotDataset.create(..., use_videos=True)`.
- Loop over buffer frames: convert images to PyTorch `(C, H, W)` tensors, call `add_frame(...)`.
- Call `dataset.save_episode()`.

- [x] **Step 4: Run test to verify it passes**

Run: `~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_lerobot_v3_exporter.py -v`
Expected: PASS (1 passed)

- [x] **Step 5: Commit changes**

```bash
git add dataset/tuinadex_dataset/lerobot_exporter.py tests/test_lerobot_v3_exporter.py
git commit -m "feat(dataset): implement multi-modal LeRobotDataset v3.0 exporter with video chunking"
```

---

### Task 6: Unified CLI Script & End-to-End Multi-Modal Check

**Files:**
- Create: `scripts/collect_teleop_data.py`
- Create: `scripts/e2e_collection_check.sh`
- Modify: `AGENTS.md` (record data collection commands)
- Modify: `README.md` (add data collection quickstart)

**Interfaces:**
- Consumes: All modules from Tasks 1~5
- Produces: Executable CLI tool and self-verifying test script.

- [x] **Step 1: Write e2e_collection_check.sh automation test**

Create `scripts/e2e_collection_check.sh`:
- Runs `collect_teleop_data.py --env sim --mode hand_only --mock-teleop --episodes 1 --frames 50 --output /tmp/tuinadex_e2e_v3`.
- Verifies returncode == 0.
- Verifies `/tmp/tuinadex_e2e_v3/meta/info.json` exists and contains `"codebase_version": "v3.0"`.
- Verifies `/tmp/tuinadex_e2e_v3/data/chunk-000` contains parquet files.
- Verifies `/tmp/tuinadex_e2e_v3/videos/chunk-000` contains mp4 files.

- [x] **Step 2: Run e2e script to verify it fails before script creation**

Run: `bash scripts/e2e_collection_check.sh`
Expected: FAIL (script does not exist yet)

- [x] **Step 3: Implement collect_teleop_data.py and e2e_collection_check.sh**

In `scripts/collect_teleop_data.py`:
- Arguments: `--env sim|real`, `--mode hand_only|arm_hand`, `--cameras head,wrist`, `--repo-id`, `--mock-teleop`, `--episodes`.
- Sets up environment via `make_tuina_env()`.
- Sets up `TuinaCollector`.
- Runs collection loop and flushes through `LeRobotDatasetWriter`.

Make `scripts/e2e_collection_check.sh` executable (`chmod +x`).

- [x] **Step 4: Run e2e test to verify it passes**

Run: `bash scripts/e2e_collection_check.sh`
Expected: Output `[E2E COLLECTION] PASS: LeRobotDataset v3.0 created with videos and parquet`

- [x] **Step 5: Run full test suites across both conda environments**

Run:
```bash
~/win_office/conda/envs/tuinadex-control/bin/python -m pytest tests/test_robot_env_spec.py tests/test_env_factory.py tests/test_episode_buffer.py tests/test_collector_fsm.py -v
~/win_office/conda/envs/tuinadex-vla/bin/python -m pytest tests/test_lerobot_v3_exporter.py -v
```
Expected: All tests PASS.

- [x] **Step 6: Commit changes**

```bash
git add scripts/ docs/ README.md AGENTS.md
git commit -m "feat(collection): deliver unified teleop data collection CLI and v3.0 e2e verification"
```
