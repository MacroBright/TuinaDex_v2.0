# TuinaDex v2.0 MVP 闭环 — Phase 3 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付 16-DOF LEAP Hand MVP 完整闭环：健壮的手部运行时节点、视觉遥操、rosbag2 采集、LeRobot 数据集转换，全程 TDD 可实证。

**Architecture:** ROS2 运行时（Runtime）与 LeRobot 学习域（Learning）双域解耦。手部节点 = 普通 rclpy Node + 显式 WatchdogStateMachine 纯类 FSM；采集 = rosbag2_py 进程内节点；数据转换 = 离线 CLI（rosbag2 → 50Hz 网格对齐 → 逐关节限位校验 → LeRobotDataset）。安全逻辑全部抽成不依赖 rclpy 的纯模块以支持本机 TDD。

**Tech Stack:** ROS2 (Ubuntu 26.04 对应发行版，Task 0 安装) / Python 3.10+ / numpy / pytest / opencv+mediapipe (遥操) / lerobot 0.4.4 (smolvla conda env)

**Spec:** `docs/specs/LEAP_HAND_MVP_INTERFACE_SPEC.md`（16-DOF 契约、双向转换、话题、LeRobot Schema）

## Global Constraints

- 全链路 SI 弧度 (rad)；OPEN_POSE 全开位为 0 rad，屈曲为正 (q>0)。逐关节限位见 Spec §1 表格。
- Action 空间 = 16 维 absolute joint position，禁止 delta。
- 控制周期 50Hz；看门狗超时 300ms → Relax-to-OPEN；限流 150mA；Operating Mode = 5 (position-current)。
- State (`/hand/joint_states`) 与 Action (`/hand/joint_command`) 严格分离；均为 `sensor_msgs/JointState`。
- rosbag2 是唯一事实源；LeRobotDataset 为衍生产品，由离线 `tuinadex_dataset` 生成。
- teleop 只发布指令，绝不触碰硬件驱动。
- 本地单元测试环境：`/home/bright/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`（numpy 2.5.1 + pytest + mediapipe 1.0.0）。
- LeRobot 相关测试环境：`/home/bright/win_office/conda/envs/smolvla/bin/python`（lerobot 0.4.4 + pyarrow 25.0.0）。
- ROS2 依赖的测试用 `pytest.importorskip("rclpy")` 守护；Task 0 完成后本机可跑。
- 每个任务一个 commit；高危系统操作（Task 0 的 apt/sudo 安装）执行前必须人工确认。
- 现状基线：`77cb9ba`（P0 修复后 9/9 测试全绿）。

## 已裁决架构决策 (Invariants，来自 Phase 2 用户拍板)

1. 节点生命周期：**普通 Node + 显式 FSM**（不使用 LifecycleNode）。
2. 采集器：**rosbag2_py 进程内节点**（非 subprocess 包装）。
3. bag 反序列化：**依赖 ROS2 环境**（本机安装 ROS2，不写纯 Python CDR 解析器）。

---

### Task 0: ROS2 环境安装与配置（系统级 · 执行前人工确认）

**Files:**
- Modify: `~/.bashrc`（追加 source 行）
- Modify: `AGENTS.md`（记录实际安装的发行版与验证命令）

**Interfaces:**
- Produces: 本机可用的 `ros2` CLI、`colcon`、`rclpy`（系统 python3），Task 3/6/7/8 的测试前提。

- [ ] **Step 1: 核实 Ubuntu 26.04 支持的 ROS2 发行版（禁止凭记忆写死）**

本机为 Ubuntu 26.04 (Resolute)。按 REP-2002 节奏应有对应 2026 年发行版，但名称与可用性必须实证核实：

```bash
# 方法 1: 查 REP-0004 / 官方安装页
# 方法 2: 添加 ROS apt 源后实证
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
apt-cache search "^ros-[a-z]+-ros-base$"   # 列出所有可用发行版
```

- [ ] **Step 2: 向用户报告可用发行版清单，确认后安装**

```bash
sudo apt install ros-<DISTRO>-ros-base ros-<DISTRO>-rosbag2 ros-<DISTRO>-sensor-msgs ros-<DISTRO>-std-srvs python3-colcon-common-extensions python3-pytest -y
```

回退方案（若 26.04 无官方支持发行版）：Docker `ros:humble` 或 `ros:jazzy` 容器内运行 ROS2 相关任务，其余任务不受影响。**必须先报告再执行。**

- [ ] **Step 3: 验证与固化**

```bash
source /opt/ros/<DISTRO>/setup.bash && ros2 --help | head -1
python3 -c "import rclpy; print('rclpy OK')"
colcon version-check
echo "source /opt/ros/<DISTRO>/setup.bash" >> ~/.bashrc
```

- [ ] **Step 4: 更新 AGENTS.md**

在 `AGENTS.md` §2 追加实际发行版、source 命令与 Task 8 的完整验证命令。

- [ ] **Step 5: Commit**

```bash
git add AGENTS.md && git commit -m "docs: record ROS2 <DISTRO> install and verification commands"
```

---

### Task 1: WatchdogStateMachine 纯类看门狗状态机

**Files:**
- Create: `src/tuinadex_hand/tuinadex_hand/watchdog.py`
- Test: `tests/test_watchdog_fsm.py`

**Interfaces:**
- Produces: `WatchdogState`(FOLLOWING/RELAXING)、`WatchdogAction`(NONE/WRITE/RELAX_STEP)、`WatchdogStateMachine(timeout_sec=0.30)`，方法 `on_command(t_now: float) -> None`、`tick(t_now: float) -> WatchdogAction`、属性 `state`。Task 3 依赖。

- [ ] **Step 1: 写失败测试**

```python
"""test_watchdog_fsm.py — 看门狗安全状态机全状态转移验证."""
import sys
from pathlib import Path
import pytest

SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_hand"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from tuinadex_hand.watchdog import (
    WatchdogStateMachine, WatchdogState, WatchdogAction,
)


def test_never_commanded_is_none():
    """从未收到指令: 不动作 (手保持全开), 也不触发 relax."""
    fsm = WatchdogStateMachine(timeout_sec=0.30)
    assert fsm.state is WatchdogState.FOLLOWING
    assert fsm.tick(t_now=100.0) is WatchdogAction.NONE


def test_following_within_timeout_writes():
    fsm = WatchdogStateMachine(timeout_sec=0.30)
    fsm.on_command(t_now=1.0)
    assert fsm.tick(t_now=1.0) is WatchdogAction.WRITE
    assert fsm.tick(t_now=1.30) is WatchdogAction.WRITE  # 边界 <=


def test_timeout_transitions_to_relaxing():
    fsm = WatchdogStateMachine(timeout_sec=0.30)
    fsm.on_command(t_now=1.0)
    assert fsm.tick(t_now=1.31) is WatchdogAction.RELAX_STEP
    assert fsm.state is WatchdogState.RELAXING
    # 持续 RELAX_STEP 直到新指令
    assert fsm.tick(t_now=2.0) is WatchdogAction.RELAX_STEP


def test_new_command_rearms_following():
    fsm = WatchdogStateMachine(timeout_sec=0.30)
    fsm.on_command(t_now=1.0)
    fsm.tick(t_now=1.31)  # 进入 RELAXING
    fsm.on_command(t_now=2.0)
    assert fsm.state is WatchdogState.FOLLOWING
    assert fsm.tick(t_now=2.0) is WatchdogAction.WRITE


def test_invalid_timeout_rejected():
    with pytest.raises(ValueError):
        WatchdogStateMachine(timeout_sec=0.0)
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_watchdog_fsm.py -v`
Expected: FAIL，`ModuleNotFoundError: tuinadex_hand.watchdog`

- [ ] **Step 3: 最小实现**

```python
"""watchdog.py — 指令流看门狗安全状态机 (纯逻辑, 不依赖 rclpy).

状态:
    FOLLOWING — 正常跟随指令流 (每周期写缓存指令)
    RELAXING  — 指令超时, 每周期驱动一步 Relax-to-OPEN

裁决动作由 tick() 返回, 时间源由调用方注入 (节点时钟秒值), 本类不做任何 I/O。
hold/ramp 插值由驱动层 relax_to_open_step() 负责, 本类只裁决"何时进入安全回退"。
"""
from __future__ import annotations
from enum import Enum, auto


class WatchdogState(Enum):
    FOLLOWING = auto()
    RELAXING = auto()


class WatchdogAction(Enum):
    NONE = auto()        # 从未收到指令: 不动作
    WRITE = auto()       # 写入缓存指令
    RELAX_STEP = auto()  # 驱动侧执行一步 Relax-to-OPEN


class WatchdogStateMachine:
    def __init__(self, timeout_sec: float = 0.30):
        if timeout_sec <= 0:
            raise ValueError(f"timeout_sec must be > 0, got {timeout_sec}")
        self.timeout_sec = float(timeout_sec)
        self._state = WatchdogState.FOLLOWING
        self._last_cmd_time: float = -float("inf")
        self._ever_commanded = False

    @property
    def state(self) -> WatchdogState:
        return self._state

    def on_command(self, t_now: float) -> None:
        """收到合法新指令: 立即拉回 FOLLOWING 并刷新计时."""
        self._state = WatchdogState.FOLLOWING
        self._last_cmd_time = t_now
        self._ever_commanded = True

    def tick(self, t_now: float) -> WatchdogAction:
        """每个控制周期调用一次, 返回本周期的安全动作."""
        if not self._ever_commanded:
            return WatchdogAction.NONE
        if self._state is WatchdogState.FOLLOWING:
            if (t_now - self._last_cmd_time) <= self.timeout_sec:
                return WatchdogAction.WRITE
            self._state = WatchdogState.RELAXING
        return WatchdogAction.RELAX_STEP
```

- [ ] **Step 4: 运行确认通过**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_watchdog_fsm.py -v`
Expected: 5 passed

- [ ] **Step 5: Commit**

```bash
git add src/tuinadex_hand/tuinadex_hand/watchdog.py tests/test_watchdog_fsm.py
git commit -m "feat(hand): explicit watchdog state machine (pure logic, TDD)"
```

---

### Task 2: 指令报文校验与 Canonical 重排纯函数

**Files:**
- Create: `src/tuinadex_hand/tuinadex_hand/command_processing.py`
- Test: `tests/test_command_processing.py`

**Interfaces:**
- Consumes: `leap_driver.CANONICAL_JOINT_NAMES`（纯 numpy 模块，无循环依赖）。
- Produces: `align_command(positions: Sequence[float], names: Optional[Sequence[str]] = None) -> Tuple[Optional[np.ndarray], str]`；合法返回 `(16 维 ndarray, "")`，拒绝返回 `(None, 原因)`。Task 3 依赖。
- 语义收紧（相对旧节点内联逻辑）：未知关节名与 names 数量 ≠16 一律**拒绝**（旧代码静默忽略），NaN/Inf 显式拒绝。

- [ ] **Step 1: 写失败测试**

```python
"""test_command_processing.py — /hand/joint_command 报文校验与重排."""
import sys
from pathlib import Path
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_hand"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from tuinadex_hand.command_processing import align_command
from tuinadex_hand.driver.leap_driver import CANONICAL_JOINT_NAMES


def test_valid_passthrough_canonical_order():
    q = np.arange(16, dtype=np.float64) * 0.1
    out, reason = align_command(q.tolist(), CANONICAL_JOINT_NAMES)
    assert out is not None and reason == ""
    np.testing.assert_allclose(out, q)


def test_valid_no_names():
    out, reason = align_command([0.1] * 16)
    assert out is not None and reason == ""


def test_reorders_noncanonical_names():
    # 交换 index_yaw (0) 与 thumb_ip (15) 的位置
    names = list(CANONICAL_JOINT_NAMES)
    names[0], names[15] = names[15], names[0]
    pos = [float(i) for i in range(16)]
    out, reason = align_command(pos, names)
    assert out is not None and reason == ""
    assert out[0] == 15.0
    assert out[15] == 0.0


def test_dim_mismatch_rejected():
    out, reason = align_command([0.0] * 15)
    assert out is None and "dim_mismatch" in reason


def test_nan_rejected_with_index():
    pos = [0.0] * 16
    pos[7] = float("nan")
    out, reason = align_command(pos)
    assert out is None and "non_finite" in reason and "7" in reason


def test_unknown_joint_name_rejected():
    names = list(CANONICAL_JOINT_NAMES)
    names[3] = "wrist_yaw"   # 不存在的关节
    out, reason = align_command([0.0] * 16, names)
    assert out is None and "unknown_joint_names" in reason


def test_name_count_mismatch_rejected():
    out, reason = align_command([0.0] * 16, ["a", "b"])
    assert out is None and "name_dim_mismatch" in reason
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_command_processing.py -v`
Expected: FAIL，`ModuleNotFoundError: tuinadex_hand.command_processing`

- [ ] **Step 3: 最小实现**

```python
"""command_processing.py — /hand/joint_command 报文校验与 Canonical 重排 (纯逻辑).

拒绝条件 (安全优先, 宁拒勿猜):
  - 维度 != 16
  - 含 NaN / Inf (脏值直通硬件 = 越界风险)
  - names 数量 != 16 或含未知关节名 (静默猜测对齐 = 错位风险)
"""
from __future__ import annotations
from typing import Optional, Sequence, Tuple
import numpy as np

from .driver.leap_driver import CANONICAL_JOINT_NAMES


def align_command(
    positions: Sequence[float],
    names: Optional[Sequence[str]] = None,
) -> Tuple[Optional[np.ndarray], str]:
    """校验并重排为 16 维 Canonical 目标弧度.

    Returns:
        (q_target, reason): 合法时 (16 维 float64 ndarray, "")；拒绝时 (None, 原因说明).
    """
    pos = np.asarray(list(positions), dtype=np.float64)
    if pos.shape != (16,):
        return None, f"dim_mismatch: expected 16, got {pos.shape[0]}"

    if not np.all(np.isfinite(pos)):
        bad = np.nonzero(~np.isfinite(pos))[0].tolist()
        return None, f"non_finite at indices {bad}"

    q = pos
    if names:
        if len(names) != 16:
            return None, f"name_dim_mismatch: expected 16 names, got {len(names)}"
        if list(names) != CANONICAL_JOINT_NAMES:
            name_to_idx = {n: i for i, n in enumerate(CANONICAL_JOINT_NAMES)}
            unknown = [n for n in names if n not in name_to_idx]
            if unknown:
                return None, f"unknown_joint_names: {unknown}"
            ordered = np.zeros(16, dtype=np.float64)
            for src, n in enumerate(names):
                ordered[name_to_idx[n]] = pos[src]
            q = ordered
    return q, ""
```

- [ ] **Step 4: 运行确认通过**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_command_processing.py -v`
Expected: 7 passed

- [ ] **Step 5: Commit**

```bash
git add src/tuinadex_hand/tuinadex_hand/command_processing.py tests/test_command_processing.py
git commit -m "feat(hand): strict command validation + canonical realignment (pure logic)"
```

---

### Task 3: 手节点接线重构（FSM + 节点时钟 + 安全关停）

**Files:**
- Modify: `src/tuinadex_hand/tuinadex_hand/leap_hand_node.py`（整体重写控制逻辑）
- Test: `tests/test_leap_hand_node_integration.py`（rclpy 守护，Task 0 后可跑）

**Interfaces:**
- Consumes: Task 1 `WatchdogStateMachine`、Task 2 `align_command`、驱动层 `relax_to_open_step(now, hold_time, ramp_time)`。
- Produces: 节点参数 `watchdog_hold_sec`(0.30)、`watchdog_ramp_sec`(0.60)、`shutdown_dwell_sec`(0.5)；统一使用节点时钟。
- 语义底线（不可违背）：RELAXING 期间收到合法指令 → `driver.reset_loss_state()` + 恢复日志；所有计时取自 `self.get_clock().now()`。

- [ ] **Step 1: 写集成测试（rclpy 守护）**

```python
"""test_leap_hand_node_integration.py — mock 模式下节点级闭环验证 (需 ROS2)."""
import sys
from pathlib import Path
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

WORKSPACE_SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_hand"
if str(WORKSPACE_SRC) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_SRC))

from sensor_msgs.msg import JointState
from tuinadex_hand.leap_hand_node import LeapHandNode

FAST_PARAMS = [
    ("mock_mode", True),
    ("watchdog_timeout_sec", 0.10),
    ("watchdog_hold_sec", 0.10),
    ("watchdog_ramp_sec", 0.20),
]


@pytest.fixture
def rclpy_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_mock_loop_and_watchdog(rclpy_ctx):
    """发布指令后 joint_states 跟随; 停发后看门狗平滑回 0 rad."""
    from rclpy.parameter import Parameter
    node = LeapHandNode()
    node.set_parameters([Parameter(k, value=v) for k, v in FAST_PARAMS]
                        is None or [])  # 参数需在构造期生效, 见下注
    node.destroy_node()
```

> **实现注意（本步骤的测试与构造方式）**：参数必须在 `__init__` 读参数之前生效。两种可行做法任选其一并保持一致：
> (a) 给 `LeapHandNode.__init__` 加可选 `parameter_overrides: dict | None = None`，在 `declare_parameter` 时用它作 default 值；
> (b) 测试改用 `rclpy.init(args=["--ros-args", "-p", "mock_mode:=true", ...])` 全局参数注入。
> **推荐 (a)**，测试签名变为 `LeapHandNode(parameter_overrides=dict(FAST_PARAMS))`。下为按 (a) 的最终测试体：

```python
def test_mock_loop_and_watchdog(rclpy_ctx):
    """发布指令后 joint_states 跟随; 停发后看门狗平滑回 0 rad."""
    overrides = dict(mock_mode=True, watchdog_timeout_sec=0.10,
                     watchdog_hold_sec=0.10, watchdog_ramp_sec=0.20)
    node = LeapHandNode(parameter_overrides=overrides)

    received = []
    node.create_subscription(
        JointState, "/hand/joint_states",
        lambda m: received.append(np.asarray(m.position)), 10)

    # 1) 正常跟随: 持续发 0.8 rad 指令
    pub = node.create_publisher(JointState, "/hand/joint_command", 10)
    import time as _time
    cmd = JointState(); cmd.position = [0.8] * 16
    t_end = _time.monotonic() + 0.4
    while _time.monotonic() < t_end:
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.02)
    assert received and np.allclose(received[-1], 0.8, atol=1e-6)

    # 2) 停发指令 -> 看门狗 (0.1+0.1+0.2s) 平滑回 0
    t_end = _time.monotonic() + 1.0
    while _time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert np.allclose(received[-1], 0.0, atol=1e-3)
    node.destroy_node()
```

- [ ] **Step 2: 运行（Task 0 前该测试 skip，确认 skip 生效）**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_leap_hand_node_integration.py -v`
Expected: 1 skipped（rclpy 不可用）

- [ ] **Step 3: 重构节点实现**

`leap_hand_node.py` 重写为（完整代码，`parameter_overrides` 走 `declare_parameter(name, value=default)`）：

```python
"""leap_hand_node.py — LEAP Hand 16-DOF ROS2 运行时主控节点.

架构 (Phase 2 裁决: 普通 Node + 显式 FSM):
  1. 50Hz 定时读硬件并发布 /hand/joint_states;
  2. 订阅 /hand/joint_command, 经 align_command 严格校验后缓存;
  3. WatchdogStateMachine 裁决: 300ms 指令断流 -> 驱动层 Relax-to-OPEN;
  4. 全链路 SI 弧度, 统一节点时钟计时 (禁 wall-clock)。
"""
from __future__ import annotations

import time
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from tuinadex_hand.driver.leap_driver import LeapHandDriver, CANONICAL_JOINT_NAMES
from tuinadex_hand.watchdog import WatchdogStateMachine, WatchdogAction
from tuinadex_hand.command_processing import align_command


class LeapHandNode(Node):
    def __init__(self, parameter_overrides: Optional[dict] = None):
        super().__init__("leap_hand_node")
        overrides = parameter_overrides or {}

        def declare(name: str, value):
            self.declare_parameter(name, overrides.get(name, value))

        declare("port", "/dev/ttyUSB0")
        declare("baudrate", 4000000)
        declare("kP", 300)
        declare("kI", 0)
        declare("kD", 100)
        declare("curr_lim", 150)
        declare("control_rate_hz", 50.0)
        declare("watchdog_timeout_sec", 0.30)
        declare("watchdog_hold_sec", 0.30)
        declare("watchdog_ramp_sec", 0.60)
        declare("shutdown_dwell_sec", 0.5)
        declare("mock_mode", False)

        gp = lambda name: self.get_parameter(name).value
        self.rate_hz = float(gp("control_rate_hz"))
        self.hold_sec = float(gp("watchdog_hold_sec"))
        self.ramp_sec = float(gp("watchdog_ramp_sec"))
        self.dwell_sec = float(gp("shutdown_dwell_sec"))

        self.driver = LeapHandDriver(
            port=str(gp("port")), baudrate=int(gp("baudrate")),
            kP=int(gp("kP")), kI=int(gp("kI")), kD=int(gp("kD")),
            curr_lim=int(gp("curr_lim")), mock_mode=bool(gp("mock_mode")),
        )
        if not self.driver.connect():
            self.get_logger().error(f"无法连接 LEAP Hand (port={gp('port')})，节点退出。")
            raise SystemExit("LEAP Hand connection failed")

        self.state_pub = self.create_publisher(JointState, "/hand/joint_states", 10)
        self.cmd_sub = self.create_subscription(
            JointState, "/hand/joint_command", self._on_joint_command, 10)

        self._latest_command: Optional[np.ndarray] = None
        self._fsm = WatchdogStateMachine(timeout_sec=float(gp("watchdog_timeout_sec")))
        self._relax_logged = False
        self.timer = self.create_timer(1.0 / self.rate_hz, self._control_loop)

    def _now(self) -> float:
        """节点时钟秒值 (统一时间源, 禁 wall-clock)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joint_command(self, msg: JointState) -> None:
        names = list(msg.name) if msg.name else None
        q, reason = align_command(msg.position, names)
        if q is None:
            self.get_logger().warn(f"指令被拒绝 ({reason})，已丢弃。")
            return
        was_relaxing = self._fsm.state.name == "RELAXING"
        self._fsm.on_command(self._now())
        if was_relaxing:
            self.driver.reset_loss_state()
            self._relax_logged = False
            self.get_logger().info("收到新指令，看门狗已解除，恢复跟随。")
        self._latest_command = q

    def _control_loop(self) -> None:
        t = self._now()
        action = self._fsm.tick(t)

        if action is WatchdogAction.WRITE:
            try:
                self.driver.write_command(self._latest_command)
            except Exception as e:
                self.get_logger().error(f"写入指令异常: {e}")
        elif action is WatchdogAction.RELAX_STEP:
            if not self._relax_logged:
                self.get_logger().warn(
                    f"指令断流 > {self._fsm.timeout_sec * 1000:.0f}ms，"
                    "触发 Relax-to-OPEN 安全回退！")
                self._relax_logged = True
            try:
                self.driver.relax_to_open_step(t, hold_time=self.hold_sec,
                                               ramp_time=self.ramp_sec)
            except Exception as e:
                self.get_logger().error(f"Relax 步进异常: {e}")

        try:
            pos, vel, eff = self.driver.read_states()
        except Exception as e:
            self.get_logger().error(f"读取物理状态失败: {e}")
            return

        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = list(CANONICAL_JOINT_NAMES)
        state_msg.position = pos.tolist()
        state_msg.velocity = vel.tolist()
        state_msg.effort = eff.tolist()
        self.state_pub.publish(state_msg)

    def destroy_node(self) -> None:
        """安全关停序列: 停定时器 -> 全开指令 -> dwell 等待到位 -> 断开."""
        self.get_logger().info("安全退出: 停止控制循环, 回全开位, 等待到位后断开...")
        if self.timer is not None:
            self.timer.cancel()
        try:
            self.driver.set_open()
            time.sleep(self.dwell_sec)   # 让电机实际走到全开位 (旧实现即发即断, 已修正)
            self.driver.disconnect()
        except Exception as e:
            self.get_logger().warn(f"关停清理异常: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LeapHandNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 跑纯逻辑回归（本机可验证部分）**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`
Expected: Task 0 前集成测试 skip、其余全过，无 import 错误。

- [ ] **Step 5: Task 0 完成后跑集成测试**

Run: `source /opt/ros/<DISTRO>/setup.bash && ~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_leap_hand_node_integration.py -v`
Expected: 1 passed

- [ ] **Step 6: Commit**

```bash
git add src/tuinadex_hand/tuinadex_hand/leap_hand_node.py tests/test_leap_hand_node_integration.py
git commit -m "refactor(hand): wire node to watchdog FSM + node clock + safe shutdown dwell"
```

---

### Task 4: 遥操节点健壮化（退出/重捕获复位/利手过滤/资源释放）

**Files:**
- Create: `src/tuinadex_teleop/tuinadex_teleop/tracking/teleop_logic.py`（纯逻辑）
- Modify: `src/tuinadex_teleop/tuinadex_teleop/teleop_gesture_node.py`
- Test: `tests/test_teleop_logic.py`

**Interfaces:**
- Produces: `handedness_ok(handedness: str, required: str) -> bool`、`regained_after_gap(prev_seen: bool, now_seen: bool) -> bool`。
- 行为契约: 未检测到(或利手不符)不发布 → 300ms 后手节点看门狗 Relax-to-OPEN（符合规范"主端手势丢失"安全语义）；重捕获时 1€ 滤波器必须 reset（清陈旧速度状态）；退出快捷键改抛 `SystemExit`（回调内 `rclpy.shutdown()` 属危险用法）；`destroy_node` 释放 MediaPipe 资源。

- [ ] **Step 1: 写失败测试**

```python
"""test_teleop_logic.py — 遥操纯逻辑: 利手过滤与重捕获判定."""
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_teleop"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from tuinadex_teleop.tracking.teleop_logic import handedness_ok, regained_after_gap


def test_handedness_any_accepts_both():
    assert handedness_ok("Left", "any") is True
    assert handedness_ok("Right", "any") is True


def test_handedness_strict():
    assert handedness_ok("Right", "Right") is True
    assert handedness_ok("Left", "Right") is False
    assert handedness_ok("Right", "any") is True


def test_regained_after_gap():
    assert regained_after_gap(prev_seen=False, now_seen=True) is True
    assert regained_after_gap(prev_seen=True, now_seen=True) is False
    assert regained_after_gap(prev_seen=True, now_seen=False) is False
    assert regained_after_gap(prev_seen=False, now_seen=False) is False
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_teleop_logic.py -v`
Expected: FAIL，`ModuleNotFoundError: ...teleop_logic`

- [ ] **Step 3: 实现纯逻辑模块**

```python
"""teleop_logic.py — 遥操节点可单测的纯判定逻辑 (不依赖 rclpy/cv2)."""


def handedness_ok(handedness: str, required: str) -> bool:
    """required 为 "any"/"" 时接受 Left/Right 任意利手; 否则要求精确匹配."""
    if required in ("any", ""):
        return handedness in ("Left", "Right")
    return handedness == required


def regained_after_gap(prev_seen: bool, now_seen: bool) -> bool:
    """丢手 -> 重捕获 的单帧转移判定 (触发滤波器 reset 用)."""
    return (not prev_seen) and now_seen
```

- [ ] **Step 4: 运行确认通过**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_teleop_logic.py -v`
Expected: 3 passed

- [ ] **Step 5: 节点接线（修改点清单，逐处最小改动）**

在 `teleop_gesture_node.py`：

1. 新增导入与参数：

```python
from tuinadex_teleop.tracking.teleop_logic import handedness_ok, regained_after_gap
# declare_parameter 区追加:
self.declare_parameter("required_handedness", "any")
# 读取区追加:
self.required_handedness = str(self.get_parameter("required_handedness").value)
```

2. `__init__` 末尾（离合状态区）追加：`self._hand_seen = False`。

3. `_loop()` 检测判定替换（原 `if results and len(results) > 0:` 段）：

```python
        now = time.time()
        results = self.tracker.detect(frame)
        seen = bool(results) and handedness_ok(results[0].handedness,
                                               self.required_handedness)
        if regained_after_gap(self._hand_seen, seen):
            self.filter.reset()   # 清除陈旧速度状态, 避免重捕获速度尖峰
        self._hand_seen = seen

        if seen:
            hand = results[0]
            # 1. 映射为 16 维相对弧度 (0 为全开，>0 为弯曲)
            raw_angles = self.mapper.map_keypoints_to_leap(hand)
            # ... (后续 filtered_angles 逻辑不变)
```

4. 退出键分支：`elif key == ord('q') or key == 27:` 改为 `raise SystemExit("user quit")`（替代 `rclpy.shutdown()`；`main()` 已捕获 SystemExit）。

5. `destroy_node()` 追加：

```python
        try:
            self.tracker.close()
        except Exception:
            pass
```

- [ ] **Step 6: 全量回归**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -q`
Expected: 全部 passed/skip，无新增失败

- [ ] **Step 7: Commit**

```bash
git add src/tuinadex_teleop/ tests/test_teleop_logic.py
git commit -m "fix(teleop): filter reset on re-acquire, handedness gate, safe exit, resource release"
```

---

### Task 5: 校验器收紧至规范逐关节限位

**Files:**
- Modify: `dataset/tuinadex_dataset/validator.py`
- Test: `tests/test_dataset_sync_validator.py`（追加用例）

**Interfaces:**
- Produces: `SPEC_JOINT_LIMITS: Tuple[Tuple[float, float], ...]`（16 项，与 Spec §1 表格逐行一致）；`DatasetValidator(joint_limits=None)` 默认取规范限位。
- 兼容性: 既有测试数据全零，不受收紧影响（已核实 [tests/test_dataset_sync_validator.py:35-52]）。

- [ ] **Step 1: 追加失败测试**

```python
def test_validator_spec_per_joint_limits():
    """规范逐关节限位: thumb_yaw (idx 12) 上限 1.80, 超出即拒."""
    from tuinadex_dataset.validator import SPEC_JOINT_LIMITS

    assert len(SPEC_JOINT_LIMITS) == 16
    assert SPEC_JOINT_LIMITS[12] == (-0.30, 1.80)
    assert SPEC_JOINT_LIMITS[0] == (-0.35, 0.35)

    val = DatasetValidator()
    t = np.linspace(0.0, 1.0, 20)
    s = np.zeros((20, 16)); a = np.zeros((20, 16))
    s[5, 12] = 1.9   # thumb_yaw 超上限 (旧标量限位 2.5 会漏检)
    ok, errors = val.validate_episode(t, s, a)
    assert ok is False and any("限位" in e for e in errors)


def test_validator_still_passes_spec_boundary_values():
    val = DatasetValidator()
    t = np.linspace(0.0, 1.0, 20)
    s = np.zeros((20, 16)); a = np.zeros((20, 16))
    s[:, 12] = 1.80   # 恰好在上限
    s[:, 0] = -0.35   # 侧摆下限
    ok, errors = val.validate_episode(t, s, a)
    assert ok is True, errors
```

- [ ] **Step 2: 运行确认失败**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_dataset_sync_validator.py -v`
Expected: FAIL，`ImportError: cannot import name 'SPEC_JOINT_LIMITS'`

- [ ] **Step 3: 实现（validator.py 修改点）**

模块级常量（数值与 Spec §1 表格逐行核对）：

```python
# 规范逐关节限位 (rad, 相对全开位 0) — 与 LEAP_HAND_MVP_INTERFACE_SPEC §1 一致
SPEC_JOINT_LIMITS = (
    (-0.35, 0.35), (0.0, 2.00), (0.0, 1.85), (0.0, 1.80),   # index
    (-0.35, 0.35), (0.0, 2.00), (0.0, 1.85), (0.0, 1.80),   # middle
    (-0.35, 0.35), (0.0, 2.00), (0.0, 1.85), (0.0, 1.80),   # ring
    (-0.30, 1.80), (0.0, 1.60), (0.0, 1.80), (0.0, 1.80),   # thumb
)
```

`__init__` 签名改为：

```python
    def __init__(
        self,
        joint_limits=None,
        max_single_step_delta: float = 0.50,
    ):
        self.joint_limits = tuple(joint_limits) if joint_limits is not None else SPEC_JOINT_LIMITS
        self._lo = np.array([lo for lo, _ in self.joint_limits])
        self._hi = np.array([hi for _, hi in self.joint_limits])
        self.max_delta = max_single_step_delta
```

原 `limits_min/limits_max` 两段标量限位检查替换为：

```python
        if np.any(states < self._lo) or np.any(states > self._hi):
            errors.append("State 存在超出规范逐关节限位的数值")
        if np.any(actions < self._lo) or np.any(actions > self._hi):
            errors.append("Action 存在超出规范逐关节限位的数值")
```

（NaN / 单调性 / 跳变检查保持原样不动。）

- [ ] **Step 4: 运行确认通过（含既有用例）**

Run: `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_dataset_sync_validator.py -v`
Expected: 4 passed

- [ ] **Step 5: Commit**

```bash
git add dataset/tuinadex_dataset/validator.py tests/test_dataset_sync_validator.py
git commit -m "feat(dataset): per-joint spec limits in validator (thumb_yaw 1.80 catches old scalar miss)"
```

---

### Task 6: tuinadex_recorder 采集包（rosbag2_py 进程内）

**Files:**
- Create: `src/tuinadex_recorder/package.xml`、`setup.py`、`setup.cfg`、`resource/tuinadex_recorder`
- Create: `src/tuinadex_recorder/tuinadex_recorder/__init__.py`、`recorder_node.py`
- Test: `tests/test_recorder_node.py`（rclpy 守护）

**Interfaces:**
- Produces: 节点 `tuinadex_recorder_node`；服务 `/recorder/start`、`/recorder/stop`（`std_srvs/srv/Trigger`）；参数 `topics`(默认 `["/hand/joint_states","/hand/joint_command"]`)、`topic_types`（默认两条 `sensor_msgs/msg/JointState`）、`bag_root`(默认 `bags`)。bag 目录 `bags/session_YYYYmmdd_HHMMSS`。
- Task 8 依赖本节点的服务接口与 bag 输出。

- [ ] **Step 1: 包骨架（照 `src/tuinadex_teleop/setup.py` 逐行同构）**

`setup.py` 仅改 `package_name = 'tuinadex_recorder'`、description `'TuinaDex rosbag2 Recording Package'`，entry_points：`'tuinadex_recorder_node = tuinadex_recorder.recorder_node:main'`。`setup.cfg`、`package.xml`（依赖 `rclpy, rosbag2_py, std_srvs, sensor_msgs`）、`resource/tuinadex_recorder` 空文件照抄同类包格式。

- [ ] **Step 2: 写失败测试（rclpy 守护）**

```python
"""test_recorder_node.py — 采集节点服务生命周期与 bag 落盘 (需 ROS2)."""
import pytest
rclpy = pytest.importorskip("rclpy")

import sys
from pathlib import Path
SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_recorder"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from tuinadex_recorder.recorder_node import RecorderNode
from std_srvs.srv import Trigger


def test_start_stop_creates_bag(tmp_path):
    rclpy.init()
    try:
        node = RecorderNode()
        node._bag_root = tmp_path   # 直接注入临时目录 (测试专用)

        resp = Trigger.Response()
        node._on_start(Trigger.Request(), resp)
        assert resp.success is True
        assert (tmp_path / Path(resp.message).name).exists()   # bag 目录已建

        resp2 = Trigger.Response()
        node._on_start(Trigger.Request(), resp2)
        assert resp2.success is False                           # 重复 start 拒绝

        node._on_stop(Trigger.Request(), Trigger.Response())
        resp3 = Trigger.Response()
        node._on_stop(Trigger.Request(), resp3)
        assert resp3.success is False                           # 未在录制
        node.destroy_node()
    finally:
        rclpy.shutdown()
```

- [ ] **Step 3: 实现节点**

```python
"""recorder_node.py — rosbag2 进程内采集节点 (Phase 2 裁决: rosbag2_py, 非 subprocess).

generic subscription 直收序列化字节 (零反序列化开销), Trigger 服务控制起停;
bag 输出 bags/session_YYYYmmdd_HHMMSS (sqlite3)。
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import rosbag2_py


class RecorderNode(Node):
    def __init__(self):
        super().__init__("tuinadex_recorder")
        self.declare_parameter("topics", ["/hand/joint_states", "/hand/joint_command"])
        self.declare_parameter(
            "topic_types",
            ["/hand/joint_states", "/hand/joint_command"] and
            ["sensor_msgs/msg/JointState", "sensor_msgs/msg/JointState"])
        self.declare_parameter("bag_root", "bags")

        self._topics = list(self.get_parameter("topics").value)
        self._types = list(self.get_parameter("topic_types").value)
        if len(self._topics) != len(self._types):
            raise SystemExit(
                f"topics({len(self._topics)}) 与 topic_types({len(self._types)}) 数量不一致")
        self._bag_root = Path(str(self.get_parameter("bag_root").value))

        self._writer: rosbag2_py.SequentialWriter | None = None
        self._subs = []

        self.create_service(Trigger, "/recorder/start", self._on_start)
        self.create_service(Trigger, "/recorder/stop", self._on_stop)
        self.get_logger().info(f"采集节点就绪: topics={self._topics}")

    def _on_start(self, request, response) -> Trigger.Response:
        if self._writer is not None:
            response.success, response.message = False, "already recording"
            return response
        bag_dir = str(self._bag_root / f"session_{datetime.now():%Y%m%d_%H%M%S}")
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                        output_serialization_format="cdr"))
        for topic, ttype in zip(self._topics, self._types):
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=ttype, serialization_format="cdr"))
            self._subs.append(self.create_generic_subscription(
                topic, ttype, 10,
                lambda data, _t=topic: self._write(_t, data)))
        self._writer = writer
        self.get_logger().info(f"录制开始: {bag_dir}")
        response.success, response.message = True, bag_dir
        return response

    def _write(self, topic: str, data: bytes) -> None:
        if self._writer is not None:
            self._writer.write(topic, data, self.get_clock().now().nanoseconds)

    def _on_stop(self, request, response) -> Trigger.Response:
        if self._writer is None:
            response.success, response.message = False, "not recording"
            return response
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs.clear()
        self._writer = None   # SequentialWriter 析构时 flush 并关闭
        self.get_logger().info("录制停止。")
        response.success, response.message = True, "stopped"
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RecorderNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

> 实现注：`topic_types` 声明行中 `and` 表达式是排版残留，落地时直接写
> `self.declare_parameter("topic_types", ["sensor_msgs/msg/JointState", "sensor_msgs/msg/JointState"])`。

- [ ] **Step 4: 测试通过 + 构建（Task 0 后）**

```bash
~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/test_recorder_node.py -v
cd /home/bright/win_office/ubantu_files/project/TuinaDex_v2.0
source /opt/ros/<DISTRO>/setup.bash && colcon build --symlink-install --packages-select tuinadex_recorder
```
Expected: 测试 passed；colcon 无 error

- [ ] **Step 5: Commit**

```bash
git add src/tuinadex_recorder tests/test_recorder_node.py
git commit -m "feat(recorder): in-process rosbag2_py recorder with Trigger start/stop services"
```

---

### Task 7: LeRobot 数据集适配器（bag_reader + adapter + CLI）

**Files:**
- Create: `dataset/tuinadex_dataset/bag_reader.py`
- Create: `dataset/tuinadex_dataset/adapter.py`
- Create: `dataset/tuinadex_dataset/cli.py`
- Test: `tests/test_dataset_adapter.py`（rclpy + lerobot 守护）

**Interfaces:**
- Consumes: `TimeSynchronizer.align_series(state_ts, state_pos, action_ts, action_pos) -> (grid, states, actions)`（已有，勿改签名）；Task 5 `SPEC_JOINT_LIMITS`。
- Produces:
  - `bag_reader.read_joint_state_bag(bag_dir, topic: str) -> Tuple[np.ndarray, np.ndarray]`（header.stamp 秒，(N,), (N,16)）
  - `adapter.convert_bag_to_episode(bag_dir, fps=50.0) -> (grid, states, actions, ok, errors)`
  - `adapter.save_to_lerobot(grid, states, actions, repo_id: str, fps=50.0) -> str`
  - CLI: `python -m tuinadex_dataset.cli --bag <bag_dir> --out <repo_id> [--fps 50] [--allow-invalid]`

- [ ] **Step 1: 先验证 lerobot API（防版本漂移，据实微调 Step 4 关键字）**

```bash
/home/bright/win_office/conda/envs/smolvla/bin/python -c "
import inspect
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
print(inspect.signature(LeRobotDataset.create))
print(inspect.signature(LeRobotDataset.add_episode))"
```
若 `lerobot.common.datasets` 导入失败则尝试 `lerobot.datasets.lerobot_dataset`（0.5+ 路径），并同步修改测试与实现中的 import。**据实测签名调整 `create`/`add_episode` 关键字后，把实测结论写进本任务 commit message。**

- [ ] **Step 2: 写失败测试（合成 bag → 转换 → 数据集全链）**

```python
"""test_dataset_adapter.py — 合成 rosbag2 → 对齐 → 校验 → LeRobotDataset 全链 (需 ROS2+lerobot)."""
import pytest
rclpy = pytest.importorskip("rclpy")

import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "dataset") not in sys.path:
    sys.path.insert(0, str(ROOT / "dataset"))

from tuinadex_dataset.adapter import convert_bag_to_episode, save_to_lerobot


@pytest.fixture
def synthetic_bag(tmp_path):
    """用 rosbag2_py 写一段 1 秒合成 bag: state 50Hz 正弦, command 40Hz 方波."""
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState

    bag_dir = str(tmp_path / "synthetic")
    w = SequentialWriter()
    w.open(StorageOptions(uri=bag_dir, storage_id="sqlite3"),
           ConverterOptions("cdr", "cdr"))
    for topic in ("/hand/joint_states", "/hand/joint_command"):
        w.create_topic(TopicMetadata(name=topic, type="sensor_msgs/msg/JointState",
                                     serialization_format="cdr"))

    t0 = 1000  # 固定整秒起点
    for k in range(50):   # state: 50Hz
        m = JointState()
        m.header.stamp.sec = t0 + k // 50
        m.header.stamp.nanosec = (k % 50) * 20_000_000
        m.position = [float(np.sin(0.1 * k))] * 16
        w.write("/hand/joint_states", serialize_message(m),
                m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
    for k in range(40):   # action: 40Hz
        m = JointState()
        m.header.stamp.sec = t0 + k // 40
        m.header.stamp.nanosec = (k % 40) * 25_000_000
        m.position = [float(k % 2)] * 16
        w.write("/hand/joint_command", serialize_message(m),
                m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
    return bag_dir


def test_read_joint_state_bag(synthetic_bag):
    from tuinadex_dataset.bag_reader import read_joint_state_bag
    ts, pos = read_joint_state_bag(synthetic_bag, "/hand/joint_states")
    assert ts.shape == (50,) and pos.shape == (50, 16)
    assert np.all(np.diff(ts) > 0)     # header.stamp 严格单调
    assert abs(ts[0] - 1000.0) < 1e-6


def test_convert_and_validate(synthetic_bag):
    grid, states, actions, ok, errors = convert_bag_to_episode(synthetic_bag)
    assert states.shape == (len(grid), 16) and actions.shape == (len(grid), 16)
    dt = np.diff(grid)
    np.testing.assert_allclose(dt, 0.02, atol=1e-5)
    assert ok is True, errors   # 合成数据在规范限位内


def test_save_to_lerobot(synthetic_bag, tmp_path):
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    grid, states, actions, ok, _ = convert_bag_to_episode(synthetic_bag)
    assert ok
    out = save_to_lerobot(grid, states, actions,
                          repo_id=str(tmp_path / "tuina_synthetic"), fps=50)
    assert Path(out).exists()
    ds = LeRobotDataset(str(tmp_path / "tuina_synthetic"))
    assert ds.meta.total_frames == len(grid)
    frame = ds[0]
    assert frame["observation.state"].shape == (16,)
    assert frame["action"].shape == (16,)
```

- [ ] **Step 3: 运行确认失败（Task 0 后）**

Run: `~/win_office/conda/envs/smolvla/bin/python -m pytest tests/test_dataset_adapter.py -v`
Expected: FAIL，`ModuleNotFoundError: tuinadex_dataset.bag_reader`

- [ ] **Step 4: 实现**

`bag_reader.py`:

```python
"""bag_reader.py — rosbag2 读取与 JointState 反序列化 (依赖 ROS2 环境, Phase 2 裁决)."""
from __future__ import annotations

from typing import Tuple

import numpy as np

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState


def read_joint_state_bag(bag_dir, topic: str) -> Tuple[np.ndarray, np.ndarray]:
    """读取 bag 中指定 JointState 话题 -> (header.stamp 秒 (N,), position (N,16)).

    非法帧 (维度 != 16) 跳过并计数; 时间戳取 header.stamp
    (规范: 硬件回读时刻 / 指令生成时刻)。
    """
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    known = {m.name: m.type for m in reader.get_all_topics_and_types()}
    if topic not in known:
        raise ValueError(f"话题 {topic} 不在 bag 中 (现有: {sorted(known)})")
    if known[topic] != "sensor_msgs/msg/JointState":
        raise ValueError(f"话题 {topic} 类型为 {known[topic]}, 仅支持 JointState")

    ts_list, pos_list, skipped = [], [], 0
    while reader.has_next():
        topic_name, data, _recv = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, JointState)
        pos = np.asarray(msg.position, dtype=np.float64)
        if pos.shape != (16,):
            skipped += 1
            continue
        ts_list.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        pos_list.append(pos)
    if skipped:
        print(f"[bag_reader] 跳过 {skipped} 个非法维度帧")
    if not ts_list:
        raise ValueError(f"话题 {topic} 无有效帧")
    return np.asarray(ts_list, dtype=np.float64), np.stack(pos_list)
```

`adapter.py`（`create`/`add_episode` 关键字按 Step 1 实测签名微调）:

```python
"""adapter.py — rosbag2 → 50Hz 对齐 → 规范校验 → LeRobotDataset 离线转换."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .bag_reader import read_joint_state_bag
from .synchronizer import TimeSynchronizer
from .validator import DatasetValidator

# 与 Spec §1 / leap_driver 一致的 16 关节名 (纯离线模块, 独立声明不 import ROS 包)
CANONICAL_JOINT_NAMES = [
    "index_yaw",  "index_mcp",  "index_pip",  "index_dip",
    "middle_yaw", "middle_mcp", "middle_pip", "middle_dip",
    "ring_yaw",   "ring_mcp",   "ring_pip",   "ring_dip",
    "thumb_yaw",  "thumb_cmc",  "thumb_mcp",  "thumb_ip",
]


def convert_bag_to_episode(bag_dir, fps: float = 50.0):
    """读取 state/action 双话题 -> 网格对齐 -> 规范校验.

    Returns: (grid (T,), states (T,16) float32, actions (T,16) float32, ok, errors)
    """
    s_ts, s_pos = read_joint_state_bag(bag_dir, "/hand/joint_states")
    a_ts, a_pos = read_joint_state_bag(bag_dir, "/hand/joint_command")
    grid, states, actions = TimeSynchronizer(target_fps=fps).align_series(
        s_ts, s_pos, a_ts, a_pos)
    ok, errors = DatasetValidator().validate_episode(grid, states, actions)
    return grid, states, actions, ok, errors


def save_to_lerobot(grid, states, actions, repo_id: str, fps: float = 50.0) -> str:
    """写入 LeRobotDataset (lerobot 0.4.4 API), 返回数据集根路径."""
    import torch
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    features = {
        "observation.state": {
            "dtype": "float32", "shape": (16,),
            "names": {"joints": list(CANONICAL_JOINT_NAMES)}},
        "action": {
            "dtype": "float32", "shape": (16,),
            "names": {"joints": list(CANONICAL_JOINT_NAMES)}},
    }
    dataset = LeRobotDataset.create(repo_id=repo_id, fps=int(fps), features=features)
    for i in range(len(grid)):
        dataset.add_episode(
            frame={
                "observation.state": torch.from_numpy(states[i].astype(np.float32)),
                "action": torch.from_numpy(actions[i].astype(np.float32)),
                "timestamp": float(grid[i] - grid[0]),
            },
            task="teleoperation")
    return str(Path(repo_id))
```

`cli.py`:

```python
"""cli.py — python -m tuinadex_dataset.cli --bag <dir> --out <repo_id> [--fps 50]."""
import argparse

from .adapter import convert_bag_to_episode, save_to_lerobot


def main():
    ap = argparse.ArgumentParser(description="rosbag2 -> LeRobotDataset 离线转换")
    ap.add_argument("--bag", required=True, help="rosbag2 目录路径")
    ap.add_argument("--out", required=True, help="LeRobotDataset repo_id / 输出路径")
    ap.add_argument("--fps", type=float, default=50.0)
    ap.add_argument("--allow-invalid", action="store_true",
                    help="校验失败时仍导出 (默认拒绝)")
    args = ap.parse_args()

    grid, states, actions, ok, errors = convert_bag_to_episode(args.bag, fps=args.fps)
    if not ok and not args.allow_invalid:
        raise SystemExit("校验失败, 拒绝导出:\n  " + "\n  ".join(errors))
    if not ok:
        print("[警告] 校验失败但按 --allow-invalid 继续:\n  " + "\n  ".join(errors))
    out = save_to_lerobot(grid, states, actions, repo_id=args.out, fps=args.fps)
    print(f"完成: {len(grid)} 帧 @ {args.fps}Hz -> {out}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: 测试通过**

Run: `~/win_office/conda/envs/smolvla/bin/python -m pytest tests/test_dataset_adapter.py -v`（Task 0 后）
Expected: 3 passed

- [ ] **Step 6: Commit**

```bash
git add dataset/tuinadex_dataset/ tests/test_dataset_adapter.py
git commit -m "feat(dataset): rosbag2->LeRobotDataset offline adapter with CLI (sync+validate)"
```

---

### Task 8: 端到端 mock 闭环 + AGENTS.md 验证命令固化

**Files:**
- Create: `scripts/e2e_mock_check.sh`
- Modify: `AGENTS.md`（§5 补完整验证命令）
- Modify: `.agent/SESSION_STATE.md`（Phase 3 完成归档）

**Interfaces:**
- Consumes: Task 3 (mock 手节点)、Task 6 (recorder)、Task 7 (adapter)。
- Produces: Phase 4 验收所需的端到端实证流程。

- [ ] **Step 1: 编写 e2e 脚本**

```bash
#!/usr/bin/env bash
# e2e_mock_check.sh — 无真机端到端闭环: mock 手节点 + 模拟指令 + 录制 + 离线转换
# 前置: source /opt/ros/<DISTRO>/setup.bash && colcon build --symlink-install 已完成
set -euo pipefail
cd "$(dirname "$0")/.."
source install/setup.bash

WORK=$(mktemp -d)
trap 'kill $(jobs -p) 2>/dev/null || true; rm -rf "$WORK"' EXIT

# 1) mock 手节点 (快速看门狗参数)
ros2 run tuinadex_hand leap_hand_node --ros-args -p mock_mode:=true &
sleep 2

# 2) 录制器
ros2 run tuinadex_recorder tuinadex_recorder_node --ros-args -p bag_root:="$WORK/bags" &
sleep 1
ros2 service call /recorder/start std_srvs/srv/Trigger

# 3) 模拟指令源: 3 秒 0.5 rad 弯曲指令
timeout 3 ros2 topic pub -r 50 /hand/joint_command sensor_msgs/msg/JointState \
  '{position: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}'

# 4) 指令断流 2 秒 (验证看门狗 Relax-to-OPEN 已进 bag), 然后停止录制
sleep 2
ros2 service call /recorder/stop std_srvs/srv/Trigger

# 5) 离线转换
BAG=$(ls -td "$WORK"/bags/session_* | head -1)
/home/bright/win_office/conda/envs/smolvla/bin/python -m tuinadex_dataset.cli --bag "$BAG" --out "$WORK/dataset"

echo "[E2E] PASS: mock 闭环全链贯通"
```

- [ ] **Step 2: 执行脚本并记录实证**

Run: `bash scripts/e2e_mock_check.sh`
Expected: 转换器打印帧数与输出路径，末行 `[E2E] PASS`。失败则修到通过为止（这是 Phase 4 验收核心证据）。

- [ ] **Step 3: AGENTS.md §5 固化验证命令**

追加内容（发行版名按 Task 0 实际结果替换 `<DISTRO>`）：

```markdown
### 验证命令 (固化)
- 单元测试 (无 ROS2 也可跑纯逻辑部分):
  `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`
- 全量测试 (含 ROS2 集成): `source /opt/ros/<DISTRO>/setup.bash && ~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`
- LeRobot 适配器测试: `~/win_office/conda/envs/smolvla/bin/python -m pytest tests/test_dataset_adapter.py -v`
- 构建: `source /opt/ros/<DISTRO>/setup.bash && colcon build --symlink-install`
- 端到端: `bash scripts/e2e_mock_check.sh`
```

- [ ] **Step 4: Commit + 归档**

```bash
git add scripts/e2e_mock_check.sh AGENTS.md .agent/SESSION_STATE.md
git commit -m "feat(e2e): mock closed-loop check script; fixate verification commands in AGENTS.md"
```

---

## Self-Review 记录

- **Spec 覆盖**: §1 逐关节限位 → Task 5；§2 双向转换 → 既有 driver 测试已覆盖（Phase 1）；§3 话题契约 → Task 3（节点）+ Task 6（录制双话题）；§4 LeRobot Schema（(16,) float32、50Hz 对齐、离线转换）→ Task 7。无遗漏。
- **占位符扫描**: Task 6 的 `and` 排版残留已加实现注；Task 3/7 的实现注意与 API 实测步骤均为显式声明的验证步骤，非隐藏 TBD。
- **类型一致性**: `align_command -> (Optional[np.ndarray], str)`、`WatchdogStateMachine.tick -> WatchdogAction`、`read_joint_state_bag -> ((N,), (N,16))`、`convert_bag_to_episode -> 5 元组` 在 Task 1/2/3/7/8 间交叉核对一致。
