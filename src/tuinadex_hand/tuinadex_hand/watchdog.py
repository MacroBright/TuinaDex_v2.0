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
            if (t_now - self._last_cmd_time) <= self.timeout_sec + 1e-9:
                return WatchdogAction.WRITE
            self._state = WatchdogState.RELAXING
        return WatchdogAction.RELAX_STEP
