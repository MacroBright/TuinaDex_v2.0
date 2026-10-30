"""Thread-safe Hub command queue and collection orchestration."""
from __future__ import annotations

import logging
import queue
import threading
import time
from enum import Enum
from typing import Callable

from .collector import CollectionState, TuinaCollector
from .data_channel import BaseDataChannel
from .episode_buffer import BufferCapacityError, EpisodeBuffer

logger = logging.getLogger(__name__)


class HubAction(str, Enum):
    """Commands accepted by the Hub control loop."""

    START = "start"
    PAUSE = "pause"
    RESUME = "resume"
    STOP = "stop"
    SAVE = "save"
    DISCARD = "discard"
    LABEL = "label"


class DataHubController:
    """Control-loop writer with an HTTP-safe submit/status boundary."""

    def __init__(
        self,
        channel: BaseDataChannel,
        collector: TuinaCollector,
        control_fps: float = 50.0,
        ui_fps: float = 25.0,
        max_staleness_s: float = 0.06,
        max_consecutive_stale: int = 3,
    ):
        if control_fps <= 0 or ui_fps <= 0:
            raise ValueError("control_fps and ui_fps must be > 0")
        if max_consecutive_stale < 1:
            raise ValueError("max_consecutive_stale must be >= 1")

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
            target=self._save_worker,
            name="episode-saver",
            daemon=True,
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

    def submit(self, action: str, payload: dict | None = None) -> dict:
        """Queue a command from any thread without touching collector state."""
        try:
            parsed_action = HubAction(action)
        except ValueError:
            return {"ok": False, "error": f"unknown action '{action}'"}
        try:
            self._commands.put_nowait((parsed_action, payload or {}))
        except queue.Full:
            return {"ok": False, "error": "command queue full"}
        return {"ok": True, "accepted": parsed_action.value}

    def status(self) -> dict:
        """Return a coherent status snapshot from any thread."""
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

    def tick(self) -> None:
        """Advance commands, recording, and telemetry on the control clock."""
        with self._lock:
            now = time.monotonic()
            was_recording = (
                self.collector.state is CollectionState.RECORDING
            )
            self._drain_commands()
            self._tick_no += 1
            if (
                was_recording
                and self.collector.state is CollectionState.RECORDING
            ):
                self._record_tick(now)
            if self._tick_no % self._ui_every == 0:
                self._publish(now)

    def shutdown(self, timeout: float = 5.0) -> None:
        """Stop the saver after pending work within the timeout."""
        try:
            self._commands.put_nowait((HubAction.STOP, {}))
        except queue.Full:
            logger.warning("Command queue full during shutdown")
        try:
            self._save_q.put(None, timeout=timeout)
        except queue.Full:
            logger.error("Save queue did not drain during shutdown")
        self._saver.join(timeout)

    def _drain_commands(self) -> None:
        while True:
            try:
                action, payload = self._commands.get_nowait()
            except queue.Empty:
                return
            self._apply(action, payload)

    def _apply(self, action: HubAction, payload: dict) -> None:
        collector = self.collector
        if action is HubAction.START:
            if collector.state is CollectionState.IDLE:
                self._begin_episode()
            collector.start_recording()
        elif action is HubAction.PAUSE:
            collector.pause_recording()
        elif action is HubAction.RESUME:
            collector.start_recording()
        elif action is HubAction.STOP:
            collector.stop_recording()
        elif action is HubAction.SAVE:
            self._finish_episode(save=True)
        elif action is HubAction.DISCARD:
            self._finish_episode(save=False)
        elif action is HubAction.LABEL:
            if "phase_label" in payload:
                self._phase_label = str(payload["phase_label"])
            if "language_instruction" in payload:
                self._instruction = str(payload["language_instruction"])
                self.collector.buffer.language_instruction = self._instruction

    def _begin_episode(self) -> None:
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
            issues = buffer.validate_continuity(
                max_gap_s=max(
                    1.5 / self.control_fps,
                    self.max_staleness_s,
                )
            )
            if issues:
                logger.warning(
                    "Episode continuity rejected: %s", issues[:5]
                )
                self._last_abort_reason = f"continuity: {issues[0]}"
                self.collector.review_decision(save=False)
                return
            buffer.seal_episode()
        self.collector.review_decision(
            save=save, defer_completion=save
        )

    def _record_tick(self, now: float) -> None:
        packets = self.channel.get_camera_packets()
        if not packets.jpeg:
            self._abort("no_camera_frames")
            return
        worst_age = packets.worst_age(now)
        if worst_age > self.max_staleness_s:
            self._dropped += 1
            self._stale_streak += 1
            if self._stale_streak >= self.max_consecutive_stale:
                self._abort(f"stale_camera_{worst_age * 1000:.0f}ms")
            return
        self._stale_streak = 0

        q, effort, forces = self.channel.get_proprioception()
        action = self.channel.get_action_label()
        relative_time = now - self._episode_t0
        try:
            self.collector.buffer.add_frame(
                state=q,
                action=action,
                effort=effort,
                contact_forces=forces,
                images=packets.jpeg,
                timestamp=relative_time,
                image_timestamps={
                    cam: relative_time for cam in packets.timestamps
                },
                phase_label=self._phase_label,
            )
        except BufferCapacityError as exc:
            self._abort(str(exc))

    def _abort(self, reason: str) -> None:
        logger.warning("Episode aborted: %s", reason)
        self._last_abort_reason = reason
        self.collector.buffer.clear()
        self.collector.state = CollectionState.IDLE
        self._dropped = 0
        self._stale_streak = 0

    def _publish(self, now: float) -> None:
        try:
            q, effort, forces = self.channel.get_proprioception()
        except Exception:  # noqa: BLE001 - telemetry must not stop recording
            logger.exception("Telemetry read failed")
            return
        self._last_telemetry = {
            "timestamp": (
                now - self._episode_t0
                if self.collector.state is CollectionState.RECORDING
                else 0.0
            ),
            "state": [float(value) for value in q],
            "effort": [float(value) for value in effort],
            "contact_forces": [float(value) for value in forces],
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
            except Exception:  # noqa: BLE001 - writer failure is isolated
                logger.exception("Episode save failed")
                with self._lock:
                    self._save_errors += 1
            finally:
                with self._lock:
                    self.collector.notify_save_complete()
