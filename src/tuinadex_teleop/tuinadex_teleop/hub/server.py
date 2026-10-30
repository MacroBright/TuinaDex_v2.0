"""Zero-dependency Hub server with REST, SSE, and MJPEG endpoints."""
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
    """Single-producer, multi-consumer frame and telemetry slots."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._frames: dict[str, tuple[bytes, float]] = {}
        self._telemetry: dict[str, Any] = {}
        self._telemetry_seq = 0
        self._stopped = False

    @property
    def stopped(self) -> bool:
        with self._cond:
            return self._stopped

    def publish_frames(self, packets: CameraPacket) -> None:
        """Publish already encoded camera frames without re-encoding."""
        if not packets.jpeg:
            return
        with self._cond:
            if self._stopped:
                return
            self._frames = {
                cam: (payload, packets.timestamps.get(cam, 0.0))
                for cam, payload in packets.jpeg.items()
            }
            self._cond.notify_all()

    def publish_telemetry(self, telemetry: dict[str, Any]) -> None:
        """Publish one telemetry snapshot and increment its sequence."""
        with self._cond:
            if self._stopped:
                return
            self._telemetry = telemetry
            self._telemetry_seq += 1
            self._cond.notify_all()

    def publish(
        self,
        packets: CameraPacket,
        telemetry: dict[str, Any] | None = None,
    ) -> None:
        """Publish frames and optional telemetry as one producer action."""
        self.publish_frames(packets)
        if telemetry is not None:
            self.publish_telemetry(telemetry)

    def wait_frame(
        self, cam: str, after_ts: float, timeout: float
    ) -> tuple[bytes | None, float]:
        """Wait for a frame newer than ``after_ts`` on one camera."""
        with self._cond:
            self._cond.wait_for(
                lambda: self._stopped
                or (
                    cam in self._frames
                    and self._frames[cam][1] > after_ts
                ),
                timeout=timeout,
            )
            item = self._frames.get(cam)
            if (
                self._stopped
                or item is None
                or item[1] <= after_ts
            ):
                return None, after_ts
            return item

    def wait_telemetry(
        self, after_seq: int, timeout: float
    ) -> tuple[dict[str, Any] | None, int]:
        """Wait for telemetry newer than ``after_seq``."""
        with self._cond:
            self._cond.wait_for(
                lambda: self._stopped
                or self._telemetry_seq > after_seq,
                timeout=timeout,
            )
            if self._stopped or self._telemetry_seq <= after_seq:
                return None, after_seq
            return dict(self._telemetry), self._telemetry_seq

    def stop(self) -> None:
        """Wake all stream consumers and reject future publishes."""
        with self._cond:
            self._stopped = True
            self._cond.notify_all()


class _HubHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class CollectionHubServer:
    """Serve the collection workbench in a background HTTP thread."""

    def __init__(
        self, controller, host: str = "127.0.0.1", port: int = 8080
    ):
        self.controller = controller
        self.stream = StreamHub()
        self._httpd = _HubHTTPServer((host, int(port)), _HubHandler)
        self._httpd.controller = controller
        self._httpd.stream = self.stream
        self._thread: threading.Thread | None = None
        self._closed = False
        if host not in ("127.0.0.1", "localhost"):
            logger.warning(
                "Hub bound to non-loopback host %s; verify network isolation",
                host,
            )

    @property
    def port(self) -> int:
        """Return the actual listening port, including ephemeral ports."""
        return int(self._httpd.server_address[1])

    def start(self) -> int:
        """Start serving and return the actual port."""
        if self._closed:
            raise RuntimeError("CollectionHubServer has been stopped")
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(
                target=self._httpd.serve_forever,
                name="hub-http",
                daemon=True,
            )
            self._thread.start()
        return self.port

    def stop(self) -> None:
        """Stop streams and close the listening socket."""
        if self._closed:
            return
        self._closed = True
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

    def do_GET(self) -> None:  # noqa: N802 - stdlib handler convention
        path = self.path.split("?", 1)[0]
        try:
            if path == "/":
                self._static("index.html", "text/html; charset=utf-8")
            elif path == "/app.js":
                self._static(
                    "app.js", "application/javascript; charset=utf-8"
                )
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
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return

    def do_POST(self) -> None:  # noqa: N802 - stdlib handler convention
        path = self.path.split("?", 1)[0]
        if not path.startswith("/api/action/"):
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        action = path.rsplit("/", 1)[-1]
        result = self.server.controller.submit(action, self._read_json())
        status = (
            HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
        )
        self._json(status, result)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            return {}
        if length > 64 * 1024:
            return {}
        raw = self.rfile.read(length)
        try:
            value = json.loads(raw.decode("utf-8") or "{}")
        except (UnicodeDecodeError, json.JSONDecodeError):
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
            self._json(
                HTTPStatus.NOT_FOUND,
                {"error": f"missing static asset {name}"},
            )
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
            self._json(
                HTTPStatus.NOT_FOUND, {"error": f"unknown camera {cam}"}
            )
            return
        self.send_response(HTTPStatus.OK)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        last_timestamp = 0.0
        while not self.server.stream.stopped:
            payload, last_timestamp = self.server.stream.wait_frame(
                cam, last_timestamp, timeout=1.0
            )
            if payload is None:
                continue
            header = (
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
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
        sequence = 0
        while not self.server.stream.stopped:
            payload, sequence = self.server.stream.wait_telemetry(
                sequence, timeout=1.0
            )
            if payload is None:
                continue
            data = json.dumps(payload, ensure_ascii=False)
            self.wfile.write(f"data: {data}\r\n\r\n".encode("utf-8"))
            self.wfile.flush()
