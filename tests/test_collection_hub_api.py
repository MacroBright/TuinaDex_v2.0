"""Hub HTTP/REST/MJPEG endpoint tests on an ephemeral port."""
import json
import time
import urllib.error
import urllib.request

import pytest

from tuinadex_teleop.collection.collector import TuinaCollector
from tuinadex_teleop.collection.data_channel import MockDataChannel
from tuinadex_teleop.collection.data_hub_controller import DataHubController
from tuinadex_teleop.hub.server import CollectionHubServer


def _post(url, payload=None):
    body = json.dumps(payload or {}).encode()
    request = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read().decode())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read().decode())


def _get_json(url):
    with urllib.request.urlopen(url, timeout=5) as response:
        return response.status, json.loads(response.read().decode())


@pytest.fixture
def hub():
    channel = MockDataChannel(shape=(48, 64, 3), dof=22)
    collector = TuinaCollector(env=None)
    controller = DataHubController(
        channel, collector, control_fps=50.0, ui_fps=25.0
    )
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
    controller.tick()
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
    with urllib.request.urlopen(f"{base}/", timeout=5) as response:
        assert response.status == 200
        assert "TuinaDex" in response.read().decode()
    for path in ("/app.js", "/style.css"):
        with urllib.request.urlopen(f"{base}{path}", timeout=5) as response:
            assert response.status == 200


def test_mjpeg_stream_yields_jpeg_frames(hub):
    base, controller, server = hub
    import threading

    stop = threading.Event()

    def _pump():
        while not stop.is_set():
            controller.tick()
            server.stream.publish(
                controller.channel.get_camera_packets(),
                controller.status(),
            )
            time.sleep(0.01)

    thread = threading.Thread(target=_pump, daemon=True)
    thread.start()
    try:
        with urllib.request.urlopen(
            f"{base}/video/overhead", timeout=5
        ) as response:
            assert response.headers["Content-Type"].startswith(
                "multipart/x-mixed-replace"
            )
            chunk = response.read(2048)
            assert (
                b"--frame" in chunk
                or b"Content-Type: image/jpeg" in chunk
            )
    finally:
        stop.set()
        thread.join(2.0)


def test_sse_stream_emits_telemetry(hub):
    base, controller, server = hub
    import threading

    stop = threading.Event()

    def _pump():
        while not stop.is_set():
            controller.tick()
            server.stream.publish(
                controller.channel.get_camera_packets(),
                controller.status(),
            )
            time.sleep(0.01)

    thread = threading.Thread(target=_pump, daemon=True)
    thread.start()
    try:
        with urllib.request.urlopen(
            f"{base}/events/telemetry", timeout=5
        ) as response:
            assert response.headers["Content-Type"].startswith(
                "text/event-stream"
            )
            payload = response.read(512).decode(errors="ignore")
            assert "data:" in payload
    finally:
        stop.set()
        thread.join(2.0)


def test_mjpeg_multipart_framing_is_rfc_compliant(hub):
    """每个分块必须是 CRLF--frame 分隔，且 Content-Length 与 JPEG 长度一致 (W11)."""
    base, controller, server = hub
    import threading

    stop = threading.Event()

    def _pump():
        while not stop.is_set():
            controller.tick()
            server.stream.publish(
                controller.channel.get_camera_packets(),
                controller.status(),
            )
            time.sleep(0.01)

    thread = threading.Thread(target=_pump, daemon=True)
    thread.start()
    buffer = b""
    try:
        with urllib.request.urlopen(
            f"{base}/video/wrist", timeout=5
        ) as response:
            assert response.headers["Content-Type"] == (
                "multipart/x-mixed-replace; boundary=frame"
            )
            deadline = time.time() + 5.0
            while buffer.count(b"--frame") < 3 and time.time() < deadline:
                chunk = response.read(4096)
                if not chunk:
                    break
                buffer += chunk
    finally:
        stop.set()
        thread.join(2.0)

    parts = buffer.split(b"--frame")
    assert len(parts) >= 3, f"expected >=2 complete parts, got {len(parts) - 1}"
    for raw in parts[1:3]:
        assert raw.startswith(b"\r\n"), "delimiter must begin with CRLF"
        head, sep, rest = raw.partition(b"\r\n\r\n")
        assert sep, "part headers must end with a blank line"
        assert b"Content-Type: image/jpeg" in head
        length = int(head.split(b"Content-Length: ")[1].split(b"\r\n")[0])
        payload = rest[:length]
        assert payload[:2] == b"\xff\xd8", "JPEG SOI missing"
        assert payload[-2:] == b"\xff\xd9", "JPEG EOI missing"
        assert rest[length:length + 2] == b"\r\n", "part must close with CRLF"
