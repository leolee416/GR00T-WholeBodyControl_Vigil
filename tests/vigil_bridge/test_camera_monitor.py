from __future__ import annotations

import base64
import json
import threading
import time
from typing import Iterator
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from gear_sonic.vigil_bridge.camera_monitor import (
    CameraMonitor,
    CameraFrameStore,
    create_camera_monitor_server,
)


JPEG = b"\xff\xd8" + (b"camera-frame" * 32) + b"\xff\xd9"


@pytest.fixture
def monitor_server() -> Iterator[tuple[str, CameraFrameStore]]:
    store = CameraFrameStore("tcp://127.0.0.1:5555")
    store.update(
        {
            "images": {"ego_view": base64.b64encode(JPEG).decode("ascii")},
            "timestamps": {"ego_view": 123.5},
        }
    )
    server = create_camera_monitor_server("127.0.0.1", 0, store)
    host, port = server.server_address
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://{host}:{port}", store
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_frame_store_accepts_base64_jpeg_and_raw_png() -> None:
    store = CameraFrameStore("tcp://camera:5555")
    png = b"\x89PNG\r\n\x1a\n" + b"payload"

    count = store.update(
        {
            "images": {
                "ego_view": base64.b64encode(JPEG).decode("ascii"),
                "depth": png,
            },
            "timestamps": {"ego_view": 1.0, "depth": 2.0},
        }
    )

    assert count == 2
    assert store.latest("ego_view").data == JPEG
    assert store.latest("ego_view").content_type == "image/jpeg"
    assert store.latest("depth").content_type == "image/png"
    assert store.health()["ok"] is True
    assert store.health()["streams"] == ["depth", "ego_view"]


def test_frame_store_reports_invalid_image_without_crashing() -> None:
    store = CameraFrameStore("tcp://camera:5555")

    count = store.update({"images": {"ego_view": "not-base64"}})

    health = store.health()
    assert count == 0
    assert health["ok"] is False
    assert health["error_message"] == "waiting for camera frames"
    assert "not valid base64" in health["decode_warning"]


def test_frame_store_marks_old_frames_as_stale() -> None:
    now = [10.0]
    store = CameraFrameStore(
        "tcp://camera:5555",
        stale_after_s=2.0,
        monotonic=lambda: now[0],
    )
    store.update({"images": {"ego_view": JPEG}})
    assert store.health()["ok"] is True

    now[0] = 12.1

    health = store.health()
    assert health["ok"] is False
    assert health["receiving_frames"] is False
    assert "stale" in health["error_message"]


def test_monitor_captures_each_source_payload_once_and_closes_source() -> None:
    payload = {"images": {"ego_view": JPEG}}

    class FakeSource:
        error = None

        def __init__(self) -> None:
            self.closed = False

        def read_latest(self):
            return payload

        def close(self) -> None:
            self.closed = True

    source = FakeSource()
    store = CameraFrameStore("tcp://camera:5555")
    monitor = CameraMonitor(source, store, poll_fps=100.0)
    monitor.start()
    deadline = time.monotonic() + 1.0
    while store.latest("ego_view") is None and time.monotonic() < deadline:
        time.sleep(0.01)
    monitor.close()

    assert store.latest("ego_view").data == JPEG
    assert store.health()["payload_count"] == 1
    assert source.closed is True


def test_dashboard_health_and_snapshot_endpoints(monitor_server) -> None:
    base_url, _store = monitor_server

    with urlopen(f"{base_url}/", timeout=2.0) as response:
        assert response.status == 200
        assert "GR00T 实时相机" in response.read().decode("utf-8")

    with urlopen(f"{base_url}/health", timeout=2.0) as response:
        health = json.loads(response.read().decode("utf-8"))
        assert response.status == 200
        assert health["ok"] is True
        assert health["streams"] == ["ego_view"]

    with urlopen(f"{base_url}/snapshot/ego_view", timeout=2.0) as response:
        assert response.headers["Content-Type"] == "image/jpeg"
        assert response.read() == JPEG


def test_mjpeg_endpoint_returns_multipart_frame(monitor_server) -> None:
    base_url, _store = monitor_server

    with urlopen(f"{base_url}/stream/ego_view.mjpg", timeout=2.0) as response:
        assert response.headers["Content-Type"] == "multipart/x-mixed-replace; boundary=frame"
        assert response.readline() == b"--frame\r\n"
        assert response.readline() == b"Content-Type: image/jpeg\r\n"
        length_header = response.readline().decode("ascii")
        assert length_header == f"Content-Length: {len(JPEG)}\r\n"
        assert response.readline() == b"\r\n"
        assert response.read(len(JPEG)) == JPEG


def test_missing_snapshot_returns_404(monitor_server) -> None:
    base_url, _store = monitor_server

    with pytest.raises(HTTPError) as exc_info:
        urlopen(f"{base_url}/snapshot/missing", timeout=2.0)

    assert exc_info.value.code == 404
    body = json.loads(exc_info.value.read().decode("utf-8"))
    assert body["ok"] is False


def test_missing_mjpeg_stream_returns_404(monitor_server) -> None:
    base_url, _store = monitor_server

    with pytest.raises(HTTPError) as exc_info:
        urlopen(f"{base_url}/stream/missing.mjpg", timeout=2.0)

    assert exc_info.value.code == 404
