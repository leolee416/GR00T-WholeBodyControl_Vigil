"""Read-only web monitor for the existing ZMQ camera stream.

The monitor subscribes to the same camera PUB endpoint used by the Vigil bridge
and exposes browser-friendly MJPEG streams.  It never publishes robot commands
and it does not start or stop the camera service.
"""

from __future__ import annotations

import argparse
import base64
import binascii
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
from typing import Any, Callable, Mapping, Protocol
from urllib.parse import unquote, urlsplit


_DASHBOARD_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>GR00T 相机监控</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; background: #101418; color: #e9eef3; }
    header { position: sticky; top: 0; z-index: 2; display: flex; gap: 14px;
      align-items: center; padding: 14px 18px; background: #171d23ee;
      border-bottom: 1px solid #2c353e; backdrop-filter: blur(8px); }
    h1 { margin: 0; font-size: 18px; font-weight: 650; }
    #status { margin-left: auto; padding: 5px 10px; border-radius: 999px;
      background: #6d4c00; color: #ffe6a1; font-size: 13px; }
    #status.ok { background: #124f34; color: #b9f6d7; }
    #details { color: #aeb9c4; font: 12px ui-monospace, monospace; }
    main { padding: 18px; }
    #empty { padding: 36px; text-align: center; color: #aeb9c4; }
    #grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
      gap: 16px; }
    figure { margin: 0; overflow: hidden; border: 1px solid #2c353e;
      border-radius: 10px; background: #080a0c; }
    img { display: block; width: 100%; min-height: 220px; object-fit: contain; }
    figcaption { display: flex; justify-content: space-between; padding: 9px 12px;
      background: #171d23; color: #cbd4dd; }
    a { color: #75bfff; text-decoration: none; }
  </style>
</head>
<body>
  <header>
    <h1>GR00T 实时相机</h1>
    <span id="details">正在连接…</span>
    <span id="status">等待画面</span>
  </header>
  <main>
    <div id="empty">正在等待相机帧，请确认相机服务和 ZMQ 端口可用。</div>
    <div id="grid"></div>
  </main>
  <script>
    const statusEl = document.getElementById('status');
    const detailsEl = document.getElementById('details');
    const emptyEl = document.getElementById('empty');
    const gridEl = document.getElementById('grid');
    let streamSignature = '';

    function renderStreams(streams) {
      const signature = streams.join('\u0000');
      if (signature === streamSignature) return;
      streamSignature = signature;
      gridEl.replaceChildren();
      for (const name of streams) {
        const figure = document.createElement('figure');
        const image = document.createElement('img');
        image.alt = name;
        image.src = '/stream/' + encodeURIComponent(name) + '.mjpg';
        const caption = document.createElement('figcaption');
        const label = document.createElement('span');
        label.textContent = name;
        const snapshot = document.createElement('a');
        snapshot.href = '/snapshot/' + encodeURIComponent(name);
        snapshot.target = '_blank';
        snapshot.rel = 'noopener';
        snapshot.textContent = '打开单帧';
        caption.append(label, snapshot);
        figure.append(image, caption);
        gridEl.append(figure);
      }
      emptyEl.hidden = streams.length > 0;
    }

    async function refreshHealth() {
      try {
        const response = await fetch('/health', {cache: 'no-store'});
        const health = await response.json();
        renderStreams(health.streams || []);
        statusEl.classList.toggle('ok', Boolean(health.ok));
        statusEl.textContent = health.ok ? '画面正常' : (health.error_message || '等待画面');
        const age = health.last_frame_age_s == null ? '-' : health.last_frame_age_s.toFixed(2) + 's';
        detailsEl.textContent = health.source_endpoint + ' · 最新帧 ' + age;
      } catch (error) {
        statusEl.classList.remove('ok');
        statusEl.textContent = '网页服务连接失败';
        detailsEl.textContent = String(error);
      }
    }
    refreshHealth();
    setInterval(refreshHealth, 1000);
  </script>
</body>
</html>
"""


class CameraPayloadSource(Protocol):
    """Minimal interface implemented by the existing ZMQ image subscriber."""

    error: str | None

    def read_latest(self) -> Mapping[str, Any] | None: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class CameraFrame:
    name: str
    data: bytes
    content_type: str
    sequence: int
    received_at: float
    source_timestamp: float | str | None


def _decode_wire_image(value: Any) -> tuple[bytes, str]:
    """Return encoded image bytes and a browser content type."""

    if isinstance(value, str):
        encoded = value
        if value.startswith("data:") and "," in value:
            encoded = value.split(",", 1)[1]
        try:
            data = base64.b64decode(encoded, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise ValueError("image string is not valid base64") from exc
    elif isinstance(value, bytes | bytearray | memoryview):
        data = bytes(value)
    else:
        raise ValueError(f"unsupported image value type: {type(value).__name__}")

    if data.startswith(b"\xff\xd8"):
        return data, "image/jpeg"
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        return data, "image/png"
    raise ValueError("image payload is neither JPEG nor PNG")


class CameraFrameStore:
    """Thread-safe latest-frame store shared by capture and HTTP threads."""

    def __init__(
        self,
        source_endpoint: str,
        stale_after_s: float = 2.0,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.source_endpoint = source_endpoint
        self.stale_after_s = stale_after_s
        self._monotonic = monotonic
        self._condition = threading.Condition()
        self._frames: dict[str, CameraFrame] = {}
        self._next_sequence = 1
        self._payload_count = 0
        self._source_error: str | None = None
        self._decode_error: str | None = None

    def update(self, payload: Mapping[str, Any]) -> int:
        raw_images = payload.get("images", {})
        raw_timestamps = payload.get("timestamps", {})
        if not isinstance(raw_images, Mapping):
            self.set_decode_error("camera payload field 'images' is not an object")
            return 0
        timestamps = raw_timestamps if isinstance(raw_timestamps, Mapping) else {}

        now = self._monotonic()
        decoded: list[tuple[str, bytes, str, float | str | None]] = []
        errors: list[str] = []
        for raw_name, value in raw_images.items():
            name = str(raw_name)
            try:
                data, content_type = _decode_wire_image(value)
            except ValueError as exc:
                errors.append(f"{name}: {exc}")
                continue
            source_timestamp = timestamps.get(raw_name, timestamps.get(name))
            if not isinstance(source_timestamp, float | int | str):
                source_timestamp = None
            decoded.append((name, data, content_type, source_timestamp))

        with self._condition:
            for name, data, content_type, source_timestamp in decoded:
                self._frames[name] = CameraFrame(
                    name=name,
                    data=data,
                    content_type=content_type,
                    sequence=self._next_sequence,
                    received_at=now,
                    source_timestamp=source_timestamp,
                )
                self._next_sequence += 1
            if decoded:
                self._payload_count += 1
                self._source_error = None
            self._decode_error = "; ".join(errors) if errors else None
            if decoded or errors:
                self._condition.notify_all()
        return len(decoded)

    def set_source_error(self, error: str | None) -> None:
        with self._condition:
            self._source_error = error
            self._condition.notify_all()

    def set_decode_error(self, error: str | None) -> None:
        with self._condition:
            self._decode_error = error
            self._condition.notify_all()

    def latest(self, name: str) -> CameraFrame | None:
        with self._condition:
            return self._frames.get(name)

    def wait_for_frame(
        self,
        name: str,
        after_sequence: int,
        timeout_s: float = 1.0,
    ) -> CameraFrame | None:
        deadline = self._monotonic() + timeout_s
        with self._condition:
            while True:
                frame = self._frames.get(name)
                if frame is not None and frame.sequence > after_sequence:
                    return frame
                remaining = deadline - self._monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)

    def health(self) -> dict[str, Any]:
        with self._condition:
            now = self._monotonic()
            frames = list(self._frames.values())
            details = [
                {
                    "name": frame.name,
                    "content_type": frame.content_type,
                    "age_s": round(max(0.0, now - frame.received_at), 3),
                    "source_timestamp": frame.source_timestamp,
                    "sequence": frame.sequence,
                }
                for frame in sorted(frames, key=lambda item: item.name)
            ]
            newest_age = min((item["age_s"] for item in details), default=None)
            receiving = newest_age is not None and newest_age <= self.stale_after_s
            if self._source_error:
                error_message = self._source_error
            elif not frames:
                error_message = "waiting for camera frames"
            elif not receiving:
                error_message = f"camera stream is stale ({newest_age:.2f}s)"
            else:
                error_message = None
            return {
                "ok": receiving and self._source_error is None,
                "error_message": error_message,
                "source_endpoint": self.source_endpoint,
                "receiving_frames": receiving,
                "last_frame_age_s": newest_age,
                "streams": [item["name"] for item in details],
                "stream_details": details,
                "payload_count": self._payload_count,
                "decode_warning": self._decode_error,
            }


class CameraMonitor:
    """Poll a camera payload source without ever writing to robot transports."""

    def __init__(
        self,
        source: CameraPayloadSource,
        frame_store: CameraFrameStore,
        poll_fps: float = 30.0,
    ) -> None:
        self.source = source
        self.frame_store = frame_store
        self.poll_interval_s = 1.0 / max(poll_fps, 1.0)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="camera-web-monitor-capture",
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.source.close()

    def _capture_loop(self) -> None:
        previous_payload: Mapping[str, Any] | None = None
        while not self._stop_event.is_set():
            try:
                payload = self.source.read_latest()
                source_error = getattr(self.source, "error", None)
                if source_error and payload is None:
                    self.frame_store.set_source_error(str(source_error))
                if payload is not None and payload is not previous_payload:
                    self.frame_store.update(payload)
                    previous_payload = payload
            except Exception as exc:  # noqa: BLE001 - keep health page available.
                self.frame_store.set_source_error(str(exc))
            self._stop_event.wait(self.poll_interval_s)


class _CameraMonitorHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def create_camera_monitor_server(
    host: str,
    port: int,
    frame_store: CameraFrameStore,
) -> ThreadingHTTPServer:
    """Create the web server; caller controls its lifecycle."""

    class CameraMonitorHTTPHandler(BaseHTTPRequestHandler):
        server_version = "GrootCameraMonitor/1.0"

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path in {"", "/"}:
                self._write_bytes(
                    HTTPStatus.OK,
                    _DASHBOARD_HTML.encode("utf-8"),
                    "text/html; charset=utf-8",
                )
                return
            if path == "/health":
                self._write_json(HTTPStatus.OK, frame_store.health())
                return
            if path == "/favicon.ico":
                self._write_bytes(HTTPStatus.NO_CONTENT, b"", "image/x-icon")
                return
            if path.startswith("/snapshot/"):
                name = unquote(path[len("/snapshot/") :])
                self._write_snapshot(name)
                return
            if path.startswith("/stream/"):
                name = unquote(path[len("/stream/") :])
                if name.endswith(".mjpg"):
                    name = name.removesuffix(".mjpg")
                self._write_stream(name)
                return
            self._write_json(
                HTTPStatus.NOT_FOUND,
                {"ok": False, "error_message": f"unsupported endpoint: {path}"},
            )

        def log_message(self, format: str, *args: Any) -> None:
            return

        def _write_snapshot(self, name: str) -> None:
            frame = frame_store.latest(name)
            if frame is None:
                self._write_json(
                    HTTPStatus.NOT_FOUND,
                    {"ok": False, "error_message": f"camera stream not found: {name}"},
                )
                return
            self._write_bytes(HTTPStatus.OK, frame.data, frame.content_type)

        def _write_stream(self, name: str) -> None:
            initial_frame = frame_store.latest(name)
            if initial_frame is None:
                self._write_json(
                    HTTPStatus.NOT_FOUND,
                    {"ok": False, "error_message": f"camera stream not found: {name}"},
                )
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.end_headers()
            sequence = -1
            try:
                while True:
                    frame = frame_store.wait_for_frame(name, sequence, timeout_s=1.0)
                    if frame is None:
                        frame = frame_store.latest(name)
                        if frame is None:
                            continue
                    sequence = frame.sequence
                    part_headers = (
                        b"--frame\r\n"
                        + f"Content-Type: {frame.content_type}\r\n".encode("ascii")
                        + f"Content-Length: {len(frame.data)}\r\n\r\n".encode("ascii")
                    )
                    self.wfile.write(part_headers)
                    self.wfile.write(frame.data)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            except OSError:
                return

        def _write_json(self, status: HTTPStatus, payload: Mapping[str, Any]) -> None:
            body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            self._write_bytes(status, body, "application/json; charset=utf-8")

        def _write_bytes(
            self,
            status: HTTPStatus,
            body: bytes,
            content_type: str,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            if body:
                self.wfile.write(body)

    return _CameraMonitorHTTPServer((host, port), CameraMonitorHTTPHandler)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Expose the existing GR00T ZMQ camera stream as a read-only web page.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address")
    parser.add_argument("--port", type=int, default=8780, help="HTTP listen port")
    parser.add_argument("--camera-host", default="127.0.0.1", help="ZMQ camera host")
    parser.add_argument("--camera-port", type=int, default=5555, help="ZMQ camera port")
    parser.add_argument("--poll-fps", type=float, default=30.0, help="ZMQ poll frequency")
    parser.add_argument(
        "--stale-after",
        type=float,
        default=2.0,
        help="seconds without a new payload before health becomes unhealthy",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.port < 0 or args.port > 65535:
        raise SystemExit("--port must be between 0 and 65535")
    if args.camera_port <= 0 or args.camera_port > 65535:
        raise SystemExit("--camera-port must be between 1 and 65535")
    if args.poll_fps <= 0:
        raise SystemExit("--poll-fps must be positive")
    if args.stale_after <= 0:
        raise SystemExit("--stale-after must be positive")

    try:
        from gear_sonic.vigil_bridge.mujoco_adapter import ZMQImageSubscriber

        source = ZMQImageSubscriber(host=args.camera_host, port=args.camera_port)
    except Exception as exc:  # noqa: BLE001 - turn optional dependency errors into CLI errors.
        raise SystemExit(f"failed to create camera subscriber: {exc}") from exc

    endpoint = f"tcp://{args.camera_host}:{args.camera_port}"
    frame_store = CameraFrameStore(endpoint, stale_after_s=args.stale_after)
    monitor = CameraMonitor(source, frame_store, poll_fps=args.poll_fps)
    try:
        server = create_camera_monitor_server(args.host, args.port, frame_store)
    except Exception:
        source.close()
        raise

    monitor.start()
    bound_host, bound_port = server.server_address[:2]
    display_host = "<host-ip>" if bound_host in {"0.0.0.0", "::"} else bound_host
    print(f"Camera source: {endpoint}")
    print(f"Camera monitor: http://{display_host}:{bound_port}/")
    print("Read-only monitor started. Press Ctrl-C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping camera monitor...")
    finally:
        server.server_close()
        monitor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
