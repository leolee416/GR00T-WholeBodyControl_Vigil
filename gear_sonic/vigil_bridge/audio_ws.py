"""Optional WebSocket transport for bridge audio sessions."""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass
import json
import queue
import threading
from typing import Any

from gear_sonic.vigil_bridge.audio import AudioFrame, AudioSessionManager


@dataclass(frozen=True)
class AudioWebSocketServerConfig:
    """Configuration for the optional audio WebSocket server."""

    start_input_on_connect: bool = True


class AudioWebSocketServer:
    """Runs a small optional WebSocket server for full-duplex audio frames."""

    def __init__(
        self,
        manager: AudioSessionManager,
        host: str,
        port: int,
        config: AudioWebSocketServerConfig | None = None,
    ) -> None:
        self.manager = manager
        self.host = host
        self.port = port
        self.config = config or AudioWebSocketServerConfig()
        self.error_message: str | None = None
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop_event: asyncio.Event | None = None

    def start(self) -> dict[str, Any]:
        if self._thread is not None and self._thread.is_alive():
            return self.health()
        self.error_message = None
        self._thread = threading.Thread(target=self._run_thread, name="vigil-audio-ws", daemon=True)
        self._thread.start()
        return self.health()

    def stop(self) -> dict[str, Any]:
        if self._loop is not None and self._stop_event is not None:
            self._loop.call_soon_threadsafe(self._stop_event.set)
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._loop = None
        self._stop_event = None
        return self.health()

    def health(self) -> dict[str, Any]:
        return {
            "ok": self.error_message is None,
            "host": self.host,
            "port": self.port,
            "running": self._thread is not None and self._thread.is_alive(),
            "error_message": self.error_message,
        }

    def _run_thread(self) -> None:
        try:
            asyncio.run(self._serve())
        except Exception as exc:  # noqa: BLE001 - surfaced through health.
            self.error_message = str(exc)

    async def _serve(self) -> None:
        try:
            import websockets
        except Exception as exc:  # noqa: BLE001 - optional dependency.
            self.error_message = f"websockets dependency is unavailable: {exc}"
            return

        self._loop = asyncio.get_running_loop()
        self._stop_event = asyncio.Event()
        async with websockets.serve(self._handle_socket, self.host, self.port):
            await self._stop_event.wait()

    async def _handle_socket(self, websocket: Any) -> None:
        session = self.manager.start_session(
            {
                "transport": "websocket",
                "input": self.config.start_input_on_connect,
            }
        )
        await websocket.send(json.dumps({"type": "session.started", "payload": session}))
        subscriber = self.manager.subscribe_input()
        producer = asyncio.create_task(self._produce_mic_frames(websocket, subscriber))
        consumer = asyncio.create_task(self._consume_output_frames(websocket))
        try:
            done, pending = await asyncio.wait(
                {producer, consumer},
                return_when=asyncio.FIRST_COMPLETED,
            )
            for task in done:
                task.result()
            for task in pending:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task
        finally:
            self.manager.unsubscribe_input(subscriber)
            self.manager.stop_session({"session_id": session.get("session_id"), "stop_output": True})

    async def _produce_mic_frames(self, websocket: Any, subscriber: queue.Queue[AudioFrame]) -> None:
        while True:
            try:
                frame = subscriber.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.02)
                continue
            await websocket.send(frame.pcm)

    async def _consume_output_frames(self, websocket: Any) -> None:
        async for message in websocket:
            if isinstance(message, bytes):
                try:
                    result = self.manager.play_output_pcm(message, normalize=True)
                except Exception as exc:  # noqa: BLE001 - keep the socket alive and report failure.
                    result = {
                        "ok": False,
                        "error_message": str(exc),
                    }
                await websocket.send(json.dumps({"type": "output.result", "payload": result}))
                continue
            payload = json.loads(message)
            message_type = str(payload.get("type", ""))
            if message_type == "ping":
                await websocket.send(json.dumps({"type": "pong", "payload": self.manager.health()}))
            elif message_type == "output.stop":
                await websocket.send(
                    json.dumps({"type": "output.stop.result", "payload": self.manager.stop_output()})
                )
            elif message_type == "session.stop":
                await websocket.send(
                    json.dumps({"type": "session.stop.result", "payload": self.manager.stop_session(payload)})
                )
                return
            else:
                await websocket.send(
                    json.dumps(
                        {
                            "type": "error",
                            "payload": {
                                "ok": False,
                                "error_message": f"unsupported audio websocket message: {message_type}",
                            },
                        }
                    )
                )
