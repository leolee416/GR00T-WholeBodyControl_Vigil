from __future__ import annotations

import asyncio
import json
import socket
import struct

import pytest

from gear_sonic.vigil_bridge.audio import AudioBridgeConfig, AudioSessionManager, FakeSpeakerClient
from gear_sonic.vigil_bridge.audio_ws import AudioWebSocketServer, AudioWebSocketServerConfig

websockets = pytest.importorskip("websockets")


def _pcm(values: list[int]) -> bytes:
    return struct.pack("<" + "h" * len(values), *values)


def _free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


async def _connect_retry(url: str):
    last_exc: Exception | None = None
    for _ in range(20):
        try:
            return await websockets.connect(url, max_size=None)
        except Exception as exc:  # noqa: BLE001 - retry until server thread binds.
            last_exc = exc
            await asyncio.sleep(0.05)
    assert last_exc is not None
    raise last_exc


async def _recv_until(websocket, message_type: str, timeout_s: float = 2.0) -> dict:
    deadline = asyncio.get_running_loop().time() + timeout_s
    while asyncio.get_running_loop().time() < deadline:
        remaining = max(deadline - asyncio.get_running_loop().time(), 0.01)
        message = await asyncio.wait_for(websocket.recv(), timeout=remaining)
        if isinstance(message, bytes):
            continue
        payload = json.loads(message)
        if payload.get("type") == message_type:
            return payload
    raise AssertionError(f"timed out waiting for {message_type}")


async def _recv_binary(websocket, timeout_s: float = 2.0) -> bytes:
    deadline = asyncio.get_running_loop().time() + timeout_s
    while asyncio.get_running_loop().time() < deadline:
        remaining = max(deadline - asyncio.get_running_loop().time(), 0.01)
        message = await asyncio.wait_for(websocket.recv(), timeout=remaining)
        if isinstance(message, bytes):
            return message
    raise AssertionError("timed out waiting for binary mic frame")


async def _exercise_full_duplex(url: str, manager: AudioSessionManager, speaker: FakeSpeakerClient) -> None:
    websocket = await _connect_retry(url)
    async with websocket:
        started = json.loads(await websocket.recv())
        assert started["type"] == "session.started"
        assert started["payload"]["ok"] is True

        manager.push_input_pcm_for_test(_pcm([111, -222, 333]))
        mic_frame = await _recv_binary(websocket)
        assert mic_frame == _pcm([111, -222, 333])

        await websocket.send(_pcm([-1000, 0, 1000]))
        output_result = await _recv_until(websocket, "output.result")
        assert output_result["payload"]["ok"] is True
        assert speaker.play_count == 1
        assert speaker.last_telemetry["audio_stats"]["peak"] == 27800

        await websocket.send(json.dumps({"type": "session.stop"}))
        stop_result = await _recv_until(websocket, "session.stop.result")
        assert stop_result["payload"]["ok"] is True


async def _exercise_persistent_output_stream(url: str, speaker: FakeSpeakerClient) -> None:
    websocket = await _connect_retry(url)
    async with websocket:
        started = json.loads(await websocket.recv())
        assert started["type"] == "session.started"

        await websocket.send(
            json.dumps(
                {
                    "type": "output.start",
                    "utterance_id": "utt-ws",
                    "normalize": False,
                }
            )
        )
        start_result = await _recv_until(websocket, "output.start.result")
        assert start_result["payload"]["ok"] is True

        chunks = [_pcm([value] * 320) for value in (1000, 2000, 3000)]
        for chunk in chunks:
            await websocket.send(chunk)
            accepted = await _recv_until(websocket, "output.result")
            assert accepted["payload"]["ok"] is True

        await websocket.send(json.dumps({"type": "output.end", "utterance_id": "utt-ws"}))
        end_result = await _recv_until(websocket, "output.end.result")
        assert end_result["payload"]["ok"] is True
        assert speaker.stream_start_count == 1
        assert speaker.stream_write_count == 3
        assert speaker.stream_end_count == 1
        assert bytes(speaker.stream_pcm) == b"".join(chunks)

        await websocket.send(json.dumps({"type": "session.stop"}))
        stop_result = await _recv_until(websocket, "session.stop.result")
        assert stop_result["payload"]["ok"] is True


def test_audio_websocket_full_duplex_with_fake_speaker() -> None:
    port = _free_tcp_port()
    speaker = FakeSpeakerClient()
    manager = AudioSessionManager(
        AudioBridgeConfig(enabled=True, fake_speaker=True),
        speaker_client=speaker,
    )
    server = AudioWebSocketServer(
        manager=manager,
        host="127.0.0.1",
        port=port,
        config=AudioWebSocketServerConfig(start_input_on_connect=False),
    )
    server.start()
    try:
        asyncio.run(_exercise_full_duplex(f"ws://127.0.0.1:{port}/audio/ws", manager, speaker))
    finally:
        server.stop()


def test_audio_websocket_persistent_output_stream() -> None:
    port = _free_tcp_port()
    speaker = FakeSpeakerClient()
    manager = AudioSessionManager(
        AudioBridgeConfig(
            enabled=True,
            fake_speaker=True,
            speaker_stream_chunk_ms=20,
            speaker_stream_prebuffer_ms=40,
            speaker_stream_send_lead_ms=2,
            speaker_stream_queue_s=1.0,
            speaker_stream_drain_ms=0,
        ),
        speaker_client=speaker,
    )
    server = AudioWebSocketServer(
        manager=manager,
        host="127.0.0.1",
        port=port,
        config=AudioWebSocketServerConfig(start_input_on_connect=False),
    )
    server.start()
    try:
        asyncio.run(_exercise_persistent_output_stream(f"ws://127.0.0.1:{port}/audio/ws", speaker))
    finally:
        server.stop()
