"""Host/VLT-side test client for the Vigil bridge audio WebSocket."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import struct
import time
import wave


def generate_tone_pcm(duration_s: float, sample_rate: int, frequency_hz: float, peak: int) -> bytes:
    sample_count = int(duration_s * sample_rate)
    samples = []
    for index in range(sample_count):
        phase = 2.0 * math.pi * frequency_hz * index / sample_rate
        samples.append(int(round(math.sin(phase) * peak)))
    return struct.pack("<" + "h" * len(samples), *samples)


def read_pcm16_wav(path: str) -> bytes:
    with wave.open(path, "rb") as wav:
        channels = wav.getnchannels()
        sample_width = wav.getsampwidth()
        sample_rate = wav.getframerate()
        frames = wav.getnframes()
        pcm = wav.readframes(frames)
    if channels != 1 or sample_width != 2 or sample_rate != 16000:
        raise ValueError(
            "WAV must be 16 kHz mono PCM16, got "
            f"sample_rate={sample_rate}, channels={channels}, sample_width={sample_width}"
        )
    return pcm


def write_pcm16_wav(path: str, pcm: bytes, sample_rate: int = 16000) -> None:
    with wave.open(path, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm)


async def run_client(args: argparse.Namespace) -> None:
    try:
        import websockets
    except Exception as exc:  # noqa: BLE001 - CLI should print a useful message.
        raise SystemExit(f"websockets dependency is unavailable: {exc}") from exc

    mic_chunks: list[bytes] = []
    json_messages: list[dict] = []

    async def recv_until_type(websocket, message_type: str, timeout_s: float = 60.0) -> dict:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = await asyncio.wait_for(
                websocket.recv(),
                timeout=max(0.01, deadline - time.monotonic()),
            )
            if isinstance(message, bytes):
                mic_chunks.append(message)
                continue
            payload = json.loads(message)
            json_messages.append(payload)
            print(json.dumps(payload, ensure_ascii=False))
            if payload.get("type") == message_type:
                return payload
        raise TimeoutError(f"timed out waiting for {message_type}")

    async with websockets.connect(args.url, max_size=None) as websocket:
        started = await websocket.recv()
        print(started)

        output_pcm: bytes | None = None
        if args.send_wav:
            output_pcm = read_pcm16_wav(args.send_wav)
            print(f"[client] sending WAV PCM bytes={len(output_pcm)}")
        elif args.send_tone:
            output_pcm = generate_tone_pcm(
                duration_s=args.tone_duration,
                sample_rate=16000,
                frequency_hz=args.tone_frequency,
                peak=args.tone_peak,
            )
            print(f"[client] sending tone PCM bytes={len(output_pcm)}")

        if output_pcm is not None and args.legacy_one_shot:
            await websocket.send(output_pcm)
        elif output_pcm is not None:
            utterance_id = f"smoke-{int(time.time() * 1000)}"
            await websocket.send(
                json.dumps(
                    {
                        "type": "output.start",
                        "utterance_id": utterance_id,
                        "normalize": not args.no_normalize,
                    }
                )
            )
            start_result = await recv_until_type(websocket, "output.start.result")
            if not start_result.get("payload", {}).get("ok", False):
                raise RuntimeError(f"output.start failed: {start_result}")
            chunk_bytes = max(2, int(32000 * args.output_chunk_ms / 1000.0))
            chunk_bytes -= chunk_bytes % 2
            for offset in range(0, len(output_pcm), chunk_bytes):
                chunk = output_pcm[offset : offset + chunk_bytes]
                await websocket.send(chunk)
                accepted = await recv_until_type(websocket, "output.result")
                if not accepted.get("payload", {}).get("ok", False):
                    raise RuntimeError(f"output chunk rejected: {accepted}")
                if args.pace_output:
                    await asyncio.sleep(len(chunk) / 32000.0)
            await websocket.send(
                json.dumps({"type": "output.end", "utterance_id": utterance_id})
            )
            end_result = await recv_until_type(websocket, "output.end.result")
            if not end_result.get("payload", {}).get("ok", False):
                raise RuntimeError(f"output.end failed: {end_result}")

        deadline = time.monotonic() + max(args.listen_seconds, 0.0)
        while time.monotonic() < deadline:
            try:
                message = await asyncio.wait_for(websocket.recv(), timeout=0.25)
            except asyncio.TimeoutError:
                continue
            if isinstance(message, bytes):
                mic_chunks.append(message)
                print(f"[client] received mic PCM bytes={len(message)}")
            else:
                try:
                    payload = json.loads(message)
                except json.JSONDecodeError:
                    payload = {"raw": message}
                json_messages.append(payload)
                print(json.dumps(payload, ensure_ascii=False))

        await websocket.send(json.dumps({"type": "session.stop"}))
        try:
            message = await asyncio.wait_for(websocket.recv(), timeout=2.0)
            print(message if isinstance(message, str) else f"[client] final binary bytes={len(message)}")
        except asyncio.TimeoutError:
            pass

    if args.record_wav:
        pcm = b"".join(mic_chunks)
        write_pcm16_wav(args.record_wav, pcm)
        print(f"[client] wrote mic recording {args.record_wav} bytes={len(pcm)}")
    print(
        "[client] summary "
        f"mic_chunks={len(mic_chunks)} mic_bytes={sum(len(chunk) for chunk in mic_chunks)} "
        f"json_messages={len(json_messages)}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Test host/VLT-side G1 audio bridge WebSocket I/O.")
    parser.add_argument("--url", default="ws://127.0.0.1:8766/audio/ws")
    parser.add_argument("--listen-seconds", type=float, default=3.0)
    parser.add_argument("--record-wav", default=None, help="Optional output WAV path for received mic PCM.")
    parser.add_argument("--send-tone", action="store_true", help="Send a generated PCM16 tone to robot speaker.")
    parser.add_argument("--tone-duration", type=float, default=1.0)
    parser.add_argument("--tone-frequency", type=float, default=440.0)
    parser.add_argument("--tone-peak", type=int, default=12000)
    parser.add_argument("--send-wav", default=None, help="Send a 16 kHz mono PCM16 WAV to robot speaker.")
    parser.add_argument(
        "--output-chunk-ms",
        type=int,
        default=40,
        help="PCM WebSocket frame duration used by persistent output streaming.",
    )
    parser.add_argument(
        "--no-pace-output",
        dest="pace_output",
        action="store_false",
        help="Send frames as fast as accepted; useful only for short queue/backpressure tests.",
    )
    parser.set_defaults(pace_output=True)
    parser.add_argument(
        "--no-normalize",
        action="store_true",
        help="Disable the bridge's fixed per-utterance PCM gain.",
    )
    parser.add_argument(
        "--legacy-one-shot",
        action="store_true",
        help="Send one unbracketed binary message through the compatibility path.",
    )
    args = parser.parse_args()
    asyncio.run(run_client(args))


if __name__ == "__main__":
    main()
