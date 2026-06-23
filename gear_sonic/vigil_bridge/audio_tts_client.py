"""Host/VLT-side test client for G1 bridge TTS output.

This is a small HTTP-only debug client. Native `/audio/tts` validates that the
G1 `AudioClient.TtsMaker` path is reachable, but it is not loudness calibrated.
For calibrated speech output, synthesize 16 kHz mono PCM16 WAV on the VLT side
and send it through `/audio/output_segment`.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import struct
from typing import Any
from urllib import request
import wave


JSONDict = dict[str, Any]


def get_json(base_url: str, path: str, timeout: float) -> JSONDict:
    url = _join_url(base_url, path)
    with request.urlopen(url, timeout=timeout) as response:  # noqa: S310 - debug client for configured robot URL.
        raw = response.read()
    return _decode_json(raw)


def post_json(base_url: str, path: str, payload: JSONDict, timeout: float) -> JSONDict:
    url = _join_url(base_url, path)
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    http_request = request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with request.urlopen(http_request, timeout=timeout) as response:  # noqa: S310 - debug client for configured robot URL.
        raw = response.read()
    return _decode_json(raw)


def read_pcm16_wav(path: str) -> tuple[bytes, JSONDict]:
    with wave.open(path, "rb") as wav:
        channels = wav.getnchannels()
        sample_width = wav.getsampwidth()
        sample_rate = wav.getframerate()
        frames = wav.getnframes()
        pcm = wav.readframes(frames)
    if sample_rate != 16000 or channels != 1 or sample_width != 2:
        raise ValueError(
            "WAV must be 16 kHz mono PCM16, got "
            f"sample_rate={sample_rate}, channels={channels}, sample_width={sample_width}"
        )
    return pcm, {
        "sample_rate": sample_rate,
        "channels": channels,
        "sample_width": sample_width,
        "frames": frames,
        "duration_s": frames / float(sample_rate),
        "pcm_bytes": len(pcm),
    }


def generate_tone_pcm(duration_s: float, frequency_hz: float, peak: int) -> bytes:
    sample_rate = 16000
    sample_count = int(max(duration_s, 0.02) * sample_rate)
    peak = max(1, min(int(peak), 32767))
    samples = []
    for index in range(sample_count):
        phase = 2.0 * math.pi * frequency_hz * index / sample_rate
        samples.append(int(round(math.sin(phase) * peak)))
    return struct.pack("<" + "h" * len(samples), *samples)


def run_native_tts(args: argparse.Namespace) -> list[JSONDict]:
    if args.text:
        payloads = [_tts_payload(args.text, args.language, args.speaker_id)]
    else:
        payloads = [
            _tts_payload(args.zh_text, "zh", None),
            _tts_payload(args.en_text, "en", None),
        ]

    responses = []
    for payload in payloads:
        print(f"[client] POST /audio/tts language={payload.get('language')} text={payload['text']!r}")
        response = post_json(args.base_url, "/audio/tts", payload, args.timeout)
        print_json(response)
        responses.append(response)
    return responses


def send_calibrated_wav(args: argparse.Namespace) -> JSONDict:
    pcm, wav_info = read_pcm16_wav(args.send_wav)
    print(f"[client] POST /audio/output_segment wav={args.send_wav}")
    print(f"[client] wav_info={json.dumps(wav_info, ensure_ascii=False)}")
    payload = {
        "encoding": "wav-base64",
        "data": base64.b64encode(_pcm16_wav_bytes(pcm)).decode("ascii"),
        "normalize": not args.no_normalize,
    }
    response = post_json(args.base_url, "/audio/output_segment", payload, args.timeout)
    print_json(response)
    return response


def send_calibrated_tone(args: argparse.Namespace) -> JSONDict:
    pcm = generate_tone_pcm(args.tone_duration, args.tone_frequency, args.tone_peak)
    print(f"[client] POST /audio/output_segment generated_tone_pcm_bytes={len(pcm)}")
    payload = {
        "encoding": "pcm16-base64",
        "data": base64.b64encode(pcm).decode("ascii"),
        "normalize": not args.no_normalize,
    }
    response = post_json(args.base_url, "/audio/output_segment", payload, args.timeout)
    print_json(response)
    return response


def _tts_payload(text: str, language: str | None, speaker_id: int | None) -> JSONDict:
    payload: JSONDict = {"text": text}
    if language:
        payload["language"] = language
    if speaker_id is not None:
        payload["speaker_id"] = speaker_id
    return payload


def _pcm16_wav_bytes(pcm: bytes) -> bytes:
    # `/audio/output_segment` can accept a WAV container; the bridge validates
    # that it is 16 kHz mono PCM16, then extracts and normalizes the PCM frames.
    import io

    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(16000)
        wav.writeframes(pcm)
    return buffer.getvalue()


def _join_url(base_url: str, path: str) -> str:
    return base_url.rstrip("/") + "/" + path.strip("/")


def _decode_json(raw: bytes) -> JSONDict:
    payload = json.loads(raw.decode("utf-8") or "{}")
    if not isinstance(payload, dict):
        raise ValueError("bridge response is not a JSON object")
    return payload


def print_json(payload: JSONDict) -> None:
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))


def main() -> None:
    parser = argparse.ArgumentParser(description="Host/VLT-side G1 bridge TTS debug client.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8765", help="Robot bridge HTTP base URL.")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--skip-health", action="store_true")
    parser.add_argument("--skip-native-tts", action="store_true")
    parser.add_argument("--text", default=None, help="If set, send one native TTS request with this text.")
    parser.add_argument("--language", choices=["zh", "en"], default=None)
    parser.add_argument("--speaker-id", type=int, default=None, help="Optional explicit G1 native TTS speaker id.")
    parser.add_argument("--zh-text", default="你好，这是 VLT 侧 G1 TTS 联调测试。")
    parser.add_argument("--en-text", default="Hello, this is a VLT side G1 TTS integration test.")
    parser.add_argument(
        "--send-wav",
        default=None,
        help="Optional 16 kHz mono PCM16 WAV to send through calibrated /audio/output_segment.",
    )
    parser.add_argument("--send-tone", action="store_true", help="Send a calibrated test tone through PlayStream.")
    parser.add_argument("--tone-duration", type=float, default=1.0)
    parser.add_argument("--tone-frequency", type=float, default=440.0)
    parser.add_argument("--tone-peak", type=int, default=12000)
    parser.add_argument("--no-normalize", action="store_true", help="Do not let bridge normalize PCM peak.")
    args = parser.parse_args()

    print(f"[client] bridge={args.base_url}")
    if not args.skip_health:
        print("[client] GET /audio/health")
        print_json(get_json(args.base_url, "/audio/health", args.timeout))

    if not args.skip_native_tts:
        run_native_tts(args)
        print(
            "[client] note: /audio/tts is native G1 TtsMaker and is not loudness-calibrated. "
            "For production volume, send synthesized PCM/WAV via /audio/output_segment."
        )

    if args.send_tone:
        send_calibrated_tone(args)

    if args.send_wav:
        send_calibrated_wav(args)


if __name__ == "__main__":
    main()
