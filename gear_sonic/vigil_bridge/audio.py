"""Audio runtime helpers for the GR00T-side Vigil bridge.

This module is intentionally runtime-facing only. It exposes G1 microphone and
speaker I/O as audio frames and telemetry; it does not know about Vigil prompts,
judges, scores, or task success.
"""

from __future__ import annotations

import base64
from collections import deque
from dataclasses import dataclass, field
import io
import math
import queue
import socket
import struct
import subprocess
import tempfile
import threading
import time
from typing import Any, Mapping
import uuid
import wave

from gear_sonic.vigil_bridge.protocol import JSONDict

DEFAULT_SAMPLE_RATE = 16000
DEFAULT_CHANNELS = 1
DEFAULT_SAMPLE_WIDTH = 2
DEFAULT_PCM_PEAK_TARGET = 27800
DEFAULT_TTS_TEXT_MAX_CHARS = 500
TTS_SPEAKER_IDS = {
    "zh": 0,
    "en": 1,
}


@dataclass(frozen=True)
class AudioBridgeConfig:
    """Configuration for robot-side bridge audio I/O."""

    enabled: bool = False
    runtime_mode: str = "dry_run"
    sample_rate: int = DEFAULT_SAMPLE_RATE
    channels: int = DEFAULT_CHANNELS
    sample_width: int = DEFAULT_SAMPLE_WIDTH
    mic_group: str = "239.168.123.161"
    mic_port: int = 5555
    mic_interface_ip: str | None = None
    mic_chunk_ms: int = 40
    ring_seconds: float = 10.0
    segment_max_s: float = 10.0
    speaker_volume: int = 100
    speaker_peak_target: int = DEFAULT_PCM_PEAK_TARGET
    speaker_runner: str | None = None
    speaker_iface: str | None = None
    speaker_timeout_s: float = 45.0
    tts_text_max_chars: int = DEFAULT_TTS_TEXT_MAX_CHARS
    fake_speaker: bool = False


@dataclass(frozen=True)
class AudioFrame:
    """A timestamped PCM16 frame."""

    sequence: int
    timestamp_monotonic: float
    pcm: bytes


@dataclass(frozen=True)
class TtsTextSegment:
    """A native TTS text segment bound to one G1 speaker voice."""

    text: str
    language: str
    speaker_id: int

    def as_payload(self) -> JSONDict:
        return {
            "text": self.text,
            "language": self.language,
            "speaker_id": self.speaker_id,
            "text_chars": len(self.text),
        }


@dataclass
class AudioRingBuffer:
    """Bounded audio frame buffer with optional streaming subscribers."""

    max_frames: int
    _frames: deque[AudioFrame] = field(default_factory=deque)
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _subscribers: list[queue.Queue[AudioFrame]] = field(default_factory=list)

    def push(self, frame: AudioFrame) -> None:
        with self._lock:
            self._frames.append(frame)
            while len(self._frames) > self.max_frames:
                self._frames.popleft()
            subscribers = list(self._subscribers)
        for subscriber in subscribers:
            _put_drop_oldest(subscriber, frame)

    def latest_pcm(self, duration_s: float, bytes_per_second: int) -> tuple[bytes, JSONDict]:
        target_bytes = max(0, int(duration_s * bytes_per_second))
        with self._lock:
            frames = list(self._frames)
        if not frames:
            return b"", {"frame_count": 0, "available_bytes": 0}

        chunks: list[bytes] = []
        total = 0
        selected = 0
        for frame in reversed(frames):
            chunks.append(frame.pcm)
            total += len(frame.pcm)
            selected += 1
            if total >= target_bytes:
                break
        pcm = b"".join(reversed(chunks))
        if target_bytes and len(pcm) > target_bytes:
            pcm = pcm[-target_bytes:]
        first_ts = frames[-selected].timestamp_monotonic if selected else None
        last_ts = frames[-1].timestamp_monotonic
        return pcm, {
            "frame_count": selected,
            "available_bytes": sum(len(frame.pcm) for frame in frames),
            "returned_bytes": len(pcm),
            "first_timestamp_monotonic": first_ts,
            "last_timestamp_monotonic": last_ts,
        }

    def subscribe(self, max_frames: int = 32) -> queue.Queue[AudioFrame]:
        subscriber: queue.Queue[AudioFrame] = queue.Queue(maxsize=max_frames)
        with self._lock:
            self._subscribers.append(subscriber)
        return subscriber

    def unsubscribe(self, subscriber: queue.Queue[AudioFrame]) -> None:
        with self._lock:
            self._subscribers = [item for item in self._subscribers if item is not subscriber]

    def stats(self) -> JSONDict:
        with self._lock:
            frame_count = len(self._frames)
            bytes_available = sum(len(frame.pcm) for frame in self._frames)
            last_ts = self._frames[-1].timestamp_monotonic if self._frames else None
        stale_s = None if last_ts is None else time.monotonic() - last_ts
        return {
            "frame_count": frame_count,
            "bytes_available": bytes_available,
            "last_frame_age_s": stale_s,
        }


class G1MicMulticastReceiver:
    """Receives the G1 microphone multicast stream as PCM16 frames."""

    def __init__(self, config: AudioBridgeConfig, ring: AudioRingBuffer) -> None:
        self.config = config
        self.ring = ring
        self.running = False
        self.packet_count = 0
        self.byte_count = 0
        self.error_message: str | None = None
        self._sequence = 0
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None

    def start(self) -> JSONDict:
        if self.running:
            return self.health()
        self.running = True
        self.error_message = None
        self._thread = threading.Thread(target=self._run, name="g1-audio-mic", daemon=True)
        self._thread.start()
        return self.health()

    def stop(self) -> JSONDict:
        self.running = False
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self._socket = None
        return self.health()

    def health(self) -> JSONDict:
        ring_stats = self.ring.stats()
        return {
            "running": self.running,
            "group": self.config.mic_group,
            "port": self.config.mic_port,
            "interface_ip": self.config.mic_interface_ip,
            "packet_count": self.packet_count,
            "byte_count": self.byte_count,
            "error_message": self.error_message,
            "ring": ring_stats,
        }

    def _run(self) -> None:
        try:
            self._socket = self._open_socket()
            while self.running:
                try:
                    data, _addr = self._socket.recvfrom(65535)
                except OSError as exc:
                    if self.running:
                        self.error_message = str(exc)
                    break
                if not data:
                    continue
                if len(data) % self.config.sample_width:
                    data = data[: -(len(data) % self.config.sample_width)]
                self.packet_count += 1
                self.byte_count += len(data)
                self._sequence += 1
                self.ring.push(
                    AudioFrame(
                        sequence=self._sequence,
                        timestamp_monotonic=time.monotonic(),
                        pcm=data,
                    )
                )
        except Exception as exc:  # noqa: BLE001 - surfaced through health.
            self.error_message = str(exc)
        finally:
            self.running = False

    def _open_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self.config.mic_port))
        iface_ip = self.config.mic_interface_ip or "0.0.0.0"
        membership = socket.inet_aton(self.config.mic_group) + socket.inet_aton(iface_ip)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        return sock


class SpeakerClient:
    """Speaker client interface used by `AudioSessionManager`."""

    def play_pcm(self, pcm: bytes, config: AudioBridgeConfig, telemetry: JSONDict) -> JSONDict:
        raise NotImplementedError

    def tts(self, text: str, speaker_id: int, config: AudioBridgeConfig, telemetry: JSONDict) -> JSONDict:
        raise NotImplementedError

    def stop(self) -> JSONDict:
        return {"ok": True, "error_message": None}

    def health(self) -> JSONDict:
        return {"available": False, "type": type(self).__name__}


class FakeSpeakerClient(SpeakerClient):
    """Test speaker that records calls without touching hardware."""

    def __init__(self) -> None:
        self.play_count = 0
        self.tts_count = 0
        self.stop_count = 0
        self.last_pcm_bytes = 0
        self.last_telemetry: JSONDict = {}
        self.last_tts: JSONDict = {}

    def play_pcm(self, pcm: bytes, config: AudioBridgeConfig, telemetry: JSONDict) -> JSONDict:
        self.play_count += 1
        self.last_pcm_bytes = len(pcm)
        self.last_telemetry = dict(telemetry)
        return {
            "ok": True,
            "error_message": None,
            "speaker": "fake",
            "play_count": self.play_count,
            "pcm_bytes": len(pcm),
            "volume": config.speaker_volume,
            "telemetry": telemetry,
        }

    def tts(self, text: str, speaker_id: int, config: AudioBridgeConfig, telemetry: JSONDict) -> JSONDict:
        self.tts_count += 1
        self.last_tts = {
            "text": text,
            "speaker_id": speaker_id,
            "telemetry": dict(telemetry),
        }
        return {
            "ok": True,
            "error_message": None,
            "speaker": "fake",
            "tts_count": self.tts_count,
            "speaker_id": speaker_id,
            "text_chars": len(text),
            "volume": config.speaker_volume,
            "telemetry": telemetry,
        }

    def stop(self) -> JSONDict:
        self.stop_count += 1
        return {"ok": True, "error_message": None, "speaker": "fake", "stop_count": self.stop_count}

    def health(self) -> JSONDict:
        return {
            "available": True,
            "type": "fake",
            "play_count": self.play_count,
            "tts_count": self.tts_count,
            "stop_count": self.stop_count,
        }


class SubprocessSpeakerClient(SpeakerClient):
    """Speaker client that invokes an external G1 PlayStream helper."""

    def play_pcm(self, pcm: bytes, config: AudioBridgeConfig, telemetry: JSONDict) -> JSONDict:
        runner_error = self._runner_error(config)
        if runner_error is not None:
            return runner_error
        with tempfile.NamedTemporaryFile(prefix="g1_bridge_audio_", suffix=".wav", delete=True) as wav_file:
            write_pcm16_wav(wav_file.name, pcm, config.sample_rate, config.channels, config.sample_width)
            cmd = [
                config.speaker_runner,
                "--volume",
                str(config.speaker_volume),
                "--wav",
                wav_file.name,
                _volume_safety_flag(config.speaker_volume),
                "--skip-tts",
            ]
            if config.speaker_iface:
                cmd.extend(["--iface", config.speaker_iface])
            completed, error = self._run(cmd, config)
            if error is not None:
                error.update({"cmd": cmd, "pcm_bytes": len(pcm), "volume": config.speaker_volume, "telemetry": telemetry})
                return error
        return {
            "ok": completed.returncode == 0,
            "error_message": None if completed.returncode == 0 else completed.stderr.strip(),
            "speaker": "subprocess",
            "returncode": completed.returncode,
            "stdout": completed.stdout[-4000:],
            "stderr": completed.stderr[-4000:],
            "pcm_bytes": len(pcm),
            "volume": config.speaker_volume,
            "telemetry": telemetry,
        }

    def tts(self, text: str, speaker_id: int, config: AudioBridgeConfig, telemetry: JSONDict) -> JSONDict:
        runner_error = self._runner_error(config)
        if runner_error is not None:
            return runner_error
        cmd = [
            config.speaker_runner,
            "--volume",
            str(config.speaker_volume),
            _volume_safety_flag(config.speaker_volume),
            "--skip-music",
            "--tts-text",
            text,
            "--tts-speaker-id",
            str(speaker_id),
        ]
        if config.speaker_iface:
            cmd.extend(["--iface", config.speaker_iface])
        completed, error = self._run(cmd, config)
        if error is not None:
            error.update(
                {
                    "cmd": cmd,
                    "speaker_id": speaker_id,
                    "text_chars": len(text),
                    "volume": config.speaker_volume,
                    "telemetry": telemetry,
                }
            )
            return error
        return {
            "ok": completed.returncode == 0,
            "error_message": None if completed.returncode == 0 else completed.stderr.strip(),
            "speaker": "subprocess",
            "returncode": completed.returncode,
            "stdout": completed.stdout[-4000:],
            "stderr": completed.stderr[-4000:],
            "speaker_id": speaker_id,
            "text_chars": len(text),
            "volume": config.speaker_volume,
            "telemetry": telemetry,
        }

    def health(self) -> JSONDict:
        return {"available": True, "type": "subprocess"}

    @staticmethod
    def _runner_error(config: AudioBridgeConfig) -> JSONDict | None:
        if not config.speaker_runner:
            return {
                "ok": False,
                "error_message": "speaker_runner is not configured",
                "speaker": "subprocess",
            }
        if config.speaker_runner.endswith("/"):
            return {
                "ok": False,
                "error_message": f"speaker_runner points to a directory, not an executable: {config.speaker_runner}",
                "speaker": "subprocess",
            }
        return None

    @staticmethod
    def _run(cmd: list[str], config: AudioBridgeConfig) -> tuple[subprocess.CompletedProcess[str] | None, JSONDict | None]:
        try:
            completed = subprocess.run(
                cmd,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=max(config.speaker_timeout_s, 1.0),
            )
        except Exception as exc:  # noqa: BLE001 - return structured WebSocket/HTTP error.
            return None, {
                "ok": False,
                "error_message": str(exc),
                "speaker": "subprocess",
            }
        return completed, None


@dataclass
class AudioSessionManager:
    """Coordinates robot-side audio input/output sessions."""

    config: AudioBridgeConfig = field(default_factory=AudioBridgeConfig)
    speaker_client: SpeakerClient | None = None
    ring: AudioRingBuffer | None = None
    mic_receiver: G1MicMulticastReceiver | None = None
    _sessions: dict[str, JSONDict] = field(default_factory=dict)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def __post_init__(self) -> None:
        max_frames = max(1, int(self.config.ring_seconds * 1000.0 / max(self.config.mic_chunk_ms, 1)))
        if self.ring is None:
            self.ring = AudioRingBuffer(max_frames=max_frames)
        if self.mic_receiver is None:
            self.mic_receiver = G1MicMulticastReceiver(self.config, self.ring)
        if self.speaker_client is None:
            self.speaker_client = FakeSpeakerClient() if self.config.fake_speaker else SubprocessSpeakerClient()

    def capabilities(self) -> JSONDict:
        transport = ["http_segment_fallback"]
        try:
            __import__("websockets")
        except Exception:
            websocket_available = False
        else:
            websocket_available = True
            transport.insert(0, "websocket")
        return {
            "enabled": self.config.enabled,
            "input": ["pcm16_16k_mono_stream", "pcm16_16k_mono_segment"],
            "output": ["pcm16_16k_mono_stream", "pcm16_16k_mono_segment", "native_tts_text"],
            "transport": transport,
            "websocket_available": websocket_available,
            "sample_rate": self.config.sample_rate,
            "channels": self.config.channels,
            "sample_width": self.config.sample_width,
            "speaker_volume": self.config.speaker_volume,
            "speaker_peak_target": self.config.speaker_peak_target,
            "tts": {
                "languages": list(TTS_SPEAKER_IDS),
                "speaker_ids": dict(TTS_SPEAKER_IDS),
                "text_max_chars": self.config.tts_text_max_chars,
                "native_loudness_calibrated": False,
                "calibrated_output_path": "/audio/output_segment",
            },
        }

    def health(self) -> JSONDict:
        assert self.ring is not None
        assert self.mic_receiver is not None
        assert self.speaker_client is not None
        return {
            "ok": True,
            "enabled": self.config.enabled,
            "runtime_mode": self.config.runtime_mode,
            "format": self._format_payload(),
            "sessions": list(self._sessions.values()),
            "mic": self.mic_receiver.health(),
            "speaker": self.speaker_client.health(),
            "ring": self.ring.stats(),
        }

    def start_session(self, payload: Mapping[str, Any] | None = None) -> JSONDict:
        if not self.config.enabled:
            return self._disabled("audio is disabled; start bridge with --audio-enabled")
        options = dict(payload or {})
        session_id = str(options.get("session_id") or uuid.uuid4())
        with self._lock:
            self._sessions[session_id] = {
                "session_id": session_id,
                "started_at_monotonic": time.monotonic(),
                "transport": str(options.get("transport", "http")),
            }
        if bool(options.get("input", True)):
            assert self.mic_receiver is not None
            self.mic_receiver.start()
        return {
            "ok": True,
            "error_message": None,
            "session_id": session_id,
            "format": self._format_payload(),
            "capabilities": self.capabilities(),
            "telemetry": self.health(),
        }

    def stop_session(self, payload: Mapping[str, Any] | None = None) -> JSONDict:
        options = dict(payload or {})
        session_id = options.get("session_id")
        if session_id is None:
            with self._lock:
                stopped = list(self._sessions)
                self._sessions.clear()
        else:
            with self._lock:
                stopped = [str(session_id)] if str(session_id) in self._sessions else []
                self._sessions.pop(str(session_id), None)
        if bool(options.get("stop_input", True)) and not self._sessions:
            assert self.mic_receiver is not None
            self.mic_receiver.stop()
        if bool(options.get("stop_output", True)):
            self.stop_output()
        return {
            "ok": True,
            "error_message": None,
            "stopped_sessions": stopped,
            "telemetry": self.health(),
        }

    def input_segment(self, payload: Mapping[str, Any] | None = None) -> JSONDict:
        if not self.config.enabled:
            return self._disabled("audio is disabled; start bridge with --audio-enabled")
        options = dict(payload or {})
        duration_s = min(
            max(_float(options.get("duration_s"), 1.0), 0.02),
            max(self.config.segment_max_s, 0.02),
        )
        fmt = str(options.get("format", "pcm"))
        assert self.ring is not None
        pcm, telemetry = self.ring.latest_pcm(duration_s, self._bytes_per_second())
        if not pcm:
            return {
                "ok": False,
                "error_message": "no microphone audio frames available",
                "format": self._format_payload(),
                "telemetry": telemetry,
            }
        stats = pcm16_stats(pcm)
        if fmt == "wav":
            data = pcm16_wav_bytes(pcm, self.config.sample_rate, self.config.channels, self.config.sample_width)
            encoding = "wav-base64"
        else:
            data = pcm
            encoding = "pcm16-base64"
        return {
            "ok": True,
            "error_message": None,
            "encoding": encoding,
            "data": base64.b64encode(data).decode("ascii"),
            "duration_s": len(pcm) / float(self._bytes_per_second()),
            "format": self._format_payload(),
            "audio_stats": stats,
            "telemetry": telemetry,
        }

    def output_segment(self, payload: Mapping[str, Any] | None = None) -> JSONDict:
        if not self.config.enabled:
            return self._disabled("audio is disabled; start bridge with --audio-enabled")
        options = dict(payload or {})
        try:
            pcm, decode_telemetry = self._decode_audio_payload(options)
        except Exception as exc:  # noqa: BLE001 - structured bridge response.
            return {"ok": False, "error_message": str(exc), "telemetry": {}}
        max_bytes = int(self.config.segment_max_s * self._bytes_per_second())
        if len(pcm) > max_bytes:
            return {
                "ok": False,
                "error_message": f"audio segment exceeds max duration {self.config.segment_max_s}s",
                "telemetry": {"pcm_bytes": len(pcm), "max_bytes": max_bytes},
            }
        result = self.play_output_pcm(pcm, normalize=bool(options.get("normalize", True)))
        result["decode_telemetry"] = decode_telemetry
        return result

    def play_output_pcm(self, pcm: bytes, normalize: bool = True) -> JSONDict:
        if not self.config.enabled:
            return self._disabled("audio is disabled; start bridge with --audio-enabled")
        if len(pcm) % self.config.sample_width:
            pcm = pcm[: -(len(pcm) % self.config.sample_width)]
        if normalize:
            pcm, telemetry = normalize_pcm16_peak(pcm, self.config.speaker_peak_target)
        else:
            telemetry = {"normalization": "skipped", "audio_stats": pcm16_stats(pcm)}
        assert self.speaker_client is not None
        return self.speaker_client.play_pcm(pcm, self.config, telemetry)

    def tts(self, payload: Mapping[str, Any] | None = None) -> JSONDict:
        if not self.config.enabled:
            return self._disabled("audio is disabled; start bridge with --audio-enabled")
        raw_payload = dict(payload or {})
        try:
            text, language, speaker_id = self._parse_tts_payload(payload or {})
            segmentation = _normalize_tts_segmentation(raw_payload.get("segmentation"))
        except ValueError as exc:
            return {
                "ok": False,
                "error_message": str(exc),
                "telemetry": {"tts": self._tts_capabilities()},
            }
        explicit_speaker_id = raw_payload.get("speaker_id") is not None
        segments = _tts_text_segments(text, language, speaker_id if explicit_speaker_id else None, segmentation)
        telemetry: JSONDict = {
            "method": "native_segmented" if len(segments) > 1 else "native",
            "language": language,
            "speaker_id": speaker_id,
            "text_chars": len(text),
            "text_max_chars": self.config.tts_text_max_chars,
            "segmentation": segmentation,
            "segments": [segment.as_payload() for segment in segments],
            "native_loudness_calibrated": False,
            "volume_model": "AudioClient.TtsMaker does not expose PCM gain; speaker_peak_target is not applied",
            "calibrated_output_path": "/audio/output_segment",
        }
        assert self.speaker_client is not None
        if len(segments) == 1:
            segment = segments[0]
            return self.speaker_client.tts(segment.text, segment.speaker_id, self.config, telemetry)
        return self._tts_segmented(segments, telemetry)

    def stop_output(self) -> JSONDict:
        assert self.speaker_client is not None
        return self.speaker_client.stop()

    def close(self) -> None:
        if self.mic_receiver is not None:
            self.mic_receiver.stop()
        self.stop_output()

    def push_input_pcm_for_test(self, pcm: bytes) -> None:
        assert self.ring is not None
        sequence = int(time.monotonic() * 1000)
        self.ring.push(AudioFrame(sequence=sequence, timestamp_monotonic=time.monotonic(), pcm=pcm))

    def subscribe_input(self) -> queue.Queue[AudioFrame]:
        assert self.ring is not None
        return self.ring.subscribe()

    def unsubscribe_input(self, subscriber: queue.Queue[AudioFrame]) -> None:
        assert self.ring is not None
        self.ring.unsubscribe(subscriber)

    def _decode_audio_payload(self, payload: Mapping[str, Any]) -> tuple[bytes, JSONDict]:
        encoding = str(payload.get("encoding", "pcm16-base64"))
        data = payload.get("data")
        if not isinstance(data, str) or not data:
            raise ValueError("audio payload requires base64 string field 'data'")
        raw = base64.b64decode(data)
        if encoding == "wav-base64":
            pcm, wav_info = pcm16_from_wav_bytes(raw)
            return pcm, {"encoding": encoding, "wav_info": wav_info}
        if encoding == "pcm16-base64":
            if len(raw) % self.config.sample_width:
                raise ValueError("PCM payload byte length must align to sample_width")
            return raw, {"encoding": encoding, "pcm_bytes": len(raw)}
        raise ValueError(f"unsupported audio encoding: {encoding}")

    def _bytes_per_second(self) -> int:
        return self.config.sample_rate * self.config.channels * self.config.sample_width

    def _format_payload(self) -> JSONDict:
        return {
            "sample_rate": self.config.sample_rate,
            "channels": self.config.channels,
            "sample_width": self.config.sample_width,
            "sample_format": "pcm16le",
        }

    def _tts_capabilities(self) -> JSONDict:
        return {
            "languages": list(TTS_SPEAKER_IDS),
            "speaker_ids": dict(TTS_SPEAKER_IDS),
            "text_max_chars": self.config.tts_text_max_chars,
            "native_loudness_calibrated": False,
            "calibrated_output_path": "/audio/output_segment",
        }

    def _parse_tts_payload(self, payload: Mapping[str, Any]) -> tuple[str, str, int]:
        raw_text = payload.get("text")
        if not isinstance(raw_text, str):
            raise ValueError("TTS payload requires string field 'text'")
        text = raw_text.strip()
        if not text:
            raise ValueError("TTS text must not be empty")
        if len(text) > self.config.tts_text_max_chars:
            raise ValueError(f"TTS text exceeds max length {self.config.tts_text_max_chars}")

        language = _normalize_tts_language(payload.get("language"), text)
        raw_speaker_id = payload.get("speaker_id")
        if raw_speaker_id is None:
            speaker_id = TTS_SPEAKER_IDS[language]
        else:
            speaker_id = _int(raw_speaker_id, -1)
            if speaker_id < 0 or speaker_id > 255:
                raise ValueError("TTS speaker_id must be in 0..255")
        return text, language, speaker_id

    def _tts_segmented(self, segments: list["TtsTextSegment"], telemetry: JSONDict) -> JSONDict:
        assert self.speaker_client is not None
        results: list[JSONDict] = []
        ok = True
        error_message = None
        for index, segment in enumerate(segments):
            segment_telemetry = dict(telemetry)
            segment_telemetry.update(
                {
                    "segment_index": index,
                    "segment_count": len(segments),
                    "language": segment.language,
                    "speaker_id": segment.speaker_id,
                    "text_chars": len(segment.text),
                }
            )
            result = self.speaker_client.tts(segment.text, segment.speaker_id, self.config, segment_telemetry)
            result["segment"] = segment.as_payload()
            results.append(result)
            if not bool(result.get("ok", False)):
                ok = False
                error_message = str(result.get("error_message") or f"TTS segment {index} failed")
                break
        return {
            "ok": ok,
            "error_message": error_message,
            "speaker": results[-1].get("speaker") if results else type(self.speaker_client).__name__,
            "segment_count": len(segments),
            "segments": [segment.as_payload() for segment in segments],
            "results": results,
            "text_chars": int(telemetry["text_chars"]),
            "volume": self.config.speaker_volume,
            "telemetry": telemetry,
        }

    @staticmethod
    def _disabled(message: str) -> JSONDict:
        return {"ok": False, "error_message": message}


def pcm16_stats(pcm: bytes) -> JSONDict:
    if not pcm:
        return {"samples": 0, "peak": 0, "rms": 0.0, "mean_abs": 0.0}
    if len(pcm) % 2:
        pcm = pcm[:-1]
    samples = struct.unpack("<" + "h" * (len(pcm) // 2), pcm)
    if not samples:
        return {"samples": 0, "peak": 0, "rms": 0.0, "mean_abs": 0.0}
    peak = max(abs(sample) for sample in samples)
    sum_abs = sum(abs(sample) for sample in samples)
    sum_sq = sum(float(sample) * float(sample) for sample in samples)
    return {
        "samples": len(samples),
        "peak": peak,
        "rms": math.sqrt(sum_sq / len(samples)),
        "mean_abs": sum_abs / float(len(samples)),
    }


def normalize_pcm16_peak(pcm: bytes, target_peak: int) -> tuple[bytes, JSONDict]:
    if not 1 <= target_peak <= 32767:
        raise ValueError("target_peak must be in 1..32767")
    if len(pcm) % 2:
        pcm = pcm[:-1]
    samples = struct.unpack("<" + "h" * (len(pcm) // 2), pcm) if pcm else ()
    if not samples:
        return b"", {"normalization": "empty", "target_peak": target_peak, "audio_stats": pcm16_stats(b"")}
    peak_in = max(abs(sample) for sample in samples)
    if peak_in == 0:
        return pcm, {"normalization": "silent", "target_peak": target_peak, "audio_stats": pcm16_stats(pcm)}
    gain = target_peak / float(peak_in)
    out: list[int] = []
    clipped = 0
    for sample in samples:
        value = int(round(sample * gain))
        if value > 32767:
            value = 32767
            clipped += 1
        elif value < -32768:
            value = -32768
            clipped += 1
        out.append(value)
    normalized = struct.pack("<" + "h" * len(out), *out)
    return normalized, {
        "normalization": "peak",
        "target_peak": target_peak,
        "gain": gain,
        "peak_in": peak_in,
        "clipped_samples": clipped,
        "audio_stats": pcm16_stats(normalized),
    }


def pcm16_wav_bytes(
    pcm: bytes,
    sample_rate: int = DEFAULT_SAMPLE_RATE,
    channels: int = DEFAULT_CHANNELS,
    sample_width: int = DEFAULT_SAMPLE_WIDTH,
) -> bytes:
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(sample_width)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm)
    return buffer.getvalue()


def write_pcm16_wav(path: str, pcm: bytes, sample_rate: int, channels: int, sample_width: int) -> None:
    with wave.open(path, "wb") as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(sample_width)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm)


def pcm16_from_wav_bytes(raw: bytes) -> tuple[bytes, JSONDict]:
    with wave.open(io.BytesIO(raw), "rb") as wav:
        channels = wav.getnchannels()
        sample_width = wav.getsampwidth()
        sample_rate = wav.getframerate()
        frames = wav.getnframes()
        pcm = wav.readframes(frames)
    if sample_rate != DEFAULT_SAMPLE_RATE or channels != DEFAULT_CHANNELS or sample_width != DEFAULT_SAMPLE_WIDTH:
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


def _put_drop_oldest(target: queue.Queue[AudioFrame], frame: AudioFrame) -> None:
    try:
        target.put_nowait(frame)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    try:
        target.put_nowait(frame)
    except queue.Full:
        pass


def _float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _normalize_tts_language(value: Any, text: str) -> str:
    if value is None:
        return "en" if _looks_like_english(text) else "zh"
    normalized = str(value).strip().lower().replace("_", "-")
    aliases = {
        "zh": "zh",
        "zh-cn": "zh",
        "cn": "zh",
        "chinese": "zh",
        "mandarin": "zh",
        "en": "en",
        "en-us": "en",
        "english": "en",
    }
    language = aliases.get(normalized)
    if language is None:
        raise ValueError("TTS language must be one of: zh, en")
    return language


def _looks_like_english(text: str) -> bool:
    letters = [char for char in text if char.isalpha()]
    return bool(letters) and all(ord(char) < 128 for char in letters)


def _tts_text_segments(
    text: str,
    default_language: str,
    forced_speaker_id: int | None = None,
    segmentation: str = "auto",
) -> list[TtsTextSegment]:
    if forced_speaker_id is not None:
        return [TtsTextSegment(text=text, language=default_language, speaker_id=forced_speaker_id)]
    if segmentation == "none":
        return [TtsTextSegment(text=text, language=default_language, speaker_id=TTS_SPEAKER_IDS[default_language])]

    segments: list[TtsTextSegment] = []
    current_language: str | None = None
    current_chars: list[str] = []

    def flush() -> None:
        nonlocal current_language, current_chars
        segment_text = "".join(current_chars).strip()
        if segment_text:
            language = current_language or default_language
            segments.append(
                TtsTextSegment(
                    text=segment_text,
                    language=language,
                    speaker_id=TTS_SPEAKER_IDS[language],
                )
            )
        current_language = None
        current_chars = []

    for char in text:
        language = _tts_char_language(char)
        if language is None:
            if current_chars:
                current_chars.append(char)
            continue
        if current_language is not None and language != current_language:
            flush()
        current_language = language
        current_chars.append(char)
    flush()
    if not segments:
        return [TtsTextSegment(text=text, language=default_language, speaker_id=TTS_SPEAKER_IDS[default_language])]
    if segmentation == "strict":
        return segments
    return _merge_short_embedded_english_segments(segments)


def _tts_char_language(char: str) -> str | None:
    codepoint = ord(char)
    if (
        0x3400 <= codepoint <= 0x4DBF
        or 0x4E00 <= codepoint <= 0x9FFF
        or 0xF900 <= codepoint <= 0xFAFF
        or 0x20000 <= codepoint <= 0x2A6DF
        or 0x2A700 <= codepoint <= 0x2B73F
        or 0x2B740 <= codepoint <= 0x2B81F
        or 0x2B820 <= codepoint <= 0x2CEAF
    ):
        return "zh"
    if char.isascii() and char.isalnum():
        return "en"
    return None


def _merge_short_embedded_english_segments(segments: list[TtsTextSegment]) -> list[TtsTextSegment]:
    merged: list[TtsTextSegment] = []
    for index, segment in enumerate(segments):
        if segment.language == "en" and _is_short_embedded_english(segments, index):
            segment = TtsTextSegment(text=segment.text, language="zh", speaker_id=TTS_SPEAKER_IDS["zh"])
        if merged and merged[-1].language == segment.language:
            previous = merged[-1]
            merged[-1] = TtsTextSegment(
                text=_join_tts_segment_text(previous.text, segment.text),
                language=previous.language,
                speaker_id=previous.speaker_id,
            )
            continue
        merged.append(segment)
    return merged


def _is_short_embedded_english(segments: list[TtsTextSegment], index: int) -> bool:
    has_left_zh = any(segment.language == "zh" for segment in segments[:index])
    has_right_zh = any(segment.language == "zh" for segment in segments[index + 1 :])
    if not (has_left_zh or has_right_zh):
        return False
    tokens = _ascii_alnum_tokens(segments[index].text)
    if not tokens:
        return False
    return len(tokens) <= 4 and sum(len(token) for token in tokens) <= 24


def _ascii_alnum_tokens(text: str) -> list[str]:
    tokens: list[str] = []
    current: list[str] = []
    for char in text:
        if char.isascii() and char.isalnum():
            current.append(char)
            continue
        if current:
            tokens.append("".join(current))
            current = []
    if current:
        tokens.append("".join(current))
    return tokens


def _join_tts_segment_text(left: str, right: str) -> str:
    if not left:
        return right
    if not right:
        return left
    if left[-1].isspace() or right[0].isspace():
        return left + right
    if _tts_char_language(left[-1]) == "en" or _tts_char_language(right[0]) == "en":
        return left + " " + right
    return left + right


def _normalize_tts_segmentation(value: Any) -> str:
    if value is None:
        return "auto"
    normalized = str(value).strip().lower().replace("_", "-")
    aliases = {
        "auto": "auto",
        "smart": "auto",
        "strict": "strict",
        "split": "strict",
        "none": "none",
        "off": "none",
        "disabled": "none",
    }
    segmentation = aliases.get(normalized)
    if segmentation is None:
        raise ValueError("TTS segmentation must be one of: auto, strict, none")
    return segmentation


def _volume_safety_flag(volume: int) -> str:
    return "--allow-volume-over-100" if volume > 100 else "--allow-volume-100"
