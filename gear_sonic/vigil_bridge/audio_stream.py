"""Queued, paced PCM output for the G1 audio bridge.

The WebSocket transport may receive PCM much faster or slower than wall clock.
This module keeps transport ingestion separate from hardware playback, applies
one fixed gain per utterance, and owns the start/write/end lifecycle exposed by
the persistent speaker runner.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import queue
import struct
import threading
import time
from typing import Any, Protocol

JSONDict = dict[str, Any]


class StreamingSpeaker(Protocol):
    """Minimal persistent-speaker interface consumed by the output worker."""

    def start_stream(self, utterance_id: str, telemetry: JSONDict) -> JSONDict:
        ...

    def write_stream_pcm(self, pcm: bytes, telemetry: JSONDict) -> JSONDict:
        ...

    def end_stream(self, telemetry: JSONDict) -> JSONDict:
        ...

    def stop(self) -> JSONDict:
        ...


@dataclass(frozen=True)
class SpeakerOutputConfig:
    sample_rate: int = 16000
    channels: int = 1
    sample_width: int = 2
    chunk_ms: int = 200
    prebuffer_ms: int = 400
    send_lead_ms: int = 20
    queue_seconds: float = 3.0
    drain_ms: int = 150
    target_peak: int = 27800
    completion_timeout_s: float = 45.0

    def __post_init__(self) -> None:
        if self.sample_rate <= 0 or self.channels <= 0 or self.sample_width <= 0:
            raise ValueError("speaker stream audio format values must be positive")
        if self.chunk_ms <= 0:
            raise ValueError("speaker stream chunk_ms must be positive")
        if self.prebuffer_ms < self.chunk_ms:
            raise ValueError("speaker stream prebuffer_ms must be >= chunk_ms")
        if not 0 <= self.send_lead_ms < self.chunk_ms:
            raise ValueError("speaker stream send_lead_ms must be in [0, chunk_ms)")
        if self.queue_seconds <= 0.0:
            raise ValueError("speaker stream queue_seconds must be positive")
        if self.drain_ms < 0:
            raise ValueError("speaker stream drain_ms must be >= 0")
        if not 1 <= self.target_peak <= 32767:
            raise ValueError("speaker stream target_peak must be in 1..32767")

    @property
    def bytes_per_second(self) -> int:
        return self.sample_rate * self.channels * self.sample_width

    @property
    def chunk_bytes(self) -> int:
        frames = max(1, round(self.sample_rate * self.chunk_ms / 1000.0))
        return frames * self.channels * self.sample_width

    @property
    def prebuffer_bytes(self) -> int:
        frames = max(1, round(self.sample_rate * self.prebuffer_ms / 1000.0))
        return frames * self.channels * self.sample_width

    @property
    def queue_bytes(self) -> int:
        return max(self.chunk_bytes, int(self.queue_seconds * self.bytes_per_second))


@dataclass
class _OutputEvent:
    kind: str
    pcm: bytes = b""


@dataclass
class QueuedSpeakerOutput:
    """One utterance worth of ordered PCM with bounded ingress and pacing."""

    speaker: StreamingSpeaker
    utterance_id: str
    config: SpeakerOutputConfig = field(default_factory=SpeakerOutputConfig)
    normalize: bool = True
    _events: queue.Queue[_OutputEvent] = field(default_factory=queue.Queue, init=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _completion: threading.Event = field(default_factory=threading.Event, init=False)
    _abort: threading.Event = field(default_factory=threading.Event, init=False)
    _thread: threading.Thread | None = field(default=None, init=False)
    _queued_pcm_bytes: int = field(default=0, init=False)
    _result: JSONDict = field(default_factory=dict, init=False)
    _telemetry: JSONDict = field(default_factory=dict, init=False)

    def start(self) -> JSONDict:
        if self._thread is not None:
            return self.status()
        self._telemetry = {
            "utterance_id": self.utterance_id,
            "state": "prebuffering",
            "normalize": self.normalize,
            "chunk_ms": self.config.chunk_ms,
            "prebuffer_ms": self.config.prebuffer_ms,
            "send_lead_ms": self.config.send_lead_ms,
            "queue_capacity_bytes": self.config.queue_bytes,
            "received_pcm_bytes": 0,
            "played_pcm_bytes": 0,
            "playstream_calls": 0,
            "underrun_count": 0,
            "clipped_samples": 0,
            "gain": None,
        }
        self._thread = threading.Thread(
            target=self._run,
            name=f"g1-speaker-{self.utterance_id}",
            daemon=True,
        )
        self._thread.start()
        return self.status()

    def write(self, pcm: bytes) -> JSONDict:
        if self._completion.is_set():
            return self._error("speaker output stream is already complete")
        if self._abort.is_set():
            return self._error("speaker output stream is stopping")
        if not pcm:
            return self._error("speaker output stream requires non-empty PCM")
        if len(pcm) % self.config.sample_width:
            return self._error("PCM payload byte length must align to sample_width")
        with self._lock:
            if self._queued_pcm_bytes + len(pcm) > self.config.queue_bytes:
                return {
                    "ok": False,
                    "error_message": "speaker output queue is full; apply upstream backpressure",
                    "backpressure": True,
                    "telemetry": self._status_telemetry_locked(),
                }
            self._queued_pcm_bytes += len(pcm)
            self._telemetry["received_pcm_bytes"] = int(
                self._telemetry.get("received_pcm_bytes", 0)
            ) + len(pcm)
        self._events.put(_OutputEvent("pcm", bytes(pcm)))
        return self.status()

    def finish(self, timeout_s: float | None = None) -> JSONDict:
        if not self._completion.is_set():
            self._events.put(_OutputEvent("end"))
        timeout = self.config.completion_timeout_s if timeout_s is None else max(timeout_s, 0.0)
        if not self._completion.wait(timeout):
            return self._error(f"speaker output did not drain within {timeout:.1f}s")
        return dict(self._result)

    def stop(self, timeout_s: float = 2.0) -> JSONDict:
        self._abort.set()
        self._events.put(_OutputEvent("stop"))
        stop_result = self.speaker.stop()
        self._completion.wait(max(timeout_s, 0.0))
        if self._result:
            return dict(self._result)
        return stop_result

    def status(self) -> JSONDict:
        with self._lock:
            telemetry = self._status_telemetry_locked()
        return {
            "ok": not self._completion.is_set() or bool(self._result.get("ok", False)),
            "error_message": self._result.get("error_message"),
            "utterance_id": self.utterance_id,
            "accepted": not self._completion.is_set(),
            "telemetry": telemetry,
        }

    @property
    def complete(self) -> bool:
        return self._completion.is_set()

    def _run(self) -> None:
        buffer = bytearray()
        ended = False
        speaker_started = False
        next_send_at = 0.0
        playback_end_at = 0.0
        gain = 1.0
        underrun_active = False
        try:
            while True:
                if self._abort.is_set():
                    self._finish_result(True, None, state="stopped")
                    return

                self._drain_available_events(buffer)
                ended = ended or bool(self._telemetry.get("input_ended", False))

                if not speaker_started:
                    if buffer and (len(buffer) >= self.config.prebuffer_bytes or ended):
                        gain = _fixed_stream_gain(buffer, self.config.target_peak) if self.normalize else 1.0
                        self._telemetry["gain"] = gain
                        start_result = self.speaker.start_stream(
                            self.utterance_id,
                            {"streaming": self._status_telemetry()},
                        )
                        if not bool(start_result.get("ok", False)):
                            start_result = self._recover_speaker(start_result)
                            self._finish_result(
                                False,
                                str(start_result.get("error_message") or "speaker stream start failed"),
                                state="failed",
                                speaker=start_result,
                            )
                            return
                        speaker_started = True
                        next_send_at = time.monotonic()
                        playback_end_at = next_send_at
                        self._telemetry["state"] = "playing"
                    elif ended:
                        self._finish_result(True, None, state="complete")
                        return
                    else:
                        self._wait_for_event(buffer, timeout=0.1)
                        continue

                now = time.monotonic()
                if buffer and now >= next_send_at:
                    chunk_size = min(len(buffer), self.config.chunk_bytes)
                    chunk = bytes(buffer[:chunk_size])
                    del buffer[:chunk_size]
                    with self._lock:
                        self._queued_pcm_bytes = max(
                            0,
                            self._queued_pcm_bytes - len(chunk),
                        )
                    normalized, clipped = _apply_pcm16_gain(chunk, gain)
                    send_started = time.monotonic()
                    play_result = self.speaker.write_stream_pcm(
                        normalized,
                        {
                            "utterance_id": self.utterance_id,
                            "chunk_index": int(self._telemetry["playstream_calls"]),
                            "gain": gain,
                        },
                    )
                    send_elapsed_ms = (time.monotonic() - send_started) * 1000.0
                    if not bool(play_result.get("ok", False)):
                        play_result = self._recover_speaker(play_result)
                        self._finish_result(
                            False,
                            str(play_result.get("error_message") or "speaker PlayStream failed"),
                            state="failed",
                            speaker=play_result,
                        )
                        return
                    self._telemetry["played_pcm_bytes"] = int(
                        self._telemetry["played_pcm_bytes"]
                    ) + len(normalized)
                    self._telemetry["playstream_calls"] = int(
                        self._telemetry["playstream_calls"]
                    ) + 1
                    self._telemetry["clipped_samples"] = int(
                        self._telemetry["clipped_samples"]
                    ) + clipped
                    self._telemetry["last_playstream_latency_ms"] = round(send_elapsed_ms, 3)
                    duration_s = len(normalized) / float(self.config.bytes_per_second)
                    playback_end_at = max(playback_end_at, send_started) + duration_s
                    lead_s = min(
                        self.config.send_lead_ms / 1000.0,
                        duration_s * 0.5,
                    )
                    next_send_at = max(send_started, playback_end_at - lead_s)
                    underrun_active = False
                    continue

                if ended and not buffer:
                    remaining = max(0.0, playback_end_at - time.monotonic())
                    remaining += self.config.drain_ms / 1000.0
                    if self._abort.wait(remaining):
                        continue
                    end_result = self.speaker.end_stream(
                        {"streaming": self._status_telemetry()}
                    )
                    if not bool(end_result.get("ok", False)):
                        end_result = self._recover_speaker(end_result)
                        self._finish_result(
                            False,
                            str(end_result.get("error_message") or "speaker stream end failed"),
                            state="failed",
                            speaker=end_result,
                        )
                        return
                    self._finish_result(True, None, state="complete", speaker=end_result)
                    return

                wait_s = 0.1
                if next_send_at > now:
                    wait_s = min(wait_s, next_send_at - now)
                elif not buffer and not underrun_active:
                    self._telemetry["underrun_count"] = int(
                        self._telemetry["underrun_count"]
                    ) + 1
                    underrun_active = True
                self._wait_for_event(buffer, timeout=max(wait_s, 0.001))
        except Exception as exc:  # noqa: BLE001 - surfaced through bridge telemetry.
            try:
                self.speaker.stop()
            except Exception:  # noqa: BLE001 - original stream failure is primary.
                pass
            self._finish_result(False, str(exc), state="failed")

    def _recover_speaker(self, failure: JSONDict) -> JSONDict:
        result = dict(failure)
        try:
            result["recovery"] = self.speaker.stop()
        except Exception as exc:  # noqa: BLE001 - preserve the original speaker failure.
            result["recovery"] = {
                "ok": False,
                "error_message": str(exc),
            }
        return result

    def _drain_available_events(self, buffer: bytearray) -> None:
        while True:
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                return
            self._handle_event(event, buffer)

    def _wait_for_event(self, buffer: bytearray, timeout: float) -> None:
        try:
            event = self._events.get(timeout=timeout)
        except queue.Empty:
            return
        self._handle_event(event, buffer)

    def _handle_event(self, event: _OutputEvent, buffer: bytearray) -> None:
        if event.kind == "pcm":
            buffer.extend(event.pcm)
            return
        if event.kind == "end":
            self._telemetry["input_ended"] = True
            return
        if event.kind == "stop":
            self._abort.set()

    def _finish_result(
        self,
        ok: bool,
        error_message: str | None,
        *,
        state: str,
        speaker: JSONDict | None = None,
    ) -> None:
        self._telemetry["state"] = state
        self._result = {
            "ok": ok,
            "error_message": error_message,
            "utterance_id": self.utterance_id,
            "telemetry": self._status_telemetry(),
        }
        if speaker is not None:
            self._result["speaker_result"] = speaker
        self._completion.set()

    def _error(self, message: str) -> JSONDict:
        return {
            "ok": False,
            "error_message": message,
            "utterance_id": self.utterance_id,
            "telemetry": self._status_telemetry(),
        }

    def _status_telemetry(self) -> JSONDict:
        with self._lock:
            return self._status_telemetry_locked()

    def _status_telemetry_locked(self) -> JSONDict:
        telemetry = dict(self._telemetry)
        telemetry["queued_pcm_bytes"] = self._queued_pcm_bytes
        telemetry["queued_audio_ms"] = round(
            self._queued_pcm_bytes * 1000.0 / self.config.bytes_per_second,
            3,
        )
        return telemetry


def _fixed_stream_gain(pcm: bytes, target_peak: int) -> float:
    if not pcm:
        return 1.0
    samples = struct.unpack("<" + "h" * (len(pcm) // 2), pcm)
    peak = max((abs(sample) for sample in samples), default=0)
    return 1.0 if peak == 0 else target_peak / float(peak)


def _apply_pcm16_gain(pcm: bytes, gain: float) -> tuple[bytes, int]:
    if gain == 1.0 or not pcm:
        return pcm, 0
    samples = struct.unpack("<" + "h" * (len(pcm) // 2), pcm)
    output: list[int] = []
    clipped = 0
    for sample in samples:
        value = int(round(sample * gain))
        if value > 32767:
            value = 32767
            clipped += 1
        elif value < -32768:
            value = -32768
            clipped += 1
        output.append(value)
    return struct.pack("<" + "h" * len(output), *output), clipped
