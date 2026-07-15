from __future__ import annotations

import base64
from pathlib import Path
import struct

from gear_sonic.vigil_bridge.audio import (
    AudioBridgeConfig,
    AudioSessionManager,
    FakeSpeakerClient,
    SubprocessSpeakerClient,
    normalize_pcm16_peak,
    pcm16_stats,
)
from gear_sonic.vigil_bridge.service import VigilBridgeService
from gear_sonic.vigil_bridge.transport import BridgeRequestRouter


def _handshake_request() -> dict:
    return {
        "protocol_version": "vigil_groot_bridge_v1",
        "client": {"name": "vigil", "component": "GrootWBCEnv"},
        "episode_id": "test_001",
        "runtime_mode": "dry_run",
        "required_capabilities": {
            "actions": ["navigate.forward"],
            "observation": ["rgb", "robot_state"],
            "oracle_source": "none",
        },
    }


def _pcm(values: list[int]) -> bytes:
    return struct.pack("<" + "h" * len(values), *values)


def test_legacy_handshake_omits_audio_capability_by_default() -> None:
    response = VigilBridgeService().handshake(_handshake_request())

    assert response["ok"] is True
    assert "audio" not in response["capabilities"]
    assert response["capabilities"]["actions"] == [
        "navigate.backward",
        "navigate.forward",
        "navigate.turn_left",
        "navigate.turn_right",
    ]
    assert response["capabilities"]["observation"] == ["rgb", "depth", "robot_state"]


def test_handshake_includes_audio_when_client_requests_it() -> None:
    service = VigilBridgeService(
        audio_manager=AudioSessionManager(
            AudioBridgeConfig(enabled=True, fake_speaker=True, runtime_mode="dry_run")
        )
    )
    request = _handshake_request()
    request["required_capabilities"]["audio"] = {"input": ["pcm16_16k_mono_stream"]}

    response = service.handshake(request)

    assert response["ok"] is True
    assert response["capabilities"]["audio"]["enabled"] is True
    assert response["capabilities"]["audio"]["speaker_peak_target"] == 27800
    assert response["capabilities"]["audio"]["tts"]["speaker_ids"] == {"zh": 0, "en": 1}
    assert response["capabilities"]["audio"]["tts"]["native_loudness_calibrated"] is False
    assert "http_segment_fallback" in response["capabilities"]["audio"]["transport"]


def test_audio_capabilities_describe_reactive_speaker_led() -> None:
    manager = AudioSessionManager(
        AudioBridgeConfig(enabled=True, fake_speaker=True, speaker_reactive_led=True)
    )

    led = manager.capabilities()["speaker_led"]

    assert led["enabled"] is True
    assert led["refresh_hz"] == 50
    assert led["speech_palette"]["low"] == [46, 14, 0]
    assert led["speech_palette"]["high"] == [255, 234, 0]
    assert led["speech_palette"]["blue_channel"] == 0
    assert led["end_animation"]["to_dark_blue_ms"] == 250
    assert led["end_animation"]["dark_to_bright_blue_ms"] == 500
    assert led["end_animation"]["bright_blue_hold_ms"] == 500


def test_pcm_peak_normalization_targets_27800_without_clipping() -> None:
    raw = _pcm([-1000, 0, 1000, 500])

    normalized, telemetry = normalize_pcm16_peak(raw, 27800)

    assert pcm16_stats(normalized)["peak"] == 27800
    assert telemetry["gain"] == 27.8
    assert telemetry["clipped_samples"] == 0


def test_audio_input_segment_returns_recent_ring_pcm() -> None:
    manager = AudioSessionManager(AudioBridgeConfig(enabled=True, fake_speaker=True))
    manager.push_input_pcm_for_test(_pcm([100, -200, 300, -400]))
    service = VigilBridgeService(audio_manager=manager)

    response = BridgeRequestRouter(service).dispatch(
        "audio/input_segment",
        {"duration_s": 0.1, "format": "pcm"},
    )

    assert response["ok"] is True
    assert response["encoding"] == "pcm16-base64"
    decoded = base64.b64decode(response["data"])
    assert decoded == _pcm([100, -200, 300, -400])
    assert response["audio_stats"]["peak"] == 400


def test_audio_output_segment_normalizes_and_calls_fake_speaker() -> None:
    speaker = FakeSpeakerClient()
    manager = AudioSessionManager(
        AudioBridgeConfig(enabled=True, fake_speaker=True, speaker_peak_target=27800),
        speaker_client=speaker,
    )
    service = VigilBridgeService(audio_manager=manager)
    payload = {
        "encoding": "pcm16-base64",
        "data": base64.b64encode(_pcm([-1000, 0, 1000])).decode("ascii"),
    }

    response = BridgeRequestRouter(service).dispatch("audio/output_segment", payload)

    assert response["ok"] is True
    assert speaker.play_count == 1
    assert speaker.last_pcm_bytes == 6
    assert speaker.last_telemetry["audio_stats"]["peak"] == 27800
    assert response["telemetry"]["target_peak"] == 27800


def test_audio_output_segment_reports_bad_speaker_runner_without_raising() -> None:
    manager = AudioSessionManager(
        AudioBridgeConfig(
            enabled=True,
            fake_speaker=False,
            speaker_runner="/tmp/",
        )
    )
    service = VigilBridgeService(audio_manager=manager)
    payload = {
        "encoding": "pcm16-base64",
        "data": base64.b64encode(_pcm([-1000, 0, 1000])).decode("ascii"),
    }

    response = BridgeRequestRouter(service).dispatch("audio/output_segment", payload)

    assert response["ok"] is False
    assert "directory" in response["error_message"]


def test_persistent_output_stream_uses_one_speaker_lifecycle_for_many_chunks() -> None:
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
    chunks = [_pcm([1000] * 320), _pcm([2000] * 320), _pcm([3000] * 320)]

    started = manager.start_output_stream(
        {"utterance_id": "utt-many", "normalize": False}
    )
    accepted = [
        manager.write_output_stream_pcm(chunk, {"utterance_id": "utt-many"})
        for chunk in chunks
    ]
    ended = manager.end_output_stream({"utterance_id": "utt-many", "timeout_s": 2.0})

    assert started["ok"] is True
    assert all(result["ok"] for result in accepted)
    assert ended["ok"] is True
    assert speaker.stream_start_count == 1
    assert speaker.stream_end_count == 1
    assert speaker.stream_write_count == 3
    assert bytes(speaker.stream_pcm) == b"".join(chunks)
    assert ended["telemetry"]["underrun_count"] == 0


def test_persistent_output_stream_reports_backpressure_without_dropping_pcm() -> None:
    speaker = FakeSpeakerClient()
    manager = AudioSessionManager(
        AudioBridgeConfig(
            enabled=True,
            fake_speaker=True,
            speaker_stream_chunk_ms=20,
            speaker_stream_prebuffer_ms=20,
            speaker_stream_send_lead_ms=0,
            speaker_stream_queue_s=0.02,
            speaker_stream_drain_ms=0,
        ),
        speaker_client=speaker,
    )

    assert manager.start_output_stream({"utterance_id": "utt-full"})["ok"] is True
    too_large = manager.write_output_stream_pcm(
        _pcm([1000] * 641),
        {"utterance_id": "utt-full"},
    )
    stopped = manager.stop_output()

    assert too_large["ok"] is False
    assert too_large["backpressure"] is True
    assert "queue is full" in too_large["error_message"]
    assert stopped["ok"] is True
    assert speaker.stop_count >= 1


def test_persistent_output_stream_recovers_after_playstream_failure() -> None:
    class FailFirstWriteSpeaker(FakeSpeakerClient):
        fail_next_write = True

        def write_stream_pcm(self, pcm: bytes, telemetry: dict) -> dict:
            if self.fail_next_write:
                self.fail_next_write = False
                return {"ok": False, "error_message": "injected PlayStream failure"}
            return super().write_stream_pcm(pcm, telemetry)

    speaker = FailFirstWriteSpeaker()
    manager = AudioSessionManager(
        AudioBridgeConfig(
            enabled=True,
            fake_speaker=True,
            speaker_stream_chunk_ms=20,
            speaker_stream_prebuffer_ms=20,
            speaker_stream_send_lead_ms=0,
            speaker_stream_queue_s=1.0,
            speaker_stream_drain_ms=0,
        ),
        speaker_client=speaker,
    )
    pcm = _pcm([1000] * 320)

    assert manager.start_output_stream({"utterance_id": "utt-fails"})["ok"] is True
    assert manager.write_output_stream_pcm(pcm)["ok"] is True
    failed = manager.end_output_stream({"timeout_s": 2.0})

    assert failed["ok"] is False
    assert failed["telemetry"]["state"] == "failed"
    assert failed["speaker_result"]["recovery"]["ok"] is True
    assert speaker.stop_count == 1

    assert manager.start_output_stream({"utterance_id": "utt-recovers"})["ok"] is True
    assert manager.write_output_stream_pcm(pcm)["ok"] is True
    recovered = manager.end_output_stream({"timeout_s": 2.0})

    assert recovered["ok"] is True
    assert speaker.stream_end_count == 1


def test_persistent_subprocess_speaker_reuses_one_runner(tmp_path: Path) -> None:
    runner = tmp_path / "fake_speaker_runner.py"
    runner.write_text(
        """#!/usr/bin/env python3
import sys

stdin = sys.stdin.buffer
print("OK READY", flush=True)
while True:
    raw = stdin.readline()
    if not raw:
        break
    line = raw.decode("utf-8").strip()
    if line.startswith("PCM "):
        size = int(line.split()[1])
        payload = stdin.read(size)
        if len(payload) != size:
            print("ERR short PCM", flush=True)
            break
        print(f"OK PCM {size}", flush=True)
    elif line.startswith("START "):
        print(f"OK {line}", flush=True)
    elif line == "END":
        print("OK END", flush=True)
    elif line == "STOP":
        print("OK STOP", flush=True)
    elif line == "QUIT":
        print("OK QUIT", flush=True)
        break
    else:
        print(f"ERR unsupported {line}", flush=True)
""",
        encoding="utf-8",
    )
    runner.chmod(0o755)
    config = AudioBridgeConfig(
        enabled=True,
        speaker_runner=str(runner),
        speaker_volume=80,
    )
    speaker = SubprocessSpeakerClient(config)

    started = speaker.start_stream("utt-ipc", {})
    pid = speaker.health()["runner_pid"]
    written = speaker.write_stream_pcm(_pcm([100, -100] * 320), {})
    ended = speaker.end_stream({})
    started_again = speaker.start_stream("utt-ipc-2", {})
    same_pid = speaker.health()["runner_pid"]
    stopped = speaker.stop()
    speaker.close()

    assert started["ok"] is True
    assert written["ok"] is True
    assert ended["ok"] is True
    assert started_again["ok"] is True
    assert pid == same_pid
    assert stopped["ok"] is True
    assert speaker.health()["runner_alive"] is False


def test_audio_tts_maps_languages_to_fake_speaker_ids() -> None:
    speaker = FakeSpeakerClient()
    manager = AudioSessionManager(AudioBridgeConfig(enabled=True, fake_speaker=True), speaker_client=speaker)
    service = VigilBridgeService(audio_manager=manager)

    zh_response = BridgeRequestRouter(service).dispatch(
        "audio/tts",
        {"text": "你好，这是中文语音测试。", "language": "zh"},
    )
    en_response = BridgeRequestRouter(service).dispatch(
        "audio/tts",
        {"text": "Hello, this is a G1 TTS test.", "language": "en"},
    )

    assert zh_response["ok"] is True
    assert zh_response["speaker_id"] == 0
    assert zh_response["telemetry"]["native_loudness_calibrated"] is False
    assert en_response["ok"] is True
    assert en_response["speaker_id"] == 1
    assert speaker.tts_count == 2


def test_audio_tts_segments_mixed_language_text() -> None:
    speaker = FakeSpeakerClient()
    manager = AudioSessionManager(AudioBridgeConfig(enabled=True, fake_speaker=True), speaker_client=speaker)
    service = VigilBridgeService(audio_manager=manager)

    response = BridgeRequestRouter(service).dispatch(
        "audio/tts",
        {"text": "你好 G1 hello 世界"},
    )

    assert response["ok"] is True
    assert response["speaker_id"] == 0
    assert response["telemetry"]["segmentation"] == "auto"
    assert response["telemetry"]["segments"] == [
        {"text": "你好 G1 hello 世界", "language": "zh", "speaker_id": 0, "text_chars": 14}
    ]
    assert speaker.tts_count == 1


def test_audio_tts_strict_segmentation_splits_mixed_language_text() -> None:
    speaker = FakeSpeakerClient()
    manager = AudioSessionManager(AudioBridgeConfig(enabled=True, fake_speaker=True), speaker_client=speaker)
    service = VigilBridgeService(audio_manager=manager)

    response = BridgeRequestRouter(service).dispatch(
        "audio/tts",
        {"text": "你好 G1 hello 世界", "segmentation": "strict"},
    )

    assert response["ok"] is True
    assert response["segment_count"] == 3
    assert response["segments"] == [
        {"text": "你好", "language": "zh", "speaker_id": 0, "text_chars": 2},
        {"text": "G1 hello", "language": "en", "speaker_id": 1, "text_chars": 8},
        {"text": "世界", "language": "zh", "speaker_id": 0, "text_chars": 2},
    ]
    assert [item["segment"]["speaker_id"] for item in response["results"]] == [0, 1, 0]
    assert speaker.tts_count == 3


def test_audio_tts_explicit_speaker_id_disables_segmentation() -> None:
    speaker = FakeSpeakerClient()
    manager = AudioSessionManager(AudioBridgeConfig(enabled=True, fake_speaker=True), speaker_client=speaker)
    service = VigilBridgeService(audio_manager=manager)

    response = BridgeRequestRouter(service).dispatch(
        "audio/tts",
        {"text": "你好 G1 hello 世界", "speaker_id": 0},
    )

    assert response["ok"] is True
    assert response["speaker_id"] == 0
    assert response["telemetry"]["segments"] == [
        {"text": "你好 G1 hello 世界", "language": "zh", "speaker_id": 0, "text_chars": 14}
    ]
    assert speaker.tts_count == 1


def test_audio_tts_rejects_empty_text() -> None:
    manager = AudioSessionManager(AudioBridgeConfig(enabled=True, fake_speaker=True))
    service = VigilBridgeService(audio_manager=manager)

    response = BridgeRequestRouter(service).dispatch("audio/tts", {"text": "   ", "language": "en"})

    assert response["ok"] is False
    assert "empty" in response["error_message"]
