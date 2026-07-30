from __future__ import annotations

import base64
import struct

from gear_sonic.vigil_bridge.audio import (
    AudioBridgeConfig,
    AudioSessionManager,
    FakeSpeakerClient,
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
        "sonic.sit_chair",
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
