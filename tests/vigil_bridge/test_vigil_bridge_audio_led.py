from __future__ import annotations

import math
from pathlib import Path
import struct
import subprocess

import pytest

from gear_sonic.vigil_bridge.audio import AudioBridgeConfig, SubprocessSpeakerClient
from gear_sonic.vigil_bridge.audio_led import build_reactive_led_plan, speech_color


def _sine_pcm(amplitudes: list[int], frames_per_amplitude: int = 40) -> bytes:
    samples: list[int] = []
    frame_samples = 320
    for amplitude in amplitudes:
        for frame in range(frames_per_amplitude):
            for index in range(frame_samples):
                phase = 2.0 * math.pi * 440.0 * (frame * frame_samples + index) / 16000.0
                samples.append(round(math.sin(phase) * amplitude))
    return struct.pack("<" + "h" * len(samples), *samples)


def test_speech_palette_is_saturated_orange_to_bright_yellow() -> None:
    assert speech_color(0.0) == (46, 14, 0)
    assert speech_color(0.5) == (158, 79, 0)
    assert speech_color(1.0) == (255, 234, 0)


def test_reactive_led_plan_has_contrast_and_no_blue_during_speech() -> None:
    plan, telemetry = build_reactive_led_plan(_sine_pcm([500, 6000, 24000]))
    colors = [tuple(plan[index : index + 3]) for index in range(0, len(plan), 3)]

    assert telemetry["refresh_hz"] == 50
    assert telemetry["frame_count"] == 120
    assert telemetry["speech_blue_channel"] == 0
    assert all(color[2] == 0 for color in colors)
    assert min(color[0] for color in colors) <= 50
    assert max(color[0] for color in colors) >= 250
    assert max(color[1] for color in colors) >= 220
    assert telemetry["end_animation"] == {
        "to_dark_blue_ms": 250,
        "dark_to_bright_blue_ms": 500,
        "bright_blue_hold_ms": 500,
        "dark_blue_rgb": [0, 0, 40],
        "bright_blue_rgb": [0, 0, 255],
    }


def test_reactive_led_plan_rejects_non_mono_pcm16() -> None:
    with pytest.raises(ValueError, match="mono PCM16"):
        build_reactive_led_plan(b"\x00\x00", channels=2)


def test_subprocess_speaker_passes_ephemeral_led_plan(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}

    def fake_run(cmd: list[str], config: AudioBridgeConfig):
        plan_index = cmd.index("--led-plan") + 1
        plan_path = Path(cmd[plan_index])
        captured["cmd"] = list(cmd)
        captured["plan"] = plan_path.read_bytes()
        return subprocess.CompletedProcess(cmd, 0, stdout="ok", stderr=""), None

    monkeypatch.setattr(SubprocessSpeakerClient, "_run", staticmethod(fake_run))
    speaker = SubprocessSpeakerClient()
    config = AudioBridgeConfig(
        enabled=True,
        speaker_runner=str(tmp_path / "runner"),
        speaker_reactive_led=True,
    )

    result = speaker.play_pcm(_sine_pcm([1000], frames_per_amplitude=2), config, {})

    assert result["ok"] is True
    assert result["reactive_led"]["enabled"] is True
    assert "--led-refresh-hz" in captured["cmd"]
    assert "--led-end-to-dark-ms" in captured["cmd"]
    assert captured["plan"]
    assert all(captured["plan"][index] == 0 for index in range(2, len(captured["plan"]), 3))
