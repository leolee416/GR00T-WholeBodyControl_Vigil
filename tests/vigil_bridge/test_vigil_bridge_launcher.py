from __future__ import annotations

from pathlib import Path

import pytest

import gear_sonic.vigil_bridge.launcher as launcher


def _parse_start_args(*args: str):
    parser = launcher._build_parser()
    return parser.parse_args(["start", *args])


def test_start_forwards_audio_and_tts_options_to_bridge_command() -> None:
    args = _parse_start_args(
        "--with-tts",
        "--audio-advertise-always",
        "--audio-ws",
        "--audio-ws-host",
        "0.0.0.0",
        "--audio-ws-port",
        "8766",
        "--audio-mic-interface-ip",
        "192.168.123.164",
        "--audio-speaker-iface",
        "enP8p1s0",
        "--audio-speaker-runner",
        "/tmp/g1_speaker_runner",
        "--audio-speaker-reactive-led",
        "--audio-speaker-volume",
        "88",
        "--audio-speaker-peak-target",
        "27800",
    )

    bridge_args = launcher._audio_bridge_args(args)

    assert bridge_args == [
        "--audio-enabled",
        "--audio-mic-group",
        "239.168.123.161",
        "--audio-mic-port",
        "5555",
        "--audio-segment-max-s",
        "10.0",
        "--audio-speaker-volume",
        "88",
        "--audio-speaker-peak-target",
        "27800",
        "--audio-speaker-stream-chunk-ms",
        "200",
        "--audio-speaker-stream-prebuffer-ms",
        "400",
        "--audio-speaker-stream-send-lead-ms",
        "20",
        "--audio-speaker-stream-queue-s",
        "3.0",
        "--audio-speaker-stream-drain-ms",
        "150",
        "--audio-advertise-always",
        "--audio-mic-interface-ip",
        "192.168.123.164",
        "--audio-speaker-runner",
        "/tmp/g1_speaker_runner",
        "--audio-speaker-iface",
        "enP8p1s0",
        "--audio-speaker-reactive-led",
        "--audio-ws",
        "--audio-ws-port",
        "8766",
        "--audio-ws-host",
        "0.0.0.0",
    ]

    script = launcher._bridge_script(args, Path("/tmp/policy.log"), Path("/tmp/bridge.log"))

    assert "--audio-enabled" in script
    assert "--audio-ws" in script
    assert "--audio-speaker-runner /tmp/g1_speaker_runner" in script


def test_audio_options_require_explicit_audio_enable(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    args = _parse_start_args("--audio-ws")
    deploy_dir = tmp_path / "deploy"
    deploy_dir.mkdir()
    tensorrt_root = tmp_path / "TensorRT"
    (tensorrt_root / "include").mkdir(parents=True)
    (tensorrt_root / "include" / "NvInfer.h").write_text("", encoding="utf-8")
    (tensorrt_root / "lib").mkdir()
    args.tensorrt_root = str(tensorrt_root)
    monkeypatch.setattr(launcher, "DEPLOY_DIR", deploy_dir)

    with pytest.raises(SystemExit, match="audio options require"):
        launcher._validate_start_inputs(args)


def test_audio_disabled_does_not_forward_audio_options() -> None:
    args = _parse_start_args()

    assert launcher._audio_bridge_args(args) == []


def test_policy_script_forwards_chair_policy_assets() -> None:
    args = _parse_start_args(
        "--checkpoint",
        "policy/sit_chair/model",
        "--obs-config",
        "policy/sit_chair/observation_config.yaml",
        "--motion-data",
        "reference/sit_chair",
    )

    script = launcher._policy_script(args, Path("/tmp/policy.log"))

    assert "--checkpoint policy/sit_chair/model" in script
    assert "--obs-config policy/sit_chair/observation_config.yaml" in script
    assert "--motion-data reference/sit_chair" in script


def test_policy_script_forwards_explicit_chair_gate_override() -> None:
    args = _parse_start_args("--disable-chair-v12-preposition-gate")

    script = launcher._policy_script(args, Path("/tmp/policy.log"))

    assert "env CHAIR_V12_DISABLE_PREPOSITION_GATE=1 ./deploy.sh" in script


def test_reactive_led_requires_speaker_runner(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    args = _parse_start_args("--with-audio", "--audio-speaker-reactive-led")
    deploy_dir = tmp_path / "deploy"
    deploy_dir.mkdir()
    tensorrt_root = tmp_path / "TensorRT"
    (tensorrt_root / "include").mkdir(parents=True)
    (tensorrt_root / "include" / "NvInfer.h").write_text("", encoding="utf-8")
    (tensorrt_root / "lib").mkdir()
    args.tensorrt_root = str(tensorrt_root)
    monkeypatch.setattr(launcher, "DEPLOY_DIR", deploy_dir)

    with pytest.raises(SystemExit, match="requires --audio-speaker-runner"):
        launcher._validate_start_inputs(args)
