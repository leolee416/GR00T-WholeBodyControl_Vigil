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
        "--audio-advertise-always",
        "--audio-mic-interface-ip",
        "192.168.123.164",
        "--audio-speaker-runner",
        "/tmp/g1_speaker_runner",
        "--audio-speaker-iface",
        "enP8p1s0",
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


def test_policy_script_preserves_release_policy_default() -> None:
    args = _parse_start_args()

    script = launcher._policy_script(args, Path("/tmp/policy.log"))

    assert "--checkpoint policy/release/model" in script
    assert (
        "--obs-config policy/release/observation_config.yaml"
        in script
    )


def test_policy_script_can_opt_in_to_facee_v73_noheight_models() -> None:
    args = _parse_start_args(
        "--policy-checkpoint",
        "policy/facee_v73_noheight/model",
        "--policy-observation-config",
        "policy/facee_v73_noheight/observation_config.yaml",
    )

    script = launcher._policy_script(args, Path("/tmp/policy.log"))

    assert "--checkpoint policy/facee_v73_noheight/model" in script
    assert (
        "--obs-config policy/facee_v73_noheight/observation_config.yaml"
        in script
    )


def test_audio_options_require_explicit_audio_enable(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    args = _parse_start_args("--audio-ws")
    deploy_dir = tmp_path / "deploy"
    deploy_dir.mkdir()
    tensorrt_root = tmp_path / "TensorRT"
    (tensorrt_root / "include").mkdir(parents=True)
    (tensorrt_root / "include" / "NvInfer.h").write_text("", encoding="utf-8")
    (tensorrt_root / "lib").mkdir()
    policy_dir = deploy_dir / "policy" / "release"
    policy_dir.mkdir(parents=True)
    (policy_dir / "model_encoder.onnx").write_bytes(b"test")
    (policy_dir / "model_decoder.onnx").write_bytes(b"test")
    (policy_dir / "observation_config.yaml").write_text("", encoding="utf-8")
    args.tensorrt_root = str(tensorrt_root)
    monkeypatch.setattr(launcher, "DEPLOY_DIR", deploy_dir)

    with pytest.raises(SystemExit, match="audio options require"):
        launcher._validate_start_inputs(args)


def test_audio_disabled_does_not_forward_audio_options() -> None:
    args = _parse_start_args()

    assert launcher._audio_bridge_args(args) == []
