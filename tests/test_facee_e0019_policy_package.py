from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
POLICY_DIR = REPO_ROOT / "gear_sonic_deploy/policy/facee_e0019_exact_v3"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def test_e0019_policy_package_is_one_shared_actor_without_distance_selector() -> None:
    manifest = json.loads((POLICY_DIR / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["source_checkpoint_sha256"] == (
        "3727209b0340fc31c499d1e069386c7e10297fce80adb4f87e9db10e840e7650"
    )
    assert manifest["source_taskset_sha256"] == (
        "35eed93b181ad0c4e1a4fc498f2cd05f99d99ce3e28c2a65ccd9e68819ddba1d"
    )
    assert manifest["architecture"]["actor_count"] == 1
    assert manifest["architecture"]["distance_expert_selector"] is False
    assert manifest["architecture"]["distance_conditioned_routing"] is False
    assert manifest["actor_inputs"]["distance_or_reference_selector"] is False
    assert manifest["startup_history"] == "repeat_reset"
    assert sorted(path.name for path in POLICY_DIR.glob("*.onnx")) == [
        "model_decoder.onnx",
        "model_encoder.onnx",
    ]


def test_e0019_policy_hashes_and_observation_contract_are_self_consistent() -> None:
    manifest = json.loads((POLICY_DIR / "manifest.json").read_text(encoding="utf-8"))
    for kind in ("encoder", "decoder"):
        model = POLICY_DIR / f"model_{kind}.onnx"
        assert sha256(model) == manifest["models"][kind]["sha256"]
        assert manifest["pytorch_onnx_parity"][kind]["allclose"] is True
    assert manifest["models"]["encoder"]["input_shape"] == [1, 1751]
    assert manifest["models"]["decoder"]["input_shape"] == [1, 994]

    observation_path = POLICY_DIR / "observation_config.yaml"
    assert sha256(observation_path) == manifest["observation_config_sha256"]
    config = yaml.safe_load(observation_path.read_text(encoding="utf-8"))
    names = {
        item["name"]
        for item in config["observations"]
        + config["encoder"]["encoder_observations"]
    }
    assert not names & {
        "height_map_z_flat",
        "chair_relative_position",
        "chair_relative_orientation",
        "reference_expert_selector",
        "distance_selector",
    }
