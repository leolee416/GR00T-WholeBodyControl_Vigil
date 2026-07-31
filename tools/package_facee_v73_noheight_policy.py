#!/usr/bin/env python3
"""Validate and package the FaceE v73 no-height-map ONNX policy."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import onnx
import onnxruntime as ort
import yaml


CHECKPOINT_REL = Path(
    "out/faceE_sonic_v1_1_noheight_v65/"
    "dagger_v69iter100_balanced_freezeenc_fixedlr2e6_30iter_v73/"
    "model_step_000025.pt"
)
CHECKPOINT_SHA256 = (
    "610bc1bd21e30ca9dd9690a5096ddf3f54ca7b3fe7fb3ddf3ed73c65ead049c9"
)
EXPECTED = {
    "encoder": {
        "input_name": "obs_dict",
        "input_shape": [1, 1751],
        "output_name": "encoded_tokens",
        "output_shape": [1, 64],
    },
    "decoder": {
        "input_name": "obs_dict",
        "input_shape": [1, 994],
        "output_name": "action",
        "output_shape": [1, 29],
    },
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _shape(value: ort.NodeArg) -> list[int | str | None]:
    return [item if isinstance(item, (int, str)) else None for item in value.shape]


def _inspect(path: Path, kind: str) -> dict[str, object]:
    model = onnx.load(path, load_external_data=False)
    onnx.checker.check_model(model)
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1 or len(outputs) != 1:
        raise ValueError(f"{kind} must have exactly one input and output")
    actual = {
        "input_name": inputs[0].name,
        "input_shape": _shape(inputs[0]),
        "output_name": outputs[0].name,
        "output_shape": _shape(outputs[0]),
    }
    if actual != EXPECTED[kind]:
        raise ValueError(f"{kind} interface mismatch: {actual} != {EXPECTED[kind]}")
    return {
        **actual,
        "opset": max(item.version for item in model.opset_import),
        "node_count": len(model.graph.node),
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--grail-root",
        type=Path,
        default=Path("/workspace/fangs1@xiaopeng.com/workspace_fs/GRAIL"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            Path(__file__).resolve().parents[1]
            / "gear_sonic_deploy/policy/facee_v73_noheight"
        ),
    )
    args = parser.parse_args()

    checkpoint = args.grail_root / CHECKPOINT_REL
    if _sha256(checkpoint) != CHECKPOINT_SHA256:
        raise ValueError(f"checkpoint checksum mismatch: {checkpoint}")
    exported = checkpoint.parent / "exported"
    source_paths = {
        kind: exported / f"model_step_000025_{kind}.onnx"
        for kind in ("encoder", "decoder")
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)

    models: dict[str, dict[str, object]] = {}
    parity: dict[str, dict[str, object]] = {}
    for kind, source in source_paths.items():
        source_info = _inspect(source, kind)
        target = args.output_dir / f"model_{kind}.onnx"
        shutil.copy2(source, target)
        target_info = _inspect(target, kind)
        if source_info["sha256"] != target_info["sha256"]:
            raise ValueError(f"{kind} changed while packaging")
        models[kind] = target_info

        parity_source = source.with_suffix(".parity.json")
        if not parity_source.is_file():
            raise FileNotFoundError(f"missing PyTorch/ONNX parity: {parity_source}")
        parity_result = json.loads(parity_source.read_text(encoding="utf-8"))
        if not parity_result.get("allclose", False):
            raise ValueError(f"{kind} parity failed: {parity_result}")
        parity[kind] = parity_result
        shutil.copy2(
            parity_source,
            args.output_dir / f"model_{kind}.parity.json",
        )

    observation_config = args.output_dir / "observation_config.yaml"
    observation_data = yaml.safe_load(
        observation_config.read_text(encoding="utf-8")
    )
    configured_observation_names = {
        item["name"]
        for item in observation_data["observations"]
        + observation_data["encoder"]["encoder_observations"]
    }
    if "height_map_z_flat" in configured_observation_names:
        raise ValueError("deployment observation config unexpectedly contains height map")

    manifest = {
        "schema_version": 1,
        "name": "facee_v73_noheight_task_accept",
        "source_checkpoint": CHECKPOINT_REL.as_posix(),
        "source_checkpoint_sha256": CHECKPOINT_SHA256,
        "observation_contract": (
            "reference_motion_plus_proprioception_no_height_map_no_chair_pose"
        ),
        "actor_inputs": {
            "reference_motion": True,
            "proprioception": True,
            "height_map_z_flat": False,
            "chair_relative_position": False,
            "chair_relative_orientation": False,
        },
        "models": models,
        "pytorch_onnx_parity": parity,
        "motion_catalog": (
            "gear_sonic/vigil_bridge/data/facee_chair_13s_v2/manifest.json"
        ),
        "acceptance": {
            "strict_tracking_successes": 16,
            "task_level_acceptances": 2,
            "task_level_acceptance_distances_m": [1.35, 1.4],
            "note": (
                "1.35/1.40 reuse the exact 1.45 robot reference and were accepted "
                "for sitting task completion despite strict anchor_ori_full termination."
            ),
        },
        "deployment_status": {
            "real_robot_authorized": False,
            "reason": (
                "ONNX export parity and Isaac task acceptance do not authorize "
                "hardware. Independent MuJoCo screening must pass before opt-in use."
            ),
        },
    }
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
