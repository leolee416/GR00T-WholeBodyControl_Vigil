#!/usr/bin/env python3
"""Validate and package the E0017 iteration-8 actor used by E0019 exact-v3."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import onnx
import onnxruntime as ort
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT_SHA256 = (
    "3727209b0340fc31c499d1e069386c7e10297fce80adb4f87e9db10e840e7650"
)
TASKSET_SHA256 = (
    "35eed93b181ad0c4e1a4fc498f2cd05f99d99ce3e28c2a65ccd9e68819ddba1d"
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


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def shape(value: ort.NodeArg) -> list[int | str | None]:
    return [item if isinstance(item, (int, str)) else None for item in value.shape]


def inspect_onnx(path: Path, kind: str) -> dict[str, object]:
    model = onnx.load(path, load_external_data=False)
    onnx.checker.check_model(model)
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1 or len(outputs) != 1:
        raise ValueError(f"{kind} must have exactly one input and output")
    actual = {
        "input_name": inputs[0].name,
        "input_shape": shape(inputs[0]),
        "output_name": outputs[0].name,
        "output_shape": shape(outputs[0]),
    }
    if actual != EXPECTED[kind]:
        raise ValueError(f"{kind} interface mismatch: {actual} != {EXPECTED[kind]}")
    return {
        **actual,
        "opset": max(item.version for item in model.opset_import),
        "node_count": len(model.graph.node),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--taskset", type=Path, required=True)
    parser.add_argument(
        "--observation-config",
        type=Path,
        default=(
            REPO_ROOT
            / "gear_sonic_deploy/policy/facee_v73_noheight/observation_config.yaml"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            REPO_ROOT / "gear_sonic_deploy/policy/facee_e0019_exact_v3"
        ),
    )
    args = parser.parse_args()

    checkpoint = args.checkpoint.resolve()
    taskset = args.taskset.resolve()
    observation_source = args.observation_config.resolve()
    output = args.output_dir.resolve()
    if sha256(checkpoint) != CHECKPOINT_SHA256:
        raise ValueError("E0017 iteration-8 checkpoint checksum mismatch")
    if sha256(taskset) != TASKSET_SHA256:
        raise ValueError("E0019 exact-v3 task-set checksum mismatch")

    exported = checkpoint.parent / "exported"
    source_models = {
        kind: exported / f"model_step_000020_{kind}.onnx"
        for kind in ("encoder", "decoder")
    }
    output.mkdir(parents=True, exist_ok=True)
    models: dict[str, dict[str, object]] = {}
    parity: dict[str, dict[str, object]] = {}
    for kind, source in source_models.items():
        source_info = inspect_onnx(source, kind)
        target = output / f"model_{kind}.onnx"
        shutil.copy2(source, target)
        target_info = inspect_onnx(target, kind)
        if target_info["sha256"] != source_info["sha256"]:
            raise ValueError(f"{kind} changed while packaging")
        models[kind] = target_info

        parity_source = source.with_suffix(".parity.json")
        parity_result = json.loads(parity_source.read_text(encoding="utf-8"))
        if not parity_result.get("allclose", False):
            raise ValueError(f"{kind} PyTorch/ONNX parity failed")
        parity[kind] = parity_result
        shutil.copy2(parity_source, output / f"model_{kind}.parity.json")

    observation_data = yaml.safe_load(observation_source.read_text(encoding="utf-8"))
    configured_names = {
        item["name"]
        for item in observation_data["observations"]
        + observation_data["encoder"]["encoder_observations"]
    }
    forbidden = {
        "height_map_z_flat",
        "chair_relative_position",
        "chair_relative_orientation",
        "reference_expert_selector",
        "distance_selector",
    }
    if configured_names & forbidden:
        raise ValueError(
            f"deployment observation config contains forbidden inputs: "
            f"{sorted(configured_names & forbidden)}"
        )
    observation_target = output / "observation_config.yaml"
    shutil.copy2(observation_source, observation_target)

    manifest = {
        "schema_version": 1,
        "name": "faceE_e0019_exact_v3_crosssim_18of18",
        "source_checkpoint": str(checkpoint),
        "source_checkpoint_sha256": sha256(checkpoint),
        "source_taskset": str(taskset),
        "source_taskset_sha256": sha256(taskset),
        "observation_contract": (
            "reference_motion_plus_proprioception_no_height_map_no_chair_pose"
        ),
        "actor_inputs": {
            "reference_motion": True,
            "proprioception": True,
            "height_map_z_flat": False,
            "vision": False,
            "chair_relative_position": False,
            "chair_relative_orientation": False,
            "distance_or_reference_selector": False,
        },
        "architecture": {
            "actor_count": 1,
            "encoder_count": 1,
            "decoder_count": 1,
            "distance_expert_selector": False,
            "distance_conditioned_routing": False,
            "shared_actor_for_all_18_references": True,
            "network_topology_change_from_v73": False,
            "encoder_mode_4_semantics": (
                "input modality token g1/teleop/smpl; not a distance selector"
            ),
        },
        "startup_history": "repeat_reset",
        "models": models,
        "pytorch_onnx_parity": parity,
        "observation_config": observation_target.name,
        "observation_config_sha256": sha256(observation_target),
        "motion_catalog": (
            "gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json"
        ),
        "mujoco_scene": (
            "gear_sonic/data/assets/robot_description/mjcf/"
            "scene_facee_training_primitives.xml"
        ),
        "simulation_acceptance": {
            "isaac_physx_strict": "18/18",
            "cpu_mujoco_strict": "18/18",
            "cpu_mujoco_no_preseat_lower_leg_kick": "18/18",
            "source": str(taskset.parent / "RESULTS.md"),
        },
        "deployment_status": {
            "default_launcher_changed": False,
            "real_robot_authorized": False,
            "reason": (
                "This package is an explicit simulation candidate. Repository-mode "
                "MuJoCo must be validated separately from the E0019 evaluator."
            ),
        },
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
