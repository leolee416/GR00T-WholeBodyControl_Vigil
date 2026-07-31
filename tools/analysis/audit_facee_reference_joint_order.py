#!/usr/bin/env python3
"""Offline audit for the FaceE protocol-v1 reference joint-order mismatch.

This script deliberately does not import Isaac Lab.  It reads the ordering
constants from their source file with ``ast.literal_eval``, loads one packaged
FaceE NPZ, and compares two otherwise-identical ONNX inputs:

* expected: the FK output is reordered from MuJoCo to IsaacLab order;
* current:  the packaged FK output is consumed without that reorder.

The experiment changes only the 10 future position frames and 10 future
velocity frames (580 scalar slots).  Robot state, root orientation, encoder
mode, inactive encoder branches, and decoder proprioception are held fixed.
It is therefore an order-only counterfactual, not a simulator rollout.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CATALOG = REPO_ROOT / "gear_sonic/vigil_bridge/data/facee_chair_13s/manifest.json"
DEFAULT_POLICY_DIR = REPO_ROOT / "gear_sonic_deploy/policy/facee_v73_noheight"
G1_SOURCE = REPO_ROOT / "gear_sonic/envs/manager_env/robots/g1.py"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _literal_assignment(path: Path, name: str) -> Any:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
                return ast.literal_eval(node.value)
        elif isinstance(node, ast.AnnAssign):
            if isinstance(node.target, ast.Name) and node.target.id == name:
                return ast.literal_eval(node.value)
    raise KeyError(f"{name} not found in {path}")


def _joint_name(link_name: str) -> str:
    if link_name == "torso_link":
        return "waist_pitch_joint"
    if not link_name.endswith("_link"):
        raise ValueError(f"unexpected G1 link name: {link_name}")
    return link_name.removesuffix("_link") + "_joint"


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.asarray([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.asarray(
        [
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
        ],
        dtype=np.float64,
    )


def _quat_to_rot6d(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.asarray(
        [
            1 - 2 * (y * y + z * z),
            2 * (x * y - w * z),
            2 * (x * y + w * z),
            1 - 2 * (x * x + z * z),
            2 * (x * z - w * y),
            2 * (y * z + w * x),
        ],
        dtype=np.float32,
    )


def _build_encoder_input(
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    root_quat_wxyz: np.ndarray,
) -> np.ndarray:
    """Build the selected G1 branch of the v1.1 1751-value encoder input."""
    sample_frames = np.arange(0, 50, 5, dtype=np.int64)
    encoded = np.zeros(1751, dtype=np.float32)
    # Mirrors GatherEncoderMode(..., fill_zeros_num=3) in the current C++
    # runtime for encode_mode=0: [0, 0, 0, 0].  This prefix is identical in
    # both branches, so it cannot contribute to the order-only delta.
    encoded[:4] = 0.0
    encoded[4:294] = joint_pos[sample_frames].reshape(-1)
    encoded[294:584] = joint_vel[sample_frames].reshape(-1)

    robot_quat = np.asarray(root_quat_wxyz[0], dtype=np.float64)
    robot_quat /= np.linalg.norm(robot_quat)
    if robot_quat[0] < 0:
        robot_quat = -robot_quat
    for output_frame, source_frame in enumerate(sample_frames):
        reference_quat = np.asarray(root_quat_wxyz[source_frame], dtype=np.float64)
        reference_quat /= np.linalg.norm(reference_quat)
        if reference_quat[0] < 0:
            reference_quat = -reference_quat
        relative = _quat_mul(_quat_conjugate(robot_quat), reference_quat)
        encoded[584 + output_frame * 6 : 584 + (output_frame + 1) * 6] = (
            _quat_to_rot6d(relative)
        )
    return encoded.reshape(1, -1)


def _build_decoder_input(token: np.ndarray) -> np.ndarray:
    """Use one fixed proprioception fixture so only the encoder token differs."""
    decoded = np.zeros((1, 994), dtype=np.float32)
    decoded[0, :64] = token.reshape(-1)
    # Layout after token: angular velocity 30, q 290, dq 290, last action 290,
    # projected gravity 30.  A static upright fixture is sufficient for this
    # counterfactual; both branches receive exactly the same fixture.
    decoded[0, 964:994] = np.tile(np.asarray([0.0, 0.0, -1.0], np.float32), 10)
    return decoded


def _metrics(expected: np.ndarray, current: np.ndarray) -> dict[str, Any]:
    delta = np.asarray(current, dtype=np.float64) - np.asarray(expected, dtype=np.float64)
    return {
        "changed_values_exact": int(np.count_nonzero(delta)),
        "max_abs": float(np.max(np.abs(delta))),
        "l2": float(np.linalg.norm(delta)),
    }


def _git_head() -> str | None:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def audit(tag: str, catalog_path: Path, policy_dir: Path) -> dict[str, Any]:
    try:
        import onnxruntime as ort
    except ImportError as exc:
        raise RuntimeError(
            "onnxruntime is required; use the restored Sonic Python, for example "
            "/opt/conda/envs/sonic/bin/python"
        ) from exc

    manifest = json.loads(catalog_path.read_text(encoding="utf-8"))
    record = next((row for row in manifest["motions"] if row["tag"] == tag), None)
    if record is None:
        available = ", ".join(row["tag"] for row in manifest["motions"])
        raise KeyError(f"tag {tag!r} not in catalog; available: {available}")
    motion_path = catalog_path.parent / record["file"]
    motion_sha256 = _sha256(motion_path)
    if motion_sha256 != record["sha256"]:
        raise ValueError(
            f"catalog checksum mismatch for {motion_path}: "
            f"manifest={record['sha256']} actual={motion_sha256}"
        )
    with np.load(motion_path, allow_pickle=False) as archive:
        joint_pos_mujoco = np.asarray(archive["joint_pos"], dtype=np.float32)
        joint_vel_mujoco = np.asarray(archive["joint_vel"], dtype=np.float32)
        root_quat_wxyz = np.asarray(archive["body_quat_w"], dtype=np.float32)
    if joint_pos_mujoco.shape != (650, 29) or joint_vel_mujoco.shape != (650, 29):
        raise ValueError("expected 650 x 29 FaceE position and velocity arrays")
    if root_quat_wxyz.shape != (650, 4):
        raise ValueError("expected a 650 x 4 root quaternion array")

    isaaclab_links = list(_literal_assignment(G1_SOURCE, "G1_ISAACLAB_JOINTS"))
    isaaclab_names = [_joint_name(name) for name in isaaclab_links[1:]]
    isaaclab_to_mujoco = np.asarray(
        _literal_assignment(G1_SOURCE, "G1_ISAACLAB_TO_MUJOCO_DOF"), dtype=np.int64
    )
    mujoco_to_isaaclab = np.asarray(
        _literal_assignment(G1_SOURCE, "G1_MUJOCO_TO_ISAACLAB_DOF"), dtype=np.int64
    )
    mujoco_names = [isaaclab_names[index] for index in isaaclab_to_mujoco]
    if not np.array_equal(isaaclab_to_mujoco[mujoco_to_isaaclab], np.arange(29)):
        raise ValueError("G1 order maps are not inverses")

    joint_pos_isaaclab = joint_pos_mujoco[:, mujoco_to_isaaclab]
    joint_vel_isaaclab = joint_vel_mujoco[:, mujoco_to_isaaclab]
    expected_encoder_input = _build_encoder_input(
        joint_pos_isaaclab, joint_vel_isaaclab, root_quat_wxyz
    )
    current_encoder_input = _build_encoder_input(
        joint_pos_mujoco, joint_vel_mujoco, root_quat_wxyz
    )

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    encoder_path = policy_dir / "model_encoder.onnx"
    decoder_path = policy_dir / "model_decoder.onnx"
    encoder = ort.InferenceSession(
        str(encoder_path), sess_options=options, providers=["CPUExecutionProvider"]
    )
    decoder = ort.InferenceSession(
        str(decoder_path), sess_options=options, providers=["CPUExecutionProvider"]
    )
    encoder_input_name = encoder.get_inputs()[0].name
    decoder_input_name = decoder.get_inputs()[0].name
    expected_token = encoder.run(None, {encoder_input_name: expected_encoder_input})[0]
    current_token = encoder.run(None, {encoder_input_name: current_encoder_input})[0]
    expected_action = decoder.run(
        None, {decoder_input_name: _build_decoder_input(expected_token)}
    )[0]
    current_action = decoder.run(
        None, {decoder_input_name: _build_decoder_input(current_token)}
    )[0]

    sample_frames = np.arange(0, 50, 5, dtype=np.int64)
    sampled_expected_reference = np.concatenate(
        [
            joint_pos_isaaclab[sample_frames].reshape(-1),
            joint_vel_isaaclab[sample_frames].reshape(-1),
        ]
    )
    sampled_current_reference = np.concatenate(
        [
            joint_pos_mujoco[sample_frames].reshape(-1),
            joint_vel_mujoco[sample_frames].reshape(-1),
        ]
    )
    action_delta = (current_action - expected_action).reshape(-1)
    top_action_indices = np.argsort(np.abs(action_delta))[::-1][:10]

    slot_table = []
    for slot in range(29):
        source_index = int(mujoco_to_isaaclab[slot])
        slot_table.append(
            {
                "encoder_slot": slot,
                "expected_isaaclab_joint": isaaclab_names[slot],
                "correct_source_mujoco_index": source_index,
                "current_unreordered_joint": mujoco_names[slot],
                "same_joint_by_coincidence": bool(source_index == slot),
            }
        )

    first_frame = []
    for slot in range(29):
        source_index = int(mujoco_to_isaaclab[slot])
        first_frame.append(
            {
                "encoder_slot": slot,
                "expected_joint": isaaclab_names[slot],
                "current_joint": mujoco_names[slot],
                "expected_value_rad": float(joint_pos_mujoco[0, source_index]),
                "current_value_rad": float(joint_pos_mujoco[0, slot]),
            }
        )

    mismatched_slots_per_frame = int(
        sum(not row["same_joint_by_coincidence"] for row in slot_table)
    )
    semantic_assignments_changed = int(
        mismatched_slots_per_frame * len(sample_frames) * 2
    )
    token_metrics = _metrics(expected_token, current_token)
    action_metrics = _metrics(expected_action, current_action)
    status = (
        "BUG_CONFIRMED"
        if mismatched_slots_per_frame > 0
        and token_metrics["changed_values_exact"] > 0
        and action_metrics["changed_values_exact"] > 0
        else "INCONCLUSIVE"
    )

    report = {
        "schema_version": 1,
        "audit": "facee_protocol_v1_reference_joint_order",
        "status": status,
        "scope": "order-only ONNX counterfactual; not a physics or hardware rollout",
        "repository": {
            "root": str(REPO_ROOT),
            "git_head": _git_head(),
        },
        "motion": {
            "tag": tag,
            "path": str(motion_path.resolve()),
            "sha256": motion_sha256,
            "frame_count": int(joint_pos_mujoco.shape[0]),
            "fps": int(record["fps"]),
            "sampled_frames": sample_frames.tolist(),
        },
        "policy": {
            "directory": str(policy_dir.resolve()),
            "encoder": str(encoder_path.resolve()),
            "encoder_sha256": _sha256(encoder_path),
            "decoder": str(decoder_path.resolve()),
            "decoder_sha256": _sha256(decoder_path),
        },
        "mapping": {
            "mujoco_to_isaaclab": mujoco_to_isaaclab.tolist(),
            "isaaclab_to_mujoco": isaaclab_to_mujoco.tolist(),
            "unchanged_slots": [
                row["encoder_slot"] for row in slot_table if row["same_joint_by_coincidence"]
            ],
            "mismatched_slots_per_frame": mismatched_slots_per_frame,
            "slot_table": slot_table,
        },
        "first_frame": first_frame,
        "order_only_metrics": {
            "semantic_joint_assignments_changed": semantic_assignments_changed,
            "sampled_reference_580": _metrics(
                sampled_expected_reference, sampled_current_reference
            ),
            "encoder_input_1751": _metrics(expected_encoder_input, current_encoder_input),
            "encoded_token_64": token_metrics,
            "raw_action_29": action_metrics,
        },
        "top_action_deltas": [
            {
                "action_index_isaaclab": int(index),
                "joint": isaaclab_names[index],
                "expected": float(expected_action[0, index]),
                "current_unreordered": float(current_action[0, index]),
                "delta": float(action_delta[index]),
            }
            for index in top_action_indices
        ],
        "controls": {
            "changed": "only 10x29 reference q and 10x29 reference dq ordering",
            "held_fixed": [
                "reference root orientation",
                "G1 encoder selector",
                "inactive teleop and SMPL encoder branches",
                "decoder angular velocity history",
                "decoder joint position history",
                "decoder joint velocity history",
                "decoder previous-action history",
                "decoder gravity history",
            ],
        },
    }
    return report


def _print_human(report: dict[str, Any]) -> None:
    metrics = report["order_only_metrics"]
    print(f"status: {report['status']}")
    print(f"motion: {report['motion']['tag']} ({report['motion']['path']})")
    print(
        "mapping: "
        f"{report['mapping']['mismatched_slots_per_frame']}/29 slots mismatch per frame; "
        f"unchanged slots={report['mapping']['unchanged_slots']}"
    )
    print(
        "sampled semantic assignments changed: "
        f"{metrics['semantic_joint_assignments_changed']}/580"
    )
    print("first three encoder slots at frame 0:")
    for row in report["first_frame"][:3]:
        print(
            f"  [{row['encoder_slot']:02d}] expects {row['expected_joint']:<24} "
            f"{row['expected_value_rad']:+.9f}; receives {row['current_joint']:<24} "
            f"{row['current_value_rad']:+.9f}"
        )
    for name in (
        "sampled_reference_580",
        "encoder_input_1751",
        "encoded_token_64",
        "raw_action_29",
    ):
        row = metrics[name]
        print(
            f"{name}: changed={row['changed_values_exact']}, "
            f"max_abs={row['max_abs']:.9g}, l2={row['l2']:.9g}"
        )
    print("largest raw-action deltas (current - expected):")
    for row in report["top_action_deltas"]:
        print(
            f"  [{row['action_index_isaaclab']:02d}] {row['joint']:<27} "
            f"{row['expected']:+.7f} -> {row['current_unreordered']:+.7f} "
            f"delta={row['delta']:+.7f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag", default="d1p70")
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--policy-dir", type=Path, default=DEFAULT_POLICY_DIR)
    parser.add_argument(
        "--json-output",
        type=Path,
        help="optional path for the complete machine-readable audit report",
    )
    args = parser.parse_args()
    report = audit(args.tag, args.catalog.resolve(), args.policy_dir.resolve())
    _print_human(report)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(f"json: {args.json_output.resolve()}")


if __name__ == "__main__":
    main()
