#!/usr/bin/env python3
"""Build the real-bridge FaceE chair-motion catalog for no-height v73.

This is an offline packaging tool.  Run it with the restored GRAIL Sonic
environment; the real-robot bridge itself only needs NumPy and the generated
manifest/NPZ files.  Most motions come from the v64 selection.  User-reviewed
cross-distance experiments deliberately replace d1.55 with the d1.70 robot
reference and d1.65 with the d1.80 robot reference.  Raw strict failures remain
recorded separately from task-level acceptance.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import sys

import joblib
import numpy as np
from omegaconf import OmegaConf
import torch


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPO_ROOT / "gear_sonic/vigil_bridge/data/facee_chair_13s_v2"
G1_ORDER_SOURCE = REPO_ROOT / "gear_sonic/envs/manager_env/robots/g1.py"
DEPLOYMENT_DECISION = (
    "out/faceE_sonic_v1_1_noheight_v65/"
    "deployment_candidate_v73_task_accept_20260730.json"
)
REFERENCE_SELECTION_DECISION = (
    "out/faceE_sonic_v1_1_noheight_v65/"
    "deployment_candidate_v73_crossdistance_reference_selection_20260731.json"
)
TASK_ACCEPT_TAGS = {"d1p35", "d1p40"}
REFERENCE_REPLACEMENT_TAGS = {"d1p55", "d1p65"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def literal_assignment(path: Path, name: str):
    """Read a literal constant without importing the Isaac-Lab-heavy module."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            if any(
                isinstance(target, ast.Name) and target.id == name
                for target in node.targets
            ):
                return ast.literal_eval(node.value)
        elif isinstance(node, ast.AnnAssign):
            if isinstance(node.target, ast.Name) and node.target.id == name:
                return ast.literal_eval(node.value)
    raise KeyError(f"{name} not found in {path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--grail-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    grail_root = args.grail_root.resolve()
    sonic_root = grail_root / "imports/SONIC"
    sys.path.insert(0, str(sonic_root))
    from gear_sonic.utils.motion_lib.torch_humanoid_batch import Humanoid_Batch

    selection_path = grail_root / "out/faceE_all_success_hold13_selected_v64/results.json"
    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    deployment_path = grail_root / DEPLOYMENT_DECISION
    deployment = json.loads(deployment_path.read_text(encoding="utf-8"))
    replacement_path = grail_root / REFERENCE_SELECTION_DECISION
    replacement = json.loads(replacement_path.read_text(encoding="utf-8"))
    checkpoint_path = Path(deployment["checkpoint"]).resolve()
    if not checkpoint_path.is_relative_to(grail_root):
        raise ValueError("deployment checkpoint must be inside --grail-root")
    if sha256(checkpoint_path) != deployment["checkpoint_sha256"]:
        raise ValueError("deployment checkpoint checksum mismatch")
    if replacement["checkpoint_sha256"] != deployment["checkpoint_sha256"]:
        raise ValueError("replacement decision checkpoint does not match deployment")
    if (
        replacement["actor_observation_contract"]
        != deployment["actor_observation_contract"]
    ):
        raise ValueError("replacement decision observation contract mismatch")
    evidence_results = (
        Path(replacement["evidence_directory"]).resolve() / "results.json"
    )
    if sha256(evidence_results) != replacement["evidence_results_sha256"]:
        raise ValueError("replacement decision evidence checksum mismatch")
    deployment_rows = {
        f"d{float(row['distance_m']):.2f}".replace(".", "p"): row
        for row in deployment["rows"]
    }
    if set(deployment_rows) != TASK_ACCEPT_TAGS:
        raise ValueError(
            f"deployment decision must override {sorted(TASK_ACCEPT_TAGS)}, "
            f"got {sorted(deployment_rows)}"
        )
    replacement_rows = {
        f"d{float(row['distance_m']):.2f}".replace(".", "p"): row
        for row in replacement["rows"]
    }
    if set(replacement_rows) != REFERENCE_REPLACEMENT_TAGS:
        raise ValueError(
            "replacement decision must override "
            f"{sorted(REFERENCE_REPLACEMENT_TAGS)}, "
            f"got {sorted(replacement_rows)}"
        )
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    mujoco_to_isaaclab = np.asarray(
        literal_assignment(G1_ORDER_SOURCE, "G1_MUJOCO_TO_ISAACLAB_DOF"),
        dtype=np.int64,
    )
    isaaclab_to_mujoco = np.asarray(
        literal_assignment(G1_ORDER_SOURCE, "G1_ISAACLAB_TO_MUJOCO_DOF"),
        dtype=np.int64,
    )
    if (
        mujoco_to_isaaclab.shape != (29,)
        or isaaclab_to_mujoco.shape != (29,)
        or not np.array_equal(
            isaaclab_to_mujoco[mujoco_to_isaaclab], np.arange(29)
        )
    ):
        raise ValueError("G1 MuJoCo/IsaacLab DOF mappings are invalid")

    cfg = OmegaConf.create(
        {
            "asset": {
                "assetRoot": str(
                    sonic_root / "gear_sonic/data/assets/robot_description/mjcf"
                ),
                "assetFileName": "g1_29dof_rev_1_0.xml",
            },
            "extend_config": [],
        }
    )
    humanoid = Humanoid_Batch(cfg, torch.device("cpu"))
    records = []
    generated: dict[str, dict[str, np.ndarray]] = {}
    for item in selection["results"]:
        tag = item["tag"]
        override_row = replacement_rows.get(tag) or deployment_rows.get(tag)
        if override_row is None:
            source_path = (
                grail_root
                / item["motion_library"]
                / "robot"
                / f"{item['motion_key']}.pkl"
            )
            motion_key = item["motion_key"]
            source_motion_library = item["motion_library"]
            task_level_status = "STRICT_TRACKING_OK"
            strict_tracking_status = "OK"
            strict_tracking_progress = 1.0
            strict_tracking_reason = ""
            reuses_reference_tag = None
            reference_distance_m = float(item["distance_m"])
            supersedes_motion_key = None
            mujoco_task_level_status = None
            mujoco_strict_status = None
            mujoco_strict_reason = None
        else:
            source_path = Path(override_row["robot_reference"]).resolve()
            if not source_path.is_relative_to(grail_root):
                raise ValueError(f"{tag}: robot reference must be inside --grail-root")
            if sha256(source_path) != override_row["robot_reference_sha256"]:
                raise ValueError(f"{tag}: selected robot reference checksum mismatch")
            motion_key = override_row["motion_key"]
            source_motion_library = str(source_path.parent.parent.relative_to(grail_root))
            task_level_status = override_row["task_level_status"]
            strict_tracking_status = override_row["strict_tracking_status"]
            strict_tracking_progress = float(override_row["strict_progress"])
            strict_tracking_reason = override_row["strict_reason"]
            if tag in replacement_rows:
                reference_distance_m = float(override_row["reference_distance_m"])
                reuses_reference_tag = (
                    f"d{reference_distance_m:.2f}".replace(".", "p")
                )
            else:
                reference_distance_m = 1.45
                reuses_reference_tag = "d1p45"
            supersedes_motion_key = override_row.get("supersedes_motion_key")
            mujoco_task_level_status = override_row.get("mujoco_task_level_status")
            mujoco_strict_status = override_row.get("mujoco_strict_status")
            mujoco_strict_reason = override_row.get("mujoco_strict_reason")

        source = joblib.load(source_path)[motion_key]
        pose = torch.as_tensor(source["pose_aa"]).float().unsqueeze(0)
        trans = torch.as_tensor(source["root_trans_offset"]).float().unsqueeze(0)
        fk = humanoid.fk_batch(
            pose,
            trans,
            return_full=True,
            fps=float(source["fps"]),
            target_fps=50,
            interpolate_data=True,
        )
        joint_pos_mujoco = fk.dof_pos[0].cpu().numpy().astype(np.float32)
        joint_vel_mujoco = fk.dof_vels[0].cpu().numpy().astype(np.float32)
        # ZMQ streamed-motion protocol v1 consumes all 29 motion-reference
        # joints in IsaacLab order.  Humanoid_Batch follows the source
        # PKL/MJCF order, so the deployment asset must cross this boundary
        # exactly once before it is serialized.
        joint_pos = joint_pos_mujoco[:, mujoco_to_isaaclab]
        joint_vel = joint_vel_mujoco[:, mujoco_to_isaaclab]
        root_quat_xyzw = fk.global_rotation[0, :, 0].cpu().numpy()
        root_quat_wxyz = root_quat_xyzw[:, [3, 0, 1, 2]].astype(np.float32)
        if joint_pos.shape != (650, 29) or joint_vel.shape != (650, 29):
            raise ValueError(f"{item['tag']}: unexpected 50 Hz shape {joint_pos.shape}")
        if root_quat_wxyz.shape != (650, 4):
            raise ValueError(f"{item['tag']}: unexpected root quaternion shape")

        motion_file = output / f"{tag}.npz"
        np.savez_compressed(
            motion_file,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            body_quat_w=root_quat_wxyz,
            frame_index=np.arange(650, dtype=np.int64),
            joint_order=np.asarray("isaaclab"),
            source_joint_order=np.asarray("mujoco"),
            protocol_version=np.asarray(1, dtype=np.int32),
        )
        generated[tag] = {
            "joint_pos": joint_pos,
            "joint_vel": joint_vel,
            "body_quat_w": root_quat_wxyz,
        }
        records.append(
            {
                "tag": tag,
                "distance_m": float(item["distance_m"]),
                "reference_distance_m": reference_distance_m,
                "motion_name": motion_key,
                "file": motion_file.name,
                "sha256": sha256(motion_file),
                "fps": 50,
                "frames": 650,
                "duration_s": 13.0,
                "encode_mode": 0,
                "protocol_version": 1,
                "source_joint_order": "mujoco",
                "joint_order": "isaaclab",
                "joint_order_mapping": "G1_MUJOCO_TO_ISAACLAB_DOF",
                "source_motion_library": source_motion_library,
                "source_motion_key": motion_key,
                "source_robot_pkl_sha256": sha256(source_path),
                "task_level_status": task_level_status,
                "strict_tracking_status": strict_tracking_status,
                "strict_tracking_progress": strict_tracking_progress,
                "strict_tracking_reason": strict_tracking_reason,
                "reuses_reference_tag": reuses_reference_tag,
                "supersedes_motion_key": supersedes_motion_key,
                "mujoco_task_level_status": mujoco_task_level_status,
                "mujoco_strict_status": mujoco_strict_status,
                "mujoco_strict_reason": mujoco_strict_reason,
            }
        )

    for tag in sorted(TASK_ACCEPT_TAGS):
        for field in ("joint_pos", "joint_vel", "body_quat_w"):
            if not np.array_equal(generated[tag][field], generated["d1p45"][field]):
                raise ValueError(f"{tag}: {field} must exactly reuse d1p45")
    for tag, donor_tag in (("d1p55", "d1p70"), ("d1p65", "d1p80")):
        for field in ("joint_pos", "joint_vel", "body_quat_w"):
            if not np.array_equal(generated[tag][field], generated[donor_tag][field]):
                raise ValueError(f"{tag}: {field} must exactly reuse {donor_tag}")

    manifest = {
        "schema_version": 2,
        "name": "faceE_v73_noheight_user_selected_real_bridge_v2",
        "protocol_version": 1,
        "source_joint_order": "mujoco",
        "joint_order": "isaaclab",
        "joint_count": 29,
        "joint_order_mapping": "G1_MUJOCO_TO_ISAACLAB_DOF",
        "joint_order_mapping_values": mujoco_to_isaaclab.tolist(),
        "joint_order_mapping_source": str(G1_ORDER_SOURCE.relative_to(REPO_ROOT)),
        "joint_order_mapping_source_sha256": sha256(G1_ORDER_SOURCE),
        "source_results": str(selection_path.relative_to(grail_root)),
        "source_results_sha256": sha256(selection_path),
        "deployment_decision": str(deployment_path.relative_to(grail_root)),
        "deployment_decision_sha256": sha256(deployment_path),
        "reference_selection_decision": str(
            replacement_path.relative_to(grail_root)
        ),
        "reference_selection_decision_sha256": sha256(replacement_path),
        "policy_checkpoint": str(checkpoint_path.relative_to(grail_root)),
        "policy_checkpoint_sha256": deployment["checkpoint_sha256"],
        "policy_observation_contract": deployment["actor_observation_contract"],
        "strict_tracking_successes": int(
            replacement["strict_tracking_successes_after_replacement"]
        ),
        "task_level_acceptances": int(
            replacement["task_level_acceptances_after_replacement"]
        ),
        "construction": (
            "Gear-SONIC Humanoid_Batch FK with the source FPS and target_fps=50; "
            "FK joint positions/velocities are converted exactly once from MuJoCo "
            "to IsaacLab order before serialization for ZMQ streamed-motion "
            "protocol v1. d1.35/d1.40 reuse the exact d1.45 "
            "robot reference. d1.55 reuses d1.70 and d1.65 reuses d1.80 after "
            "user review of independent Isaac/MuJoCo cross-distance rollouts. "
            "Task-level acceptance does not overwrite raw strict failures."
        ),
        "selection_policy": (
            "ceil measured distance to the next available 0.05 m reference; "
            "measured range [1.10, 2.00] m"
        ),
        "count": len(records),
        "motions": records,
    }
    if len(records) != 18:
        raise ValueError(f"expected 18 motions, got {len(records)}")
    manifest_path = output / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(manifest_path)


if __name__ == "__main__":
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    main()
