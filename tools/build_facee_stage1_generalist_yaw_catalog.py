#!/usr/bin/env python3
"""Build Stage-1 Sit-only distance/yaw deployment motion catalogs.

``clean6`` packages only the six two-simulator-clean hardware-preflight
conditions. ``full403`` packages every Sit condition from 0.90 to 2.40 m and
-30 to +30 degrees for explicit, non-clean opt-in testing. Stand motions are
never packaged. Each generated reference uses the selected robot PKL's
finalized ``dof`` stream, resampled to 50 Hz and reordered exactly once.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import joblib
import numpy as np

from build_facee_exact_v3_catalog import (
    G1_ORDER_SOURCE,
    literal_assignment,
    resample_e0019_motion,
    sha256,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CLEAN_OUTPUT = REPO_ROOT / "gear_sonic/vigil_bridge/data/facee_stage1_generalist_yaw_clean6"
DEFAULT_FULL_OUTPUT = REPO_ROOT / "gear_sonic/vigil_bridge/data/facee_stage1_generalist_yaw_full403"
EXPECTED_CHECKPOINT_SHA256 = "dc543f99fb51207727799d9c939cbd6b58e83af354947ddc843dd9ffa913607d"
EXPECTED_SOURCE_MANIFEST_SHA256 = "a6fd1df8569933ebb0db8c97b30ce3e0d9a2f247e653e5da8b9bb52785149954"
EXPECTED_ISAAC_RESULTS_SHA256 = "b743230ccbe602835b32ea2ecb8f6eb5ba80d13b0a26219f3480ff0dc930224b"
EXPECTED_MUJOCO_RESULTS_SHA256 = "1e88b9c918e75e1c34442e1cbe5e3265dd06656fd70912103420874e37adb8b3"
EXPECTED_CLEAN_TAGS = tuple(f"d1p45_sit_yaw_p{yaw:02d}" for yaw in range(0, 30, 5))
EXPECTED_DISTANCE_GRID = tuple(round(0.90 + 0.05 * index, 2) for index in range(31))
EXPECTED_YAW_GRID = tuple(range(-30, 31, 5))


def _records(payload: dict[str, object], path: Path) -> list[dict[str, object]]:
    for name in ("rows", "records"):
        value = payload.get(name)
        if isinstance(value, list):
            return value
    raise ValueError(f"{path}: missing rows/records list")


def _indexed(payload: dict[str, object], path: Path) -> dict[str, dict[str, object]]:
    rows = _records(payload, path)
    result = {str(row["tag"]): row for row in rows}
    if len(result) != len(rows):
        raise ValueError(f"{path}: duplicate tags")
    return result


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-manifest", type=Path, required=True)
    parser.add_argument("--isaac-results", type=Path, required=True)
    parser.add_argument("--mujoco-results", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--scope", choices=("clean6", "full403"), default="clean6")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    source_manifest = args.source_manifest.resolve()
    isaac_results = args.isaac_results.resolve()
    mujoco_results = args.mujoco_results.resolve()
    checkpoint = args.checkpoint.resolve()
    expected_hashes = {
        source_manifest: EXPECTED_SOURCE_MANIFEST_SHA256,
        isaac_results: EXPECTED_ISAAC_RESULTS_SHA256,
        mujoco_results: EXPECTED_MUJOCO_RESULTS_SHA256,
        checkpoint: EXPECTED_CHECKPOINT_SHA256,
    }
    for path, expected in expected_hashes.items():
        if sha256(path) != expected:
            raise ValueError(f"checksum mismatch: {path}")

    source_payload = json.loads(source_manifest.read_text(encoding="utf-8"))
    isaac_payload = json.loads(isaac_results.read_text(encoding="utf-8"))
    mujoco_payload = json.loads(mujoco_results.read_text(encoding="utf-8"))
    source_by_tag = _indexed(source_payload, source_manifest)
    isaac_by_tag = _indexed(isaac_payload, isaac_results)
    mujoco_by_tag = _indexed(mujoco_payload, mujoco_results)

    clean_tags = tuple(
        sorted(
            tag
            for tag, record in mujoco_by_tag.items()
            if bool(record.get("clean_pass")) and bool(isaac_by_tag.get(tag, {}).get("strict_ok"))
        )
    )
    if clean_tags != EXPECTED_CLEAN_TAGS:
        raise ValueError(f"expected exactly the six audited clean tags, got {clean_tags}")
    sit_rows = [row for row in source_by_tag.values() if str(row.get("mode")) == "sit"]
    sit_rows.sort(key=lambda row: (float(row["distance_m"]), int(row["chair_yaw_delta_deg"])))
    sit_tags = tuple(str(row["tag"]) for row in sit_rows)
    distance_grid = tuple(sorted({float(row["distance_m"]) for row in sit_rows}))
    yaw_grid = tuple(sorted({int(row["chair_yaw_delta_deg"]) for row in sit_rows}))
    if (
        len(sit_tags) != 403
        or distance_grid != EXPECTED_DISTANCE_GRID
        or yaw_grid != EXPECTED_YAW_GRID
    ):
        raise ValueError("source manifest must contain the complete 31x13 Sit grid")
    selected_tags = EXPECTED_CLEAN_TAGS if args.scope == "clean6" else sit_tags

    mujoco_to_isaaclab = np.asarray(
        literal_assignment(G1_ORDER_SOURCE, "G1_MUJOCO_TO_ISAACLAB_DOF"),
        dtype=np.int64,
    )
    if mujoco_to_isaaclab.shape != (29,) or not np.array_equal(
        np.sort(mujoco_to_isaaclab), np.arange(29)
    ):
        raise ValueError("invalid MuJoCo-to-IsaacLab mapping")

    default_output = DEFAULT_CLEAN_OUTPUT if args.scope == "clean6" else DEFAULT_FULL_OUTPUT
    output = (args.output or default_output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    motions: list[dict[str, object]] = []
    for tag in selected_tags:
        source_row = source_by_tag[tag]
        isaac_row = isaac_by_tag[tag]
        mujoco_row = mujoco_by_tag[tag]
        if source_row.get("mode") != "sit":
            raise ValueError(f"{tag}: source mode is not sit")
        distance_m = float(source_row["distance_m"])
        yaw_deg = int(source_row["chair_yaw_delta_deg"])

        source_path = Path(str(source_row["motion"])).resolve()
        if sha256(source_path) != str(source_row["robot_sha256"]):
            raise ValueError(f"{tag}: source robot PKL checksum mismatch")
        motion_key = str(source_row["motion_key"])
        source_bundle = joblib.load(source_path)
        if list(source_bundle) != [motion_key]:
            raise ValueError(f"{tag}: source PKL must contain exactly {motion_key}")
        sampled = resample_e0019_motion(source_bundle[motion_key])
        joint_pos_mujoco = sampled["joint_pos_mujoco"]
        joint_vel_mujoco = sampled["joint_vel_mujoco"]
        joint_pos = joint_pos_mujoco[:, mujoco_to_isaaclab]
        joint_vel = joint_vel_mujoco[:, mujoco_to_isaaclab]
        if joint_pos.shape != (650, 29):
            raise ValueError(f"{tag}: expected 650x29 deploy reference")

        motion_file = output / f"{tag}.npz"
        np.savez_compressed(
            motion_file,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            joint_pos_mujoco=joint_pos_mujoco,
            joint_vel_mujoco=joint_vel_mujoco,
            body_quat_w=sampled["root_quat_wxyz"].astype(np.float32),
            root_pos=sampled["root_pos"],
            reset_root_quat_w=sampled["root_quat_wxyz"],
            root_lin_vel_w=sampled["root_lin_vel_w"],
            root_ang_vel_w=sampled["root_ang_vel_w"],
            frame_index=np.arange(650, dtype=np.int64),
            joint_order=np.asarray("isaaclab"),
            source_joint_order=np.asarray("mujoco"),
            protocol_version=np.asarray(1, dtype=np.int32),
        )
        motions.append(
            {
                "tag": tag,
                "distance_m": distance_m,
                "reference_distance_m": distance_m,
                "chair_yaw_deg": yaw_deg,
                "reference_yaw_deg": yaw_deg,
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
                "source_motion": str(source_path),
                "source_motion_sha256": sha256(source_path),
                "source_motion_key": motion_key,
                "source_asset_status": source_row.get("asset_status"),
                "source_construction": source_row.get("construction"),
                "crosssim_acceptance": {
                    "isaac_physx_strict": bool(isaac_row["strict_ok"]),
                    "isaac_mpjpe_mm": float(isaac_row["mpjpe_mm"]),
                    "direct_mujoco_action_completed": bool(mujoco_row["action_completed"]),
                    "direct_mujoco_clean": bool(mujoco_row["clean_pass"]),
                    "direct_mujoco_lower_leg_chair_peak_n": float(
                        mujoco_row["lower_leg_chair_peak_n"]
                    ),
                    "direct_mujoco_final_torso_tilt_deg": float(mujoco_row["final_torso_tilt_deg"]),
                },
            }
        )

    is_clean_scope = args.scope == "clean6"
    selection_contract = {
        "method": "nearest_available_reference",
        "tie_break": "lower_distance_then_lower_yaw",
        "measured_distance_range_m": ([1.425, 1.475] if is_clean_scope else [0.875, 2.425]),
        "measured_yaw_range_deg": ([-2.5, 27.5] if is_clean_scope else [-32.5, 32.5]),
        "available_distance_m": ([1.45] if is_clean_scope else list(EXPECTED_DISTANCE_GRID)),
        "available_yaw_deg": (list(range(0, 30, 5)) if is_clean_scope else list(EXPECTED_YAW_GRID)),
        "chair_yaw_deg_required": True,
        "selector_source": "external_measurement_or_vision",
        "require_explicit_non_clean_opt_in": not is_clean_scope,
    }
    manifest = {
        "schema_version": 3,
        "name": (
            "facee_stage1_generalist_yaw_clean6"
            if is_clean_scope
            else "facee_stage1_generalist_yaw_full403_sit_only"
        ),
        "protocol_version": 1,
        "source_joint_order": "mujoco",
        "joint_order": "isaaclab",
        "joint_count": 29,
        "joint_order_mapping": "G1_MUJOCO_TO_ISAACLAB_DOF",
        "joint_order_mapping_values": mujoco_to_isaaclab.tolist(),
        "joint_order_mapping_source": str(G1_ORDER_SOURCE.relative_to(REPO_ROOT)),
        "joint_order_mapping_source_sha256": sha256(G1_ORDER_SOURCE),
        "policy_checkpoint": str(checkpoint),
        "policy_checkpoint_sha256": sha256(checkpoint),
        "source_manifest": str(source_manifest),
        "source_manifest_sha256": sha256(source_manifest),
        "isaac_results": str(isaac_results),
        "isaac_results_sha256": sha256(isaac_results),
        "mujoco_results": str(mujoco_results),
        "mujoco_results_sha256": sha256(mujoco_results),
        "policy_observation_contract": (
            "reference_motion_plus_proprioception_no_height_map_no_chair_pose"
        ),
        "startup_history": "repeat_reset",
        "selection_contract": selection_contract,
        "construction": (
            f"Stage-1 {args.scope} Sit robot PKLs; finalized dof channels; "
            "authoritative 50 Hz interpolation; MuJoCo-to-IsaacLab reorder once"
        ),
        "deployment_status": {
            "real_robot_authorized": False,
            "scope": (
                "hardware preflight candidate only"
                if is_clean_scope
                else "complete Sit simulation catalog; explicit non-clean opt-in"
            ),
            "reason": (
                "simulation evidence does not authorize hardware; only six of "
                "403 conditions pass the two-simulator clean gate"
            ),
        },
        "mode": "sit",
        "stand_included": False,
        "count": len(motions),
        "motions": motions,
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(manifest_path)


if __name__ == "__main__":
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    main()
