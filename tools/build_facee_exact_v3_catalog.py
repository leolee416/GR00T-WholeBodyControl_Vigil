#!/usr/bin/env python3
"""Package the FaceE E0019 exact-v3 task set for ZMQ streamed motion.

The authoritative E0019 evaluator consumes the selected PKL's finalized
``dof`` channels directly, resamples them from the declared source FPS to
50 Hz, and only then converts MuJoCo/MJCF order to IsaacLab order.  This tool
reproduces that contract exactly.  It must not reconstruct DOFs from
``pose_aa``: that produces a visibly different frame-zero state and changes
the closed-loop task even though both arrays live in the same PKL.

The generated catalog is deliberately opt-in.  It is paired with the E0017
iteration-8 actor and must not silently replace the older v73 catalog.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import joblib
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation, Slerp


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT / "gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3"
)
G1_ORDER_SOURCE = REPO_ROOT / "gear_sonic/envs/manager_env/robots/g1.py"
FACEE_CHAIR_MESH = (
    REPO_ROOT
    / "gear_sonic/data/robot_model/model_data/g1/meshes/facee_backless_41cm_proxy.obj"
)
EXPECTED_CHAIR_MESH_SHA256 = (
    "47bf22f5589627e4cb04c43bf7625e9f96a464027344a8b28ab1be11f970b8cd"
)
EXPECTED_TASKSET_SHA256 = (
    "35eed93b181ad0c4e1a4fc498f2cd05f99d99ce3e28c2a65ccd9e68819ddba1d"
)
EXPECTED_ACTOR_SHA256 = (
    "3727209b0340fc31c499d1e069386c7e10297fce80adb4f87e9db10e840e7650"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def literal_assignment(path: Path, name: str):
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


def yaw_from_quat_xyzw(quat: np.ndarray) -> float:
    x, y, z, w = [float(value) for value in quat]
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def chair_layout_in_initial_robot_frame(
    root_xy: np.ndarray,
    root_yaw: float,
    chair_world_xy: np.ndarray,
    chair_world_yaw: float,
) -> dict[str, object]:
    delta = np.asarray(chair_world_xy, dtype=np.float64) - np.asarray(
        root_xy, dtype=np.float64
    )
    cosine = math.cos(root_yaw)
    sine = math.sin(root_yaw)
    world_to_robot = np.asarray([[cosine, sine], [-sine, cosine]])
    center_robot = world_to_robot @ delta
    return {
        "frame": "initial_robot_heading",
        "center_xy_m": center_robot.tolist(),
        "center_distance_m": float(np.linalg.norm(center_robot)),
        "yaw_rad": wrap_pi(float(chair_world_yaw) - root_yaw),
        "source_root_world_xy_m": np.asarray(root_xy, dtype=np.float64).tolist(),
        "source_root_world_yaw_rad": root_yaw,
        "source_chair_world_xy_m": np.asarray(
            chair_world_xy, dtype=np.float64
        ).tolist(),
        "source_chair_world_yaw_rad": float(chair_world_yaw),
    }


def resample_e0019_motion(
    source: dict[str, object], target_hz: float = 50.0
) -> dict[str, np.ndarray]:
    """Mirror ``load_resampled_motion`` in the authoritative CPU evaluator."""

    joint_source = np.asarray(source["dof"], dtype=np.float64)
    root_source = np.asarray(source["root_trans_offset"], dtype=np.float64)
    quat_xyzw_source = np.asarray(source["root_rot"], dtype=np.float64)
    source_hz = float(source["fps"])
    if joint_source.ndim != 2 or joint_source.shape[1] != 29:
        raise ValueError(f"expected source dof shape [N,29], got {joint_source.shape}")
    if root_source.shape != (len(joint_source), 3):
        raise ValueError("root_trans_offset does not match dof frames")
    if quat_xyzw_source.shape != (len(joint_source), 4):
        raise ValueError("root_rot does not match dof frames")

    source_duration = (len(joint_source) - 1) / source_hz
    source_time = np.arange(len(joint_source), dtype=np.float64) / source_hz
    # SONIC excludes the source endpoint.  The E0019 crops are authored so this
    # produces exactly 650 policy frames at 50 Hz.
    target_time = np.arange(
        0.0, source_duration, 1.0 / target_hz, dtype=np.float64
    )
    joint_pos_mujoco = np.column_stack(
        [
            np.interp(target_time, source_time, joint_source[:, index])
            for index in range(29)
        ]
    )
    root_pos = np.column_stack(
        [
            np.interp(target_time, source_time, root_source[:, index])
            for index in range(3)
        ]
    )
    rotations = Rotation.from_quat(quat_xyzw_source)
    quat_xyzw = Slerp(source_time, rotations)(target_time).as_quat()
    root_quat_wxyz = quat_xyzw[:, [3, 0, 1, 2]]

    joint_vel_mujoco = np.empty_like(joint_pos_mujoco)
    joint_vel_mujoco[:-1] = np.diff(joint_pos_mujoco, axis=0) * target_hz
    joint_vel_mujoco[-1] = joint_vel_mujoco[-3]
    root_lin_vel_w = gaussian_filter1d(
        np.gradient(root_pos, 1.0 / target_hz, axis=0),
        2,
        axis=0,
        mode="nearest",
    )
    target_rotations = Rotation.from_quat(quat_xyzw)
    angular_step = (
        target_rotations[1:] * target_rotations[:-1].inv()
    ).as_rotvec() * target_hz
    root_ang_vel_w = gaussian_filter1d(
        np.vstack([angular_step, np.zeros((1, 3), dtype=np.float64)]),
        2,
        axis=0,
        mode="nearest",
    )
    return {
        "joint_pos_mujoco": joint_pos_mujoco.astype(np.float32),
        "joint_vel_mujoco": joint_vel_mujoco.astype(np.float32),
        # Keep the authored floating-base reset in float64.  E0019 starts with
        # several foot collision primitives slightly penetrating the floor;
        # rounding these four arrays to float32 changes the initial contact set
        # (23 instead of 28 contacts for d1p50) and immediately selects a
        # different MuJoCo solver branch.  The streamed reference remains f32
        # below, matching protocol v1 and the actor's observation tensors.
        "root_pos": root_pos,
        "root_quat_wxyz": root_quat_wxyz,
        "root_lin_vel_w": root_lin_vel_w,
        "root_ang_vel_w": root_ang_vel_w,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--grail-root", type=Path, required=True)
    parser.add_argument("--taskset", type=Path, required=True)
    parser.add_argument("--actor-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    grail_root = args.grail_root.resolve()
    taskset_path = args.taskset.resolve()
    actor_checkpoint = args.actor_checkpoint.resolve()
    if sha256(taskset_path) != EXPECTED_TASKSET_SHA256:
        raise ValueError("E0019 exact-v3 task-set checksum mismatch")
    if sha256(actor_checkpoint) != EXPECTED_ACTOR_SHA256:
        raise ValueError("E0017 iteration-8 actor checkpoint checksum mismatch")
    if sha256(FACEE_CHAIR_MESH) != EXPECTED_CHAIR_MESH_SHA256:
        raise ValueError("E0019 backless chair collision mesh checksum mismatch")

    taskset = json.loads(taskset_path.read_text(encoding="utf-8"))
    tasks = taskset.get("tasks")
    if taskset.get("count") != 18 or not isinstance(tasks, list) or len(tasks) != 18:
        raise ValueError("E0019 exact-v3 task set must contain 18 tasks")
    expected_tags = {f"d{distance / 100:.2f}".replace(".", "p") for distance in range(115, 201, 5)}
    if {str(task.get("tag")) for task in tasks} != expected_tags:
        raise ValueError("E0019 exact-v3 task tags must be d1p15..d2p00")

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

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, object]] = []

    for task in sorted(tasks, key=lambda item: float(item["distance_m"])):
        tag = str(task["tag"])
        source_path = Path(task["motion"]).resolve()
        task_manifest_path = Path(task["manifest"]).resolve()
        motion_key = str(task["motion_key"])
        task_manifest = json.loads(task_manifest_path.read_text(encoding="utf-8"))
        if task_manifest.get("backrest") is not False:
            raise ValueError(f"{tag}: exact-v3 chair must be backless")
        dimensions = task_manifest.get("dimensions_m", {})
        if dimensions != {"width": 0.5, "depth": 0.45, "seat_height": 0.41}:
            raise ValueError(f"{tag}: unexpected exact-v3 chair dimensions")
        rows = task_manifest.get("records")
        if not isinstance(rows, list) or len(rows) != 1:
            raise ValueError(f"{tag}: task manifest must contain one record")
        task_row = rows[0]
        if str(task_row.get("clip")) != motion_key:
            raise ValueError(f"{tag}: motion key/manifest mismatch")
        if sha256(source_path) != str(task_row.get("robot_pkl_sha256")):
            raise ValueError(f"{tag}: source robot PKL checksum mismatch")

        source_bundle = joblib.load(source_path)
        if list(source_bundle) != [motion_key]:
            raise ValueError(f"{tag}: source PKL must contain exactly {motion_key}")
        source = source_bundle[motion_key]
        sampled = resample_e0019_motion(source)
        joint_pos_mujoco = sampled["joint_pos_mujoco"]
        joint_vel_mujoco = sampled["joint_vel_mujoco"]
        joint_pos = joint_pos_mujoco[:, mujoco_to_isaaclab]
        joint_vel = joint_vel_mujoco[:, mujoco_to_isaaclab]
        root_quat_wxyz = sampled["root_quat_wxyz"]
        root_pos = sampled["root_pos"]
        root_lin_vel_w = sampled["root_lin_vel_w"]
        root_ang_vel_w = sampled["root_ang_vel_w"]
        if joint_pos.shape != (650, 29) or joint_vel.shape != (650, 29):
            raise ValueError(f"{tag}: unexpected 50 Hz shape {joint_pos.shape}")
        if root_quat_wxyz.shape != (650, 4):
            raise ValueError(f"{tag}: unexpected root quaternion shape")

        source_root_xy = np.asarray(source["root_trans_offset"][0, :2])
        source_root_yaw = yaw_from_quat_xyzw(
            np.asarray(source["root_rot"][0], dtype=np.float64)
        )
        layout = chair_layout_in_initial_robot_frame(
            source_root_xy,
            source_root_yaw,
            np.asarray(task_row["stool_center_xy"], dtype=np.float64),
            float(task_row["stool_yaw_rad"]),
        )
        layout.update(
            {
                "nominal_distance_m": float(task["distance_m"]),
                "seat_height_m": 0.41,
                "width_m": 0.5,
                "depth_m": 0.45,
                "backrest": False,
            }
        )

        motion_file = output / f"{tag}.npz"
        np.savez_compressed(
            motion_file,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            joint_pos_mujoco=joint_pos_mujoco,
            joint_vel_mujoco=joint_vel_mujoco,
            body_quat_w=root_quat_wxyz.astype(np.float32),
            root_pos=root_pos,
            reset_root_quat_w=root_quat_wxyz,
            root_lin_vel_w=root_lin_vel_w,
            root_ang_vel_w=root_ang_vel_w,
            frame_index=np.arange(650, dtype=np.int64),
            joint_order=np.asarray("isaaclab"),
            source_joint_order=np.asarray("mujoco"),
            protocol_version=np.asarray(1, dtype=np.int32),
        )
        records.append(
            {
                "tag": tag,
                "distance_m": float(task["distance_m"]),
                "reference_distance_m": float(task["distance_m"]),
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
                "source_task_manifest": str(task_manifest_path),
                "source_task_manifest_sha256": sha256(task_manifest_path),
                "provenance": task.get("provenance"),
                "chair_layout": layout,
                "crosssim_acceptance": {
                    "isaac_physx_strict": True,
                    "cpu_mujoco_strict": True,
                    "cpu_mujoco_no_preseat_lower_leg_kick": True,
                },
            }
        )

    manifest = {
        "schema_version": 2,
        "name": "faceE_e0019_exact_v3_crosssim_18of18",
        "protocol_version": 1,
        "source_joint_order": "mujoco",
        "joint_order": "isaaclab",
        "joint_count": 29,
        "joint_order_mapping": "G1_MUJOCO_TO_ISAACLAB_DOF",
        "joint_order_mapping_values": mujoco_to_isaaclab.tolist(),
        "joint_order_mapping_source": str(G1_ORDER_SOURCE.relative_to(REPO_ROOT)),
        "joint_order_mapping_source_sha256": sha256(G1_ORDER_SOURCE),
        "source_taskset": str(taskset_path),
        "source_taskset_sha256": sha256(taskset_path),
        "policy_checkpoint": str(actor_checkpoint),
        "policy_checkpoint_sha256": sha256(actor_checkpoint),
        "policy_observation_contract": (
            "reference_motion_plus_proprioception_no_height_map_no_chair_pose"
        ),
        "startup_history": "repeat_reset",
        "selection_policy": (
            "ceil measured distance to the next available 0.05 m reference; "
            "measured range [1.10, 2.00] m"
        ),
        "construction": (
            "E0019 exact-v3 selected PKL finalized dof channels; authoritative "
            "evaluator-equivalent interpolation to 50 Hz; "
            "MuJoCo-to-IsaacLab joint reorder exactly once; 650 frames/13 seconds."
        ),
        "chair_geometry": {
            "backrest": False,
            "width_m": 0.5,
            "depth_m": 0.45,
            "seat_height_m": 0.41,
            "collision_semantics": "E0019 audited rounded backless seat proxy",
            "mesh": str(FACEE_CHAIR_MESH.relative_to(REPO_ROOT)),
            "mesh_sha256": sha256(FACEE_CHAIR_MESH),
            "vertices": 258,
            "triangles": 512,
        },
        "count": len(records),
        "motions": records,
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
