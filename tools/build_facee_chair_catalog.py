#!/usr/bin/env python3
"""Build the real-bridge FaceE chair-motion catalog from GRAIL v64.

This is an offline packaging tool.  Run it with the restored GRAIL Sonic
environment; the real-robot bridge itself only needs NumPy and the generated
manifest/NPZ files.
"""

from __future__ import annotations

import argparse
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
DEFAULT_OUTPUT = REPO_ROOT / "gear_sonic/vigil_bridge/data/facee_chair_13s"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

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
    for item in selection["results"]:
        source_path = (
            grail_root
            / item["motion_library"]
            / "robot"
            / f"{item['motion_key']}.pkl"
        )
        source = joblib.load(source_path)[item["motion_key"]]
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
        joint_pos = fk.dof_pos[0].cpu().numpy().astype(np.float32)
        joint_vel = fk.dof_vels[0].cpu().numpy().astype(np.float32)
        root_quat_xyzw = fk.global_rotation[0, :, 0].cpu().numpy()
        root_quat_wxyz = root_quat_xyzw[:, [3, 0, 1, 2]].astype(np.float32)
        if joint_pos.shape != (650, 29) or joint_vel.shape != (650, 29):
            raise ValueError(f"{item['tag']}: unexpected 50 Hz shape {joint_pos.shape}")
        if root_quat_wxyz.shape != (650, 4):
            raise ValueError(f"{item['tag']}: unexpected root quaternion shape")

        tag = item["tag"]
        motion_file = output / f"{tag}.npz"
        np.savez_compressed(
            motion_file,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            body_quat_w=root_quat_wxyz,
            frame_index=np.arange(650, dtype=np.int64),
        )
        records.append(
            {
                "tag": tag,
                "distance_m": float(item["distance_m"]),
                "motion_name": item["motion_key"],
                "file": motion_file.name,
                "sha256": sha256(motion_file),
                "fps": 50,
                "frames": 650,
                "duration_s": 13.0,
                "encode_mode": 0,
                "source_motion_library": item["motion_library"],
                "source_motion_key": item["motion_key"],
                "source_robot_pkl_sha256": sha256(source_path),
            }
        )

    manifest = {
        "schema_version": 1,
        "name": "faceE_all_success_hold13_selected_v64_real_bridge",
        "source_results": str(selection_path.relative_to(grail_root)),
        "source_results_sha256": sha256(selection_path),
        "construction": (
            "Gear-SONIC Humanoid_Batch FK with the source FPS and target_fps=50; "
            "joint positions/velocities plus root quaternion are serialized for "
            "ZMQ streamed-motion protocol v1."
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
