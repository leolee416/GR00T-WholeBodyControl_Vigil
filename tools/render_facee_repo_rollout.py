#!/usr/bin/env python3
"""Render and summarize a recorded repository-mode FaceE MuJoCo rollout.

This does not run policy inference and does not re-simulate physics.  It renders
the measured MuJoCo state captured from ``g1_debug``/``rt/odostate`` so the MP4
is a faithful visualization of an already completed closed-loop rollout.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v2 as imageio
import mujoco
import numpy as np

from gear_sonic.utils.mujoco_sim.facee_chair_scene import (
    compile_facee_scene_model,
    load_facee_chair_layout,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = (
    REPO_ROOT
    / "gear_sonic/data/assets/robot_description/mjcf/scene_facee_training_primitives.xml"
)
DEFAULT_CATALOG = (
    REPO_ROOT
    / "gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json"
)


def tilt_deg(rotation: np.ndarray) -> float:
    return math.degrees(
        math.acos(float(np.clip(rotation.reshape(3, 3)[2, 2], -1.0, 1.0)))
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("rollout", type=Path, help="raw_real_rollout.npz")
    parser.add_argument("--distance-m", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metrics", type=Path)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    layout = load_facee_chair_layout(
        args.distance_m, args.catalog.resolve(), REPO_ROOT
    )
    model = compile_facee_scene_model(
        args.scene.resolve(), layout, preserve_authored_world_frame=True
    )
    data = mujoco.MjData(model)
    joint_ids = model.actuator_trnid[:, 0].astype(np.int64)
    qpos_indices = model.jnt_qposadr[joint_ids]
    joint_names = np.asarray([model.joint(int(index)).name for index in joint_ids])

    with np.load(args.rollout.resolve(), allow_pickle=False) as archive:
        action = archive["capture_phase"] == "action"
        q = np.asarray(archive["body_q"][action], dtype=np.float64)
        root_pos = np.asarray(archive["base_xyz_world"][action], dtype=np.float64)
        root_quat = np.asarray(archive["base_quat_wxyz"][action], dtype=np.float64)
        recorded_joint_names = np.asarray(archive["joint_order"])
    if q.shape != (650, 29) or root_pos.shape != (650, 3):
        raise ValueError(
            f"expected exactly 650 action samples, got q={q.shape}, root={root_pos.shape}"
        )
    if not np.array_equal(recorded_joint_names, joint_names):
        raise ValueError("recorded and MuJoCo joint order do not match")

    torso_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "torso_link"
    )
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = 48.0
    camera.elevation = -17.0
    camera.distance = 3.25
    camera.lookat[:] = [-0.24, 0.35, 0.66]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(
        args.output,
        fps=25,
        codec="libx264",
        output_params=["-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart"],
    )
    torso_tilts = []
    pelvis_tilts = []
    try:
        for index in range(650):
            mujoco.mj_resetData(model, data)
            data.qpos[:3] = root_pos[index]
            data.qpos[3:7] = root_quat[index]
            data.qpos[qpos_indices] = q[index]
            mujoco.mj_forward(model, data)
            pelvis_tilts.append(tilt_deg(data.xmat[model.body("pelvis").id]))
            torso_tilts.append(tilt_deg(data.xmat[torso_id]))
            if index % 2 == 0:
                renderer.update_scene(data, camera=camera)
                writer.append_data(renderer.render())
    finally:
        writer.close()
        renderer.close()

    lower = model.jnt_range[joint_ids, 0]
    upper = model.jnt_range[joint_ids, 1]
    joint_margin = np.minimum(q - lower, upper - q)
    worst_sample, worst_joint = np.unravel_index(
        int(np.argmin(joint_margin)), joint_margin.shape
    )
    final_window = slice(-100, None)
    chair_delta = root_pos[-1, :2] - np.asarray(layout.source_chair_world_xy_m)
    payload = {
        "schema_version": 1,
        "artifact_kind": "measured_closed_loop_state_replay",
        "source_rollout": str(args.rollout.resolve()),
        "video": str(args.output.resolve()),
        "distance_m": args.distance_m,
        "reference_tag": layout.tag,
        "samples_50hz": 650,
        "video_frames_25hz": 325,
        "duration_s": 13.0,
        "final_pelvis_xyz_m": root_pos[-1].tolist(),
        "final_pelvis_to_chair_center_xy_m": chair_delta.tolist(),
        "final_pelvis_to_chair_center_distance_xy_m": float(
            np.linalg.norm(chair_delta)
        ),
        "final_pelvis_tilt_deg": float(pelvis_tilts[-1]),
        "final_torso_tilt_deg": float(torso_tilts[-1]),
        "final_2s_pelvis_height_range_m": [
            float(root_pos[final_window, 2].min()),
            float(root_pos[final_window, 2].max()),
        ],
        "final_2s_max_torso_tilt_deg": float(
            np.max(np.asarray(torso_tilts)[final_window])
        ),
        "minimum_joint_limit_margin_rad": float(joint_margin.min()),
        "minimum_joint_limit_margin_joint": str(joint_names[worst_joint]),
        "minimum_joint_limit_margin_time_s": float(worst_sample / 50.0),
        "contact_force_metrics_available": False,
        "strict_no_kick_claim": False,
        "note": (
            "The MP4 visualizes measured closed-loop MuJoCo state. Contact-force/no-kick "
            "acceptance requires physics-step contact instrumentation and is not inferred "
            "from this kinematic replay."
        ),
    }
    metrics = args.metrics or args.output.with_suffix(".json")
    metrics.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(metrics.resolve())


if __name__ == "__main__":
    main()
