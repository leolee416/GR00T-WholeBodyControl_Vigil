from __future__ import annotations

import ast
import json
from pathlib import Path

import numpy as np

from gear_sonic.vigil_bridge.chair_motion_catalog import DEFAULT_CATALOG


REPO_ROOT = Path(__file__).resolve().parents[1]
G1_SOURCE = REPO_ROOT / "gear_sonic/envs/manager_env/robots/g1.py"


def _literal(name: str):
    tree = ast.parse(G1_SOURCE.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name
            for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise KeyError(name)


def test_manifest_mapping_matches_g1_source_and_is_inverse() -> None:
    manifest = json.loads(DEFAULT_CATALOG.read_text(encoding="utf-8"))
    mj_to_il = np.asarray(_literal("G1_MUJOCO_TO_ISAACLAB_DOF"), dtype=np.int64)
    il_to_mj = np.asarray(_literal("G1_ISAACLAB_TO_MUJOCO_DOF"), dtype=np.int64)
    assert manifest["joint_order_mapping_values"] == mj_to_il.tolist()
    assert np.array_equal(il_to_mj[mj_to_il], np.arange(29))


def test_asymmetric_sentinel_crosses_the_order_boundary_once() -> None:
    mj_to_il = np.asarray(_literal("G1_MUJOCO_TO_ISAACLAB_DOF"), dtype=np.int64)
    q_mujoco = np.arange(29, dtype=np.float32) + 1000.0
    q_isaaclab = q_mujoco[mj_to_il]
    assert q_isaaclab.tolist() == [float(1000 + index) for index in mj_to_il]
    assert q_isaaclab[0] == 1000.0
    assert q_isaaclab[1] == 1006.0
    assert q_isaaclab[2] == 1012.0


def test_all_18_v2_assets_are_exact_q_and_dq_reorders_of_v1() -> None:
    """The v2 migration changes semantics, not the authored trajectories."""
    mj_to_il = np.asarray(_literal("G1_MUJOCO_TO_ISAACLAB_DOF"), dtype=np.int64)
    old_dir = DEFAULT_CATALOG.parent.parent / "facee_chair_13s"
    for new_path in sorted(DEFAULT_CATALOG.parent.glob("d*.npz")):
        old_path = old_dir / new_path.name
        assert old_path.is_file(), old_path
        with np.load(old_path, allow_pickle=False) as old, np.load(
            new_path, allow_pickle=False
        ) as new:
            assert np.array_equal(new["joint_pos"], old["joint_pos"][:, mj_to_il])
            assert np.array_equal(new["joint_vel"], old["joint_vel"][:, mj_to_il])
            assert np.array_equal(new["body_quat_w"], old["body_quat_w"])
