from __future__ import annotations

import json
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
POLICY_DIR = REPO_ROOT / "gear_sonic_deploy/policy/facee_v73_noheight"


def test_noheight_policy_manifest_has_expected_actor_contract() -> None:
    manifest = json.loads((POLICY_DIR / "manifest.json").read_text(encoding="utf-8"))

    assert manifest["actor_inputs"] == {
        "reference_motion": True,
        "proprioception": True,
        "height_map_z_flat": False,
        "chair_relative_position": False,
        "chair_relative_orientation": False,
    }
    assert manifest["models"]["encoder"]["input_shape"] == [1, 1751]
    assert manifest["models"]["decoder"]["input_shape"] == [1, 994]
    assert manifest["pytorch_onnx_parity"]["encoder"]["allclose"] is True
    assert manifest["pytorch_onnx_parity"]["decoder"]["allclose"] is True


def test_noheight_observation_config_matches_cpp_deploy_registry_layout() -> None:
    config = yaml.safe_load(
        (POLICY_DIR / "observation_config.yaml").read_text(encoding="utf-8")
    )
    dimensions = {
        "encoder_mode_4": 4,
        "motion_joint_positions_10frame_step5": 290,
        "motion_joint_velocities_10frame_step5": 290,
        "motion_anchor_orientation_10frame_step5": 60,
        "motion_joint_positions_lowerbody_10frame_step5": 120,
        "motion_joint_velocities_lowerbody_10frame_step5": 120,
        "vr_3point_local_target": 9,
        "vr_3point_local_orn_target": 12,
        "motion_anchor_orientation": 6,
        "smpl_joints_10frame_step1": 720,
        "smpl_anchor_orientation_10frame_step1": 60,
        "motion_joint_positions_wrists_10frame_step1": 60,
    }
    names = [
        item["name"]
        for item in config["encoder"]["encoder_observations"]
        if item["enabled"]
    ]

    assert sum(dimensions[name] for name in names) == 1751
    assert "height_map_z_flat" not in names
    assert config["encoder"]["encoder_modes"][0]["required_observations"] == [
        "encoder_mode_4",
        "motion_joint_positions_10frame_step5",
        "motion_joint_velocities_10frame_step5",
        "motion_anchor_orientation_10frame_step5",
    ]

    deploy_source = (
        REPO_ROOT
        / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/"
        "g1_deploy_onnx_ref.cpp"
    ).read_text(encoding="utf-8")
    for name in names:
        assert f'{{"{name}"' in deploy_source
