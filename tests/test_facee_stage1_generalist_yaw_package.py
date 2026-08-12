from __future__ import annotations

import json
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
POLICY_DIR = REPO_ROOT / "gear_sonic_deploy/policy/facee_stage1_generalist_yaw"


def test_stage1_generalist_uses_same_deploy_structure_as_real_robot_models() -> None:
    manifest = json.loads((POLICY_DIR / "manifest.json").read_text(encoding="utf-8"))
    comparison = manifest["structure_comparison_to_real_robot_models"]
    assert comparison["references"] == {
        "facee_e0019_exact_v3_last_real_robot": {
            "encoder_sha256": "1cf476669348344f9ccda0049fb5b708112bdcada73e9403f227321872e7e459",
            "decoder_sha256": "5464bd07f7877b9c42d38cb2a9fc73ad601b857941604e8d3e372298e67d7382",
        },
        "facee_v73_noheight_real_bridge_history": {
            "encoder_sha256": "33c88827b13d0270a1e7f036b882284e6a2429e37f9bdb21b60c706d916ca849",
            "decoder_sha256": "1bea1e29e8f243cc312e15fe0858c751d48e97ccbaa45ef419e6e4db210c14f7",
        },
    }
    for key in (
        "same_input_output_names_and_shapes",
        "same_opset",
        "same_node_counts",
        "same_operator_histograms",
        "same_initializer_shape_sequences",
    ):
        assert comparison[key] is True
    assert comparison["same_weight_values"] is False
    assert comparison["conclusion"] == (
        "deployment architecture compatible; model parameters differ"
    )
    assert manifest["models"]["encoder"]["input_shape"] == [1, 1751]
    assert manifest["models"]["encoder"]["output_shape"] == [1, 64]
    assert manifest["models"]["decoder"]["input_shape"] == [1, 994]
    assert manifest["models"]["decoder"]["output_shape"] == [1, 29]
    assert manifest["models"]["encoder"]["node_count"] == 241
    assert manifest["models"]["decoder"]["node_count"] == 48


def test_stage1_generalist_package_is_explicit_opt_in_and_oss_backed() -> None:
    manifest = json.loads((POLICY_DIR / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["deployment_status"]["real_robot_authorized"] is False
    assert manifest["distribution"] == {
        "oss_prefix": "oss://xrobot-data/fs/r2s_ego_exp/GR00T-WholeBodyControl_Vigil/policy/facee_stage1_generalist_yaw/",
        "checkpoint_uploaded": True,
        "onnx_pair_uploaded": True,
        "motion_catalog_uploaded": True,
    }
    assert manifest["packaged_preflight_scope"] == {
        "distance_m": 1.45,
        "chair_yaw_deg": [0, 5, 10, 15, 20, 25],
        "condition_count": 6,
        "isaac_strict": 6,
        "direct_mujoco_clean": 6,
    }
    assert manifest["source_checkpoint_sha256"] == (
        "dc543f99fb51207727799d9c939cbd6b58e83af354947ddc843dd9ffa913607d"
    )
    full_catalog = manifest["motion_catalogs"]["full_sit_403"]
    assert full_catalog["count"] == 403
    assert full_catalog["distance_range_m"] == [0.9, 2.4]
    assert full_catalog["yaw_range_deg"] == [-30, 30]
    assert full_catalog["mode"] == "sit"
    assert full_catalog["stand_included"] is False
    assert full_catalog["non_clean_reference_requires_explicit_opt_in"] is True

    fetch_script = (REPO_ROOT / "tools/fetch_facee_stage1_generalist_yaw_assets.sh").read_text(
        encoding="utf-8"
    )
    assert "--with-full-catalog" in fetch_script
    assert "full catalog must contain exactly 403 Sit motions" in fetch_script
    assert "unexpectedly contains Stand motion" in fetch_script


def test_stage1_observation_layout_and_cpp_mode_token_match_export() -> None:
    config = yaml.safe_load((POLICY_DIR / "observation_config.yaml").read_text(encoding="utf-8"))
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
    names = [item["name"] for item in config["encoder"]["encoder_observations"] if item["enabled"]]
    assert sum(dimensions[name] for name in names) == 1751
    source = (
        REPO_ROOT / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp"
    ).read_text(encoding="utf-8")
    assert "target_buffer[offset + 1 + static_cast<size_t>(mode)] = 1.0;" in source
    assert "For G1 mode 0 this slot must be [0, 1, 0, 0]" in source
