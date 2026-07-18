import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONFIG = (
    ROOT
    / "gear_sonic_deploy"
    / "policy"
    / "upper_body_fixed_safe_v12.json"
)
PARAMETERS = (
    ROOT
    / "gear_sonic_deploy"
    / "src"
    / "g1"
    / "g1_deploy_onnx_ref"
    / "include"
    / "policy_parameters.hpp"
)
DEPLOY_SOURCE = (
    ROOT
    / "gear_sonic_deploy"
    / "src"
    / "g1"
    / "g1_deploy_onnx_ref"
    / "src"
    / "g1_deploy_onnx_ref.cpp"
)


def test_released_v12_config_values():
    config = json.loads(CONFIG.read_text(encoding="utf-8"))

    assert config["name"] == "upper_body_fixed_safe_v12"
    assert config["application_layer"]["policy_observations"] == "unchanged"
    assert config["application_layer"]["reference_motion"] == "unchanged"
    assert config["reset_preposition"]["required"] is True
    assert config["fixed_pose_targets_rad"] == {
        "left_shoulder_pitch_joint": 0.2,
        "left_shoulder_roll_joint": 0.65,
        "left_shoulder_yaw_joint": 0.0,
        "left_elbow_joint": 0.9,
        "left_wrist_roll_joint": 0.0,
        "left_wrist_pitch_joint": 0.0,
        "left_wrist_yaw_joint": 0.0,
        "right_shoulder_pitch_joint": 0.2,
        "right_shoulder_roll_joint": -0.65,
        "right_shoulder_yaw_joint": 0.0,
        "right_elbow_joint": 0.9,
        "right_wrist_roll_joint": 0.0,
        "right_wrist_pitch_joint": 0.0,
        "right_wrist_yaw_joint": 0.0,
    }
    assert config["presit_tuck"] == {
        "targets_rad": {
            "right_shoulder_roll_joint": -0.85,
            "right_shoulder_pitch_joint": 0.45,
        },
        "window_s_from_motion_start": [4.3, 5.0],
        "blend_ramp_s": 0.2,
        "blend_shape": "trapezoid",
    }


def test_cpp_deploy_applies_v12_after_policy_and_freezes_preposition():
    parameters = PARAMETERS.read_text(encoding="utf-8")
    source = DEPLOY_SOURCE.read_text(encoding="utf-8")

    assert "chair_v12_fixed_arm_targets_rad" in parameters
    assert "chair_v12_motion_id = 4" in parameters
    assert "0.2, 0.65, 0.0, 0.9" in parameters
    assert "0.2, -0.65, 0.0, 0.9" in parameters
    assert "chair_v12_tuck_start_s = 4.3" in parameters
    assert "chair_v12_tuck_end_s = 5.0" in parameters
    assert "chair_v12_tuck_ramp_s = 0.2" in parameters
    assert "chair_v12_tuck_right_shoulder_pitch_rad = 0.45" in parameters
    assert "chair_v12_tuck_right_shoulder_roll_rad = -0.85" in parameters

    function_start = source.index("bool CreatePolicyCommand(")
    function_end = source.index(
        "bool CurrentFrameAdvancement()", function_start
    )
    function_body = source[function_start:function_end]
    infer_position = function_body.index("policy_engine_->Infer()")
    override_position = function_body.index(
        "chair_v12_fixed_arm_targets_rad[arm_slot]"
    )
    command_position = function_body.index("motor_command_buffer_.SetData")
    assert infer_position < override_position < command_position
    assert "motion->GetMotionId() == chair_v12_motion_id" in function_body

    advancement_start = source.index(
        "bool CurrentFrameAdvancement()", function_end
    )
    advancement_body = source[advancement_start:]
    preposition_guard = advancement_body.index(
        "chair_v12_phase_ == ChairV12Phase::PREPOSITION"
    )
    frame_freeze = advancement_body.index(
        "return true;", preposition_guard
    )
    normal_advancement = advancement_body.index(
        "int new_frame = current_frame_ + 1;", frame_freeze
    )
    assert preposition_guard < frame_freeze < normal_advancement
