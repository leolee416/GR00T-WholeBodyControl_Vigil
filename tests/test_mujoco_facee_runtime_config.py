from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig


def test_facee_validation_runtime_overrides_legacy_support_and_reset() -> None:
    config = SimLoopConfig(
        interface="lo",
        enable_elastic_band=False,
        auto_reset_on_fall=False,
        wait_for_low_cmd_before_step=True,
        wait_for_policy_control_before_step=True,
        facee_initialize_deploy_standing_pose=True,
        facee_initialize_reference_pose=False,
        facee_preserve_authored_world_frame=True,
        facee_policy_lockstep_decimation=4,
        facee_initial_ground_clearance_m=0.001,
    ).load_wbc_yaml()

    assert config["ENABLE_ELASTIC_BAND"] is False
    assert config["AUTO_RESET_ON_FALL"] is False
    assert config["WAIT_FOR_LOW_CMD_BEFORE_STEP"] is True
    assert config["WAIT_FOR_POLICY_CONTROL_BEFORE_STEP"] is True
    assert config["FACEE_INITIALIZE_DEPLOY_STANDING_POSE"] is True
    assert config["FACEE_INITIALIZE_REFERENCE_POSE"] is False
    assert config["FACEE_PRESERVE_AUTHORED_WORLD_FRAME"] is True
    assert config["FACEE_POLICY_LOCKSTEP_DECIMATION"] == 4
    assert config["FACEE_INITIAL_GROUND_CLEARANCE_M"] == 0.001


def test_no_hands_zeroes_legacy_hand_channel_counts() -> None:
    config = SimLoopConfig(with_hands=False).load_wbc_yaml()
    assert config["with_hands"] is False
    assert config["NUM_HAND_JOINTS"] == 0
    assert config["NUM_HAND_MOTORS"] == 0
    assert len(config["motor_effort_limit_list"]) == 29
    assert config["motor_effort_limit_list"][:6] == [
        139.0, 139.0, 88.0, 139.0, 50.0, 50.0
    ]
