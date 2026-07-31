from __future__ import annotations

import base64
import json
import time
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from gear_sonic.vigil_bridge.mujoco_adapter import (
    MujocoBridgeConfig,
    MujocoRobotState,
    MujocoRuntimeClient,
)
from gear_sonic.vigil_bridge.primitive_executor import DryRunPrimitiveExecutor
from gear_sonic.vigil_bridge.real_adapter import RealBridgeConfig, RealRuntimeClient
from gear_sonic.vigil_bridge.rollout_recorder import G1_JOINT_ORDER, RolloutRecorder
from gear_sonic.vigil_bridge.service import VigilBridgeService
from gear_sonic.vigil_bridge.transport import BridgeRequestRouter


def _g1_debug_sample(index: int, value: float) -> dict:
    return {
        "index": index,
        "ros_timestamp": 1000.0 + index * 0.02,
        "base_quat": [1.0, 0.0, 0.0, 0.0],
        "base_ang_vel": [0.0, 0.0, value],
        "body_q": [value + joint * 0.001 for joint in range(29)],
        "body_dq": [value for _ in range(29)],
        "body_q_target": [value + 0.1 for _ in range(29)],
        "last_action": [value + 0.2 for _ in range(29)],
        # This known fixed visualization placeholder must always be ignored.
        "base_trans_measured": [0.0, -1.0, 0.793],
    }


def test_rollout_recorder_exports_raw_3dgs_and_reference(tmp_path) -> None:
    recorder = RolloutRecorder(tmp_path, runtime_mode="real", localization_max_age_s=1.0)
    start = recorder.start(
        {
            "session_name": "chair_run",
            "capture_mode": "continuous",
            "checkpoint": "checkpoint.pt",
            "chair_world_pose": {
                "position_xyz": [2.0, 1.0, 0.0],
                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                "frame_id": "map",
                "source": "3dgs_registration",
                "estimated": True,
            },
        }
    )
    recorder.set_motion_context(
        {
            "skill_name": "sonic.sit_chair",
            "motion_name": "sit_060cm",
            "reference_distance_m": 0.6,
            "chair_distance_m": 0.61,
        }
    )

    base_monotonic = time.monotonic()
    for index in range(5):
        recorder.update_localization(
            {
                "base_xyz": [index * 0.01, 0.0, 0.79],
                "source": "vio",
                "frame_id": "map",
                "valid": True,
                "estimated": True,
                "timestamp_s": 2000.0 + index * 0.02,
            }
        )
        assert recorder.record_g1_debug(
            _g1_debug_sample(index, index * 0.01),
            received_monotonic_s=base_monotonic + index * 0.02,
            received_wall_s=3000.0 + index * 0.02,
        )

    result = recorder.stop({"target_fps": 50.0, "smoothing_window": 3})

    assert start["active"] is True
    assert result["ok"] is True
    assert result["sample_count"] == 5
    assert result["spatial_3dgs_replay_ready"] is True
    assert result["chair_relative_replay_ready"] is True

    with np.load(result["raw_real_rollout"], allow_pickle=False) as raw:
        assert raw["body_q"].shape == (5, 29)
        assert raw["body_dq"].shape == (5, 29)
        assert raw["base_xyz_world"].shape == (5, 3)
        assert raw["base_xyz_valid"].all()
        assert raw["joint_order"].tolist() == list(G1_JOINT_ORDER)
        assert raw["motion_name"].tolist() == ["sit_060cm"] * 5
        metadata = json.loads(str(raw["metadata_json"].item()))
        assert metadata["checkpoint"] == "checkpoint.pt"
        assert len(metadata["git_commit"]) == 40
        assert metadata["joint_order"] == list(G1_JOINT_ORDER)
        assert metadata["base_translation_policy"].startswith("external localization only")

    with np.load(result["3dgs_replay"], allow_pickle=False) as replay:
        assert bool(replay["kinematic_replay_ready"]) is True
        assert bool(replay["spatial_replay_ready"]) is True
        assert replay["root_pos_world"][-1].tolist() == pytest.approx([0.04, 0.0, 0.79])
        assert bool(replay["chair_world_pose_valid"]) is True

    with np.load(result["gear_sonic_reference"], allow_pickle=False) as reference:
        assert reference["joint_pos"].shape[1] == 29
        assert reference["joint_vel"].shape == reference["joint_pos"].shape
        assert reference["body_quat_w"].shape[1] == 4
        assert bool(reference["physical_validation_required"]) is True
        assert bool(reference["contact_correction_applied"]) is False


def test_missing_external_localization_stays_invalid_and_nan(tmp_path) -> None:
    recorder = RolloutRecorder(tmp_path, runtime_mode="real")
    recorder.start({"session_name": "no_odom", "capture_mode": "continuous"})
    assert recorder.record_g1_debug(_g1_debug_sample(1, 0.0))

    result = recorder.stop()

    assert result["spatial_3dgs_replay_ready"] is False
    with np.load(result["raw_real_rollout"], allow_pickle=False) as raw:
        assert bool(raw["base_xyz_valid"][0]) is False
        assert np.isnan(raw["base_xyz_world"][0]).all()
        assert raw["base_xyz_source"][0] == "none"


def test_rollout_recorder_exports_camera_frame_with_pose_association(tmp_path) -> None:
    recorder = RolloutRecorder(tmp_path, runtime_mode="real")
    recorder.start({"session_name": "camera", "capture_mode": "continuous"})
    now = time.monotonic()
    assert recorder.record_g1_debug(
        _g1_debug_sample(12, 0.2),
        received_monotonic_s=now,
        received_wall_s=1234.5,
    )
    jpeg = b"\xff\xd8camera-frame\xff\xd9"
    assert recorder.record_camera_payload(
        {
            "images": {"head/front": base64.b64encode(jpeg).decode("ascii")},
            "timestamps": {"head/front": 77.25},
        },
        received_monotonic_s=now,
        received_wall_s=1234.5,
        pose_source_index=12,
    ) == 1
    assert recorder.record_camera_payload(
        {"images": {"head/front": jpeg}},
        received_monotonic_s=now + 0.01,
        pose_source_index=13,
    ) == 0

    result = recorder.stop()

    camera_index = json.loads(Path(result["camera_index"]).read_text())
    assert camera_index["frame_count"] == 1
    frame = camera_index["frames"][0]
    assert frame["pose_source_index"] == 12
    assert frame["camera_timestamp"] == pytest.approx(77.25)
    assert (Path(result["session_dir"]) / frame["path"]).read_bytes() == jpeg


def test_sit_window_keeps_only_three_seconds_before_and_after(tmp_path) -> None:
    recorder = RolloutRecorder(tmp_path, runtime_mode="real")
    base_monotonic = time.monotonic()

    for index in range(5):
        assert not recorder.record_g1_debug(
            _g1_debug_sample(index, float(index)),
            received_monotonic_s=base_monotonic + index,
        )

    recorder.start(
        {
            "session_name": "bounded_sit",
            "capture_mode": "skill_window",
            "capture_skill": "sonic.sit_chair",
            "pre_roll_s": 3.0,
            "post_roll_s": 3.0,
            "auto_export": False,
        }
    )
    recorder.begin_action(
        {"skill_name": "sonic.sit_chair"},
        timestamp_monotonic_s=base_monotonic + 5.0,
    )
    for index in range(5, 8):
        assert recorder.record_g1_debug(
            _g1_debug_sample(index, float(index)),
            received_monotonic_s=base_monotonic + index,
        )
    recorder.finish_action(
        {
            "skill_name": "sonic.sit_chair",
            "action_status": "completed",
            "motion_commanded": True,
        },
        timestamp_monotonic_s=base_monotonic + 7.0,
    )
    for index in range(8, 12):
        recorded = recorder.record_g1_debug(
            _g1_debug_sample(index, float(index)),
            received_monotonic_s=base_monotonic + index,
        )
        assert recorded is (index <= 10)

    result = recorder.stop()

    assert result["recorded_pre_roll_s"] == pytest.approx(3.0)
    assert result["recorded_post_roll_s"] == pytest.approx(3.0)
    with np.load(result["raw_real_rollout"], allow_pickle=False) as raw:
        assert raw["source_index"].tolist() == list(range(2, 11))
        assert raw["capture_phase"].tolist() == (
            ["pre"] * 3 + ["action"] * 3 + ["post"] * 3
        )


def test_sit_window_auto_exports_after_post_roll(tmp_path) -> None:
    recorder = RolloutRecorder(tmp_path, runtime_mode="real")
    recorder.start(
        {
            "session_name": "auto_export",
            "capture_mode": "skill_window",
            "pre_roll_s": 0.05,
            "post_roll_s": 0.05,
            "auto_export": True,
        }
    )
    recorder.record_g1_debug(_g1_debug_sample(0, 0.0))
    recorder.begin_action({"skill_name": "sonic.sit_chair"})
    recorder.record_g1_debug(_g1_debug_sample(1, 1.0))
    recorder.finish_action(
        {
            "skill_name": "sonic.sit_chair",
            "action_status": "completed",
            "motion_commanded": True,
        }
    )
    time.sleep(0.02)
    recorder.record_g1_debug(_g1_debug_sample(2, 2.0))
    time.sleep(0.08)

    status = recorder.status()

    assert status["active"] is False
    assert status["last_export"]["ok"] is True
    assert status["last_export"]["recorded_pre_roll_s"] <= 0.05
    assert status["last_export"]["recorded_post_roll_s"] <= 0.05


def test_reference_dispatch_rebases_pre_roll_window(tmp_path) -> None:
    recorder = RolloutRecorder(tmp_path, runtime_mode="real")
    base_monotonic = time.monotonic()
    for index in range(6):
        recorder.record_g1_debug(
            _g1_debug_sample(index, float(index)),
            received_monotonic_s=base_monotonic + index,
        )
    recorder.start(
        {
            "capture_mode": "skill_window",
            "pre_roll_s": 3.0,
            "post_roll_s": 0.0,
            "auto_export": False,
        }
    )
    recorder.begin_action(
        {"skill_name": "sonic.sit_chair"},
        timestamp_monotonic_s=base_monotonic + 5.0,
    )
    recorder.record_g1_debug(
        _g1_debug_sample(6, 6.0),
        received_monotonic_s=base_monotonic + 6.0,
    )
    recorder.mark_action_dispatched(
        {"skill_name": "sonic.sit_chair"},
        timestamp_monotonic_s=base_monotonic + 7.0,
    )
    recorder.record_g1_debug(
        _g1_debug_sample(7, 7.0),
        received_monotonic_s=base_monotonic + 7.0,
    )
    recorder.finish_action(
        {
            "skill_name": "sonic.sit_chair",
            "motion_commanded": True,
        },
        timestamp_monotonic_s=base_monotonic + 7.0,
    )
    result = recorder.stop()

    with np.load(result["raw_real_rollout"], allow_pickle=False) as raw:
        assert raw["source_index"].tolist() == [4, 5, 6, 7]
        assert raw["capture_phase"].tolist() == [
            "pre",
            "pre",
            "pre",
            "action",
        ]


def test_service_auto_arms_sit_window_without_rollout_start(tmp_path) -> None:
    recorder = RolloutRecorder(tmp_path, runtime_mode="real")
    recorder.record_g1_debug(_g1_debug_sample(0, 0.0))

    class SitExecutor(DryRunPrimitiveExecutor):
        def execute_action(self, skill_name, arguments, safety):
            recorder.record_g1_debug(_g1_debug_sample(1, 1.0))
            return {
                "ok": True,
                "error_message": None,
                "action_status": "completed",
                "executed_arguments": {
                    "motion_name": "sit_060cm",
                    "reference_distance_m": 0.6,
                    "duration_s": 0.05,
                },
                "telemetry": {
                    "completion": {
                        "motion_commanded": True,
                    }
                },
            }

    service = VigilBridgeService(
        executor=SitExecutor(runtime_mode="real"),
        rollout_recorder=recorder,
        runtime_mode="real",
    )
    response = service.execute_action(
        {
            "episode_id": "sit_auto",
            "step_id": 3,
            "runtime_mode": "real",
            "skill_name": "sonic.sit_chair",
            "arguments": {"chair_distance_m": 0.61},
            "safety": {},
        }
    )
    recorder.record_g1_debug(_g1_debug_sample(2, 2.0))
    assert recorder.status()["capture"]["action_finished"] is False
    time.sleep(0.06)
    recorder.record_g1_debug(_g1_debug_sample(3, 3.0))
    result = recorder.stop()

    assert response["ok"] is True
    assert result["ok"] is True
    with np.load(result["raw_real_rollout"], allow_pickle=False) as raw:
        assert raw["source_index"].tolist() == [0, 1, 2, 3]
        assert raw["capture_phase"].tolist() == [
            "pre",
            "action",
            "action",
            "post",
        ]


def test_rejected_sit_does_not_export_pose_files(tmp_path) -> None:
    recorder = RolloutRecorder(tmp_path, runtime_mode="real")
    recorder.record_g1_debug(_g1_debug_sample(0, 0.0))
    recorder.begin_action({"skill_name": "sonic.sit_chair"})

    status = recorder.finish_action(
        {
            "skill_name": "sonic.sit_chair",
            "action_status": "rejected",
            "motion_commanded": False,
        }
    )

    assert status["active"] is False
    assert status["last_export"]["export_status"] == "skipped_no_motion_commanded"
    session_dir = Path(status["session_dir"])
    assert (session_dir / "manifest.json").exists()
    assert not (session_dir / "raw_real_rollout.npz").exists()
    assert not (session_dir / "3dgs_replay.npz").exists()
    assert not (session_dir / "gear_sonic_reference.npz").exists()


def test_router_exposes_rollout_lifecycle(tmp_path) -> None:
    recorder = RolloutRecorder(tmp_path, runtime_mode="real")
    router = BridgeRequestRouter(
        VigilBridgeService(
            rollout_recorder=recorder,
            runtime_mode="real",
        )
    )

    start = router.dispatch(
        "rollout/start",
        {"session_name": "router", "capture_mode": "continuous"},
    )
    localization = router.dispatch(
        "rollout/localization",
        {
            "base_xyz": [0.0, 0.0, 0.8],
            "source": "mocap",
            "frame_id": "world",
            "valid": True,
            "estimated": False,
        },
    )
    recorder.record_g1_debug(_g1_debug_sample(0, 0.0))
    status = router.dispatch("rollout/status", {})
    stop = router.dispatch("rollout/stop", {})

    assert start["active"] is True
    assert localization["localization"]["source"] == "mocap"
    assert status["sample_count"] == 1
    assert stop["ok"] is True


def test_real_and_mujoco_robot_state_expose_all_measured_joints(tmp_path) -> None:
    state = MujocoRobotState(
        yaw=0.0,
        base_quat=[1.0, 0.0, 0.0, 0.0],
        delta_heading=0.0,
        timestamp=time.monotonic(),
        yaw_rate=0.1,
        base_ang_vel=[0.0, 0.0, 0.1],
        body_q=[float(index) for index in range(29)],
        body_dq=[0.1] * 29,
        body_q_target=[0.2] * 29,
        policy_action=[0.3] * 29,
    )

    class FakeStateSubscriber:
        def latest(self) -> Any:
            return state

    recorder = RolloutRecorder(tmp_path, runtime_mode="real")
    recorder.update_localization(
        {
            "base_xyz": [1.0, 2.0, 0.8],
            "source": "mocap",
            "frame_id": "map",
            "valid": True,
            "estimated": False,
        }
    )
    real = RealRuntimeClient(
        RealBridgeConfig(camera_enabled=False, camera_required=False),
        rollout_recorder=recorder,
    )
    real._state_sub = FakeStateSubscriber()  # type: ignore[assignment]
    mujoco = MujocoRuntimeClient(MujocoBridgeConfig(odom_source="off"))
    mujoco._state_sub = FakeStateSubscriber()  # type: ignore[assignment]

    real_payload = real.get_robot_state_payload()
    mujoco_payload = mujoco.get_robot_state_payload()

    assert real_payload is not None
    assert mujoco_payload is not None
    assert list(real_payload["joint_positions"]) == list(G1_JOINT_ORDER)
    assert list(mujoco_payload["joint_positions"]) == list(G1_JOINT_ORDER)
    assert len(real_payload["joint_velocities"]) == 29
    assert len(mujoco_payload["joint_targets"]) == 29
    assert real_payload["base_pose"]["x_m"] == 1.0
    assert real_payload["base_pose"]["frame_id"] == "map"
    assert real_payload["base_pose"]["translation_valid"] is True
