from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import pytest

from gear_sonic.vigil_bridge.chair_motion_catalog import ChairMotionCatalog
from gear_sonic.vigil_bridge.primitive_executor import DryRunPrimitiveExecutor
from gear_sonic.vigil_bridge.rollout_recorder import G1_JOINT_ORDER
from gear_sonic.vigil_bridge.sit_chair_diagnostics import (
    ISAACLAB_TO_MUJOCO,
    analyze_rollout,
    diagnose_preflight,
    select_rollout_session,
)


@dataclass
class FakeStateProvider:
    robot_state: dict[str, Any]

    def get_robot_state(self) -> dict[str, Any]:
        return {
            "ok": True,
            "error_message": None,
            "robot_state": self.robot_state,
            "telemetry": {
                "state_connected": True,
                "ready_for_motion": True,
            },
        }


def test_preflight_is_read_only_and_matches_reference_frame_zero() -> None:
    catalog = ChairMotionCatalog()
    motion = catalog.load(1.80)
    reference_first = np.asarray(motion.frames["joint_pos"])[0, list(ISAACLAB_TO_MUJOCO)]
    state = {
        "joint_positions": dict(zip(G1_JOINT_ORDER, reference_first, strict=True)),
        "joint_velocities": dict(zip(G1_JOINT_ORDER, np.zeros(29), strict=True)),
    }
    executor = DryRunPrimitiveExecutor(chair_motion_catalog=catalog)

    response = diagnose_preflight(
        executor=executor,
        state_response=FakeStateProvider(state).get_robot_state(),
        request={"chair_distance_m": 1.80},
    )

    assert response["ok"] is True
    assert response["motion_commanded"] is False
    assert response["ready"] is True
    assert response["selected_reference"]["tag"] == "d1p80"
    assert response["selected_reference"]["source_frame_count"] == 650
    assert response["selected_reference"]["transmitted_frame_count"] == 696
    assert response["initial_alignment"]["rms_rad"] == 0.0


def test_preflight_reports_large_initial_joint_mismatch() -> None:
    catalog = ChairMotionCatalog()
    state = {
        "joint_positions": dict(zip(G1_JOINT_ORDER, np.zeros(29), strict=True)),
        "joint_velocities": dict(zip(G1_JOINT_ORDER, np.zeros(29), strict=True)),
    }

    response = diagnose_preflight(
        executor=DryRunPrimitiveExecutor(chair_motion_catalog=catalog),
        state_response=FakeStateProvider(state).get_robot_state(),
        request={
            "chair_distance_m": 1.80,
            "initial_rms_limit_rad": 0.01,
            "initial_max_limit_rad": 0.01,
        },
    )

    assert response["ok"] is True
    assert response["ready"] is False
    assert response["checks"]["initial_pose_close_to_reference"] is False
    assert response["initial_alignment"]["top_joint_errors"]


def test_rollout_analysis_reports_first_sustained_anomaly(tmp_path) -> None:
    session_dir = tmp_path / "diagnostic"
    session_dir.mkdir()
    sample_count = 150
    timestamps = np.arange(sample_count, dtype=np.float64) / 50.0
    phase = np.asarray(["pre"] * 50 + ["action"] * 100)
    body_q = np.zeros((sample_count, 29), dtype=np.float32)
    target_q = np.zeros_like(body_q)
    body_q[70:, 0] = 2.0
    base_quat = np.zeros((sample_count, 4), dtype=np.float32)
    base_quat[:, 0] = 1.0
    metadata = {
        "runtime_mode": "real",
        "source_rate_hz_estimate": 50.0,
        "capture_expected_duration_s": 2.0,
        "capture_final_motion_context": {"tag": "d1p80"},
    }
    np.savez_compressed(
        session_dir / "raw_real_rollout.npz",
        metadata_json=np.asarray(__import__("json").dumps(metadata)),
        timestamp_relative_s=timestamps,
        capture_phase=phase,
        body_q=body_q,
        body_q_target=target_q,
        base_quat_wxyz=base_quat,
        joint_order=np.asarray(G1_JOINT_ORDER),
        source_index=np.arange(sample_count, dtype=np.int64),
    )
    (session_dir / "camera_index.json").write_text(
        __import__("json").dumps(
            {
                "frames": [
                    {
                        "camera_name": "ego_view",
                        "frame_index": 4,
                        "path": "observations/ego_view/000004.jpg",
                        "pose_source_index": 69,
                        "capture_phase": "action",
                        "camera_timestamp": 123.4,
                        "received_wall_s": 125.0,
                    },
                    {
                        "camera_name": "ego_view",
                        "frame_index": 6,
                        "path": "observations/ego_view/000006.jpg",
                        "pose_source_index": 73,
                        "capture_phase": "action",
                    },
                ]
            }
        ),
        encoding="utf-8",
    )

    response = analyze_rollout(session_dir)

    first = response["anomaly_timeline"]["first_anomaly"]
    assert first["signal"] == "joint_tracking_rms"
    assert first["action_time_s"] == pytest.approx(0.4)
    assert first["source_index"] == 70
    assert first["top_joint_errors"][0]["joint"] == G1_JOINT_ORDER[0]
    rgb = first["nearest_observations"]["ego_view"]
    assert rgb["pose_source_index"] == 69
    assert rgb["source_index_delta"] == 1
    assert rgb["path"].endswith("observations/ego_view/000004.jpg")


def test_latest_rollout_does_not_silently_fall_back_while_exporting(tmp_path) -> None:
    exported = tmp_path / "older_exported"
    exported.mkdir()
    (exported / "raw_real_rollout.npz").touch()
    latest = tmp_path / "newest_still_recording"
    latest.mkdir()

    with pytest.raises(ValueError, match="not exported yet"):
        select_rollout_session(tmp_path)
