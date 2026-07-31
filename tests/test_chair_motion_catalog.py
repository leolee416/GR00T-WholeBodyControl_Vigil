from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from gear_sonic.vigil_bridge.chair_motion_catalog import (
    ChairMotionCatalog,
    DEFAULT_CATALOG,
)
from gear_sonic.vigil_bridge.primitive_executor import DryRunPrimitiveExecutor
from gear_sonic.vigil_bridge.real_adapter import (
    RealBridgeConfig,
    RealPrimitiveExecutor,
)


def test_official_catalog_is_exact_18_distance_grid() -> None:
    catalog = ChairMotionCatalog()
    assert catalog.distances_m == [value / 100 for value in range(115, 201, 5)]
    for distance in catalog.distances_m:
        motion = catalog.load(distance)
        assert motion.frames["joint_pos"].shape == (650, 29)
        assert motion.frames["joint_vel"].shape == (650, 29)
        assert motion.frames["body_quat_w"].shape == (650, 4)
        assert motion.frames["joint_order"].item() == "isaaclab"
        assert motion.frames["source_joint_order"].item() == "mujoco"
        assert motion.frames["protocol_version"].item() == 1
        assert motion.duration_s == 13.0

    manifest = json.loads(DEFAULT_CATALOG.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == 2
    assert manifest["protocol_version"] == 1
    assert manifest["source_joint_order"] == "mujoco"
    assert manifest["joint_order"] == "isaaclab"
    assert manifest["joint_count"] == 29
    assert manifest["joint_order_mapping"] == "G1_MUJOCO_TO_ISAACLAB_DOF"


def test_user_selected_reference_reuse_and_audit_status() -> None:
    catalog = ChairMotionCatalog()
    d145 = catalog.load(1.45)
    for distance in (1.35, 1.40):
        motion = catalog.load(distance)
        for field in ("joint_pos", "joint_vel", "body_quat_w"):
            assert np.array_equal(motion.frames[field], d145.frames[field])
    for target, donor in ((1.55, 1.70), (1.65, 1.80)):
        motion = catalog.load(target)
        donor_motion = catalog.load(donor)
        for field in ("joint_pos", "joint_vel", "body_quat_w"):
            assert np.array_equal(motion.frames[field], donor_motion.frames[field])

    manifest = json.loads(DEFAULT_CATALOG.read_text(encoding="utf-8"))
    rows = {row["tag"]: row for row in manifest["motions"]}
    assert manifest["policy_observation_contract"].endswith(
        "no_height_map_no_chair_pose"
    )
    assert manifest["strict_tracking_successes"] == 15
    assert manifest["task_level_acceptances"] == 3
    for tag in ("d1p35", "d1p40"):
        assert rows[tag]["task_level_status"] == "ACCEPT"
        assert rows[tag]["strict_tracking_status"] == "FAIL"
        assert rows[tag]["strict_tracking_reason"] == "anchor_ori_full"
        assert rows[tag]["reuses_reference_tag"] == "d1p45"
    assert rows["d1p55"]["task_level_status"] == "STRICT_TRACKING_OK"
    assert rows["d1p55"]["strict_tracking_status"] == "OK"
    assert rows["d1p55"]["reuses_reference_tag"] == "d1p70"
    assert rows["d1p55"]["mujoco_strict_status"] == "PASS"
    assert rows["d1p65"]["task_level_status"] == "ACCEPT"
    assert rows["d1p65"]["strict_tracking_status"] == "FAIL"
    assert rows["d1p65"]["strict_tracking_reason"] == "ee_body_pos"
    assert rows["d1p65"]["reuses_reference_tag"] == "d1p80"
    assert rows["d1p65"]["mujoco_task_level_status"] == "USER_ACCEPT"
    assert rows["d1p65"]["mujoco_strict_status"] == "FAIL"


@pytest.mark.parametrize(
    ("measured", "selected"),
    [(1.12, 1.15), (1.15, 1.15), (1.151, 1.20), (1.17, 1.20), (1.96, 2.00)],
)
def test_catalog_selects_next_longer_reference(
    measured: float, selected: float
) -> None:
    motion = ChairMotionCatalog().load(measured)
    assert motion.requested_distance_m == measured
    assert motion.reference_distance_m == selected


@pytest.mark.parametrize("distance", [1.09, 2.001, float("nan"), "bad", None])
def test_catalog_rejects_out_of_range_or_invalid_distance(distance: object) -> None:
    with pytest.raises(ValueError):
        ChairMotionCatalog().load(distance)


def test_catalog_rejects_modified_asset(tmp_path: Path) -> None:
    payload = json.loads(DEFAULT_CATALOG.read_text(encoding="utf-8"))
    source = DEFAULT_CATALOG.parent / payload["motions"][0]["file"]
    target = tmp_path / source.name
    target.write_bytes(source.read_bytes() + b"tamper")
    payload["motions"] = [dict(row) for row in payload["motions"]]
    for row in payload["motions"][1:]:
        source_item = DEFAULT_CATALOG.parent / row["file"]
        (tmp_path / row["file"]).write_bytes(source_item.read_bytes())
    (tmp_path / "manifest.json").write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="checksum"):
        ChairMotionCatalog(tmp_path / "manifest.json").load(1.15)


@pytest.mark.parametrize(
    ("field", "bad_value"),
    [
        ("schema_version", 1),
        ("protocol_version", 2),
        ("joint_order", "mujoco"),
        ("source_joint_order", "isaaclab"),
        ("joint_count", 28),
    ],
)
def test_catalog_rejects_wrong_top_level_contract(
    tmp_path: Path, field: str, bad_value: object
) -> None:
    payload = json.loads(DEFAULT_CATALOG.read_text(encoding="utf-8"))
    payload[field] = bad_value
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError):
        ChairMotionCatalog(manifest)


def test_d1p70_first_frame_has_isaaclab_semantics() -> None:
    first = ChairMotionCatalog().load(1.70).frames["joint_pos"][0]
    # IsaacLab slots 0/1/2 are left hip pitch, right hip pitch, waist yaw.
    assert first[:3] == pytest.approx(
        [0.176505998, 0.198895991, 0.030484870], abs=1e-8
    )


def test_dry_run_selects_only_requested_reference() -> None:
    executor = DryRunPrimitiveExecutor(chair_motion_catalog=ChairMotionCatalog())
    response = executor.execute_action(
        "sonic.sit_chair", {"chair_distance_m": 1.60}, {}
    )
    assert response["ok"] is True
    assert response["executed_arguments"]["chair_distance_m"] == 1.60
    assert response["executed_arguments"]["reference_distance_m"] == 1.60
    assert response["executed_arguments"]["tag"] == "d1p60"
    assert response["executed_arguments"]["frame_count"] == 650


def test_dry_run_reports_measured_and_selected_distance() -> None:
    executor = DryRunPrimitiveExecutor(chair_motion_catalog=ChairMotionCatalog())
    response = executor.execute_action(
        "sonic.sit_chair", {"chair_distance_m": 1.17}, {}
    )
    assert response["ok"] is True
    assert response["executed_arguments"]["chair_distance_m"] == 1.17
    assert response["executed_arguments"]["reference_distance_m"] == 1.20
    assert response["executed_arguments"]["tag"] == "d1p20"


def test_real_executor_fails_closed_when_motion_disabled() -> None:
    class RuntimeMustNotRun:
        def start(self):  # pragma: no cover - failure is the assertion
            raise AssertionError("runtime must not start")

    executor = RealPrimitiveExecutor(
        config=RealBridgeConfig(
            motion_enabled=False,
            chair_motion_catalog=str(DEFAULT_CATALOG),
        ),
        runtime=RuntimeMustNotRun(),  # type: ignore[arg-type]
    )
    response = executor.execute_action(
        "sonic.sit_chair", {"chair_distance_m": 1.15}, {}
    )
    assert response["ok"] is False
    assert response["action_status"] == "rejected"
    assert "disabled" in str(response["error_message"])
