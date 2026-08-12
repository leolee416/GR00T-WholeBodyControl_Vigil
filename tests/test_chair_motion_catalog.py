from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from gear_sonic.vigil_bridge.chair_motion_catalog import (
    ChairMotionCatalog,
    DEFAULT_CATALOG,
    EXACT_V3_CATALOG,
    STAGE1_GENERALIST_YAW_CATALOG,
    STAGE1_GENERALIST_YAW_HARDWARE_CATALOG,
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


def test_e0019_exact_v3_catalog_is_complete_and_crosssim_accepted() -> None:
    catalog = ChairMotionCatalog(EXACT_V3_CATALOG)
    manifest = json.loads(EXACT_V3_CATALOG.read_text(encoding="utf-8"))
    assert catalog.distances_m == [value / 100 for value in range(115, 201, 5)]
    assert manifest["name"] == "faceE_e0019_exact_v3_crosssim_18of18"
    assert manifest["startup_history"] == "repeat_reset"
    assert manifest["chair_geometry"]["backrest"] is False
    assert manifest["chair_geometry"]["width_m"] == 0.5
    assert manifest["chair_geometry"]["depth_m"] == 0.45
    assert manifest["chair_geometry"]["seat_height_m"] == 0.41
    assert manifest["chair_geometry"]["vertices"] == 258
    assert manifest["chair_geometry"]["triangles"] == 512
    for distance in catalog.distances_m:
        motion = catalog.load(distance)
        assert motion.frames["joint_pos"].shape == (650, 29)
        record = next(item for item in manifest["motions"] if item["tag"] == motion.tag)
        assert record["crosssim_acceptance"] == {
            "isaac_physx_strict": True,
            "cpu_mujoco_strict": True,
            "cpu_mujoco_no_preseat_lower_leg_kick": True,
        }
        assert record["chair_layout"]["backrest"] is False


def test_stage1_yaw_catalog_contains_only_two_simulator_clean_subset() -> None:
    catalog = ChairMotionCatalog(STAGE1_GENERALIST_YAW_CATALOG)
    manifest = json.loads(STAGE1_GENERALIST_YAW_CATALOG.read_text(encoding="utf-8"))
    assert catalog.schema_version == 3
    assert catalog.distances_m == [1.45]
    assert catalog.yaws_deg == [0, 5, 10, 15, 20, 25]
    assert manifest["count"] == 6
    assert manifest["mode"] == "sit"
    assert manifest["stand_included"] is False
    assert manifest["deployment_status"]["real_robot_authorized"] is False
    for yaw in catalog.yaws_deg:
        motion = catalog.load(1.45, yaw)
        assert motion.requested_yaw_deg == yaw
        assert motion.reference_yaw_deg == yaw
        assert motion.frames["joint_pos"].shape == (650, 29)
        record = next(row for row in manifest["motions"] if row["tag"] == motion.tag)
        assert record["crosssim_acceptance"]["isaac_physx_strict"] is True
        assert record["crosssim_acceptance"]["direct_mujoco_clean"] is True
        assert record["crosssim_acceptance"]["direct_mujoco_lower_leg_chair_peak_n"] == 0.0


@pytest.mark.parametrize(
    ("measured_yaw", "selected_yaw"),
    [(-2.5, 0.0), (2.49, 0.0), (2.5, 0.0), (2.51, 5.0), (12.4, 10.0), (27.5, 25.0)],
)
def test_stage1_yaw_catalog_selects_nearest_reference(
    measured_yaw: float, selected_yaw: float
) -> None:
    motion = ChairMotionCatalog(STAGE1_GENERALIST_YAW_CATALOG).load(1.46, measured_yaw)
    assert motion.reference_distance_m == 1.45
    assert motion.reference_yaw_deg == selected_yaw


@pytest.mark.parametrize(
    ("distance", "yaw"),
    [(1.40, 0), (1.50, 0), (1.45, -2.51), (1.45, 27.51), (1.45, None)],
)
def test_stage1_yaw_catalog_fails_closed_outside_packaged_scope(
    distance: object, yaw: object
) -> None:
    with pytest.raises(ValueError):
        ChairMotionCatalog(STAGE1_GENERALIST_YAW_CATALOG).load(distance, yaw)


def test_stage1_hardware18_is_exact_sit_only_subset() -> None:
    catalog = ChairMotionCatalog(STAGE1_GENERALIST_YAW_HARDWARE_CATALOG)
    manifest = json.loads(STAGE1_GENERALIST_YAW_HARDWARE_CATALOG.read_text(encoding="utf-8"))
    records_by_distance = {
        distance: [row for row in manifest["motions"] if float(row["distance_m"]) == distance]
        for distance in (1.45, 2.0, 2.4)
    }
    assert manifest["name"] == "facee_stage1_generalist_yaw_hardware18_sit_only"
    assert manifest["count"] == 18
    assert manifest["mode"] == "sit"
    assert manifest["stand_included"] is False
    assert manifest["subset_provenance"]["reuse_exact_npz_bytes"] is True
    assert catalog.distances_m == [1.45, 2.0, 2.4]
    assert catalog.yaws_deg == [-15, -10, -5, 0, 5, 10, 15, 20, 25]
    assert [row["chair_yaw_deg"] for row in records_by_distance[1.45]] == [
        0,
        5,
        10,
        15,
        20,
        25,
    ]
    for distance in (2.0, 2.4):
        assert [row["chair_yaw_deg"] for row in records_by_distance[distance]] == [
            -15,
            -10,
            -5,
            0,
            5,
            10,
        ]
    assert all("stand" not in row["tag"].lower() for row in manifest["motions"])


def test_stage1_hardware18_reuses_clean6_assets_byte_for_byte() -> None:
    clean = json.loads(STAGE1_GENERALIST_YAW_CATALOG.read_text(encoding="utf-8"))
    hardware = json.loads(STAGE1_GENERALIST_YAW_HARDWARE_CATALOG.read_text(encoding="utf-8"))
    hardware_by_tag = {row["tag"]: row for row in hardware["motions"]}
    for clean_record in clean["motions"]:
        hardware_record = hardware_by_tag[clean_record["tag"]]
        assert hardware_record["sha256"] == clean_record["sha256"]
        clean_asset = STAGE1_GENERALIST_YAW_CATALOG.parent / clean_record["file"]
        hardware_asset = STAGE1_GENERALIST_YAW_HARDWARE_CATALOG.parent / hardware_record["file"]
        assert hardware_asset.read_bytes() == clean_asset.read_bytes()


@pytest.mark.parametrize("distance", [2.0, 2.4])
@pytest.mark.parametrize("yaw", [-15, -10, -5, 0, 5, 10])
def test_stage1_hardware18_non_clean_distance_requires_explicit_opt_in(
    distance: float, yaw: int
) -> None:
    catalog = ChairMotionCatalog(STAGE1_GENERALIST_YAW_HARDWARE_CATALOG)
    with pytest.raises(ValueError, match="allow_non_clean_reference=true"):
        catalog.load(distance, yaw)
    motion = catalog.load(distance, yaw, allow_non_clean=True)
    assert motion.reference_distance_m == distance
    assert motion.reference_yaw_deg == yaw
    assert motion.reference_isaac_strict is True
    assert motion.reference_action_completed is True
    assert motion.reference_clean is False


def test_stage1_hardware18_clean_distance_does_not_require_non_clean_opt_in() -> None:
    motion = ChairMotionCatalog(STAGE1_GENERALIST_YAW_HARDWARE_CATALOG).load(1.45, 25)
    assert motion.tag == "d1p45_sit_yaw_p25"
    assert motion.reference_clean is True


@pytest.mark.parametrize(
    ("distance", "yaw"),
    [(1.70, 0), (2.20, 0), (2.00, 12.51), (2.40, 15), (1.45, -2.51)],
)
def test_stage1_hardware18_rejects_unpackaged_distance_yaw_regions(
    distance: float, yaw: float
) -> None:
    with pytest.raises(ValueError):
        ChairMotionCatalog(STAGE1_GENERALIST_YAW_HARDWARE_CATALOG).load(
            distance, yaw, allow_non_clean=True
        )


def test_stage1_hardware18_dry_run_reports_non_clean_selection() -> None:
    executor = DryRunPrimitiveExecutor(
        chair_motion_catalog=ChairMotionCatalog(STAGE1_GENERALIST_YAW_HARDWARE_CATALOG)
    )
    response = executor.execute_action(
        "sonic.sit_chair",
        {
            "chair_distance_m": 2.4,
            "chair_yaw_deg": 0,
            "allow_non_clean_reference": True,
        },
        {},
    )
    assert response["ok"] is True
    assert response["executed_arguments"]["tag"] == "d2p40_sit_yaw_p00"
    assert response["executed_arguments"]["reference_distance_m"] == 2.4
    assert response["executed_arguments"]["reference_yaw_deg"] == 0.0
    assert response["executed_arguments"]["reference_clean"] is False
    assert response["executed_arguments"]["allow_non_clean_reference"] is True


def test_full_yaw_catalog_requires_explicit_non_clean_opt_in(
    tmp_path: Path,
) -> None:
    payload = json.loads(STAGE1_GENERALIST_YAW_CATALOG.read_text(encoding="utf-8"))
    source_record = payload["motions"][0]
    source_motion = STAGE1_GENERALIST_YAW_CATALOG.parent / source_record["file"]
    target_motion = tmp_path / source_motion.name
    target_motion.write_bytes(source_motion.read_bytes())
    record = dict(source_record)
    record["crosssim_acceptance"] = {
        **record["crosssim_acceptance"],
        "direct_mujoco_clean": False,
        "direct_mujoco_lower_leg_chair_peak_n": 192.0,
    }
    payload["name"] = "test_full403_non_clean_gate"
    payload["selection_contract"] = {
        **payload["selection_contract"],
        "require_explicit_non_clean_opt_in": True,
    }
    payload["count"] = 1
    payload["motions"] = [record]
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps(payload), encoding="utf-8")

    catalog = ChairMotionCatalog(manifest)
    with pytest.raises(ValueError, match="allow_non_clean_reference=true"):
        catalog.load(1.45, 0)
    motion = catalog.load(1.45, 0, allow_non_clean=True)
    assert motion.reference_clean is False
    assert motion.reference_action_completed is True


def test_non_clean_opt_in_must_be_boolean() -> None:
    catalog = ChairMotionCatalog(STAGE1_GENERALIST_YAW_CATALOG)
    with pytest.raises(ValueError, match="must be boolean"):
        catalog.load(1.45, 0, allow_non_clean="true")  # type: ignore[arg-type]


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
    assert manifest["policy_observation_contract"].endswith("no_height_map_no_chair_pose")
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
def test_catalog_selects_next_longer_reference(measured: float, selected: float) -> None:
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
    assert first[:3] == pytest.approx([0.176505998, 0.198895991, 0.030484870], abs=1e-8)


def test_dry_run_selects_only_requested_reference() -> None:
    executor = DryRunPrimitiveExecutor(chair_motion_catalog=ChairMotionCatalog())
    response = executor.execute_action("sonic.sit_chair", {"chair_distance_m": 1.60}, {})
    assert response["ok"] is True
    assert response["executed_arguments"]["chair_distance_m"] == 1.60
    assert response["executed_arguments"]["reference_distance_m"] == 1.60
    assert response["executed_arguments"]["tag"] == "d1p60"
    assert response["executed_arguments"]["frame_count"] == 650


def test_dry_run_reports_measured_and_selected_distance() -> None:
    executor = DryRunPrimitiveExecutor(chair_motion_catalog=ChairMotionCatalog())
    response = executor.execute_action("sonic.sit_chair", {"chair_distance_m": 1.17}, {})
    assert response["ok"] is True
    assert response["executed_arguments"]["chair_distance_m"] == 1.17
    assert response["executed_arguments"]["reference_distance_m"] == 1.20
    assert response["executed_arguments"]["tag"] == "d1p20"


def test_dry_run_reports_measured_and_selected_yaw() -> None:
    executor = DryRunPrimitiveExecutor(
        chair_motion_catalog=ChairMotionCatalog(STAGE1_GENERALIST_YAW_CATALOG)
    )
    response = executor.execute_action(
        "sonic.sit_chair",
        {"chair_distance_m": 1.45, "chair_yaw_deg": 12.4},
        {},
    )
    assert response["ok"] is True
    assert response["executed_arguments"]["chair_yaw_deg"] == 12.4
    assert response["executed_arguments"]["reference_yaw_deg"] == 10.0
    assert response["executed_arguments"]["tag"] == "d1p45_sit_yaw_p10"
    assert response["executed_arguments"]["reference_clean"] is True
    assert response["executed_arguments"]["allow_non_clean_reference"] is False


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
    response = executor.execute_action("sonic.sit_chair", {"chair_distance_m": 1.15}, {})
    assert response["ok"] is False
    assert response["action_status"] == "rejected"
    assert "disabled" in str(response["error_message"])
