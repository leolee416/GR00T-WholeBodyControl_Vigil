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
        assert motion.duration_s == 13.0


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
