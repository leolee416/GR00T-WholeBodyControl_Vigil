#!/usr/bin/env python3
"""Extract the audited Sit-only hardware18 subset from the full403 catalog.

This tool never resamples a motion. It copies the selected full403 NPZ files
byte-for-byte, verifies every checksum and writes a narrower selection
contract for the three explicitly packaged distance bands.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPO_ROOT / "gear_sonic/vigil_bridge/data/facee_stage1_generalist_yaw_hardware18"
EXPECTED_FULL_CATALOG_NAME = "facee_stage1_generalist_yaw_full403_sit_only"
EXPECTED_FULL_CATALOG_SHA256 = "ab1f592d77dfafdd17d21f892ecb46423816902bb8337ee01edae96e4e778eee"
EXPECTED_CHECKPOINT_SHA256 = "dc543f99fb51207727799d9c939cbd6b58e83af354947ddc843dd9ffa913607d"
CLEAN_YAWS = (0, 5, 10, 15, 20, 25)
ACTION_COMPLETE_YAWS = (-15, -10, -5, 0, 5, 10)
TARGETS = (
    *((1.45, yaw, True) for yaw in CLEAN_YAWS),
    *((2.00, yaw, False) for yaw in ACTION_COMPLETE_YAWS),
    *((2.40, yaw, False) for yaw in ACTION_COMPLETE_YAWS),
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--full-catalog",
        type=Path,
        required=True,
        help="verified full403 manifest or its containing directory",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_full_catalog(path: Path) -> tuple[Path, dict[str, Any]]:
    manifest_path = path / "manifest.json" if path.is_dir() else path
    manifest_path = manifest_path.expanduser().resolve()
    if _sha256(manifest_path) != EXPECTED_FULL_CATALOG_SHA256:
        raise ValueError("full403 manifest checksum does not match the uploaded audited catalog")
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    if payload.get("name") != EXPECTED_FULL_CATALOG_NAME:
        raise ValueError("unexpected full403 catalog name")
    if payload.get("count") != 403 or payload.get("mode") != "sit":
        raise ValueError("source catalog must contain exactly 403 Sit motions")
    if payload.get("stand_included") is not False:
        raise ValueError("source catalog must exclude Stand motions")
    if payload.get("policy_checkpoint_sha256") != EXPECTED_CHECKPOINT_SHA256:
        raise ValueError("source catalog uses an unexpected policy checkpoint")
    records = payload.get("motions")
    if not isinstance(records, list) or len(records) != 403:
        raise ValueError("source manifest motions do not match count=403")
    if any("stand" in str(record.get("tag", "")).lower() for record in records):
        raise ValueError("source catalog unexpectedly contains a Stand motion")
    return manifest_path, payload


def _select_records(payload: dict[str, Any]) -> list[dict[str, Any]]:
    by_condition = {
        (round(float(record["distance_m"]), 2), int(record["chair_yaw_deg"])): record
        for record in payload["motions"]
    }
    selected: list[dict[str, Any]] = []
    for distance_m, yaw_deg, expected_clean in TARGETS:
        record = by_condition.get((distance_m, yaw_deg))
        if record is None:
            raise ValueError(f"missing full403 condition {distance_m:.2f} m / {yaw_deg} deg")
        acceptance = record.get("crosssim_acceptance")
        if not isinstance(acceptance, dict):
            raise ValueError(f"{record['tag']}: missing cross-simulator acceptance")
        if acceptance.get("isaac_physx_strict") is not True:
            raise ValueError(f"{record['tag']}: expected Isaac strict pass")
        if acceptance.get("direct_mujoco_action_completed") is not True:
            raise ValueError(f"{record['tag']}: expected MuJoCo action completion")
        if acceptance.get("direct_mujoco_clean") is not expected_clean:
            raise ValueError(f"{record['tag']}: expected direct_mujoco_clean={expected_clean}")
        selected.append(copy.deepcopy(record))
    if len({record["tag"] for record in selected}) != 18:
        raise ValueError("hardware18 selection must contain 18 unique motions")
    return selected


def _copy_exact_assets(source_dir: Path, output_dir: Path, records: list[dict[str, Any]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    expected_files = {str(record["file"]) for record in records}
    unexpected = {path.name for path in output_dir.glob("*.npz") if path.name not in expected_files}
    if unexpected:
        raise ValueError(f"output contains unexpected NPZ files: {sorted(unexpected)}")
    for record in records:
        source = source_dir / str(record["file"])
        destination = output_dir / str(record["file"])
        expected_sha256 = str(record["sha256"])
        if _sha256(source) != expected_sha256:
            raise ValueError(f"{record['tag']}: source NPZ checksum mismatch")
        shutil.copyfile(source, destination)
        if _sha256(destination) != expected_sha256:
            raise ValueError(f"{record['tag']}: copied NPZ checksum mismatch")


def _build_manifest(source: dict[str, Any], records: list[dict[str, Any]]) -> dict[str, Any]:
    payload = copy.deepcopy(source)
    payload.update(
        {
            "name": "facee_stage1_generalist_yaw_hardware18_sit_only",
            "count": 18,
            "construction": (
                "Exact byte-for-byte subset of the audited full403 Sit catalog; "
                "no motion regeneration, resampling, or numeric modification"
            ),
            "selection_contract": {
                "method": "nearest_available_reference",
                "tie_break": "lower_distance_then_lower_yaw",
                "measured_distance_range_m": [1.425, 2.425],
                "measured_distance_windows_m": [
                    [1.425, 1.475],
                    [1.975, 2.025],
                    [2.375, 2.425],
                ],
                "measured_yaw_range_deg": [-17.5, 27.5],
                "measured_yaw_range_deg_by_reference_distance": {
                    "1.45": [-2.5, 27.5],
                    "2.00": [-17.5, 12.5],
                    "2.40": [-17.5, 12.5],
                },
                "available_distance_m": [1.45, 2.0, 2.4],
                "available_yaw_deg": [-15, -10, -5, 0, 5, 10, 15, 20, 25],
                "chair_yaw_deg_required": True,
                "selector_source": "external_measurement_or_vision",
                "require_explicit_non_clean_opt_in": True,
            },
            "deployment_status": {
                "real_robot_authorized": False,
                "scope": (
                    "Sit-only requested hardware test subset; 2.00/2.40 m require "
                    "explicit non-clean opt-in"
                ),
                "reason": (
                    "6/18 conditions are two-simulator clean; the 12 conditions at "
                    "2.00/2.40 m complete in both simulators but are non-clean"
                ),
            },
            "subset_provenance": {
                "source_catalog_name": EXPECTED_FULL_CATALOG_NAME,
                "source_manifest_sha256": EXPECTED_FULL_CATALOG_SHA256,
                "source_oss_prefix": (
                    "oss://xrobot-data/fs/r2s_ego_exp/GR00T-WholeBodyControl_Vigil/"
                    "policy/facee_stage1_generalist_yaw/motion_catalog_full403/"
                ),
                "reuse_exact_npz_bytes": True,
            },
            "motions": records,
        }
    )
    return payload


def main() -> None:
    args = _parse_args()
    manifest_path, source = _load_full_catalog(args.full_catalog)
    records = _select_records(source)
    output = args.output.expanduser().resolve()
    _copy_exact_assets(manifest_path.parent, output, records)
    payload = _build_manifest(source, records)
    manifest_output = output / "manifest.json"
    manifest_output.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    clean_count = sum(
        bool(record["crosssim_acceptance"]["direct_mujoco_clean"]) for record in records
    )
    print(
        f"wrote {manifest_output}: {len(records)} Sit motions "
        f"({clean_count} clean, {len(records) - clean_count} explicit non-clean opt-in)"
    )


if __name__ == "__main__":
    main()
