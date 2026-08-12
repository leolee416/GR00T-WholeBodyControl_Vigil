"""Validated FaceE chair reference catalogs, with optional yaw selection."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_CEILING
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_CATALOG = Path(__file__).resolve().parent / "data/facee_chair_13s_v2/manifest.json"
EXACT_V3_CATALOG = Path(__file__).resolve().parent / "data/facee_chair_13s_exact_v3/manifest.json"
STAGE1_GENERALIST_YAW_CATALOG = (
    Path(__file__).resolve().parent / "data/facee_stage1_generalist_yaw_clean6/manifest.json"
)
STAGE1_GENERALIST_YAW_HARDWARE_CATALOG = (
    Path(__file__).resolve().parent / "data/facee_stage1_generalist_yaw_hardware18/manifest.json"
)
STAGE1_GENERALIST_YAW_FULL_CATALOG = (
    Path(__file__).resolve().parent / "data/facee_stage1_generalist_yaw_full403/manifest.json"
)


@dataclass(frozen=True)
class ChairMotion:
    requested_distance_m: float
    reference_distance_m: float
    tag: str
    motion_name: str
    duration_s: float
    frames: dict[str, Any]
    requested_yaw_deg: float | None = None
    reference_yaw_deg: float | None = None
    reference_isaac_strict: bool | None = None
    reference_action_completed: bool | None = None
    reference_clean: bool | None = None


def select_chair_reference_distance_m(distance_m: Any) -> tuple[float, int]:
    """Apply the deployment contract: ceil to the next 5 cm grid point."""
    try:
        measured = Decimal(str(distance_m))
    except (InvalidOperation, TypeError, ValueError) as exc:
        raise ValueError("chair_distance_m must be numeric") from exc
    if not measured.is_finite():
        raise ValueError("chair_distance_m must be finite")
    if measured < Decimal("1.10") or measured > Decimal("2.00"):
        raise ValueError(
            f"chair_distance_m={measured} is outside the supported measured range " "[1.10, 2.00] m"
        )
    selected = (measured / Decimal("0.05")).to_integral_value(rounding=ROUND_CEILING) * Decimal(
        "0.05"
    )
    selected = max(selected, Decimal("1.15"))
    return float(selected), int(selected * 100)


class ChairMotionCatalog:
    """Select a validated reference using a manifest-declared contract."""

    def __init__(self, manifest_path: str | Path = DEFAULT_CATALOG) -> None:
        self.manifest_path = Path(manifest_path).expanduser().resolve()
        payload = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        records = payload.get("motions")
        self.schema_version = payload.get("schema_version")
        if self.schema_version not in (2, 3) or not isinstance(records, list):
            raise ValueError("unsupported chair-motion manifest")
        required_contract = {
            "protocol_version": 1,
            "source_joint_order": "mujoco",
            "joint_order": "isaaclab",
            "joint_count": 29,
            "joint_order_mapping": "G1_MUJOCO_TO_ISAACLAB_DOF",
        }
        for name, expected_value in required_contract.items():
            if payload.get(name) != expected_value:
                raise ValueError(f"chair-motion manifest {name} must be {expected_value!r}")
        mapping = payload.get("joint_order_mapping_values")
        if not isinstance(mapping, list) or sorted(mapping) != list(range(29)):
            raise ValueError("chair-motion manifest has invalid joint-order mapping")
        self._records: dict[int, dict[str, Any]] = {}
        self._yaw_records: dict[tuple[int, int], dict[str, Any]] = {}
        for record in records:
            for name in ("protocol_version", "source_joint_order", "joint_order"):
                if record.get(name) != required_contract[name]:
                    raise ValueError(
                        f"{record.get('tag', '<unknown>')}: {name} must be "
                        f"{required_contract[name]!r}"
                    )
            if record.get("joint_order_mapping") != required_contract["joint_order_mapping"]:
                raise ValueError(f"{record.get('tag', '<unknown>')}: invalid joint-order mapping")
            distance = float(record["distance_m"])
            key = round(distance * 100)
            if abs(distance * 100 - key) > 1e-6:
                raise ValueError(f"invalid/duplicate chair distance: {distance}")
            if self.schema_version == 2:
                if key in self._records:
                    raise ValueError(f"invalid/duplicate chair distance: {distance}")
                self._records[key] = dict(record)
            else:
                yaw = _finite_float(record.get("chair_yaw_deg"), "chair_yaw_deg")
                yaw_key = round(yaw)
                if abs(yaw - yaw_key) > 1e-6 or (key, yaw_key) in self._yaw_records:
                    raise ValueError(f"invalid/duplicate chair distance/yaw: {distance}, {yaw}")
                self._yaw_records[(key, yaw_key)] = dict(record)
        if self.schema_version == 2:
            expected = set(range(115, 201, 5))
            if set(self._records) != expected:
                raise ValueError("chair catalog must contain exactly 1.15..2.00 m at 0.05 m")
        else:
            if not self._yaw_records:
                raise ValueError("yaw-aware chair catalog must not be empty")
            self._validate_yaw_selection_contract(payload)
            self._selection_contract = dict(payload["selection_contract"])

    @property
    def distances_m(self) -> list[float]:
        keys = self._records if self.schema_version == 2 else {key[0] for key in self._yaw_records}
        return [key / 100.0 for key in sorted(keys)]

    @property
    def yaws_deg(self) -> list[int]:
        if self.schema_version != 3:
            return []
        return sorted({key[1] for key in self._yaw_records})

    def load(
        self,
        distance_m: Any,
        yaw_deg: Any | None = None,
        *,
        allow_non_clean: bool = False,
    ) -> ChairMotion:
        if not isinstance(allow_non_clean, bool):
            raise ValueError("allow_non_clean must be boolean")
        if self.schema_version == 2:
            if yaw_deg is not None and abs(_finite_float(yaw_deg, "chair_yaw_deg")) > 1e-9:
                raise ValueError("the selected chair catalog does not support chair_yaw_deg")
            selected, key = select_chair_reference_distance_m(distance_m)
            selected_yaw: float | None = None
            requested_yaw: float | None = None
            record = self._records[key]
        else:
            selected, selected_yaw, record = self._select_yaw_record(distance_m, yaw_deg)
            requested_yaw = _finite_float(yaw_deg, "chair_yaw_deg")
        acceptance = record.get("crosssim_acceptance")
        isaac_strict: bool | None = None
        action_completed: bool | None = None
        clean: bool | None = None
        if isinstance(acceptance, dict):
            isaac_strict = bool(acceptance.get("isaac_physx_strict"))
            action_completed = bool(acceptance.get("direct_mujoco_action_completed"))
            clean = bool(acceptance.get("direct_mujoco_clean")) and isaac_strict
        require_opt_in = bool(
            getattr(self, "_selection_contract", {}).get("require_explicit_non_clean_opt_in", False)
        )
        if require_opt_in and not clean and not allow_non_clean:
            raise ValueError(
                f"{record.get('tag', '<unknown>')} is not two-simulator clean; "
                "set allow_non_clean_reference=true only for an explicitly "
                "approved non-clean test"
            )
        path = self.manifest_path.parent / str(record["file"])
        if self._sha256(path) != record["sha256"]:
            raise ValueError(f"chair motion checksum mismatch: {path}")
        with np.load(path, allow_pickle=False) as archive:
            frames = {name: archive[name] for name in archive.files}
        self._validate_frames(frames, record)
        frames["encode_mode"] = int(record.get("encode_mode", 0))
        frames["motion_id"] = -1
        return ChairMotion(
            requested_distance_m=float(distance_m),
            reference_distance_m=selected,
            tag=str(record["tag"]),
            motion_name=str(record["motion_name"]),
            duration_s=float(record["duration_s"]),
            frames=frames,
            requested_yaw_deg=requested_yaw,
            reference_yaw_deg=selected_yaw,
            reference_isaac_strict=isaac_strict,
            reference_action_completed=action_completed,
            reference_clean=clean,
        )

    def _validate_yaw_selection_contract(self, payload: dict[str, Any]) -> None:
        contract = payload.get("selection_contract")
        if not isinstance(contract, dict):
            raise ValueError("yaw-aware chair catalog requires selection_contract")
        if contract.get("method") != "nearest_available_reference":
            raise ValueError("unsupported yaw-aware selection method")
        for name in ("measured_distance_range_m", "measured_yaw_range_deg"):
            bounds = contract.get(name)
            if not isinstance(bounds, list) or len(bounds) != 2:
                raise ValueError(f"selection_contract.{name} must contain two values")
            lower = _finite_float(bounds[0], name)
            upper = _finite_float(bounds[1], name)
            if lower > upper:
                raise ValueError(f"selection_contract.{name} is reversed")
        windows = contract.get("measured_distance_windows_m")
        if windows is not None:
            if not isinstance(windows, list) or not windows:
                raise ValueError("selection_contract.measured_distance_windows_m must be non-empty")
            for bounds in windows:
                self._validate_bounds(bounds, "measured_distance_windows_m")
        yaw_ranges = contract.get("measured_yaw_range_deg_by_reference_distance")
        if yaw_ranges is not None:
            if not isinstance(yaw_ranges, dict) or not yaw_ranges:
                raise ValueError(
                    "selection_contract.measured_yaw_range_deg_by_reference_distance "
                    "must be non-empty"
                )
            for distance, bounds in yaw_ranges.items():
                _finite_float(distance, "reference_distance_m")
                self._validate_bounds(bounds, "measured_yaw_range_deg_by_reference_distance")

    @staticmethod
    def _validate_bounds(bounds: Any, name: str) -> tuple[float, float]:
        if not isinstance(bounds, list) or len(bounds) != 2:
            raise ValueError(f"selection_contract.{name} must contain two values")
        lower = _finite_float(bounds[0], name)
        upper = _finite_float(bounds[1], name)
        if lower > upper:
            raise ValueError(f"selection_contract.{name} is reversed")
        return lower, upper

    def _select_yaw_record(
        self, distance_m: Any, yaw_deg: Any | None
    ) -> tuple[float, float, dict[str, Any]]:
        measured_distance = _finite_float(distance_m, "chair_distance_m")
        measured_yaw = _finite_float(yaw_deg, "chair_yaw_deg")
        distance_bounds = self._selection_contract["measured_distance_range_m"]
        yaw_bounds = self._selection_contract["measured_yaw_range_deg"]
        if not float(distance_bounds[0]) <= measured_distance <= float(distance_bounds[1]):
            raise ValueError(
                f"chair_distance_m={measured_distance} is outside the supported "
                f"measured range [{distance_bounds[0]}, {distance_bounds[1]}] m"
            )
        distance_windows = self._selection_contract.get("measured_distance_windows_m")
        if distance_windows is not None and not any(
            float(bounds[0]) <= measured_distance <= float(bounds[1]) for bounds in distance_windows
        ):
            raise ValueError(
                f"chair_distance_m={measured_distance} is outside the supported "
                f"measured distance windows {distance_windows} m"
            )
        if not float(yaw_bounds[0]) <= measured_yaw <= float(yaw_bounds[1]):
            raise ValueError(
                f"chair_yaw_deg={measured_yaw} is outside the supported measured "
                f"range [{yaw_bounds[0]}, {yaw_bounds[1]}] deg"
            )
        distance_key = min(
            {key[0] for key in self._yaw_records},
            key=lambda item: (abs(item / 100.0 - measured_distance), item),
        )
        yaw_ranges = self._selection_contract.get("measured_yaw_range_deg_by_reference_distance")
        if yaw_ranges is not None:
            range_key = f"{distance_key / 100.0:.2f}"
            per_distance_bounds = yaw_ranges.get(range_key)
            if per_distance_bounds is None:
                raise ValueError(f"selection contract has no yaw range for {range_key} m")
            if not float(per_distance_bounds[0]) <= measured_yaw <= float(per_distance_bounds[1]):
                raise ValueError(
                    f"chair_yaw_deg={measured_yaw} is outside the supported "
                    f"measured range {per_distance_bounds} deg at {range_key} m"
                )
        key = min(
            (key for key in self._yaw_records if key[0] == distance_key),
            key=lambda item: (
                abs(item[1] - measured_yaw),
                item[1],
            ),
        )
        return key[0] / 100.0, float(key[1]), self._yaw_records[key]

    @staticmethod
    def _validate_frames(frames: dict[str, Any], record: dict[str, Any]) -> None:
        count = int(record["frames"])
        expected = {
            "joint_pos": (count, 29),
            "joint_vel": (count, 29),
            "body_quat_w": (count, 4),
            "frame_index": (count,),
        }
        for name, shape in expected.items():
            value = frames.get(name)
            if not isinstance(value, np.ndarray) or value.shape != shape:
                raise ValueError(f"{record['tag']}: {name} must have shape {shape}")
            if not np.isfinite(value).all():
                raise ValueError(f"{record['tag']}: {name} contains non-finite values")
        if not np.array_equal(frames["frame_index"], np.arange(count)):
            raise ValueError(f"{record['tag']}: frame_index must be contiguous from zero")
        scalar_contract = {
            "joint_order": "isaaclab",
            "source_joint_order": "mujoco",
            "protocol_version": 1,
        }
        for name, expected_value in scalar_contract.items():
            value = frames.get(name)
            if not isinstance(value, np.ndarray) or value.shape != ():
                raise ValueError(f"{record['tag']}: {name} must be a scalar array")
            if value.item() != expected_value:
                raise ValueError(f"{record['tag']}: {name} must be {expected_value!r}")

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()


def _finite_float(value: Any, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not np.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed
