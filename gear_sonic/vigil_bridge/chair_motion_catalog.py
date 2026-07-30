"""Validated, exact-distance FaceE chair reference catalog."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_CEILING
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_CATALOG = Path(__file__).resolve().parent / "data/facee_chair_13s/manifest.json"


@dataclass(frozen=True)
class ChairMotion:
    requested_distance_m: float
    reference_distance_m: float
    tag: str
    motion_name: str
    duration_s: float
    frames: dict[str, Any]


class ChairMotionCatalog:
    """Map a measured distance upward to the next validated 5 cm reference."""

    def __init__(self, manifest_path: str | Path = DEFAULT_CATALOG) -> None:
        self.manifest_path = Path(manifest_path).expanduser().resolve()
        payload = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        records = payload.get("motions")
        if payload.get("schema_version") != 1 or not isinstance(records, list):
            raise ValueError("unsupported chair-motion manifest")
        self._records: dict[int, dict[str, Any]] = {}
        for record in records:
            distance = float(record["distance_m"])
            key = round(distance * 100)
            if abs(distance * 100 - key) > 1e-6 or key in self._records:
                raise ValueError(f"invalid/duplicate chair distance: {distance}")
            self._records[key] = dict(record)
        expected = set(range(115, 201, 5))
        if set(self._records) != expected:
            raise ValueError("chair catalog must contain exactly 1.15..2.00 m at 0.05 m")

    @property
    def distances_m(self) -> list[float]:
        return [key / 100.0 for key in sorted(self._records)]

    def load(self, distance_m: Any) -> ChairMotion:
        try:
            measured = Decimal(str(distance_m))
        except (InvalidOperation, TypeError, ValueError) as exc:
            raise ValueError("chair_distance_m must be numeric") from exc
        if not measured.is_finite():
            raise ValueError("chair_distance_m must be finite")
        if measured < Decimal("1.10") or measured > Decimal("2.00"):
            raise ValueError(
                f"chair_distance_m={measured} is outside the supported measured range "
                "[1.10, 2.00] m"
            )
        selected = (
            (measured / Decimal("0.05")).to_integral_value(rounding=ROUND_CEILING)
            * Decimal("0.05")
        )
        selected = max(selected, Decimal("1.15"))
        key = int(selected * 100)
        record = self._records[key]
        path = self.manifest_path.parent / str(record["file"])
        if self._sha256(path) != record["sha256"]:
            raise ValueError(f"chair motion checksum mismatch: {path}")
        with np.load(path, allow_pickle=False) as archive:
            frames = {name: archive[name] for name in archive.files}
        self._validate_frames(frames, record)
        frames["encode_mode"] = int(record.get("encode_mode", 0))
        frames["motion_id"] = -1
        return ChairMotion(
            requested_distance_m=float(measured),
            reference_distance_m=key / 100.0,
            tag=str(record["tag"]),
            motion_name=str(record["motion_name"]),
            duration_s=float(record["duration_s"]),
            frames=frames,
        )

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

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
