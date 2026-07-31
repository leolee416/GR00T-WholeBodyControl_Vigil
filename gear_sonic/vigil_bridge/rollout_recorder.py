"""Bridge-layer recording and export of measured G1 rollout state.

The recorder consumes the public ``g1_debug`` stream.  It deliberately does
not read the deploy CSV directory or modify policy/control code.  Root
translation is accepted only from an explicitly identified localization
source; the fixed ``base_trans_measured`` visualization placeholder is never
used as measured odometry.
"""

from __future__ import annotations

import base64
import binascii
import hashlib
import json
import math
import os
import re
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

try:
    import numpy as np
except ImportError:  # Keep dry-run bridge imports usable in lightweight environments.
    np = None  # type: ignore[assignment]


REPO_ROOT = Path(__file__).resolve().parents[2]
ROLLOUT_SCHEMA_VERSION = "groot_real_rollout_v2"
MAX_ACTION_CONTEXT_S = 3.0

G1_JOINT_ORDER: tuple[str, ...] = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


@dataclass(frozen=True)
class LocalizationSample:
    base_xyz: tuple[float, float, float]
    source: str
    frame_id: str
    valid: bool
    estimated: bool
    source_timestamp_s: float
    received_monotonic_s: float


class RolloutRecorder:
    """Thread-safe recorder for measured bridge state and replay exports."""

    def __init__(
        self,
        output_root: str | Path = "outputs/vigil_rollouts",
        runtime_mode: str = "real",
        localization_max_age_s: float = 0.5,
    ) -> None:
        path = Path(output_root).expanduser()
        self.output_root = path if path.is_absolute() else REPO_ROOT / path
        self.runtime_mode = runtime_mode
        self.localization_max_age_s = max(float(localization_max_age_s), 0.0)
        self._lock = threading.RLock()
        self._active = False
        self._samples: list[dict[str, Any]] = []
        self._rolling_buffer: list[dict[str, Any]] = []
        self._camera_frames: list[dict[str, Any]] = []
        self._camera_digests: dict[str, str] = {}
        self._session_id: str | None = None
        self._session_dir: Path | None = None
        self._started_wall_s: float | None = None
        self._started_monotonic_s: float | None = None
        self._metadata: dict[str, Any] = {}
        self._motion_context: dict[str, Any] = {}
        self._chair_world_pose = self._empty_chair_pose()
        self._latest_localization: LocalizationSample | None = None
        self._dropped_samples = 0
        self._last_error: str | None = None
        self._last_export: dict[str, Any] | None = None
        self._capture_mode = "skill_window"
        self._capture_skill = "sonic.sit_chair"
        self._pre_roll_s = MAX_ACTION_CONTEXT_S
        self._post_roll_s = MAX_ACTION_CONTEXT_S
        self._capture_started_monotonic_s: float | None = None
        self._capture_finished_monotonic_s: float | None = None
        self._capture_deadline_monotonic_s: float | None = None
        self._capture_complete = False
        self._auto_started = False
        self._auto_export = True
        self._auto_export_options: dict[str, Any] = {}
        self._auto_export_timer: threading.Timer | None = None

    def start(self, payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if np is None:
            raise RuntimeError("numpy is required for rollout recording and NPZ export")
        request = dict(payload or {})
        with self._lock:
            if self._active:
                raise RuntimeError(f"rollout recording is already active: {self._session_id}")

            capture_mode = str(request.get("capture_mode", "skill_window")).strip().lower()
            if capture_mode not in {"skill_window", "continuous"}:
                raise ValueError("capture_mode must be 'skill_window' or 'continuous'")
            capture_skill = str(
                request.get("capture_skill", "sonic.sit_chair")
            ).strip().lower()
            if capture_mode == "skill_window" and not capture_skill:
                raise ValueError("capture_skill is required for skill_window mode")
            pre_roll_s = self._bounded_float(
                request.get("pre_roll_s", MAX_ACTION_CONTEXT_S),
                0.0,
                MAX_ACTION_CONTEXT_S,
                "pre_roll_s",
            )
            post_roll_s = self._bounded_float(
                request.get("post_roll_s", MAX_ACTION_CONTEXT_S),
                0.0,
                MAX_ACTION_CONTEXT_S,
                "post_roll_s",
            )
            target_fps = self._bounded_float(
                request.get("target_fps", 50.0),
                1.0,
                240.0,
                "target_fps",
            )
            smoothing_window = self._bounded_int(
                request.get("smoothing_window", 5),
                1,
                101,
                "smoothing_window",
            )
            now_wall = time.time()
            session_name = self._sanitize_name(str(request.get("session_name", "rollout")))
            timestamp = datetime.fromtimestamp(now_wall, tz=timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            session_id = f"{timestamp}_{session_name}"
            chair_world_pose = (
                self._normalize_chair_pose(
                    request.get("chair_world_pose"),
                    allow_empty=True,
                )
                if "chair_world_pose" in request
                else dict(self._chair_world_pose)
            )
            session_dir = self.output_root / session_id
            suffix = 1
            while session_dir.exists():
                session_dir = self.output_root / f"{session_id}_{suffix:02d}"
                suffix += 1
            session_dir.mkdir(parents=True, exist_ok=False)

            self._active = True
            self._samples = []
            self._camera_frames = []
            self._camera_digests = {}
            self._session_id = session_dir.name
            self._session_dir = session_dir
            self._started_wall_s = now_wall
            self._started_monotonic_s = time.monotonic()
            self._chair_world_pose = chair_world_pose
            self._dropped_samples = 0
            self._last_error = None
            self._capture_mode = capture_mode
            self._capture_skill = capture_skill
            self._pre_roll_s = pre_roll_s
            self._post_roll_s = post_roll_s
            self._capture_started_monotonic_s = None
            self._capture_finished_monotonic_s = None
            self._capture_deadline_monotonic_s = None
            self._capture_complete = False
            self._auto_started = bool(request.get("auto_started", False))
            self._auto_export = bool(
                request.get("auto_export", capture_mode == "skill_window")
            )
            self._auto_export_options = {
                "target_fps": target_fps,
                "smoothing_window": smoothing_window,
            }
            self._cancel_auto_export_timer_locked()

            supplied_metadata = request.get("metadata")
            metadata = dict(supplied_metadata) if isinstance(supplied_metadata, Mapping) else {}
            for key in (
                "checkpoint",
                "checkpoint_id",
                "checkpoint_hash",
                "policy_config",
                "episode_id",
                "operator",
                "notes",
            ):
                if key in request:
                    metadata[key] = request[key]
            metadata.setdefault("checkpoint", None)
            metadata.setdefault("checkpoint_id", None)
            metadata.setdefault("checkpoint_hash", None)
            self._metadata = {
                **metadata,
                "schema_version": ROLLOUT_SCHEMA_VERSION,
                "runtime_mode": self.runtime_mode,
                "session_id": self._session_id,
                "started_at_utc": datetime.fromtimestamp(now_wall, tz=timezone.utc).isoformat(),
                "git_commit": self._git_commit(),
                "joint_order": list(G1_JOINT_ORDER),
                "joint_order_source": "g1_debug_mujoco_order",
                "checkpoint_metadata_source": "rollout/start request",
                "measured_state_source": "g1_debug.body_q/body_dq (LowState-derived)",
                "policy_action_semantics": (
                    "g1_debug.last_action remapped to MuJoCo order, scaled, and offset; "
                    "it is a commanded joint-position target, not the raw network tensor"
                ),
                "base_translation_policy": (
                    "external localization only; g1_debug.base_trans_measured is ignored "
                    "because deploy publishes a fixed visualization placeholder"
                ),
                "capture_mode": self._capture_mode,
                "capture_skill": self._capture_skill,
                "pre_roll_s": self._pre_roll_s,
                "post_roll_s": self._post_roll_s,
                "auto_started": self._auto_started,
                "auto_export": self._auto_export,
            }
            if isinstance(request.get("motion_context"), Mapping):
                self._motion_context.update(dict(request["motion_context"]))
            return self.status()

    def stop(self, payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        request = dict(payload or {})
        with self._lock:
            if not self._active:
                raise RuntimeError("rollout recording is not active")
            if not self._samples:
                raise RuntimeError("rollout contains no valid 29-DoF g1_debug samples")

            target_fps = self._bounded_float(
                request.get(
                    "target_fps",
                    self._auto_export_options.get("target_fps", 50.0),
                ),
                1.0,
                240.0,
                "target_fps",
            )
            smoothing_window = self._bounded_int(
                request.get(
                    "smoothing_window",
                    self._auto_export_options.get("smoothing_window", 5),
                ),
                1,
                101,
                "smoothing_window",
            )
            if smoothing_window % 2 == 0:
                smoothing_window += 1

            samples = [dict(sample) for sample in self._samples]
            camera_frames = [dict(frame) for frame in self._camera_frames]
            session_id = self._session_id
            session_dir = self._session_dir
            metadata = dict(self._metadata)
            chair_world_pose = dict(self._chair_world_pose)
            dropped_samples = self._dropped_samples
            self._cancel_auto_export_timer_locked()
            self._capture_complete = True
            self._active = False

        assert session_id is not None
        assert session_dir is not None

        try:
            export_result = self._export(
                samples=samples,
                session_dir=session_dir,
                metadata=metadata,
                chair_world_pose=chair_world_pose,
                camera_frames=camera_frames,
                target_fps=target_fps,
                smoothing_window=smoothing_window,
                dropped_samples=dropped_samples,
            )
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._active = True
            raise

        with self._lock:
            self._last_export = export_result
            self._samples = []
            self._camera_frames = []
            self._last_error = None
        return export_result

    def record_g1_debug(
        self,
        payload: Mapping[str, Any],
        *,
        received_monotonic_s: float | None = None,
        received_wall_s: float | None = None,
        localization: Mapping[str, Any] | None = None,
    ) -> bool:
        now_monotonic = time.monotonic() if received_monotonic_s is None else float(received_monotonic_s)
        now_wall = time.time() if received_wall_s is None else float(received_wall_s)

        with self._lock:
            try:
                body_q = self._required_vector(
                    payload.get("body_q_measured", payload.get("body_q")),
                    29,
                    "body_q",
                )
                body_dq = self._optional_vector(payload.get("body_dq"), 29)
                base_quat = self._required_vector(
                    payload.get("base_quat_measured", payload.get("base_quat")),
                    4,
                    "base_quat",
                )
                base_quat = self._normalize_quaternion(base_quat)
                base_ang_vel = self._optional_vector(payload.get("base_ang_vel"), 3)
                body_q_target = self._optional_vector(payload.get("body_q_target"), 29)
                policy_action = self._optional_vector(payload.get("last_action"), 29)
            except (TypeError, ValueError) as exc:
                self._dropped_samples += 1
                self._last_error = str(exc)
                return False

            if localization is not None:
                localized = self._parse_localization(localization, now_monotonic)
            else:
                localized = self._latest_localization
            localization_is_fresh = (
                localized is not None
                and localized.valid
                and now_monotonic - localized.received_monotonic_s <= self.localization_max_age_s
            )
            if localization_is_fresh:
                assert localized is not None
                base_xyz = list(localized.base_xyz)
                base_xyz_valid = True
                base_xyz_source = localized.source
                base_xyz_frame_id = localized.frame_id
                base_xyz_estimated = localized.estimated
                localization_timestamp_s = localized.source_timestamp_s
            else:
                base_xyz = [math.nan, math.nan, math.nan]
                base_xyz_valid = False
                base_xyz_source = "none"
                base_xyz_frame_id = ""
                base_xyz_estimated = True
                localization_timestamp_s = math.nan

            sample = {
                "timestamp_monotonic_s": now_monotonic,
                "timestamp_wall_s": now_wall,
                "source_index": self._safe_int(payload.get("index"), -1),
                "ros_timestamp_s": self._safe_float(payload.get("ros_timestamp"), math.nan),
                "body_q": body_q,
                "body_dq": body_dq,
                "base_quat": base_quat,
                "base_ang_vel": base_ang_vel,
                "base_xyz": base_xyz,
                "base_xyz_valid": base_xyz_valid,
                "base_xyz_source": base_xyz_source,
                "base_xyz_frame_id": base_xyz_frame_id,
                "base_xyz_estimated": base_xyz_estimated,
                "localization_timestamp_s": localization_timestamp_s,
                "body_q_target": body_q_target,
                "policy_action": policy_action,
                "motion_name": str(self._motion_context.get("motion_name", "")),
                "skill_name": str(self._motion_context.get("skill_name", "")),
                "reference_distance_m": self._safe_float(
                    self._motion_context.get("reference_distance_m"),
                    math.nan,
                ),
                "chair_distance_m": self._safe_float(
                    self._motion_context.get("chair_distance_m"),
                    math.nan,
                ),
            }
            self._append_rolling_sample_locked(sample, now_monotonic)
            if not self._active:
                return False
            if self._capture_mode == "continuous":
                self._samples.append(sample)
            elif self._capture_started_monotonic_s is None:
                return False
            elif (
                self._capture_deadline_monotonic_s is None
                or now_monotonic <= self._capture_deadline_monotonic_s
            ):
                self._samples.append(sample)
            else:
                self._capture_complete = True
                return False
            self._last_error = None
            return True

    def record_camera_payload(
        self,
        payload: Mapping[str, Any] | None,
        *,
        received_monotonic_s: float | None = None,
        received_wall_s: float | None = None,
        pose_source_index: Any = -1,
    ) -> int:
        """Save newly received JPEG frames and link them to the pose sample.

        The camera transport exposes its latest frame rather than a hardware
        synchronizer.  ``camera_index.json`` therefore preserves the camera
        timestamp when provided and the bridge receive time used for pairing.
        """
        if not isinstance(payload, Mapping):
            return 0
        images = payload.get("images")
        if not isinstance(images, Mapping):
            return 0
        timestamps = payload.get("timestamps")
        timestamps = timestamps if isinstance(timestamps, Mapping) else {}
        now_monotonic = time.monotonic() if received_monotonic_s is None else float(received_monotonic_s)
        now_wall = time.time() if received_wall_s is None else float(received_wall_s)

        with self._lock:
            if not self._active or self._session_dir is None:
                return 0
            saved = 0
            for camera_name, value in images.items():
                image_bytes = self._camera_bytes(value)
                if image_bytes is None:
                    continue
                name = self._sanitize_name(str(camera_name))
                digest = hashlib.sha256(image_bytes).hexdigest()
                if self._camera_digests.get(name) == digest:
                    continue
                self._camera_digests[name] = digest
                frame_index = len(self._camera_frames)
                relative_path = Path("observations") / name / f"{frame_index:06d}.jpg"
                self._atomic_write_bytes(self._session_dir / relative_path, image_bytes)
                self._camera_frames.append(
                    {
                        "frame_index": frame_index,
                        "camera_name": str(camera_name),
                        "path": str(relative_path),
                        "sha256": digest,
                        "byte_size": len(image_bytes),
                        "camera_timestamp": self._json_safe_scalar(timestamps.get(camera_name)),
                        "received_monotonic_s": now_monotonic,
                        "received_wall_s": now_wall,
                        "pose_source_index": self._safe_int(pose_source_index, -1),
                        "capture_phase": self._capture_phase_for_timestamp_locked(now_monotonic),
                    }
                )
                saved += 1
            return saved

    def update_localization(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            now_monotonic = time.monotonic()
            sample = self._parse_localization(payload, now_monotonic)
            self._latest_localization = sample
            if "chair_world_pose" in payload:
                self._chair_world_pose = self._normalize_chair_pose(
                    payload.get("chair_world_pose"),
                    allow_empty=False,
                )
            result = self.status()
            result["localization"] = self._localization_status(sample, now_monotonic)
            return result

    def set_motion_context(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        allowed = (
            "motion_name",
            "skill_name",
            "reference_distance_m",
            "chair_distance_m",
            "episode_id",
            "step_id",
            "tag",
        )
        with self._lock:
            for key in allowed:
                if key in payload:
                    self._motion_context[key] = payload[key]
            if "chair_world_pose" in payload:
                self._chair_world_pose = self._normalize_chair_pose(
                    payload.get("chair_world_pose"),
                    allow_empty=False,
                )
            result = self.status()
            result["motion_context"] = dict(self._motion_context)
            return result

    def begin_action(
        self,
        payload: Mapping[str, Any],
        *,
        timestamp_monotonic_s: float | None = None,
    ) -> dict[str, Any]:
        """Start a bounded action window, auto-arming sit capture when needed."""
        skill_name = str(payload.get("skill_name", "")).strip().lower()
        now_monotonic = (
            time.monotonic()
            if timestamp_monotonic_s is None
            else float(timestamp_monotonic_s)
        )
        with self._lock:
            target_skill = self._capture_skill if self._active else "sonic.sit_chair"
            if skill_name != target_skill:
                return self.status()
            if not self._active:
                self.start(
                    {
                        "session_name": f"{skill_name.replace('.', '_')}_auto",
                        "capture_mode": "skill_window",
                        "capture_skill": skill_name,
                        "pre_roll_s": MAX_ACTION_CONTEXT_S,
                        "post_roll_s": MAX_ACTION_CONTEXT_S,
                        "auto_started": True,
                        "auto_export": True,
                        "episode_id": payload.get("episode_id"),
                        "motion_context": payload,
                    }
                )
            if self._capture_mode != "skill_window":
                return self.status()
            if skill_name != self._capture_skill:
                return self.status()
            if self._capture_started_monotonic_s is not None:
                raise RuntimeError("a rollout action window is already being captured")

            cutoff = now_monotonic - self._pre_roll_s
            self._samples = [
                dict(sample)
                for sample in self._rolling_buffer
                if cutoff <= sample["timestamp_monotonic_s"] <= now_monotonic
            ]
            self._capture_started_monotonic_s = now_monotonic
            self._capture_finished_monotonic_s = None
            self._capture_deadline_monotonic_s = None
            self._capture_complete = False
            self._metadata.update(
                {
                    "capture_action_started_monotonic_s": now_monotonic,
                    "capture_action_finished_monotonic_s": None,
                    "capture_deadline_monotonic_s": None,
                    "capture_prebuffer_sample_count": len(self._samples),
                    "capture_action_status": "running",
                }
            )
            for key in (
                "episode_id",
                "step_id",
                "motion_name",
                "reference_distance_m",
                "chair_distance_m",
                "tag",
            ):
                if key in payload:
                    self._motion_context[key] = payload[key]
            self._motion_context["skill_name"] = skill_name
            return self.status()

    def finish_action(
        self,
        payload: Mapping[str, Any],
        *,
        timestamp_monotonic_s: float | None = None,
    ) -> dict[str, Any]:
        """Mark action completion and schedule export after at most 3 s post-roll."""
        skill_name = str(payload.get("skill_name", "")).strip().lower()
        now_monotonic = (
            time.monotonic()
            if timestamp_monotonic_s is None
            else float(timestamp_monotonic_s)
        )
        with self._lock:
            if (
                not self._active
                or self._capture_mode != "skill_window"
                or skill_name != self._capture_skill
                or self._capture_started_monotonic_s is None
            ):
                return self.status()
            if payload.get("motion_commanded") is False:
                self._cancel_auto_export_timer_locked()
                self._capture_complete = True
                self._active = False
                self._samples = []
                skipped = {
                    "ok": True,
                    "error_message": None,
                    "session_id": self._session_id,
                    "session_dir": (
                        str(self._session_dir)
                        if self._session_dir is not None
                        else None
                    ),
                    "sample_count": 0,
                    "export_status": "skipped_no_motion_commanded",
                    "action_status": str(payload.get("action_status", "rejected")),
                }
                self._last_export = skipped
                if self._session_dir is not None:
                    self._atomic_write_json(
                        self._session_dir / "manifest.json",
                        skipped,
                    )
                return self.status()
            now_monotonic = max(
                now_monotonic,
                self._capture_started_monotonic_s,
            )
            expected_duration_s = self._optional_finite_float(
                payload.get("duration_s")
            )
            if expected_duration_s is not None and expected_duration_s > 0.0:
                action_finished_monotonic_s = now_monotonic + expected_duration_s
                action_duration_source = "executed_arguments.duration_s_after_dispatch"
            else:
                action_finished_monotonic_s = now_monotonic
                action_duration_source = "execute_action_return"
            self._capture_finished_monotonic_s = action_finished_monotonic_s
            self._capture_deadline_monotonic_s = (
                action_finished_monotonic_s + self._post_roll_s
            )
            self._capture_complete = (
                self._capture_deadline_monotonic_s <= time.monotonic()
            )
            self._metadata.update(
                {
                    "capture_action_dispatch_return_monotonic_s": now_monotonic,
                    "capture_action_finished_monotonic_s": (
                        action_finished_monotonic_s
                    ),
                    "capture_action_duration_source": action_duration_source,
                    "capture_expected_duration_s": expected_duration_s,
                    "capture_deadline_monotonic_s": self._capture_deadline_monotonic_s,
                    "capture_action_status": str(
                        payload.get("action_status", "completed")
                    ),
                    "capture_motion_commanded": payload.get("motion_commanded"),
                    "capture_final_motion_context": dict(self._motion_context),
                }
            )
            self._cancel_auto_export_timer_locked()
            if self._auto_export:
                session_id = self._session_id
                auto_export_delay_s = max(
                    0.0,
                    self._capture_deadline_monotonic_s - time.monotonic(),
                )
                timer = threading.Timer(
                    auto_export_delay_s,
                    self._auto_finalize,
                    args=(session_id,),
                )
                timer.daemon = True
                self._auto_export_timer = timer
                timer.start()
            return self.status()

    def mark_action_dispatched(
        self,
        payload: Mapping[str, Any],
        *,
        timestamp_monotonic_s: float | None = None,
    ) -> dict[str, Any]:
        """Rebase the window to the actual reference-dispatch time."""
        skill_name = str(payload.get("skill_name", "")).strip().lower()
        now_monotonic = (
            time.monotonic()
            if timestamp_monotonic_s is None
            else float(timestamp_monotonic_s)
        )
        with self._lock:
            if (
                not self._active
                or self._capture_mode != "skill_window"
                or skill_name != self._capture_skill
                or self._capture_started_monotonic_s is None
                or self._capture_finished_monotonic_s is not None
            ):
                return self.status()
            cutoff = now_monotonic - self._pre_roll_s
            self._samples = [
                dict(sample)
                for sample in self._rolling_buffer
                if cutoff <= sample["timestamp_monotonic_s"] <= now_monotonic
            ]
            self._capture_started_monotonic_s = now_monotonic
            self._metadata.update(
                {
                    "capture_action_started_monotonic_s": now_monotonic,
                    "capture_start_source": "reference_dispatch",
                    "capture_prebuffer_sample_count": len(self._samples),
                }
            )
            for key in (
                "motion_name",
                "reference_distance_m",
                "chair_distance_m",
                "tag",
            ):
                if key in payload:
                    self._motion_context[key] = payload[key]
            return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            now_monotonic = time.monotonic()
            return {
                "ok": True,
                "error_message": self._last_error,
                "active": self._active,
                "session_id": self._session_id,
                "session_dir": str(self._session_dir) if self._session_dir is not None else None,
                "sample_count": len(self._samples),
                "camera_frame_count": len(self._camera_frames),
                "prebuffer_sample_count": len(self._rolling_buffer),
                "dropped_sample_count": self._dropped_samples,
                "elapsed_s": (
                    now_monotonic - self._started_monotonic_s
                    if self._active and self._started_monotonic_s is not None
                    else 0.0
                ),
                "localization": self._localization_status(self._latest_localization, now_monotonic),
                "motion_context": dict(self._motion_context),
                "chair_world_pose": self._json_safe_pose(self._chair_world_pose),
                "capture": {
                    "mode": self._capture_mode,
                    "skill": self._capture_skill,
                    "pre_roll_s": self._pre_roll_s,
                    "post_roll_s": self._post_roll_s,
                    "action_started": self._capture_started_monotonic_s is not None,
                    "action_finished": (
                        self._capture_finished_monotonic_s is not None
                        and now_monotonic >= self._capture_finished_monotonic_s
                    ),
                    "action_remaining_s": (
                        max(
                            0.0,
                            self._capture_finished_monotonic_s - now_monotonic,
                        )
                        if self._capture_finished_monotonic_s is not None
                        else None
                    ),
                    "complete": self._capture_complete,
                    "auto_started": self._auto_started,
                    "auto_export": self._auto_export,
                    "post_remaining_s": (
                        max(
                            0.0,
                            self._capture_deadline_monotonic_s
                            - max(
                                now_monotonic,
                                self._capture_finished_monotonic_s
                                or now_monotonic,
                            ),
                        )
                        if self._capture_deadline_monotonic_s is not None
                        else None
                    ),
                },
                "last_export": self._last_export,
            }

    def close(self) -> None:
        """Best-effort finalization so bridge shutdown does not lose samples."""
        with self._lock:
            if not self._active:
                return
            has_samples = bool(self._samples)
            session_dir = self._session_dir
        if has_samples:
            try:
                self.stop()
                return
            except Exception as exc:  # noqa: BLE001 - shutdown must continue.
                error_message = str(exc)
        else:
            error_message = "bridge closed before any valid g1_debug sample was recorded"
        with self._lock:
            self._cancel_auto_export_timer_locked()
            self._active = False
            self._last_error = error_message
        if session_dir is not None:
            self._atomic_write_json(
                session_dir / "manifest.json",
                {
                    "ok": False,
                    "error_message": error_message,
                    "session_id": session_dir.name,
                    "sample_count": 0 if not has_samples else len(self._samples),
                    "export_status": "aborted_during_bridge_close",
                },
            )

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def latest_localization(self) -> dict[str, Any] | None:
        """Return the latest valid, fresh external localization sample."""
        with self._lock:
            sample = self._latest_localization
            now_monotonic = time.monotonic()
            if (
                sample is None
                or not sample.valid
                or now_monotonic - sample.received_monotonic_s > self.localization_max_age_s
            ):
                return None
            return {
                "base_xyz": list(sample.base_xyz),
                "source": sample.source,
                "frame_id": sample.frame_id,
                "estimated": sample.estimated,
                "timestamp_s": sample.source_timestamp_s,
                "age_s": max(0.0, now_monotonic - sample.received_monotonic_s),
            }

    def _append_rolling_sample_locked(
        self,
        sample: dict[str, Any],
        now_monotonic_s: float,
    ) -> None:
        self._rolling_buffer.append(sample)
        cutoff = now_monotonic_s - MAX_ACTION_CONTEXT_S
        self._rolling_buffer = [
            buffered
            for buffered in self._rolling_buffer
            if buffered["timestamp_monotonic_s"] >= cutoff
        ]

    def _auto_finalize(self, session_id: str | None) -> None:
        with self._lock:
            if not self._active or self._session_id != session_id:
                return
            self._capture_complete = True
            options = dict(self._auto_export_options)
        try:
            self.stop(options)
        except Exception as exc:  # noqa: BLE001 - surface through rollout status.
            with self._lock:
                self._last_error = f"automatic rollout export failed: {exc}"

    def _cancel_auto_export_timer_locked(self) -> None:
        if self._auto_export_timer is not None:
            self._auto_export_timer.cancel()
            self._auto_export_timer = None

    def _export(
        self,
        *,
        samples: list[dict[str, Any]],
        session_dir: Path,
        metadata: dict[str, Any],
        chair_world_pose: dict[str, Any],
        camera_frames: list[dict[str, Any]],
        target_fps: float,
        smoothing_window: int,
        dropped_samples: int,
    ) -> dict[str, Any]:
        monotonic_s = np.asarray([sample["timestamp_monotonic_s"] for sample in samples], dtype=np.float64)
        relative_s = monotonic_s - monotonic_s[0]
        body_q = np.asarray([sample["body_q"] for sample in samples], dtype=np.float32)
        body_dq = np.asarray([sample["body_dq"] for sample in samples], dtype=np.float32)
        base_quat = np.asarray([sample["base_quat"] for sample in samples], dtype=np.float32)
        base_ang_vel = np.asarray([sample["base_ang_vel"] for sample in samples], dtype=np.float32)
        base_xyz = np.asarray([sample["base_xyz"] for sample in samples], dtype=np.float32)
        base_xyz_valid = np.asarray([sample["base_xyz_valid"] for sample in samples], dtype=np.bool_)
        body_q_target = np.asarray([sample["body_q_target"] for sample in samples], dtype=np.float32)
        policy_action = np.asarray([sample["policy_action"] for sample in samples], dtype=np.float32)
        action_started_s = self._optional_finite_float(
            metadata.get("capture_action_started_monotonic_s")
        )
        action_finished_s = self._optional_finite_float(
            metadata.get("capture_action_finished_monotonic_s")
        )
        capture_phase = self._capture_phases(
            monotonic_s,
            action_started_s,
            action_finished_s,
        )
        recorded_pre_roll_s = (
            max(0.0, action_started_s - float(monotonic_s[0]))
            if action_started_s is not None
            else 0.0
        )
        recorded_post_roll_s = (
            max(0.0, float(monotonic_s[-1]) - action_finished_s)
            if action_finished_s is not None
            else 0.0
        )

        metadata.update(
            {
                "stopped_at_utc": datetime.now(timezone.utc).isoformat(),
                "sample_count": len(samples),
                "dropped_sample_count": dropped_samples,
                "duration_s": float(relative_s[-1]),
                "source_rate_hz_estimate": self._estimate_rate_hz(monotonic_s),
                "base_xyz_valid_samples": int(base_xyz_valid.sum()),
                "base_xyz_all_valid": bool(base_xyz_valid.all()),
                "chair_world_pose": self._json_safe_pose(chair_world_pose),
                "capture_action_start_offset_s": (
                    action_started_s - float(monotonic_s[0])
                    if action_started_s is not None
                    else None
                ),
                "capture_action_end_offset_s": (
                    action_finished_s - float(monotonic_s[0])
                    if action_finished_s is not None
                    else None
                ),
                "recorded_pre_roll_s": recorded_pre_roll_s,
                "recorded_post_roll_s": recorded_post_roll_s,
            }
        )
        metadata_json = json.dumps(metadata, ensure_ascii=False, sort_keys=True)
        chair_arrays = self._chair_arrays(chair_world_pose)

        raw_path = session_dir / "raw_real_rollout.npz"
        self._atomic_savez(
            raw_path,
            schema_version=np.asarray(ROLLOUT_SCHEMA_VERSION),
            metadata_json=np.asarray(metadata_json),
            timestamp_monotonic_s=monotonic_s,
            timestamp_wall_s=np.asarray([sample["timestamp_wall_s"] for sample in samples], dtype=np.float64),
            timestamp_relative_s=relative_s,
            source_index=np.asarray([sample["source_index"] for sample in samples], dtype=np.int64),
            ros_timestamp_s=np.asarray([sample["ros_timestamp_s"] for sample in samples], dtype=np.float64),
            body_q=body_q,
            body_dq=body_dq,
            base_quat_wxyz=base_quat,
            base_ang_vel=base_ang_vel,
            base_xyz_world=base_xyz,
            base_xyz_valid=base_xyz_valid,
            base_xyz_source=np.asarray([sample["base_xyz_source"] for sample in samples], dtype=np.str_),
            base_xyz_frame_id=np.asarray([sample["base_xyz_frame_id"] for sample in samples], dtype=np.str_),
            base_xyz_estimated=np.asarray(
                [sample["base_xyz_estimated"] for sample in samples],
                dtype=np.bool_,
            ),
            localization_timestamp_s=np.asarray(
                [sample["localization_timestamp_s"] for sample in samples],
                dtype=np.float64,
            ),
            body_q_target=body_q_target,
            policy_action=policy_action,
            capture_phase=capture_phase,
            motion_name=np.asarray([sample["motion_name"] for sample in samples], dtype=np.str_),
            skill_name=np.asarray([sample["skill_name"] for sample in samples], dtype=np.str_),
            reference_distance_m=np.asarray(
                [sample["reference_distance_m"] for sample in samples],
                dtype=np.float32,
            ),
            chair_distance_m=np.asarray(
                [sample["chair_distance_m"] for sample in samples],
                dtype=np.float32,
            ),
            joint_order=np.asarray(G1_JOINT_ORDER, dtype=np.str_),
            **chair_arrays,
        )

        valid_frame_ids = {
            str(sample["base_xyz_frame_id"])
            for sample in samples
            if sample["base_xyz_valid"]
        }
        base_frame_id = next(iter(valid_frame_ids)) if len(valid_frame_ids) == 1 else ""
        spatial_replay_ready = bool(
            base_xyz_valid.all()
            and base_xyz_valid.size > 0
            and len(valid_frame_ids) == 1
        )
        chair_relative_replay_ready = bool(
            spatial_replay_ready
            and chair_world_pose.get("valid", False)
            and str(chair_world_pose.get("frame_id", "")) == base_frame_id
        )
        replay_path = session_dir / "3dgs_replay.npz"
        self._atomic_savez(
            replay_path,
            schema_version=np.asarray("groot_3dgs_replay_v1"),
            metadata_json=np.asarray(metadata_json),
            timestamp_s=relative_s,
            root_pos_world=base_xyz,
            root_pos_valid=base_xyz_valid,
            root_pos_source=np.asarray([sample["base_xyz_source"] for sample in samples], dtype=np.str_),
            root_pos_frame_id=np.asarray(
                [sample["base_xyz_frame_id"] for sample in samples],
                dtype=np.str_,
            ),
            root_pos_estimated=np.asarray(
                [sample["base_xyz_estimated"] for sample in samples],
                dtype=np.bool_,
            ),
            root_quat_wxyz=base_quat,
            joint_pos=body_q,
            capture_phase=capture_phase,
            joint_order=np.asarray(G1_JOINT_ORDER, dtype=np.str_),
            spatial_replay_ready=np.asarray(spatial_replay_ready, dtype=np.bool_),
            chair_relative_replay_ready=np.asarray(
                chair_relative_replay_ready,
                dtype=np.bool_,
            ),
            kinematic_replay_ready=np.asarray(True, dtype=np.bool_),
            **chair_arrays,
        )

        target_s = self._uniform_timestamps(relative_s, target_fps)
        reference_q = self._resample_matrix(relative_s, body_q, target_s)
        reference_q = self._moving_average(reference_q, smoothing_window)
        reference_dq = (
            np.gradient(reference_q, target_s, axis=0)
            if len(target_s) > 1
            else np.zeros_like(reference_q)
        )
        reference_quat = self._resample_quaternions(relative_s, base_quat, target_s)
        reference_root, reference_root_valid = self._resample_root(
            relative_s,
            base_xyz,
            base_xyz_valid,
            target_s,
        )
        reference_capture_phase = self._capture_phases(
            target_s + monotonic_s[0],
            action_started_s,
            action_finished_s,
        )

        reference_metadata = {
            **metadata,
            "schema_version": "gear_sonic_reference_candidate_v1",
            "target_fps": target_fps,
            "smoothing_window": smoothing_window,
            "resampling": "linear; quaternion sign-continuous normalized linear interpolation",
            "joint_velocity_source": "gradient_of_smoothed_resampled_joint_position",
            "contact_correction_applied": False,
            "ground_height_correction_applied": False,
            "physical_validation_required": True,
            "deployment_ready": False,
        }
        reference_path = session_dir / "gear_sonic_reference.npz"
        self._atomic_savez(
            reference_path,
            schema_version=np.asarray("gear_sonic_reference_candidate_v1"),
            metadata_json=np.asarray(json.dumps(reference_metadata, ensure_ascii=False, sort_keys=True)),
            joint_pos=reference_q.astype(np.float32),
            joint_vel=reference_dq.astype(np.float32),
            body_quat_w=reference_quat.astype(np.float32),
            root_pos_world=reference_root.astype(np.float32),
            root_pos_valid=reference_root_valid,
            frame_index=np.arange(len(target_s), dtype=np.int64),
            timestamp_s=target_s,
            capture_phase=reference_capture_phase,
            encode_mode=np.asarray(0, dtype=np.int32),
            motion_id=np.asarray(-1, dtype=np.int32),
            joint_order=np.asarray(G1_JOINT_ORDER, dtype=np.str_),
            physical_validation_required=np.asarray(True, dtype=np.bool_),
            contact_correction_applied=np.asarray(False, dtype=np.bool_),
        )

        export_warnings = self._export_warnings(
            base_xyz_valid,
            valid_frame_ids,
            chair_world_pose,
        )
        if not any(
            metadata.get(key)
            for key in ("checkpoint", "checkpoint_id", "checkpoint_hash")
        ):
            export_warnings.append(
                "checkpoint metadata was not supplied to /rollout/start"
            )
        camera_index_path = session_dir / "camera_index.json"
        self._atomic_write_json(
            camera_index_path,
            {
                "schema_version": "groot_rollout_camera_index_v1",
                "frame_count": len(camera_frames),
                "association": (
                    "bridge receive time paired with the triggering g1_debug sample; "
                    "camera timestamps are source-provided when available"
                ),
                "frames": camera_frames,
            },
        )
        manifest = {
            "ok": True,
            "error_message": None,
            "session_id": session_dir.name,
            "session_dir": str(session_dir),
            "sample_count": len(samples),
            "dropped_sample_count": dropped_samples,
            "duration_s": float(relative_s[-1]),
            "raw_real_rollout": str(raw_path),
            "3dgs_replay": str(replay_path),
            "gear_sonic_reference": str(reference_path),
            "camera_index": str(camera_index_path),
            "camera_frame_count": len(camera_frames),
            "kinematic_replay_ready": True,
            "spatial_3dgs_replay_ready": spatial_replay_ready,
            "chair_relative_replay_ready": chair_relative_replay_ready,
            "base_world_frame_id": base_frame_id or None,
            "base_xyz_valid_samples": int(base_xyz_valid.sum()),
            "base_xyz_total_samples": len(samples),
            "capture_phase_sample_count": {
                phase: int(np.count_nonzero(capture_phase == phase))
                for phase in ("pre", "action", "post", "continuous")
            },
            "recorded_pre_roll_s": recorded_pre_roll_s,
            "recorded_post_roll_s": recorded_post_roll_s,
            "gear_sonic_reference_status": "candidate_requires_physical_validation",
            "warnings": export_warnings,
        }
        self._atomic_write_json(session_dir / "manifest.json", manifest)
        return manifest

    @staticmethod
    def _required_vector(value: Any, size: int, name: str) -> list[float]:
        vector = RolloutRecorder._optional_vector(value, size)
        if not all(math.isfinite(item) for item in vector):
            raise ValueError(f"{name} must contain {size} finite values")
        return vector

    @staticmethod
    def _optional_vector(value: Any, size: int) -> list[float]:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != size:
            return [math.nan] * size
        try:
            vector = [float(item) for item in value]
        except (TypeError, ValueError):
            return [math.nan] * size
        return vector

    @staticmethod
    def _normalize_quaternion(quaternion: list[float]) -> list[float]:
        if not all(math.isfinite(value) for value in quaternion):
            raise ValueError("base_quat must contain 4 finite values")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1e-9:
            raise ValueError("base_quat norm is zero")
        return [value / norm for value in quaternion]

    def _parse_localization(
        self,
        payload: Mapping[str, Any],
        received_monotonic_s: float,
    ) -> LocalizationSample:
        base_xyz = self._required_vector(payload.get("base_xyz"), 3, "base_xyz")
        source = str(payload.get("source", "")).strip()
        if not source or source.lower() in {"none", "base_trans_measured", "g1_debug"}:
            raise ValueError(
                "localization source must identify external odometry/VIO/mocap/3DGS; "
                "g1_debug.base_trans_measured is a fixed placeholder"
            )
        frame_id = str(payload.get("frame_id", "world")).strip() or "world"
        source_timestamp_s = self._safe_float(payload.get("timestamp_s"), time.time())
        return LocalizationSample(
            base_xyz=(base_xyz[0], base_xyz[1], base_xyz[2]),
            source=source,
            frame_id=frame_id,
            valid=bool(payload.get("valid", True)),
            estimated=bool(payload.get("estimated", True)),
            source_timestamp_s=source_timestamp_s,
            received_monotonic_s=received_monotonic_s,
        )

    @staticmethod
    def _localization_status(
        sample: LocalizationSample | None,
        now_monotonic_s: float,
    ) -> dict[str, Any]:
        if sample is None:
            return {
                "available": False,
                "valid": False,
                "source": "none",
                "frame_id": None,
                "age_s": None,
            }
        return {
            "available": True,
            "valid": sample.valid,
            "source": sample.source,
            "frame_id": sample.frame_id,
            "estimated": sample.estimated,
            "age_s": max(0.0, now_monotonic_s - sample.received_monotonic_s),
        }

    @staticmethod
    def _empty_chair_pose() -> dict[str, Any]:
        return {
            "valid": False,
            "position_xyz": [math.nan, math.nan, math.nan],
            "orientation_wxyz": [math.nan, math.nan, math.nan, math.nan],
            "frame_id": "",
            "source": "none",
            "estimated": True,
        }

    def _normalize_chair_pose(self, value: Any, *, allow_empty: bool) -> dict[str, Any]:
        if value is None and allow_empty:
            return self._empty_chair_pose()
        if not isinstance(value, Mapping):
            raise ValueError("chair_world_pose must be an object")
        valid = bool(value.get("valid", True))
        if not valid:
            pose = self._empty_chair_pose()
            pose.update(
                {
                    "source": str(value.get("source", "none")),
                    "frame_id": str(value.get("frame_id", "")),
                    "estimated": bool(value.get("estimated", True)),
                }
            )
            return pose
        position = self._required_vector(value.get("position_xyz"), 3, "chair_world_pose.position_xyz")
        orientation = self._normalize_quaternion(
            self._required_vector(
                value.get("orientation_wxyz"),
                4,
                "chair_world_pose.orientation_wxyz",
            )
        )
        source = str(value.get("source", "")).strip()
        if not source:
            raise ValueError("chair_world_pose.source is required")
        return {
            "valid": True,
            "position_xyz": position,
            "orientation_wxyz": orientation,
            "frame_id": str(value.get("frame_id", "world")).strip() or "world",
            "source": source,
            "estimated": bool(value.get("estimated", True)),
        }

    @staticmethod
    def _chair_arrays(pose: Mapping[str, Any]) -> dict[str, np.ndarray]:
        return {
            "chair_world_position_xyz": np.asarray(pose["position_xyz"], dtype=np.float32),
            "chair_world_orientation_wxyz": np.asarray(pose["orientation_wxyz"], dtype=np.float32),
            "chair_world_pose_valid": np.asarray(bool(pose["valid"]), dtype=np.bool_),
            "chair_world_pose_frame_id": np.asarray(str(pose["frame_id"])),
            "chair_world_pose_source": np.asarray(str(pose["source"])),
            "chair_world_pose_estimated": np.asarray(bool(pose["estimated"]), dtype=np.bool_),
        }

    @staticmethod
    def _json_safe_pose(pose: Mapping[str, Any]) -> dict[str, Any]:
        result = dict(pose)
        for key in ("position_xyz", "orientation_wxyz"):
            result[key] = [
                float(value) if math.isfinite(float(value)) else None
                for value in pose.get(key, [])
            ]
        return result

    @staticmethod
    def _uniform_timestamps(source_s: np.ndarray, target_fps: float) -> np.ndarray:
        duration = float(source_s[-1])
        if len(source_s) == 1 or duration <= 0.0:
            return np.asarray([0.0], dtype=np.float64)
        count = max(2, int(math.floor(duration * target_fps)) + 1)
        return np.linspace(0.0, duration, count, dtype=np.float64)

    @staticmethod
    def _unique_time_rows(source_s: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if len(source_s) <= 1:
            return source_s, values
        keep = np.concatenate(([True], np.diff(source_s) > 1e-9))
        return source_s[keep], values[keep]

    @classmethod
    def _resample_matrix(
        cls,
        source_s: np.ndarray,
        values: np.ndarray,
        target_s: np.ndarray,
    ) -> np.ndarray:
        source_s, values = cls._unique_time_rows(source_s, values)
        if len(source_s) == 1:
            return np.repeat(values[:1], len(target_s), axis=0)
        columns = [
            np.interp(target_s, source_s, values[:, column])
            for column in range(values.shape[1])
        ]
        return np.stack(columns, axis=1)

    @classmethod
    def _resample_quaternions(
        cls,
        source_s: np.ndarray,
        values: np.ndarray,
        target_s: np.ndarray,
    ) -> np.ndarray:
        continuous = values.astype(np.float64, copy=True)
        for index in range(1, len(continuous)):
            if float(np.dot(continuous[index - 1], continuous[index])) < 0.0:
                continuous[index] *= -1.0
        result = cls._resample_matrix(source_s, continuous, target_s)
        norms = np.linalg.norm(result, axis=1, keepdims=True)
        norms[norms <= 1e-9] = 1.0
        return result / norms

    @classmethod
    def _resample_root(
        cls,
        source_s: np.ndarray,
        root_xyz: np.ndarray,
        valid: np.ndarray,
        target_s: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        valid_indices = np.flatnonzero(valid)
        result = np.full((len(target_s), 3), np.nan, dtype=np.float64)
        target_valid = np.zeros(len(target_s), dtype=np.bool_)
        if len(valid_indices) == 0:
            return result, target_valid
        segment_starts = np.concatenate(([0], np.flatnonzero(np.diff(valid_indices) > 1) + 1))
        segment_ends = np.concatenate((segment_starts[1:], [len(valid_indices)]))
        for segment_start, segment_end in zip(segment_starts, segment_ends, strict=True):
            segment_indices = valid_indices[segment_start:segment_end]
            valid_s = source_s[segment_indices]
            valid_xyz = root_xyz[segment_indices]
            valid_s, valid_xyz = cls._unique_time_rows(valid_s, valid_xyz)
            if len(valid_s) == 1:
                nearest = int(np.argmin(np.abs(target_s - valid_s[0])))
                result[nearest] = valid_xyz[0]
                target_valid[nearest] = True
                continue
            in_range = (target_s >= valid_s[0]) & (target_s <= valid_s[-1])
            result[in_range] = cls._resample_matrix(valid_s, valid_xyz, target_s[in_range])
            target_valid[in_range] = True
        return result, target_valid

    @staticmethod
    def _moving_average(values: np.ndarray, window: int) -> np.ndarray:
        if window <= 1 or len(values) <= 2:
            return values
        effective = min(window, len(values) if len(values) % 2 == 1 else len(values) - 1)
        if effective <= 1:
            return values
        pad = effective // 2
        padded = np.pad(values, ((pad, pad), (0, 0)), mode="edge")
        kernel = np.ones(effective, dtype=np.float64) / effective
        return np.stack(
            [np.convolve(padded[:, column], kernel, mode="valid") for column in range(values.shape[1])],
            axis=1,
        )

    @staticmethod
    def _estimate_rate_hz(timestamps: np.ndarray) -> float | None:
        if len(timestamps) < 2:
            return None
        deltas = np.diff(timestamps)
        deltas = deltas[deltas > 1e-9]
        if len(deltas) == 0:
            return None
        return float(1.0 / np.median(deltas))

    @staticmethod
    def _capture_phases(
        timestamps: np.ndarray,
        action_started_s: float | None,
        action_finished_s: float | None,
    ) -> np.ndarray:
        if action_started_s is None:
            return np.full(len(timestamps), "continuous", dtype="<U10")
        phases = np.full(len(timestamps), "action", dtype="<U10")
        phases[timestamps < action_started_s] = "pre"
        if action_finished_s is not None:
            phases[timestamps > action_finished_s] = "post"
        return phases

    def _capture_phase_for_timestamp_locked(self, timestamp_s: float) -> str:
        if self._capture_started_monotonic_s is None:
            return "continuous"
        if timestamp_s < self._capture_started_monotonic_s:
            return "pre"
        if (
            self._capture_finished_monotonic_s is not None
            and timestamp_s > self._capture_finished_monotonic_s
        ):
            return "post"
        return "action"

    @staticmethod
    def _camera_bytes(value: Any) -> bytes | None:
        if isinstance(value, bytes | bytearray):
            return bytes(value)
        if isinstance(value, str):
            try:
                return base64.b64decode(value, validate=True)
            except (ValueError, binascii.Error):
                return None
        return None

    @staticmethod
    def _json_safe_scalar(value: Any) -> float | str | None:
        if value is None:
            return None
        try:
            result = float(value)
        except (TypeError, ValueError):
            return str(value)
        return result if math.isfinite(result) else None

    @staticmethod
    def _export_warnings(
        base_xyz_valid: np.ndarray,
        base_frame_ids: set[str],
        chair_world_pose: Mapping[str, Any],
    ) -> list[str]:
        warnings: list[str] = []
        if not bool(base_xyz_valid.all()):
            warnings.append(
                "base_xyz is missing or stale for one or more frames; spatial 3DGS replay is not ready"
            )
        if len(base_frame_ids) > 1:
            warnings.append(
                "base_xyz frames use different frame_id values; transform them into one world frame"
            )
        if not bool(chair_world_pose.get("valid", False)):
            warnings.append("chair_world_pose is missing; chair-relative scene replay is not ready")
        elif len(base_frame_ids) == 1 and str(chair_world_pose.get("frame_id", "")) not in base_frame_ids:
            warnings.append(
                "chair_world_pose.frame_id does not match base_xyz frame_id; "
                "chair-relative replay is not ready"
            )
        warnings.append(
            "gear_sonic_reference.npz is a smoothed/resampled candidate; "
            "contact correction and physical validation are still required"
        )
        return warnings

    @staticmethod
    def _atomic_savez(path: Path, **arrays: np.ndarray) -> None:
        temporary = path.with_name(f".{path.name}.tmp")
        with temporary.open("wb") as handle:
            np.savez_compressed(handle, **arrays)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)

    @staticmethod
    def _atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
        temporary = path.with_name(f".{path.name}.tmp")
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)

    @staticmethod
    def _atomic_write_bytes(path: Path, payload: bytes) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.tmp")
        with temporary.open("wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)

    @staticmethod
    def _git_commit() -> str | None:
        try:
            result = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=REPO_ROOT,
                check=True,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        return result.stdout.strip() or None

    @staticmethod
    def _sanitize_name(value: str) -> str:
        sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("._")
        return (sanitized or "rollout")[:80]

    @staticmethod
    def _safe_float(value: Any, default: float) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return default
        return result if math.isfinite(result) else default

    @staticmethod
    def _optional_finite_float(value: Any) -> float | None:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    @staticmethod
    def _safe_int(value: Any, default: int) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _bounded_float(value: Any, minimum: float, maximum: float, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a number") from exc
        if not minimum <= result <= maximum:
            raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
        return result

    @staticmethod
    def _bounded_int(value: Any, minimum: int, maximum: int, name: str) -> int:
        try:
            result = int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be an integer") from exc
        if not minimum <= result <= maximum:
            raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
        return result
