"""Read-only diagnostics for FaceE sit-chair reference execution."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from gear_sonic.vigil_bridge.reference_motion import (
    STREAMED_REFERENCE_TERMINAL_HOLD_FRAMES,
)
from gear_sonic.vigil_bridge.rollout_recorder import G1_JOINT_ORDER


# C++ policy_parameters.hpp: for each MuJoCo-order output joint, select the
# corresponding IsaacLab-order reference joint.
ISAACLAB_TO_MUJOCO: tuple[int, ...] = (
    0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8,
    11, 15, 19, 22, 25, 27, 28, 12, 16, 20, 23, 26, 21, 24,
)


def diagnose_preflight(
    *,
    executor: Any,
    state_response: Mapping[str, Any],
    request: Mapping[str, Any],
) -> dict[str, Any]:
    """Compare live robot state with the selected reference without motion."""
    catalog = getattr(executor, "chair_motion_catalog", None)
    if catalog is None:
        raise ValueError("chair motion catalog is not configured")
    if "chair_distance_m" not in request:
        raise ValueError("chair_distance_m is required")

    motion = catalog.load(request["chair_distance_m"])
    robot_state = _mapping(state_response.get("robot_state"))
    joint_positions = _mapping(robot_state.get("joint_positions"))
    joint_velocities = _mapping(robot_state.get("joint_velocities"))
    measured = _joint_vector(joint_positions, "joint_positions")
    measured_dq = _joint_vector(joint_velocities, "joint_velocities")

    reference_source = np.asarray(motion.frames["joint_pos"], dtype=np.float64)
    reference_first = reference_source[0, list(ISAACLAB_TO_MUJOCO)]
    error = measured - reference_first
    abs_error = np.abs(error)
    rms_rad = float(np.sqrt(np.mean(error * error)))
    max_index = int(np.argmax(abs_error))
    max_abs_dq = float(np.max(np.abs(measured_dq)))

    rms_limit = _positive_float(request.get("initial_rms_limit_rad", 0.20), "initial_rms_limit_rad")
    max_limit = _positive_float(request.get("initial_max_limit_rad", 0.50), "initial_max_limit_rad")
    dq_limit = _positive_float(request.get("stationary_dq_limit_rad_s", 0.50), "stationary_dq_limit_rad_s")
    telemetry = _mapping(state_response.get("telemetry"))
    runtime_ready = bool(
        state_response.get("ok", False)
        and telemetry.get("state_connected", True)
        and telemetry.get("ready_for_motion", True)
    )
    initial_pose_ok = rms_rad <= rms_limit and float(abs_error[max_index]) <= max_limit
    stationary_ok = max_abs_dq <= dq_limit

    top_indices = np.argsort(abs_error)[-8:][::-1]
    checks = {
        "runtime_ready": runtime_ready,
        "initial_pose_close_to_reference": initial_pose_ok,
        "robot_stationary": stationary_ok,
        "catalog_contract": True,
    }
    warnings: list[str] = []
    if not runtime_ready:
        warnings.append("runtime/state/camera is not ready for motion")
    if not initial_pose_ok:
        warnings.append("measured joints are not close enough to reference frame 0")
    if not stationary_ok:
        warnings.append("robot joint velocity is above the stationary diagnostic threshold")

    return {
        "ok": True,
        "error_message": None,
        "diagnostic": "sonic.sit_chair_preflight",
        "motion_commanded": False,
        "ready": all(checks.values()),
        "checks": checks,
        "warnings": warnings,
        "selected_reference": {
            "requested_distance_m": motion.requested_distance_m,
            "reference_distance_m": motion.reference_distance_m,
            "tag": motion.tag,
            "motion_name": motion.motion_name,
            "source_frame_count": int(reference_source.shape[0]),
            "terminal_hold_frame_count": STREAMED_REFERENCE_TERMINAL_HOLD_FRAMES,
            "transmitted_frame_count": int(
                reference_source.shape[0] + STREAMED_REFERENCE_TERMINAL_HOLD_FRAMES
            ),
            "duration_s": motion.duration_s,
            "encode_mode": int(motion.frames.get("encode_mode", 0)),
            "motion_id": int(motion.frames.get("motion_id", -1)),
        },
        "initial_alignment": {
            "rms_rad": rms_rad,
            "max_abs_rad": float(abs_error[max_index]),
            "max_abs_joint": G1_JOINT_ORDER[max_index],
            "max_abs_dq_rad_s": max_abs_dq,
            "limits": {
                "rms_rad": rms_limit,
                "max_abs_rad": max_limit,
                "stationary_dq_rad_s": dq_limit,
            },
            "top_joint_errors": [
                {
                    "joint": G1_JOINT_ORDER[int(index)],
                    "error_rad": float(error[index]),
                    "abs_error_rad": float(abs_error[index]),
                    "measured_rad": float(measured[index]),
                    "reference_frame0_rad": float(reference_first[index]),
                }
                for index in top_indices
            ],
        },
        "model_contract": {
            "expected_checkpoint": "policy/facee_v73_noheight/model",
            "expected_observation_config": (
                "policy/facee_v73_noheight/observation_config.yaml"
            ),
            "runtime_verification": "inspect launcher policy.log; paths are not present in g1_debug",
        },
    }


def analyze_rollout(session_dir: Path) -> dict[str, Any]:
    """Classify transition, tracking, timing, and tilt evidence in one rollout."""
    raw_path = session_dir / "raw_real_rollout.npz"
    if not raw_path.is_file():
        raise ValueError(f"rollout export not found: {raw_path}")

    with np.load(raw_path, allow_pickle=True) as archive:
        metadata = json.loads(str(archive["metadata_json"].item()))
        timestamps = np.asarray(archive["timestamp_relative_s"], dtype=np.float64)
        phase = np.asarray(archive["capture_phase"]).astype(str)
        body_q = np.asarray(archive["body_q"], dtype=np.float64)
        target_q = np.asarray(archive["body_q_target"], dtype=np.float64)
        base_quat = np.asarray(archive["base_quat_wxyz"], dtype=np.float64)
        joint_order = tuple(str(value) for value in archive["joint_order"])
        source_index = (
            np.asarray(archive["source_index"], dtype=np.int64)
            if "source_index" in archive.files
            else np.arange(len(timestamps), dtype=np.int64)
        )

    action = phase == "action"
    pre = phase == "pre"
    if not np.any(action):
        raise ValueError("rollout has no action samples")
    action_indices = np.flatnonzero(action)
    action_start = float(timestamps[action_indices[0]])
    action_end = float(timestamps[action_indices[-1]])
    early = action & (timestamps <= action_start + 0.5)
    late = action & (timestamps >= action_end - 1.0)

    per_sample_error = np.sqrt(np.nanmean((body_q - target_q) ** 2, axis=1))
    per_joint_action = np.sqrt(np.nanmean((body_q[action] - target_q[action]) ** 2, axis=0))
    top_joint_indices = np.argsort(np.nan_to_num(per_joint_action, nan=-1.0))[-8:][::-1]
    tilt_deg = _tilt_degrees(base_quat)
    source_rate_hz = float(metadata.get("source_rate_hz_estimate", math.nan))
    expected_duration_s = float(metadata.get("capture_expected_duration_s", math.nan))
    action_duration_s = max(action_end - action_start, 0.0)

    windows = {
        "pre": _window_metrics(per_sample_error, tilt_deg, pre),
        "early_0p5s": _window_metrics(per_sample_error, tilt_deg, early),
        "full_action": _window_metrics(per_sample_error, tilt_deg, action),
        "late_1s": _window_metrics(per_sample_error, tilt_deg, late),
    }
    tracking_threshold = _baseline_threshold(
        per_sample_error[pre],
        absolute_floor=0.25,
        minimum_margin=0.08,
    )
    tilt_threshold = _baseline_threshold(
        tilt_deg[pre],
        absolute_floor=20.0,
        minimum_margin=5.0,
    )
    timeline_onsets: list[dict[str, Any]] = []
    tracking_onset = _sustained_onset(
        values=per_sample_error,
        action_mask=action,
        threshold=tracking_threshold,
        timestamps=timestamps,
        action_start=action_start,
        source_index=source_index,
        body_q=body_q,
        target_q=target_q,
        joint_order=joint_order,
    )
    if tracking_onset is not None:
        tracking_onset["signal"] = "joint_tracking_rms"
        tracking_onset["unit"] = "rad"
        timeline_onsets.append(tracking_onset)
    tilt_onset = _sustained_onset(
        values=tilt_deg,
        action_mask=action,
        threshold=tilt_threshold,
        timestamps=timestamps,
        action_start=action_start,
        source_index=source_index,
    )
    if tilt_onset is not None:
        tilt_onset["signal"] = "base_tilt"
        tilt_onset["unit"] = "deg"
        timeline_onsets.append(tilt_onset)
    for onset in timeline_onsets:
        nearest_observations = _nearest_camera_observations(
            session_dir,
            int(onset["source_index"]),
        )
        if nearest_observations:
            onset["nearest_observations"] = nearest_observations
    timeline_onsets.sort(key=lambda item: float(item["action_time_s"]))
    first_anomaly = timeline_onsets[0] if timeline_onsets else None
    findings: list[dict[str, Any]] = []
    pre_rms = windows["pre"]["tracking_rms_rad"]
    early_rms = windows["early_0p5s"]["tracking_rms_rad"]
    late_rms = windows["late_1s"]["tracking_rms_rad"]
    full_tilt = windows["full_action"]["max_tilt_deg"]

    if early_rms is not None and (
        early_rms >= 0.25
        or (pre_rms is not None and early_rms >= max(pre_rms * 1.5, pre_rms + 0.08))
    ):
        findings.append(
            {
                "code": "start_transition_mismatch",
                "priority": "high",
                "evidence": {
                    "pre_tracking_rms_rad": pre_rms,
                    "early_tracking_rms_rad": early_rms,
                },
                "interpretation": "reference start/pose transition is a leading suspect",
            }
        )
    if late_rms is not None and early_rms is not None and late_rms > max(early_rms * 1.35, early_rms + 0.10):
        findings.append(
            {
                "code": "tracking_divergence",
                "priority": "high",
                "evidence": {
                    "early_tracking_rms_rad": early_rms,
                    "late_tracking_rms_rad": late_rms,
                },
                "interpretation": "tracking error grows during execution",
            }
        )
    if math.isfinite(expected_duration_s) and action_duration_s < expected_duration_s - 0.5:
        findings.append(
            {
                "code": "execution_window_short",
                "priority": "high",
                "evidence": {
                    "expected_duration_s": expected_duration_s,
                    "recorded_action_duration_s": action_duration_s,
                },
                "interpretation": "streamed motion may have stopped before its declared duration",
            }
        )
    if math.isfinite(source_rate_hz) and not 45.0 <= source_rate_hz <= 55.0:
        findings.append(
            {
                "code": "state_rate_mismatch",
                "priority": "medium",
                "evidence": {"source_rate_hz": source_rate_hz},
                "interpretation": "g1_debug sampling rate is outside the expected 50 Hz range",
            }
        )
    if full_tilt is not None and full_tilt >= 25.0:
        findings.append(
            {
                "code": "large_body_tilt",
                "priority": "high",
                "evidence": {"max_tilt_deg": full_tilt},
                "interpretation": "rollout contains a large non-yaw base orientation change",
            }
        )
    if not findings:
        findings.append(
            {
                "code": "no_single_dominant_runtime_fault",
                "priority": "info",
                "evidence": {},
                "interpretation": "inspect chair-relative geometry and camera frames next",
            }
        )

    return {
        "ok": True,
        "error_message": None,
        "diagnostic": "sonic.sit_chair_rollout_analysis",
        "session_id": session_dir.name,
        "runtime_mode": metadata.get("runtime_mode"),
        "motion_context": metadata.get("capture_final_motion_context", {}),
        "sample_count": int(len(timestamps)),
        "action_sample_count": int(np.count_nonzero(action)),
        "action_duration_s": action_duration_s,
        "expected_duration_s": expected_duration_s if math.isfinite(expected_duration_s) else None,
        "source_rate_hz": source_rate_hz if math.isfinite(source_rate_hz) else None,
        "windows": windows,
        "anomaly_timeline": {
            "method": (
                "first 5 consecutive 50 Hz samples above a robust pre-action "
                "baseline threshold"
            ),
            "required_consecutive_samples": 5,
            "thresholds": {
                "joint_tracking_rms_rad": tracking_threshold,
                "base_tilt_deg": tilt_threshold,
            },
            "first_anomaly": first_anomaly,
            "onsets": timeline_onsets,
        },
        "top_action_tracking_joints": [
            {
                "joint": joint_order[int(index)],
                "rms_rad": float(per_joint_action[index]),
            }
            for index in top_joint_indices
            if math.isfinite(float(per_joint_action[index]))
        ],
        "findings": findings,
    }


def select_rollout_session(output_root: Path, requested: Any = None) -> Path:
    """Resolve one session below the configured rollout root."""
    root = output_root.resolve()
    if requested is not None and str(requested).strip():
        name = Path(str(requested).strip()).name
        candidate = (root / name).resolve()
        if candidate.parent != root:
            raise ValueError("rollout session must be a direct child of the output directory")
        return candidate
    candidates = [path for path in root.iterdir() if path.is_dir()]
    if not candidates:
        raise ValueError(f"no rollout session found under {root}")
    latest = max(candidates, key=lambda path: path.stat().st_mtime)
    if not (latest / "raw_real_rollout.npz").is_file():
        raise ValueError(
            f"latest rollout {latest.name!r} is not exported yet; "
            "wait for the action and 3 s post-roll to finish"
        )
    return latest


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _joint_vector(values: Mapping[str, Any], name: str) -> np.ndarray:
    if any(joint not in values for joint in G1_JOINT_ORDER):
        raise ValueError(f"{name} does not contain all 29 G1 joints")
    result = np.asarray([values[joint] for joint in G1_JOINT_ORDER], dtype=np.float64)
    if result.shape != (29,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain 29 finite values")
    return result


def _positive_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return result


def _tilt_degrees(quat_wxyz: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quat_wxyz, axis=1, keepdims=True)
    normalized = np.divide(
        quat_wxyz,
        norm,
        out=np.full_like(quat_wxyz, np.nan),
        where=norm > 1e-9,
    )
    up_z = 1.0 - 2.0 * (normalized[:, 1] ** 2 + normalized[:, 2] ** 2)
    return np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0)))


def _window_metrics(
    tracking_error: np.ndarray,
    tilt_deg: np.ndarray,
    mask: np.ndarray,
) -> dict[str, Any]:
    if not np.any(mask):
        return {
            "sample_count": 0,
            "tracking_rms_rad": None,
            "tracking_max_rad": None,
            "max_tilt_deg": None,
        }
    values = tracking_error[mask]
    tilts = tilt_deg[mask]
    return {
        "sample_count": int(np.count_nonzero(mask)),
        "tracking_rms_rad": float(np.nanmean(values)),
        "tracking_max_rad": float(np.nanmax(values)),
        "max_tilt_deg": float(np.nanmax(tilts)),
    }


def _baseline_threshold(
    baseline: np.ndarray,
    *,
    absolute_floor: float,
    minimum_margin: float,
) -> float:
    finite = baseline[np.isfinite(baseline)]
    if finite.size == 0:
        return absolute_floor
    median = float(np.median(finite))
    mad = float(np.median(np.abs(finite - median)))
    robust_margin = max(minimum_margin, 6.0 * 1.4826 * mad)
    return max(absolute_floor, median + robust_margin)


def _sustained_onset(
    *,
    values: np.ndarray,
    action_mask: np.ndarray,
    threshold: float,
    timestamps: np.ndarray,
    action_start: float,
    source_index: np.ndarray,
    body_q: np.ndarray | None = None,
    target_q: np.ndarray | None = None,
    joint_order: tuple[str, ...] | None = None,
    required_samples: int = 5,
) -> dict[str, Any] | None:
    candidates = action_mask & np.isfinite(values) & (values > threshold)
    run = 0
    onset_index: int | None = None
    for index, candidate in enumerate(candidates):
        run = run + 1 if candidate else 0
        if run >= required_samples:
            onset_index = index - required_samples + 1
            break
    if onset_index is None:
        return None

    result: dict[str, Any] = {
        "action_time_s": float(timestamps[onset_index] - action_start),
        "rollout_time_s": float(timestamps[onset_index]),
        "source_index": int(source_index[onset_index]),
        "value": float(values[onset_index]),
        "threshold": float(threshold),
        "sustained_for_samples": required_samples,
    }
    if body_q is not None and target_q is not None and joint_order is not None:
        joint_error = np.abs(body_q[onset_index] - target_q[onset_index])
        top = np.argsort(np.nan_to_num(joint_error, nan=-1.0))[-6:][::-1]
        result["top_joint_errors"] = [
            {
                "joint": joint_order[int(index)],
                "abs_error_rad": float(joint_error[index]),
                "measured_rad": float(body_q[onset_index, index]),
                "target_rad": float(target_q[onset_index, index]),
            }
            for index in top
            if math.isfinite(float(joint_error[index]))
        ]
    return result


def _nearest_camera_observations(
    session_dir: Path,
    pose_source_index: int,
) -> dict[str, dict[str, Any]]:
    index_path = session_dir / "camera_index.json"
    if not index_path.is_file():
        return {}
    try:
        payload = json.loads(index_path.read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return {}
    frames = payload.get("frames")
    if not isinstance(frames, list):
        return {}

    nearest: dict[str, tuple[int, Mapping[str, Any]]] = {}
    for frame in frames:
        if not isinstance(frame, Mapping):
            continue
        camera_name = str(frame.get("camera_name", "")).strip()
        try:
            frame_source_index = int(frame["pose_source_index"])
        except (KeyError, TypeError, ValueError):
            continue
        if not camera_name:
            continue
        distance = abs(frame_source_index - pose_source_index)
        current = nearest.get(camera_name)
        if current is None or distance < current[0]:
            nearest[camera_name] = (distance, frame)

    result: dict[str, dict[str, Any]] = {}
    for camera_name, (distance, frame) in nearest.items():
        relative_path = str(frame.get("path", "")).strip()
        result[camera_name] = {
            "path": (
                str((session_dir / relative_path).resolve())
                if relative_path
                else None
            ),
            "frame_index": frame.get("frame_index"),
            "pose_source_index": int(frame["pose_source_index"]),
            "source_index_delta": distance,
            "capture_phase": frame.get("capture_phase"),
            "camera_timestamp": frame.get("camera_timestamp"),
            "received_wall_s": frame.get("received_wall_s"),
        }
    return result
