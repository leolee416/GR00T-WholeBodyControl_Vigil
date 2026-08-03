"""FaceE backless-chair placement for the deployment MuJoCo scene."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from gear_sonic.vigil_bridge.chair_motion_catalog import (
    EXACT_V3_CATALOG,
    select_chair_reference_distance_m,
)


FACEE_CHAIR_BODY = "facee_chair"

# Keep this simulation-only reset pose byte-for-byte aligned with
# gear_sonic_deploy/.../include/policy_parameters.hpp::default_angles.  The
# repository-mode validation intentionally exercises the real C++ INIT state,
# which ramps to this pose before accepting a streamed reference.
FACEE_DEPLOY_DEFAULT_ANGLES_MUJOCO = np.asarray(
    [
        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
        0.0, 0.0, 0.0,
        0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
        0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class FaceEChairLayout:
    requested_distance_m: float
    reference_distance_m: float
    tag: str
    center_xy_m: tuple[float, float]
    center_distance_m: float
    yaw_rad: float
    source_root_world_xy_m: tuple[float, float]
    source_root_world_yaw_rad: float
    source_chair_world_xy_m: tuple[float, float]
    source_chair_world_yaw_rad: float
    seat_height_m: float
    width_m: float
    depth_m: float
    backrest: bool
    catalog_path: Path
    motion_path: Path


@dataclass(frozen=True)
class FaceEDeployReset:
    root_height_m: float
    requested_ground_clearance_m: float
    measured_ground_clearance_m: float
    closest_robot_geom: str


@dataclass(frozen=True)
class FaceEReferenceReset:
    tag: str
    root_height_m: float
    root_quaternion_wxyz: tuple[float, float, float, float]
    initial_body_qpos_mujoco: tuple[float, ...]
    initial_body_qvel_mujoco: tuple[float, ...]
    source_motion_path: Path


def resolve_facee_catalog_path(
    catalog_path: str | Path | None,
    repo_root: str | Path | None = None,
) -> Path:
    path = Path(catalog_path) if catalog_path else EXACT_V3_CATALOG
    path = path.expanduser()
    if not path.is_absolute():
        if repo_root is None:
            raise ValueError("repo_root is required for a relative FaceE catalog path")
        path = Path(repo_root) / path
    return path.resolve()


def load_facee_chair_layout(
    distance_m: Any,
    catalog_path: str | Path | None = None,
    repo_root: str | Path | None = None,
) -> FaceEChairLayout:
    requested = float(distance_m)
    selected, key = select_chair_reference_distance_m(distance_m)
    manifest_path = resolve_facee_catalog_path(catalog_path, repo_root)
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    geometry = payload.get("chair_geometry")
    if payload.get("schema_version") != 2 or not isinstance(geometry, dict):
        raise ValueError("FaceE scene requires an exact-v3 chair catalog")
    records = {
        round(float(record["distance_m"]) * 100): record
        for record in payload.get("motions", [])
    }
    if set(records) != set(range(115, 201, 5)):
        raise ValueError("FaceE scene catalog must contain the complete 1.15..2.00 grid")
    record = records[key]
    layout = record.get("chair_layout")
    if not isinstance(layout, dict) or layout.get("frame") != "initial_robot_heading":
        raise ValueError(f"{record.get('tag')}: missing initial-robot-frame chair layout")
    center_xy = tuple(float(value) for value in layout["center_xy_m"])
    if len(center_xy) != 2 or not np.isfinite(center_xy).all():
        raise ValueError(f"{record.get('tag')}: invalid chair center")
    result = FaceEChairLayout(
        requested_distance_m=requested,
        reference_distance_m=selected,
        tag=str(record["tag"]),
        center_xy_m=center_xy,
        center_distance_m=float(layout["center_distance_m"]),
        yaw_rad=float(layout["yaw_rad"]),
        source_root_world_xy_m=tuple(
            float(value) for value in layout["source_root_world_xy_m"]
        ),
        source_root_world_yaw_rad=float(layout["source_root_world_yaw_rad"]),
        source_chair_world_xy_m=tuple(
            float(value) for value in layout["source_chair_world_xy_m"]
        ),
        source_chair_world_yaw_rad=float(layout["source_chair_world_yaw_rad"]),
        seat_height_m=float(layout["seat_height_m"]),
        width_m=float(layout["width_m"]),
        depth_m=float(layout["depth_m"]),
        backrest=bool(layout["backrest"]),
        catalog_path=manifest_path,
        motion_path=(manifest_path.parent / str(record["file"])).resolve(),
    )
    if result.backrest:
        raise ValueError(f"{result.tag}: FaceE deployment scene must be backless")
    if not math.isclose(result.seat_height_m, 0.41, abs_tol=1e-9):
        raise ValueError(f"{result.tag}: FaceE seat top must be 0.41 m")
    if not math.isclose(result.width_m, 0.5, abs_tol=1e-9):
        raise ValueError(f"{result.tag}: FaceE chair width must be 0.50 m")
    if not math.isclose(result.depth_m, 0.45, abs_tol=1e-9):
        raise ValueError(f"{result.tag}: FaceE chair depth must be 0.45 m")
    return result


def apply_facee_chair_layout(
    model: mujoco.MjModel,
    layout: FaceEChairLayout,
    body_name: str = FACEE_CHAIR_BODY,
    preserve_authored_world_frame: bool = False,
) -> None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(
            f"FaceE distance configured but MuJoCo scene has no body {body_name!r}"
        )
    center_xy = (
        layout.source_chair_world_xy_m
        if preserve_authored_world_frame
        else layout.center_xy_m
    )
    yaw = (
        layout.source_chair_world_yaw_rad
        if preserve_authored_world_frame
        else layout.yaw_rad
    )
    model.body_pos[body_id] = [center_xy[0], center_xy[1], 0.0]
    half_yaw = 0.5 * yaw
    model.body_quat[body_id] = [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)]


def compile_facee_scene_model(
    scene_path: str | Path,
    layout: FaceEChairLayout,
    body_name: str = FACEE_CHAIR_BODY,
    preserve_authored_world_frame: bool = False,
) -> mujoco.MjModel:
    """Compile a FaceE scene with the static chair already at its final pose.

    Moving a massless/static body by assigning ``MjModel.body_pos`` after XML
    compilation bypasses the normal model compilation path for derived static
    geometry and broad-phase data.  It can therefore alter contact ordering at
    reset and miss the chair later in the rollout.  Editing the spec before
    compilation keeps the complete MuJoCo model internally consistent.
    """

    path = Path(scene_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    spec = mujoco.MjSpec.from_file(str(path))
    body = spec.body(body_name)
    if body is None:
        raise ValueError(
            f"FaceE distance configured but MuJoCo scene has no body {body_name!r}"
        )
    center_xy = (
        layout.source_chair_world_xy_m
        if preserve_authored_world_frame
        else layout.center_xy_m
    )
    yaw = (
        layout.source_chair_world_yaw_rad
        if preserve_authored_world_frame
        else layout.yaw_rad
    )
    body.pos = np.asarray([center_xy[0], center_xy[1], 0.0], dtype=np.float64)
    half_yaw = 0.5 * yaw
    body.quat = np.asarray(
        [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)], dtype=np.float64
    )
    return spec.compile()


def _descends_from(model: mujoco.MjModel, body_id: int, root_body_id: int) -> bool:
    current = int(body_id)
    while current > 0:
        if current == root_body_id:
            return True
        current = int(model.body_parentid[current])
    return False


def _minimum_robot_floor_distance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    root_body_name: str = "pelvis",
    floor_geom_name: str = "floor",
) -> tuple[float, int]:
    root_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, root_body_name
    )
    floor_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, floor_geom_name
    )
    if root_body_id < 0 or floor_geom_id < 0:
        raise ValueError("FaceE deploy reset requires pelvis and floor")
    robot_geoms = [
        geom_id
        for geom_id, body_id in enumerate(model.geom_bodyid)
        if geom_id != floor_geom_id
        and _descends_from(model, int(body_id), root_body_id)
        and (
            int(model.geom_contype[geom_id]) != 0
            or int(model.geom_conaffinity[geom_id]) != 0
        )
    ]
    if not robot_geoms:
        raise RuntimeError("FaceE deploy reset found no collidable robot geometry")
    fromto = np.zeros(6, dtype=np.float64)
    distances = np.asarray(
        [
            mujoco.mj_geomDistance(
                model, data, floor_geom_id, geom_id, 2.0, fromto
            )
            for geom_id in robot_geoms
        ],
        dtype=np.float64,
    )
    closest = int(np.argmin(distances))
    return float(distances[closest]), int(robot_geoms[closest])


def configure_facee_deploy_reset(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_joint_ids: np.ndarray,
    ground_clearance_m: float = 0.001,
) -> FaceEDeployReset:
    """Set qpos0 to the C++ deploy pose and place its lowest geom at the floor.

    This is a repository/deployment startup contract, not the E0019 evaluator's
    authored-reference reset.  Updating ``qpos0`` makes explicit/manual resets
    return to the same deploy state as the initial launch.
    """

    clearance = float(ground_clearance_m)
    if not math.isfinite(clearance) or clearance < 0.0 or clearance > 0.02:
        raise ValueError("FaceE ground clearance must be within [0, 0.02] m")
    joint_ids = np.asarray(body_joint_ids, dtype=np.int64)
    if joint_ids.shape != (29,):
        raise ValueError(f"FaceE deploy reset requires 29 joint ids, got {joint_ids.shape}")
    qpos_indices = np.asarray(model.jnt_qposadr[joint_ids], dtype=np.int64)
    model.qpos0[0:2] = 0.0
    model.qpos0[3:7] = [1.0, 0.0, 0.0, 0.0]
    model.qpos0[qpos_indices] = FACEE_DEPLOY_DEFAULT_ANGLES_MUJOCO
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    baseline_distance, _ = _minimum_robot_floor_distance(model, data)
    # All robot geoms translate rigidly with the free root, so this one exact
    # vertical shift places the lowest geom at the requested clearance.
    model.qpos0[2] += clearance - baseline_distance
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    final_distance, closest_geom_id = _minimum_robot_floor_distance(model, data)
    if not math.isclose(final_distance, clearance, abs_tol=2e-6):
        raise AssertionError(
            "FaceE deploy reset grounding mismatch: "
            f"expected={clearance:.9f}, measured={final_distance:.9f}"
        )
    closest_name = mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_GEOM, closest_geom_id
    ) or f"geom_{closest_geom_id}"
    return FaceEDeployReset(
        root_height_m=float(model.qpos0[2]),
        requested_ground_clearance_m=clearance,
        measured_ground_clearance_m=final_distance,
        closest_robot_geom=closest_name,
    )


def _quat_multiply_wxyz(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.asarray(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )


def _quat_rotation_matrix_wxyz(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def configure_facee_reference_reset(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_joint_ids: np.ndarray,
    layout: FaceEChairLayout,
    preserve_authored_world_frame: bool = False,
) -> FaceEReferenceReset:
    """Reset to the selected E0019 reference frame zero in robot-heading frame."""

    if not layout.motion_path.is_file():
        raise FileNotFoundError(layout.motion_path)
    with np.load(layout.motion_path, allow_pickle=False) as motion:
        required = {
            "joint_pos_mujoco",
            "joint_vel_mujoco",
            "root_pos",
            "root_lin_vel_w",
            "root_ang_vel_w",
            "reset_root_quat_w",
        }
        missing = sorted(required - set(motion.files))
        if missing:
            raise ValueError(
                f"{layout.tag}: catalog must be rebuilt with reset state fields: {missing}"
            )
        joint_pos = np.asarray(motion["joint_pos_mujoco"][0], dtype=np.float64)
        joint_vel = np.asarray(motion["joint_vel_mujoco"][0], dtype=np.float64)
        root_pos = np.asarray(motion["root_pos"][0], dtype=np.float64)
        root_quat = np.asarray(
            motion["reset_root_quat_w"][0], dtype=np.float64
        )
        root_lin_vel = np.asarray(motion["root_lin_vel_w"][0], dtype=np.float64)
        root_ang_vel = np.asarray(motion["root_ang_vel_w"][0], dtype=np.float64)
    if joint_pos.shape != (29,) or joint_vel.shape != (29,):
        raise ValueError(f"{layout.tag}: invalid frame-zero joint state")

    if preserve_authored_world_frame:
        # Initial floor penetration creates a highly sensitive multi-contact
        # impulse.  A mathematically rigid yaw/translation rebase can choose a
        # different friction-pyramid solution, so authoritative E0019 replay
        # must keep the exact authored world state byte-for-byte.
        root_pos_local = root_pos
        root_quat_local = root_quat / np.linalg.norm(root_quat)
        root_lin_vel_local_world = root_lin_vel
        root_ang_vel_local_world = root_ang_vel
    else:
        # Chair layout is expressed in the initial robot-heading frame. Remove
        # only source yaw while preserving the authored roll/pitch reset.
        w, x, y, z = root_quat
        source_yaw = math.atan2(
            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
        )
        yaw_inverse = np.asarray(
            [math.cos(-source_yaw / 2.0), 0.0, 0.0, math.sin(-source_yaw / 2.0)],
            dtype=np.float64,
        )
        root_quat_local = _quat_multiply_wxyz(yaw_inverse, root_quat)
        root_quat_local /= np.linalg.norm(root_quat_local)
        cosine, sine = math.cos(-source_yaw), math.sin(-source_yaw)
        world_rotation = np.asarray(
            [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        root_pos_local = np.asarray([0.0, 0.0, root_pos[2]], dtype=np.float64)
        root_lin_vel_local_world = world_rotation @ root_lin_vel
        root_ang_vel_local_world = world_rotation @ root_ang_vel

    joint_ids = np.asarray(body_joint_ids, dtype=np.int64)
    if joint_ids.shape != (29,):
        raise ValueError(f"FaceE reference reset requires 29 joint ids, got {joint_ids.shape}")
    qpos_indices = np.asarray(model.jnt_qposadr[joint_ids], dtype=np.int64)
    qvel_indices = np.asarray(model.jnt_dofadr[joint_ids], dtype=np.int64)
    # The authoritative E0019 evaluator resets MjData and writes the authored
    # state into data.qpos; it does *not* rewrite model.qpos0.  qpos0 is part
    # of MuJoCo's compiled reference model, and changing it here alters the
    # very first contact solve even when all joint stiffness values are zero
    # (d1p50 changes from 28 contacts to 23).  Keep the model immutable.
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = root_pos_local
    data.qpos[3:7] = root_quat_local
    data.qpos[qpos_indices] = joint_pos
    data.qvel[0:3] = root_lin_vel_local_world
    # MuJoCo stores a free joint's angular velocity in the child frame.
    data.qvel[3:6] = (
        _quat_rotation_matrix_wxyz(root_quat_local).T @ root_ang_vel_local_world
    )
    data.qvel[qvel_indices] = joint_vel
    mujoco.mj_forward(model, data)
    return FaceEReferenceReset(
        tag=layout.tag,
        root_height_m=float(data.qpos[2]),
        root_quaternion_wxyz=tuple(float(value) for value in root_quat_local),
        initial_body_qpos_mujoco=tuple(float(value) for value in joint_pos),
        initial_body_qvel_mujoco=tuple(float(value) for value in joint_vel),
        source_motion_path=layout.motion_path,
    )


def is_facee_cpp_init_target(
    target_mujoco: np.ndarray,
    initial_mujoco: np.ndarray,
    tolerance_rad: float = 5e-4,
) -> bool:
    """Return true when a target lies on C++ INIT's initial-to-default ramp."""

    target = np.asarray(target_mujoco, dtype=np.float64)
    initial = np.asarray(initial_mujoco, dtype=np.float64)
    if target.shape != (29,) or initial.shape != (29,):
        raise ValueError("FaceE C++ INIT classifier requires two 29-vectors")
    direction = FACEE_DEPLOY_DEFAULT_ANGLES_MUJOCO - initial
    norm_sq = float(direction @ direction)
    if norm_sq < 1e-12:
        return bool(np.max(np.abs(target - initial)) <= tolerance_rad)
    ratio = float((target - initial) @ direction / norm_sq)
    residual = target - (initial + np.clip(ratio, 0.0, 1.0) * direction)
    return -1e-3 <= ratio <= 1.001 and bool(
        np.max(np.abs(residual)) <= tolerance_rad
    )
