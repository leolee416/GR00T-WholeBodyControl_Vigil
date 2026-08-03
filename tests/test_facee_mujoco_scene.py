from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from gear_sonic.utils.mujoco_sim.facee_chair_scene import (
    FACEE_DEPLOY_DEFAULT_ANGLES_MUJOCO,
    apply_facee_chair_layout,
    compile_facee_scene_model,
    configure_facee_deploy_reset,
    configure_facee_reference_reset,
    is_facee_cpp_init_target,
    load_facee_chair_layout,
)
from gear_sonic.vigil_bridge.chair_motion_catalog import EXACT_V3_CATALOG


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE = (
    REPO_ROOT
    / "gear_sonic/data/assets/robot_description/mjcf/scene_facee_training_primitives.xml"
)


@pytest.mark.parametrize(
    ("requested_distance_m", "expected_tag"),
    [(1.12, "d1p15"), (1.17, "d1p20"), (1.50, "d1p50"), (2.00, "d2p00")],
)
def test_facee_scene_uses_same_ceiling_selection_as_reference_catalog(
    requested_distance_m: float,
    expected_tag: str,
) -> None:
    layout = load_facee_chair_layout(requested_distance_m, EXACT_V3_CATALOG)
    assert layout.tag == expected_tag


@pytest.mark.parametrize("distance_m", [1.15, 1.50, 2.00])
def test_facee_scene_applies_exact_v3_pose_and_geometry(distance_m: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    layout = load_facee_chair_layout(distance_m, EXACT_V3_CATALOG)
    apply_facee_chair_layout(model, layout)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    chair = model.body("facee_chair")
    np.testing.assert_allclose(chair.pos[:2], layout.center_xy_m, atol=1e-12)
    assert chair.pos[2] == pytest.approx(0.0)
    assert layout.seat_height_m == pytest.approx(0.41)
    assert layout.backrest is False

    geom = model.geom("facee_chair_collision")
    assert geom.type == mujoco.mjtGeom.mjGEOM_MESH
    mesh = model.mesh("facee_backless_41cm_proxy")
    assert int(mesh.vertnum.item()) == 258
    mesh_element = ET.parse(SCENE).getroot().find(
        "asset/mesh[@name='facee_backless_41cm_proxy']"
    )
    assert mesh_element is not None
    vertices = np.fromstring(mesh_element.get("vertex", ""), sep=" ").reshape(-1, 3)
    np.testing.assert_allclose(vertices.min(axis=0), [-0.225, -0.25, 0.0], atol=1e-7)
    np.testing.assert_allclose(vertices.max(axis=0), [0.225, 0.25, 0.41], atol=1e-7)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chair_back") == -1

    expected_quat = np.asarray(
        [math.cos(layout.yaw_rad / 2), 0.0, 0.0, math.sin(layout.yaw_rad / 2)]
    )
    if np.dot(chair.quat, expected_quat) < 0:
        expected_quat *= -1
    np.testing.assert_allclose(chair.quat, expected_quat, atol=1e-12)


def test_facee_scene_matches_e0019_training_physics_contract() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    assert model.nu == 29
    assert model.njnt == 30  # free root + 29 actuated body joints
    assert model.opt.timestep == pytest.approx(0.005)
    assert model.opt.solver == mujoco.mjtSolver.mjSOL_NEWTON
    assert model.opt.iterations == 50

    geom_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        for geom_id in range(model.ngeom)
    ]
    primitive_ids = [
        geom_id
        for geom_id, name in enumerate(geom_names)
        if name.startswith("isaac_collision_")
    ]
    assert len(primitive_ids) == 29
    assert np.all(model.geom_contype[primitive_ids] == 1)
    assert np.all(model.geom_conaffinity[primitive_ids] == 2)

    robot_mesh_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
        and geom_names[geom_id] != "facee_chair_collision"
    ]
    assert robot_mesh_ids
    assert np.all(model.geom_contype[robot_mesh_ids] == 0)
    assert np.all(model.geom_conaffinity[robot_mesh_ids] == 0)

    for joint_name, expected_armature in (
        ("left_hip_pitch_joint", 0.025101925),
        ("left_hip_yaw_joint", 0.010177520),
        ("left_ankle_pitch_joint", 0.007219450),
        ("left_shoulder_pitch_joint", 0.003609725),
        ("left_wrist_pitch_joint", 0.00425),
    ):
        joint_id = model.joint(joint_name).id
        dof_id = int(model.jnt_dofadr[joint_id])
        assert model.dof_armature[dof_id] == pytest.approx(expected_armature)
        assert model.dof_damping[dof_id] == 0.0
        assert model.dof_frictionloss[dof_id] == 0.0


def test_facee_deploy_reset_matches_cpp_default_and_grounds_robot() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    joint_ids = np.asarray(
        [
            joint_id
            for joint_id in range(model.njnt)
            if model.joint(joint_id).name
            and any(
                token in model.joint(joint_id).name
                for token in (
                    "hip", "knee", "ankle", "waist", "shoulder", "elbow", "wrist"
                )
            )
        ],
        dtype=np.int64,
    )
    result = configure_facee_deploy_reset(model, data, joint_ids, 0.001)

    qpos_indices = model.jnt_qposadr[joint_ids]
    np.testing.assert_allclose(
        model.qpos0[qpos_indices], FACEE_DEPLOY_DEFAULT_ANGLES_MUJOCO, atol=0.0
    )
    np.testing.assert_allclose(data.qpos, model.qpos0, atol=0.0)
    assert result.root_height_m == pytest.approx(0.793, abs=0.005)
    assert result.measured_ground_clearance_m == pytest.approx(0.001, abs=2e-6)


def test_facee_deploy_reset_constant_matches_cpp_policy_header() -> None:
    import re

    header = (
        REPO_ROOT
        / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp"
    ).read_text(encoding="utf-8")
    block = re.search(
        r"default_angles\s*=\s*\{(?P<body>.*?)\};", header, flags=re.DOTALL
    )
    assert block is not None
    values = [
        float(value)
        for value in re.findall(
            r"^\s*(-?\d+(?:\.\d+)?)\s*,?", block.group("body"), flags=re.MULTILINE
        )
    ]
    np.testing.assert_allclose(values, FACEE_DEPLOY_DEFAULT_ANGLES_MUJOCO, atol=0.0)


def test_facee_reference_reset_uses_packaged_frame_zero() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    layout = load_facee_chair_layout(1.50, EXACT_V3_CATALOG)
    joint_ids = np.asarray(
        [
            joint_id
            for joint_id in range(model.njnt)
            if model.joint(joint_id).name
            and any(
                token in model.joint(joint_id).name
                for token in (
                    "hip", "knee", "ankle", "waist", "shoulder", "elbow", "wrist"
                )
            )
        ],
        dtype=np.int64,
    )
    result = configure_facee_reference_reset(model, data, joint_ids, layout)

    with np.load(layout.motion_path, allow_pickle=False) as motion:
        np.testing.assert_allclose(
            data.qpos[model.jnt_qposadr[joint_ids]],
            motion["joint_pos_mujoco"][0],
            atol=0.0,
        )
        assert result.root_height_m == pytest.approx(float(motion["root_pos"][0, 2]))
    quaternion = np.asarray(result.root_quaternion_wxyz)
    yaw = math.atan2(
        2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
        1 - 2 * (quaternion[2] ** 2 + quaternion[3] ** 2),
    )
    assert yaw == pytest.approx(0.0, abs=1e-7)


def test_facee_authoritative_reset_preserves_source_world_pose() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    original_qpos0 = model.qpos0.copy()
    layout = load_facee_chair_layout(1.50, EXACT_V3_CATALOG)
    apply_facee_chair_layout(
        model, layout, preserve_authored_world_frame=True
    )
    joint_ids = np.asarray(model.actuator_trnid[:, 0], dtype=np.int64)
    configure_facee_reference_reset(
        model,
        data,
        joint_ids,
        layout,
        preserve_authored_world_frame=True,
    )

    with np.load(layout.motion_path, allow_pickle=False) as motion:
        np.testing.assert_allclose(data.qpos[:3], motion["root_pos"][0], atol=0.0)
        np.testing.assert_allclose(
            data.qpos[3:7], motion["reset_root_quat_w"][0], atol=1e-12
        )
    np.testing.assert_array_equal(model.qpos0, original_qpos0)


def test_facee_scene_compiles_static_chair_at_final_pose() -> None:
    layout = load_facee_chair_layout(1.50, EXACT_V3_CATALOG, REPO_ROOT)
    model = compile_facee_scene_model(
        SCENE,
        layout,
        preserve_authored_world_frame=True,
    )
    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "facee_chair"
    )
    expected_xy = np.asarray(layout.source_chair_world_xy_m)
    np.testing.assert_allclose(model.body_pos[body_id, :2], expected_xy, atol=1e-12)
    np.testing.assert_allclose(
        model.body("facee_chair").pos[:2],
        layout.source_chair_world_xy_m,
        atol=0.0,
    )
    expected_chair_quat = np.asarray(
        [
            math.cos(layout.source_chair_world_yaw_rad / 2),
            0.0,
            0.0,
            math.sin(layout.source_chair_world_yaw_rad / 2),
        ]
    )
    actual_chair_quat = model.body("facee_chair").quat.copy()
    if np.dot(actual_chair_quat, expected_chair_quat) < 0:
        expected_chair_quat *= -1
    np.testing.assert_allclose(actual_chair_quat, expected_chair_quat, atol=1e-12)

    # body_ipos/body_iquat are the *local inertial frame*, not the body's
    # parent-frame pose.  Validate the compiled runtime transforms instead.
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.xpos[body_id, :2], expected_xy, atol=1e-12)
    actual_runtime_quat = data.xquat[body_id].copy()
    if np.dot(actual_runtime_quat, expected_chair_quat) < 0:
        actual_runtime_quat *= -1
    np.testing.assert_allclose(
        actual_runtime_quat, expected_chair_quat, atol=1e-12
    )
    geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "facee_chair_collision"
    )
    np.testing.assert_allclose(data.geom_xpos[geom_id, :2], expected_xy, atol=1e-9)
    assert data.geom_xpos[geom_id, 2] == pytest.approx(
        layout.seat_height_m / 2, abs=1e-8
    )


def test_facee_cpp_init_target_classifier_rejects_actor_target() -> None:
    initial = FACEE_DEPLOY_DEFAULT_ANGLES_MUJOCO.copy()
    initial[0] += 0.4
    initial[3] -= 0.2
    for ratio in (0.0, 0.01, 0.5, 1.0):
        target = initial + ratio * (FACEE_DEPLOY_DEFAULT_ANGLES_MUJOCO - initial)
        assert is_facee_cpp_init_target(target, initial)
    actor_target = initial.copy()
    actor_target[7] += 0.1
    assert not is_facee_cpp_init_target(actor_target, initial)
