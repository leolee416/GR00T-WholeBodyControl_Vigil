from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

from gear_sonic.vigil_bridge.chair_motion_catalog import ChairMotionCatalog
from gear_sonic.vigil_bridge.mujoco_adapter import HEADER_SIZE, PackedPublisher


def test_reference_message_uses_cpp_protocol_v1_fields() -> None:
    publisher = object.__new__(PackedPublisher)
    captured = {}

    def capture(topic, header, data):
        captured.update(topic=topic, header=header, data=data)

    publisher._send_packed = capture  # type: ignore[method-assign]
    publisher.send_reference_motion(ChairMotionCatalog().load(2.00).frames)
    assert captured["topic"] == "pose"
    assert captured["header"]["v"] == 1
    assert captured["header"]["count"] == 650
    assert [field["name"] for field in captured["header"]["fields"]] == [
        "joint_pos",
        "joint_vel",
        "body_quat_w",
        "encode_mode",
        "motion_id",
        "frame_index",
        "catch_up",
    ]
    assert len(json.dumps(captured["header"]).encode()) < HEADER_SIZE
    expected_bytes = 650 * 29 * 4 * 2 + 650 * 4 * 4 + 4 + 4 + 650 * 8 + 1
    assert len(captured["data"]) == expected_bytes

    motion = ChairMotionCatalog().load(2.00)
    q_count = 650 * 29
    q_bytes = q_count * np.dtype("<f4").itemsize
    wire_q = np.frombuffer(captured["data"][:q_bytes], dtype="<f4").reshape(650, 29)
    wire_dq = np.frombuffer(
        captured["data"][q_bytes : 2 * q_bytes], dtype="<f4"
    ).reshape(650, 29)
    assert np.array_equal(wire_q, motion.frames["joint_pos"])
    assert np.array_equal(wire_dq, motion.frames["joint_vel"])


@pytest.mark.parametrize(
    "frames_update",
    [
        {"joint_order": "mujoco"},
        {"joint_order": None},
        {"protocol_version": 2},
        {"protocol_version": None},
    ],
)
def test_reference_publisher_fails_closed_on_semantic_contract(
    frames_update: dict[str, object]
) -> None:
    publisher = object.__new__(PackedPublisher)
    publisher._send_packed = lambda *_args: None  # type: ignore[method-assign]
    frames = dict(ChairMotionCatalog().load(1.70).frames)
    frames.update(frames_update)
    with pytest.raises(ValueError):
        publisher.send_reference_motion(frames)


def test_cpp_motion_sequence_observation_matches_wire_payload(tmp_path: Path) -> None:
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("g++ is unavailable")

    publisher = object.__new__(PackedPublisher)
    captured = {}

    def capture(topic, header, data):
        captured.update(topic=topic, header=header, data=data)

    publisher._send_packed = capture  # type: ignore[method-assign]
    motion = ChairMotionCatalog().load(1.70)
    publisher.send_reference_motion(motion.frames)
    payload = tmp_path / "pose_v1_payload.bin"
    payload.write_bytes(captured["data"])

    repo_root = Path(__file__).resolve().parents[1]
    source = repo_root / "tests/cpp/facee_motion_sequence_parity.cpp"
    include = (
        repo_root
        / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include"
    )
    executable = tmp_path / "facee_motion_sequence_parity"
    subprocess.run(
        [compiler, "-std=c++17", "-O2", "-I", str(include), str(source), "-o", str(executable)],
        check=True,
        capture_output=True,
        text=True,
    )
    gathered_path = tmp_path / "gathered_580.bin"
    subprocess.run(
        [str(executable), str(payload), str(gathered_path), "650"],
        check=True,
        capture_output=True,
        text=True,
    )
    gathered = np.fromfile(gathered_path, dtype="<f8")
    sampled = np.arange(0, 50, 5)
    expected = np.concatenate(
        [
            motion.frames["joint_pos"][sampled].reshape(-1),
            motion.frames["joint_vel"][sampled].reshape(-1),
        ]
    ).astype(np.float64)
    assert gathered.shape == (580,)
    assert np.array_equal(gathered, expected)
