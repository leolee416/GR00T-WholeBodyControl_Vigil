from __future__ import annotations

import json

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
