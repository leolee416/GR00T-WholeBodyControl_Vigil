from __future__ import annotations

import struct

from gear_sonic.vigil_bridge.mujoco_adapter import PackedPublisher


def test_reference_motion_packs_full_body_fk_and_encode_mode() -> None:
    publisher = object.__new__(PackedPublisher)
    captured = {}

    def capture(topic, header, data):
        captured.update(topic=topic, header=header, data=data)

    publisher._send_packed = capture  # type: ignore[method-assign]
    publisher.send_reference_motion(
        {
            "joint_pos": [[0.0] * 29],
            "joint_vel": [[0.0] * 29],
            "body_quat_w": [[1.0, 0.0, 0.0, 0.0] * 2],
            "body_pos": [[0.0, 0.0, 0.8, 0.1, 0.2, 0.3]],
            "body_part_indexes": [0, 28],
            "encode_mode": 1,
            "motion_id": 4,
            "frame_index": [7],
        }
    )

    assert captured["topic"] == "pose"
    fields = {field["name"]: field for field in captured["header"]["fields"]}
    assert fields["body_pos"]["shape"] == [1, 2, 3]
    assert fields["body_part_indexes"]["shape"] == [2]
    assert fields["encode_mode"]["shape"] == [1]
    assert fields["motion_id"]["shape"] == [1]

    offset = (29 + 29 + 8 + 6) * 4
    assert struct.unpack_from("<ii", captured["data"], offset) == (0, 28)
    assert struct.unpack_from("<i", captured["data"], offset + 8) == (1,)
    assert struct.unpack_from("<i", captured["data"], offset + 12) == (4,)


def test_reference_motion_requires_body_indexes_with_body_positions() -> None:
    publisher = object.__new__(PackedPublisher)
    publisher._send_packed = lambda *_: None  # type: ignore[method-assign]

    try:
        publisher.send_reference_motion(
            {
                "joint_pos": [[0.0] * 29],
                "joint_vel": [[0.0] * 29],
                "body_quat_w": [[1.0, 0.0, 0.0, 0.0]],
                "body_pos": [[0.0, 0.0, 0.8]],
            }
        )
    except ValueError as exc:
        assert "body_part_indexes is required" in str(exc)
    else:
        raise AssertionError("missing body_part_indexes was accepted")
