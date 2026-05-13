from __future__ import annotations

import ast
from pathlib import Path

from gear_sonic.vigil_bridge import VigilBridgeService


def _handshake_request() -> dict:
    return {
        "protocol_version": "vigil_groot_bridge_v1",
        "client": {"name": "vigil", "component": "GrootWBCEnv"},
        "episode_id": "test_001",
        "runtime_mode": "mujoco",
        "required_capabilities": {
            "actions": [
                "navigate.backward",
                "navigate.forward",
                "navigate.turn_left",
                "navigate.turn_right",
            ],
            "observation": ["rgb", "robot_state"],
            "oracle_source": "none",
        },
    }


def _execute(skill_name: str, arguments: dict | None = None) -> dict:
    service = VigilBridgeService()
    return service.execute_action(
        {
            "episode_id": "test_episode",
            "step_id": 1,
            "runtime_mode": "mujoco",
            "skill_name": skill_name,
            "arguments": arguments or {"magnitude": 1},
            "safety": {"max_speed_mps": 0.5, "timeout_s": 8.0},
        }
    )


def test_handshake_returns_vigil_protocol_fields() -> None:
    response = VigilBridgeService().handshake(_handshake_request())

    assert response == {
        "ok": True,
        "error_message": None,
        "protocol_version": "vigil_groot_bridge_v1",
        "runtime_mode": "mujoco",
        "capabilities": {
            "actions": [
                "navigate.backward",
                "navigate.forward",
                "navigate.turn_left",
                "navigate.turn_right",
            ],
            "observation": ["rgb", "robot_state"],
            "oracle_source": "none",
        },
        "bridge": {
            "name": "gear_sonic_vigil_bridge",
            "version": "dry_run_phase1",
        },
    }


def test_navigate_forward_maps_to_positive_move() -> None:
    response = _execute("navigate.forward")

    assert response["ok"] is True
    assert response["action_status"] == "completed"
    assert response["executed_arguments"]["skill_name"] == "navigate.forward"
    assert response["executed_arguments"]["primitive"] == "move_model"
    assert response["executed_arguments"]["distance_m"] > 0.0
    assert response["telemetry"]["completion"]["motion_commanded"] is True
    assert response["telemetry"]["completion"]["capture_timing"] == "after_completion"


def test_navigate_backward_maps_to_negative_move() -> None:
    response = _execute("navigate.backward")

    assert response["ok"] is True
    assert response["executed_arguments"]["primitive"] == "move_model"
    assert response["executed_arguments"]["distance_m"] < 0.0


def test_navigate_turn_left_maps_to_positive_rotation() -> None:
    response = _execute("navigate.turn_left")

    assert response["ok"] is True
    assert response["executed_arguments"]["primitive"] == "rotate"
    assert response["executed_arguments"]["degrees"] > 0.0


def test_navigate_turn_right_maps_to_negative_rotation() -> None:
    response = _execute("navigate.turn_right")

    assert response["ok"] is True
    assert response["executed_arguments"]["primitive"] == "rotate"
    assert response["executed_arguments"]["degrees"] < 0.0


def test_unsupported_action_returns_structured_failure() -> None:
    response = _execute("pick_up")

    assert response["ok"] is False
    assert response["action_status"] == "rejected"
    assert "unsupported skill_name" in response["error_message"]
    assert response["executed_arguments"] == {}
    assert response["telemetry"]["completion"] == {
        "motion_commanded": False,
        "completion_source": "validation",
        "capture_timing": "not_started",
        "settled": False,
        "duration_s": 0.0,
    }


def test_vigil_bridge_package_does_not_import_vigil() -> None:
    package_root = Path(__file__).resolve().parents[2] / "gear_sonic" / "vigil_bridge"

    for path in package_root.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_roots = {alias.name.split(".")[0] for alias in node.names}
                assert "vigil" not in imported_roots
            elif isinstance(node, ast.ImportFrom):
                module_root = (node.module or "").split(".")[0]
                assert module_root != "vigil"
