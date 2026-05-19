from __future__ import annotations

import json
import threading
from typing import Iterator
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from gear_sonic.vigil_bridge import BridgeRequestRouter, VigilBridgeService, create_http_server


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
            "observation": ["rgb", "depth", "robot_state"],
            "oracle_source": "none",
        },
    }


@pytest.fixture
def http_server_url() -> Iterator[str]:
    server = create_http_server(host="127.0.0.1", port=0, service=VigilBridgeService())
    host, port = server.server_address
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://{host}:{port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def _post_json(base_url: str, endpoint: str, payload: dict) -> tuple[int, dict]:
    body = json.dumps(payload).encode("utf-8")
    request = Request(
        f"{base_url}/{endpoint}",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=2.0) as response:
        response_body = json.loads(response.read().decode("utf-8"))
        return response.status, response_body


def test_router_dispatches_handshake() -> None:
    response = BridgeRequestRouter(VigilBridgeService()).dispatch("handshake", _handshake_request())

    assert response["ok"] is True
    assert response["protocol_version"] == "vigil_groot_bridge_v1"
    assert response["runtime_mode"] == "mujoco"
    assert response["capabilities"]["oracle_source"] == "none"
    assert response["bridge"]["name"] == "gear_sonic_vigil_bridge"


def test_http_handshake_endpoint(http_server_url: str) -> None:
    status, response = _post_json(http_server_url, "handshake", _handshake_request())

    assert status == 200
    assert response["ok"] is True
    assert response["protocol_version"] == "vigil_groot_bridge_v1"
    assert response["runtime_mode"] == "mujoco"
    assert response["capabilities"]["actions"] == [
        "navigate.backward",
        "navigate.forward",
        "navigate.turn_left",
        "navigate.turn_right",
    ]
    assert response["capabilities"]["observation"] == ["rgb", "depth", "robot_state"]
    assert response["bridge"] == {
        "name": "gear_sonic_vigil_bridge",
        "version": "dry_run_phase1",
    }


def test_http_execute_action_endpoint(http_server_url: str) -> None:
    status, response = _post_json(
        http_server_url,
        "execute_action",
        {
            "episode_id": "test_episode",
            "step_id": 1,
            "runtime_mode": "mujoco",
            "skill_name": "navigate.forward",
            "arguments": {"magnitude": 1},
            "safety": {"max_speed_mps": 0.5, "timeout_s": 8.0},
        },
    )

    assert status == 200
    assert response["ok"] is True
    assert response["executed_arguments"]["primitive"] == "move_model"
    assert response["executed_arguments"]["distance_m"] > 0.0


def test_http_unknown_endpoint_returns_structured_404(http_server_url: str) -> None:
    with pytest.raises(HTTPError) as exc_info:
        _post_json(http_server_url, "missing", {})

    assert exc_info.value.code == 404
    response = json.loads(exc_info.value.read().decode("utf-8"))
    assert response["ok"] is False
    assert "unsupported endpoint" in response["error_message"]
