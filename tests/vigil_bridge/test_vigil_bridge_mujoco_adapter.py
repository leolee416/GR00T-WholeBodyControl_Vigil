from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any

from gear_sonic.vigil_bridge.mujoco_adapter import (
    MujocoBridgeConfig,
    MujocoPrimitiveExecutor,
    MujocoSensorProvider,
    create_mujoco_bridge_service,
)
from gear_sonic.vigil_bridge.service import VigilBridgeService


@dataclass
class FakeMujocoRuntime:
    config: MujocoBridgeConfig = field(
        default_factory=lambda: MujocoBridgeConfig(runtime_mode="mujoco", camera_enabled=True)
    )
    started: bool = False
    halted: bool = False
    closed: bool = False
    stop_command_sent: bool = False
    rotate_completed: bool = True
    moves: list[dict[str, float]] = field(default_factory=list)
    rotates: list[dict[str, float]] = field(default_factory=list)
    robot_state: dict[str, Any] | None = field(
        default_factory=lambda: {
            "state_id": "fake_mujoco_state",
            "base_pose": {"x_m": 1.0, "y_m": 2.0, "yaw_deg": 15.0},
            "estimated": False,
            "source": "fake_mujoco",
        }
    )
    camera_payload: dict[str, Any] = field(
        default_factory=lambda: {
            "images": {"ego_view": "ZmFrZS1qcGVn"},
            "timestamps": {"ego_view": 123.0},
        }
    )

    def start(self) -> dict:
        self.started = True
        return self.get_health()

    def halt(self) -> dict:
        self.halted = True
        self.stop_command_sent = bool(self.config.stop_on_halt)
        self.started = False
        return self.get_health()

    def close(self) -> None:
        self.closed = True
        self.started = False

    def move(self, distance_m: float, speed_mps: float, duration_s: float) -> dict:
        self.moves.append(
            {
                "distance_m": distance_m,
                "speed_mps": speed_mps,
                "duration_s": duration_s,
            }
        )
        return {
            "motion": "move",
            "completion": {
                "motion_commanded": True,
                "completion_source": "duration_and_settle",
                "capture_timing": "after_settle",
                "settled": True,
                "duration_s": duration_s + 0.8,
                "command_duration_s": duration_s,
                "settle_duration_s": 0.8,
            },
            "motion_result": {
                "actual_distance_m": distance_m,
                "actual_distance_source": "fake_odom",
            },
        }

    def rotate(self, degrees: float, rate_deg_s: float, timeout_s: float) -> dict:
        self.rotates.append(
            {
                "degrees": degrees,
                "rate_deg_s": rate_deg_s,
                "timeout_s": timeout_s,
            }
        )
        return {
            "motion": "rotate",
            "completed": self.rotate_completed,
            "completion": {
                "motion_commanded": True,
                "completion_source": "yaw_closed_loop",
                "capture_timing": "after_settle" if self.rotate_completed else "after_timeout",
                "settled": self.rotate_completed,
                "duration_s": timeout_s,
            },
            "motion_result": {
                "actual_degrees": degrees if self.rotate_completed else 0.0,
                "final_error_deg": 0.0 if self.rotate_completed else degrees,
            },
        }

    def get_robot_state_payload(self) -> dict[str, Any] | None:
        return self.robot_state

    def latest_camera_payload(self) -> dict[str, Any] | None:
        return self.camera_payload

    def get_health(self, sensor_connected: bool = True) -> dict:
        return {
            "ok": True,
            "runtime_mode": self.config.runtime_mode,
            "executor_started": self.started,
            "sensor_connected": self.robot_state is not None,
            "error_message": None,
            "telemetry": {
                "executor": "fake_mujoco",
                "camera_enabled": self.config.camera_enabled,
            },
        }


def _service(runtime: FakeMujocoRuntime) -> VigilBridgeService:
    config = runtime.config
    return VigilBridgeService(
        executor=MujocoPrimitiveExecutor(config=config, runtime=runtime),
        sensor_provider=MujocoSensorProvider(runtime=runtime, runtime_mode=config.runtime_mode),
        runtime_mode=config.runtime_mode,
    )


def _write_move_model(tmp_path: Path) -> Path:
    path = tmp_path / "move_model.json"
    path.write_text(
        json.dumps(
            {
                "models": {
                    "forward": [
                        {"magnitude_abs": 0.25, "rate": 1.0, "execute_time": 0.45},
                        {"magnitude_abs": 0.5, "rate": 0.75, "execute_time": 0.8},
                    ],
                    "backward": [
                        {"magnitude_abs": 0.25, "rate": 0.8, "execute_time": 0.5},
                        {"magnitude_abs": 0.5, "rate": 0.6, "execute_time": 0.9},
                    ],
                }
            }
        ),
        encoding="utf-8",
    )
    return path


def test_mujoco_executor_maps_forward_to_runtime_move_model(tmp_path: Path) -> None:
    model_path = _write_move_model(tmp_path)
    runtime = FakeMujocoRuntime(
        config=MujocoBridgeConfig(runtime_mode="mujoco", camera_enabled=True, move_model_file=str(model_path))
    )
    service = _service(runtime)

    response = service.execute_action(
        {
            "episode_id": "sim_episode",
            "step_id": 1,
            "runtime_mode": "mujoco",
            "skill_name": "navigate.forward",
            "arguments": {"magnitude": 2},
            "safety": {"max_speed_mps": 0.5, "timeout_s": 8.0},
        }
    )

    assert response["ok"] is True
    assert response["action_status"] == "completed"
    assert response["executed_arguments"]["skill_name"] == "navigate.forward"
    assert response["executed_arguments"]["primitive"] == "move_model"
    assert response["executed_arguments"]["distance_m"] == 0.5
    assert response["executed_arguments"]["max_speed_mps"] == 0.5
    assert response["executed_arguments"]["move_model_file"] == str(model_path)
    assert runtime.moves == [{"distance_m": 0.5, "speed_mps": 0.5, "duration_s": 0.8}]
    assert response["telemetry"]["dry_run"] is False
    assert response["telemetry"]["phase"] == "mujoco_adapter_phase3"
    assert response["telemetry"]["completion"]["capture_timing"] == "after_settle"
    assert response["telemetry"]["completion"]["settled"] is True
    assert response["telemetry"]["move_model"]["enabled"] is True
    assert response["telemetry"]["motion_result"] == {
        "actual_distance_m": 0.5,
        "actual_distance_source": "fake_odom",
    }
    assert response["robot_state_before"]["source"] == "fake_mujoco"


def test_mujoco_executor_uses_default_max_speed_for_move_model(tmp_path: Path) -> None:
    model_path = _write_move_model(tmp_path)
    runtime = FakeMujocoRuntime(
        config=MujocoBridgeConfig(runtime_mode="mujoco", camera_enabled=True, move_model_file=str(model_path))
    )
    service = _service(runtime)

    response = service.execute_action(
        {
            "episode_id": "sim_episode",
            "step_id": 1,
            "runtime_mode": "mujoco",
            "skill_name": "navigate.forward",
            "arguments": {"magnitude": 2},
            "safety": {"timeout_s": 8.0},
        }
    )

    assert response["ok"] is True
    assert response["executed_arguments"]["max_speed_mps"] == 1.0
    assert runtime.moves == [{"distance_m": 0.5, "speed_mps": 0.75, "duration_s": 0.8}]


def test_mujoco_rotate_timeout_is_structured_failure() -> None:
    runtime = FakeMujocoRuntime(rotate_completed=False)
    service = _service(runtime)

    response = service.execute_action(
        {
            "episode_id": "sim_episode",
            "step_id": 2,
            "runtime_mode": "mujoco",
            "skill_name": "navigate.turn_left",
            "arguments": {"degrees": 45},
            "safety": {"max_rate_deg_s": 30.0, "timeout_s": 2.0},
        }
    )

    assert response["ok"] is False
    assert response["action_status"] == "timeout"
    assert "timed out" in response["error_message"]
    assert response["executed_arguments"] == {}
    assert runtime.rotates == [{"degrees": 45.0, "rate_deg_s": 30.0, "timeout_s": 2.0}]
    assert response["telemetry"]["dry_run"] is False
    assert response["telemetry"]["completion"]["capture_timing"] == "after_timeout"
    assert response["telemetry"]["completion"]["settled"] is False


def test_mujoco_sensor_provider_normalizes_camera_payload() -> None:
    runtime = FakeMujocoRuntime()
    provider = MujocoSensorProvider(runtime=runtime, runtime_mode="mujoco")

    observation = provider.get_observation()

    assert observation["runtime_mode"] == "mujoco"
    assert observation["images"]["ego_view"] == {
        "encoding": "jpeg-base64",
        "data": "ZmFrZS1qcGVn",
    }
    assert observation["camera_timestamps"]["ego_view"] == 123.0
    assert observation["robot_state"]["source"] == "fake_mujoco"
    assert observation["perception"]["source"] == "none"


def test_mujoco_service_close_releases_runtime() -> None:
    runtime = FakeMujocoRuntime()
    service = _service(runtime)

    service.close()

    assert runtime.closed is True
    assert runtime.halted is False
    assert runtime.stop_command_sent is False


def test_mujoco_halt_does_not_send_deploy_stop_by_default() -> None:
    runtime = FakeMujocoRuntime()
    service = _service(runtime)

    service.halt()

    assert runtime.halted is True
    assert runtime.stop_command_sent is False


def test_mujoco_halt_can_opt_in_to_deploy_stop() -> None:
    runtime = FakeMujocoRuntime(
        config=MujocoBridgeConfig(runtime_mode="mujoco", stop_on_halt=True)
    )
    service = _service(runtime)

    service.halt()

    assert runtime.halted is True
    assert runtime.stop_command_sent is True


def test_mujoco_service_factory_does_not_start_runtime_on_handshake() -> None:
    service = create_mujoco_bridge_service(MujocoBridgeConfig(runtime_mode="mujoco"))

    response = service.handshake(
        {
            "protocol_version": "vigil_groot_bridge_v1",
            "runtime_mode": "mujoco",
        }
    )

    assert response["ok"] is True
    assert response["runtime_mode"] == "mujoco"
    assert response["capabilities"]["oracle_source"] == "none"
    service.close()
