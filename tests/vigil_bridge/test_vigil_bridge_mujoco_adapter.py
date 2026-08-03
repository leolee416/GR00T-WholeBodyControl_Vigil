from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any

from gear_sonic.vigil_bridge.mujoco_adapter import (
    MujocoBridgeConfig,
    MujocoPrimitiveExecutor,
    MujocoRuntimeClient,
    MujocoSensorProvider,
    create_mujoco_bridge_service,
)
from gear_sonic.vigil_bridge.chair_motion_catalog import (
    ChairMotionCatalog,
    DEFAULT_CATALOG,
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
    reference_motions: list[dict[str, Any]] = field(default_factory=list)
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
            "images": {"ego_view": "ZmFrZS1qcGVn", "ego_view_depth": "ZmFrZS1kZXB0aA=="},
            "timestamps": {"ego_view": 123.0, "ego_view_depth": 123.0},
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

    def play_sonic_reference_motion(self, payload: dict[str, Any]) -> dict:
        self.reference_motions.append(payload)
        frames = payload["frames"]
        duration_s = float(payload["duration_s"])
        return {
            "motion": "sonic_reference_motion",
            "sonic_input": "reference_motion",
            "chair_distance_m": float(payload["chair_distance_m"]),
            "reference_distance_m": float(payload["reference_distance_m"]),
            "motion_name": str(payload["motion_name"]),
            "tag": str(payload["tag"]),
            "duration_s": duration_s,
            "frame_count": len(frames["joint_pos"]),
            "completion": {
                "motion_commanded": True,
                "completion_source": "reference_dispatched",
                "capture_timing": "after_dispatch",
                "settled": False,
                "duration_s": 0.0,
                "command_duration_s": duration_s,
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
    assert observation["images"]["ego_view_depth"] == {
        "encoding": "jpeg-base64",
        "data": "ZmFrZS1kZXB0aA==",
    }
    assert observation["camera_timestamps"]["ego_view"] == 123.0
    assert observation["camera_timestamps"]["ego_view_depth"] == 123.0
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


def test_mujoco_executor_loads_catalog_and_dispatches_reference() -> None:
    runtime = FakeMujocoRuntime(
        config=MujocoBridgeConfig(
            runtime_mode="mujoco",
            use_move_model=False,
            chair_motion_catalog=str(DEFAULT_CATALOG),
        )
    )
    service = _service(runtime)

    response = service.execute_action(
        {
            "episode_id": "facee_sim",
            "step_id": 1,
            "runtime_mode": "mujoco",
            "skill_name": "sonic.sit_chair",
            "arguments": {"chair_distance_m": 1.17},
            "safety": {},
        }
    )

    assert response["ok"] is True
    assert response["action_status"] == "completed"
    assert response["executed_arguments"]["reference_distance_m"] == 1.20
    assert response["executed_arguments"]["frame_count"] == 650
    assert response["telemetry"]["controller"] == "mujoco_zmq_wbc"
    assert response["telemetry"]["dry_run"] is False
    assert response["telemetry"]["completion"] == {
        "motion_commanded": True,
        "completion_source": "reference_dispatched",
        "capture_timing": "after_dispatch",
        "settled": False,
        "duration_s": 0.0,
        "command_duration_s": 13.0,
    }
    assert len(runtime.reference_motions) == 1
    assert runtime.reference_motions[0]["tag"] == "d1p20"
    assert runtime.reference_motions[0]["frames"]["joint_pos"].shape == (650, 29)


def test_mujoco_factory_configures_catalog_without_starting_runtime() -> None:
    service = create_mujoco_bridge_service(
        MujocoBridgeConfig(
            runtime_mode="mujoco",
            chair_motion_catalog=str(DEFAULT_CATALOG),
        )
    )
    assert isinstance(service.executor.chair_motion_catalog, ChairMotionCatalog)
    assert service.executor.started is False
    service.close()


def test_mujoco_runtime_reference_uses_command_then_single_pose() -> None:
    class FakePackedPublisher:
        def __init__(self) -> None:
            self.events: list[tuple[str, Any]] = []

        def send_command(
            self,
            start: bool,
            stop: bool,
            planner: bool = True,
            pause: bool = False,
        ) -> None:
            self.events.append(
                ("command", {"start": start, "stop": stop, "planner": planner, "pause": pause})
            )

        def send_reference_motion(self, frames: dict[str, Any]) -> None:
            self.events.append(("pose", frames))

        def send_planner(self, *args: Any, **kwargs: Any) -> None:
            self.events.append(("planner", {"args": args, "kwargs": kwargs}))

    config = MujocoBridgeConfig(
        runtime_mode="mujoco",
        startup_command_burst_s=0.0,
        startup_command_period_s=0.01,
    )
    runtime = MujocoRuntimeClient(config)
    publisher = FakePackedPublisher()
    runtime._publisher = publisher  # type: ignore[assignment]
    runtime.started = True
    runtime.latest_state = lambda: None  # type: ignore[method-assign]
    runtime.wait_for_state = lambda timeout: object()  # type: ignore[method-assign]
    motion = ChairMotionCatalog().load(1.50)

    telemetry = runtime.play_sonic_reference_motion(
        {
            "chair_distance_m": motion.requested_distance_m,
            "reference_distance_m": motion.reference_distance_m,
            "motion_name": motion.motion_name,
            "tag": motion.tag,
            "duration_s": motion.duration_s,
            "frames": motion.frames,
        }
    )

    event_names = [name for name, _payload in publisher.events]
    assert event_names.count("pose") == 1
    pose_index = event_names.index("pose")
    command_payloads = [payload for name, payload in publisher.events if name == "command"]
    assert command_payloads
    streamed_mode_switch = {
        "start": False,
        "stop": False,
        "planner": False,
        "pause": False,
    }
    streamed_start = {
        "start": True,
        "stop": False,
        "planner": False,
        "pause": False,
    }
    assert pose_index > 0
    command_events_before_pose = [
        payload for name, payload in publisher.events[:pose_index] if name == "command"
    ]
    command_events_after_pose = [
        payload for name, payload in publisher.events[pose_index + 1 :] if name == "command"
    ]
    assert command_events_before_pose
    assert command_events_after_pose
    assert all(payload == streamed_mode_switch for payload in command_events_before_pose)
    assert all(payload == streamed_start for payload in command_events_after_pose)
    assert all(name != "planner" for name in event_names)
    assert telemetry["frame_count"] == 650
    assert telemetry["completion"]["completion_source"] == "reference_dispatched"
