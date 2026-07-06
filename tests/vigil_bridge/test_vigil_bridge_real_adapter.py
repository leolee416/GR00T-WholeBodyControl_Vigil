from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any

import pytest

from gear_sonic.vigil_bridge.real_adapter import (
    RealBridgeConfig,
    RealPrimitiveExecutor,
    RealRuntimeClient,
    RealSensorProvider,
    create_real_bridge_service,
)
from gear_sonic.vigil_bridge.service import VigilBridgeService


@dataclass
class FakeRealRuntime:
    config: RealBridgeConfig = field(
        default_factory=lambda: RealBridgeConfig(
            motion_enabled=True,
            camera_enabled=True,
            camera_required=True,
        )
    )
    started: bool = False
    halted: bool = False
    paused: bool = False
    closed: bool = False
    fail_move: bool = False
    moves: list[dict[str, float]] = field(default_factory=list)
    rotates: list[dict[str, float]] = field(default_factory=list)
    robot_state: dict[str, Any] | None = field(
        default_factory=lambda: {
            "state_id": "fake_real_state",
            "base_pose": {"x_m": None, "y_m": None, "yaw_deg": 0.0},
            "estimated": True,
            "source": "fake_real",
        }
    )
    camera_payload: dict[str, Any] | None = field(
        default_factory=lambda: {
            "images": {"ego_view": "ZmFrZS1yZWFsLWpwZWc=", "ego_view_depth": "ZmFrZS1yZWFsLWRlcHRo"},
            "timestamps": {"ego_view": 456.0, "ego_view_depth": 456.0},
        }
    )

    def start(self) -> dict:
        self.started = True
        return self.get_health()

    def halt(self) -> dict:
        self.halted = True
        self.paused = False
        self.started = False
        return self.get_health()

    def pause(self) -> dict:
        self.paused = True
        self.started = False
        return self.get_health()

    def resume(self) -> dict:
        self.paused = False
        self.started = True
        return self.get_health()

    def close(self) -> None:
        self.closed = True
        self.paused = False
        self.started = False

    def move(self, distance_m: float, speed_mps: float, duration_s: float) -> dict:
        if self.fail_move:
            raise RuntimeError("simulated real runtime move failure")
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
                "duration_s": duration_s + self.config.move_settle_time_s,
                "command_duration_s": duration_s,
                "settle_duration_s": self.config.move_settle_time_s,
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
            "completed": True,
            "completion": {
                "motion_commanded": True,
                "completion_source": "yaw_closed_loop",
                "capture_timing": "after_settle",
                "settled": True,
                "duration_s": timeout_s,
            },
            "motion_result": {
                "estimated_degrees": degrees,
                "estimated_degrees_source": "fake_real_heading",
            },
        }

    def get_robot_state_payload(self) -> dict[str, Any] | None:
        return self.robot_state

    def latest_camera_payload(self) -> dict[str, Any] | None:
        return self.camera_payload

    def get_health(self, sensor_connected: bool = True) -> dict:
        state_connected = self.robot_state is not None
        camera_connected = self.camera_payload is not None
        return {
            "ok": state_connected and (camera_connected or not self.config.camera_required),
            "runtime_mode": self.config.runtime_mode,
            "executor_started": self.started,
            "sensor_connected": state_connected and (camera_connected or not self.config.camera_required),
            "error_message": None,
            "telemetry": {
                "executor": "fake_real",
                "hardware": True,
                "motion_enabled": self.config.motion_enabled,
                "state_connected": state_connected,
                "camera_connected": camera_connected,
            },
        }


@dataclass
class FakeHeadingState:
    yaw: float = 0.0
    base_quat: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    delta_heading: float = 0.0
    yaw_rate: float = 0.0
    timestamp: float = 0.0


class StaticHeadingSubscriber:
    def __init__(self, state: FakeHeadingState) -> None:
        self.state = state

    def latest(self) -> FakeHeadingState:
        return self.state

    def wait_for_state(self, timeout: float) -> FakeHeadingState:
        return self.state


class FinalHeadingSubscriber:
    def __init__(self, initial_yaw: float, final_yaw: float) -> None:
        self.initial_state = FakeHeadingState(yaw=initial_yaw)
        self.final_state = FakeHeadingState(yaw=final_yaw)

    def latest(self) -> FakeHeadingState:
        return self.final_state

    def wait_for_state(self, timeout: float) -> FakeHeadingState:
        return self.initial_state


class RecordingPublisher:
    def __init__(self) -> None:
        self.facings: list[list[float]] = []

    def send_planner(
        self,
        mode: int,
        movement: list[float],
        facing: list[float],
        speed: float = -1.0,
        height: float = -1.0,
    ) -> None:
        self.facings.append(facing)

    def send_command(self, start: bool, stop: bool, planner: bool = True, pause: bool = False) -> None:
        pass


def _attach_runtime_fakes(
    runtime: RealRuntimeClient,
    state_subscriber: StaticHeadingSubscriber | FinalHeadingSubscriber | None = None,
) -> RecordingPublisher:
    publisher = RecordingPublisher()
    runtime._publisher = publisher  # type: ignore[assignment]
    runtime._state_sub = state_subscriber or StaticHeadingSubscriber(FakeHeadingState())  # type: ignore[assignment]
    runtime.send_idle_burst = lambda duration, preserve_facing=False: None  # type: ignore[method-assign]
    return publisher


def _service(runtime: FakeRealRuntime) -> VigilBridgeService:
    config = runtime.config
    return VigilBridgeService(
        executor=RealPrimitiveExecutor(config=config, runtime=runtime),
        sensor_provider=RealSensorProvider(runtime=runtime, runtime_mode=config.runtime_mode),
        runtime_mode=config.runtime_mode,
    )


def _write_move_model(tmp_path: Path) -> Path:
    path = tmp_path / "move_model.json"
    path.write_text(
        """{
          "models": {
            "forward": [
              {"magnitude_abs": 0.25, "rate": 0.40, "execute_time": 0.70},
              {"magnitude_abs": 0.50, "rate": 0.75, "execute_time": 0.80}
            ],
            "backward": [
              {"magnitude_abs": 0.25, "rate": 0.35, "execute_time": 0.75},
              {"magnitude_abs": 0.50, "rate": 0.65, "execute_time": 0.90}
            ]
          }
        }""",
        encoding="utf-8",
    )
    return path


def test_real_backend_rejects_motion_by_default() -> None:
    runtime = FakeRealRuntime(config=RealBridgeConfig(motion_enabled=False))
    service = _service(runtime)

    response = service.execute_action(
        {
            "episode_id": "real_episode",
            "step_id": 1,
            "runtime_mode": "real",
            "skill_name": "navigate.forward",
            "arguments": {"distance_m": 0.10},
            "safety": {"max_speed_mps": 0.20, "timeout_s": 5.0},
        }
    )

    assert response["ok"] is False
    assert response["action_status"] == "rejected"
    assert "real motion is disabled" in response["error_message"]
    assert response["telemetry"]["phase"] == "real_adapter_phase1"
    assert response["telemetry"]["dry_run"] is False
    assert response["telemetry"]["hardware"] is True
    assert response["telemetry"]["completion"]["motion_commanded"] is False
    assert runtime.moves == []


def test_real_service_pause_and_resume_keep_runtime_open() -> None:
    runtime = FakeRealRuntime()
    service = _service(runtime)

    pause_response = service.pause()
    assert pause_response["ok"] is True
    assert runtime.paused is True
    assert runtime.closed is False

    resume_response = service.resume()
    assert runtime.paused is False
    assert runtime.closed is False
    assert resume_response["executor_started"] is True


def test_real_executor_maps_forward_to_runtime_move_model(tmp_path: Path) -> None:
    model_path = _write_move_model(tmp_path)
    runtime = FakeRealRuntime(
        config=RealBridgeConfig(
            motion_enabled=True,
            camera_enabled=True,
            camera_required=True,
            default_move_speed_mps=0.15,
            max_move_speed_mps=1.50,
            move_model_file=str(model_path),
        )
    )
    service = _service(runtime)

    response = service.execute_action(
        {
            "episode_id": "real_episode",
            "step_id": 2,
            "runtime_mode": "real",
            "skill_name": "navigate.forward",
            "arguments": {"distance_m": 0.50},
            "safety": {"timeout_s": 5.0},
        }
    )

    assert response["ok"] is True
    assert response["action_status"] == "completed"
    assert response["executed_arguments"]["skill_name"] == "navigate.forward"
    assert response["executed_arguments"]["primitive"] == "move_model"
    assert response["executed_arguments"]["distance_m"] == 0.50
    assert response["executed_arguments"]["max_speed_mps"] == 1.50
    assert response["executed_arguments"]["move_model_file"] == str(model_path)
    assert runtime.moves == [
        {
            "distance_m": 0.50,
            "speed_mps": 0.75,
            "duration_s": 0.80,
        }
    ]
    assert response["telemetry"]["phase"] == "real_adapter_phase1"
    assert response["telemetry"]["dry_run"] is False
    assert response["telemetry"]["completion"]["capture_timing"] == "after_settle"
    assert response["telemetry"]["move_model"]["enabled"] is True


def test_real_executor_clamps_rotate_rate_to_real_safety_limit() -> None:
    runtime = FakeRealRuntime(
        config=RealBridgeConfig(
            motion_enabled=True,
            use_move_model=False,
            max_rotate_rate_deg_s=135.0,
        )
    )
    service = _service(runtime)

    response = service.execute_action(
        {
            "episode_id": "real_episode",
            "step_id": 4,
            "runtime_mode": "real",
            "skill_name": "navigate.turn_right",
            "arguments": {"degrees": 30},
            "safety": {"rate_deg_s": 180, "timeout_s": 5.0},
        }
    )

    assert response["ok"] is True
    assert runtime.rotates == [{"degrees": -30.0, "rate_deg_s": 135.0, "timeout_s": 5.0}]


def test_real_runtime_rotate_applies_yaw_error_feedback_to_facing_command() -> None:
    runtime = RealRuntimeClient(
        RealBridgeConfig(
            motion_enabled=True,
            camera_enabled=False,
            camera_required=False,
            rate_hz=1000.0,
            rotate_correction_retries=0,
            rotate_feedback_gain=1.0,
            rotate_feedback_limit_deg=20.0,
        )
    )
    publisher = _attach_runtime_fakes(runtime)

    telemetry = runtime.rotate(degrees=10.0, rate_deg_s=1.0, timeout_s=0.02)

    commanded_yaws = [math.degrees(math.atan2(facing[1], facing[0])) for facing in publisher.facings]
    assert max(commanded_yaws) > 8.0
    assert telemetry["rotate_controller"]["feedback_gain"] == 1.0
    assert telemetry["completion"]["attempt_count"] == 1


def test_real_runtime_rotate_main_attempt_has_minimum_window() -> None:
    runtime = RealRuntimeClient(
        RealBridgeConfig(
            motion_enabled=True,
            camera_enabled=False,
            camera_required=False,
            max_rotate_rate_deg_s=135.0,
            rotate_correction_retries=1,
            rotate_main_min_time_s=2.0,
            rotate_settle_time_s=0.5,
        )
    )
    attempt_timeouts: list[float] = []
    _attach_runtime_fakes(runtime)

    def record_timeout(**kwargs: Any) -> tuple[bool, float]:
        attempt_timeouts.append(float(kwargs["timeout_s"]))
        return False, math.radians(30.0)

    runtime._run_rotate_attempt = record_timeout  # type: ignore[method-assign]

    telemetry = runtime.rotate(degrees=-30.0, rate_deg_s=135.0, timeout_s=5.0)

    assert attempt_timeouts[0] == pytest.approx(2.0)
    assert telemetry["rotate_controller"]["main_min_time_s"] == 2.0


def test_real_runtime_rotate_uses_boosted_residual_correction_attempt() -> None:
    runtime = RealRuntimeClient(
        RealBridgeConfig(
            motion_enabled=True,
            camera_enabled=False,
            camera_required=False,
            rate_hz=1000.0,
            rotate_correction_retries=1,
            rotate_correction_boost_deg=10.0,
            rotate_correction_min_time_s=0.01,
            rotate_correction_extra_time_s=0.0,
            rotate_feedback_gain=0.0,
            rotate_main_min_time_s=0.0,
            rotate_settle_time_s=0.0,
            rotate_tolerance_deg=0.1,
        )
    )
    _attach_runtime_fakes(runtime)

    telemetry = runtime.rotate(degrees=1.0, rate_deg_s=1000.0, timeout_s=0.05)

    attempts = telemetry["rotate_controller"]["attempts"]
    assert telemetry["completed"] is False
    assert telemetry["completion"]["attempt_count"] == 2
    assert attempts[1]["command_target_deg"] == pytest.approx(11.0)


def test_real_runtime_rotate_accepts_final_error_inside_tolerance_after_idle() -> None:
    runtime = RealRuntimeClient(
        RealBridgeConfig(
            motion_enabled=True,
            camera_enabled=False,
            camera_required=False,
            rotate_correction_retries=0,
            rotate_tolerance_deg=5.0,
        )
    )
    _attach_runtime_fakes(runtime, FinalHeadingSubscriber(0.0, math.radians(29.0)))

    def timeout_inside_tolerance(**kwargs: Any) -> tuple[bool, float]:
        return False, math.radians(1.0)

    runtime._run_rotate_attempt = timeout_inside_tolerance  # type: ignore[method-assign]

    telemetry = runtime.rotate(degrees=30.0, rate_deg_s=30.0, timeout_s=1.5)

    assert telemetry["completed"] is True
    assert telemetry["completion"]["completion_detail"] == "yaw_tolerance_after_idle"
    assert telemetry["motion_result"]["final_error_deg"] == pytest.approx(1.0)
    assert telemetry["motion_result"]["estimated_degrees"] == pytest.approx(29.0)


def test_real_executor_halts_on_runtime_exception() -> None:
    runtime = FakeRealRuntime(config=RealBridgeConfig(motion_enabled=True, use_move_model=False), fail_move=True)
    service = _service(runtime)

    response = service.execute_action(
        {
            "episode_id": "real_episode",
            "step_id": 3,
            "runtime_mode": "real",
            "skill_name": "navigate.forward",
            "arguments": {"distance_m": 0.10},
            "safety": {"max_speed_mps": 0.20, "timeout_s": 5.0},
        }
    )

    assert response["ok"] is False
    assert response["action_status"] == "failed"
    assert "simulated real runtime move failure" in response["error_message"]
    assert runtime.halted is True
    assert response["telemetry"]["halt_called"] is True


def test_real_sensor_provider_normalizes_camera_payload() -> None:
    runtime = FakeRealRuntime()
    provider = RealSensorProvider(runtime=runtime, runtime_mode="real")

    observation = provider.get_observation()

    assert observation["runtime_mode"] == "real"
    assert observation["images"]["ego_view"] == {
        "encoding": "jpeg-base64",
        "data": "ZmFrZS1yZWFsLWpwZWc=",
    }
    assert observation["images"]["ego_view_depth"] == {
        "encoding": "jpeg-base64",
        "data": "ZmFrZS1yZWFsLWRlcHRo",
    }
    assert observation["camera_timestamps"]["ego_view"] == 456.0
    assert observation["camera_timestamps"]["ego_view_depth"] == 456.0
    assert observation["robot_state"]["source"] == "fake_real"
    assert observation["perception"]["source"] == "none"


def test_real_service_factory_does_not_enable_motion_by_default() -> None:
    service = create_real_bridge_service(RealBridgeConfig(runtime_mode="real"))

    response = service.handshake(
        {
            "protocol_version": "vigil_groot_bridge_v1",
            "runtime_mode": "real",
        }
    )

    assert response["ok"] is True
    assert response["runtime_mode"] == "real"
    assert response["capabilities"]["oracle_source"] == "none"
    service.close()


def test_real_auto_start_sends_command_before_waiting_for_state(monkeypatch: pytest.MonkeyPatch) -> None:
    events: list[str] = []

    class FakePublisher:
        def __init__(self, bind_host: str, port: int, verbose: bool = False) -> None:
            events.append(f"publisher:{bind_host}:{port}")

        def send_command(self, start: bool, stop: bool, planner: bool = True, pause: bool = False) -> None:
            events.append(f"command:{int(start)}:{int(stop)}:{int(planner)}:{int(pause)}")

        def send_planner(
            self,
            mode: int,
            movement: list[float],
            facing: list[float],
            speed: float = -1.0,
            height: float = -1.0,
        ) -> None:
            events.append(f"planner:{mode}")

        def close(self) -> None:
            events.append("publisher_closed")

    class FakeStateSubscriber:
        def __init__(self, host: str, port: int, topic: str) -> None:
            events.append(f"state_sub:{host}:{port}:{topic}")

        def start(self) -> None:
            events.append("state_sub_started")

        def latest(self) -> None:
            return None

        def wait_for_state(self, timeout: float) -> Any:
            events.append("wait_for_state")
            assert "command:1:0:1:0" in events

            @dataclass
            class State:
                yaw: float = 0.0
                base_quat: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
                delta_heading: float = 0.0
                yaw_rate: float = 0.0
                timestamp: float = 0.0

            return State()

        def close(self) -> None:
            events.append("state_sub_closed")

    monkeypatch.setattr("gear_sonic.vigil_bridge.real_adapter.PackedPublisher", FakePublisher)
    monkeypatch.setattr("gear_sonic.vigil_bridge.real_adapter.StateSubscriber", FakeStateSubscriber)

    runtime = RealRuntimeClient(
        RealBridgeConfig(
            motion_enabled=True,
            auto_start_control=True,
            camera_enabled=False,
            camera_required=False,
            state_timeout_s=0.01,
            startup_command_burst_s=0.0,
        )
    )

    health = runtime.start()

    assert health["ok"] is True
    assert health["executor_started"] is True
    assert events.index("command:1:0:1:0") < events.index("wait_for_state")
