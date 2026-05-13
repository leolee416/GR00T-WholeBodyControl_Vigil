"""Service facade for the GR00T-side Vigil bridge skeleton."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from gear_sonic.vigil_bridge.primitive_executor import DryRunPrimitiveExecutor
from gear_sonic.vigil_bridge.protocol import (
    BRIDGE_NAME,
    BRIDGE_VERSION,
    ExecuteActionResponse,
    HandshakeResponse,
    ObservationResponse,
    ORACLE_SOURCE,
    PROTOCOL_VERSION,
    ResetEpisodeResponse,
    RobotStateResponse,
    RuntimeHealth,
    SUPPORTED_ACTIONS,
    SUPPORTED_OBSERVATIONS,
)
from gear_sonic.vigil_bridge.sensors import FakeSensorProvider


@dataclass
class VigilBridgeService:
    """Composes a primitive executor and sensor provider behind the bridge API."""

    executor: DryRunPrimitiveExecutor | None = None
    sensor_provider: FakeSensorProvider | None = None
    runtime_mode: str = "dry_run"
    _closed: bool = False

    def __post_init__(self) -> None:
        if self.executor is None:
            self.executor = DryRunPrimitiveExecutor(runtime_mode=self.runtime_mode)
        if self.sensor_provider is None:
            self.sensor_provider = FakeSensorProvider(runtime_mode=self.runtime_mode)

    def handshake(self, payload: dict[str, Any]) -> HandshakeResponse:
        self._set_runtime_mode(str(payload["runtime_mode"]))
        return {
            "ok": True,
            "error_message": None,
            "protocol_version": PROTOCOL_VERSION,
            "runtime_mode": self.runtime_mode,
            "capabilities": {
                "actions": list(SUPPORTED_ACTIONS),
                "observation": list(SUPPORTED_OBSERVATIONS),
                "oracle_source": ORACLE_SOURCE,
            },
            "bridge": {
                "name": BRIDGE_NAME,
                "version": BRIDGE_VERSION,
            },
        }

    def reset_episode(self, payload: Mapping[str, Any]) -> ResetEpisodeResponse:
        if self._closed:
            return self._reset_error("bridge service is closed")

        self._set_runtime_mode(str(payload.get("runtime_mode", self.runtime_mode)))
        assert self.executor is not None
        assert self.sensor_provider is not None

        self.executor.start()
        state_response = self.sensor_provider.get_robot_state()
        return {
            "ok": bool(state_response.get("ok", True)),
            "error_message": state_response.get("error_message"),
            "episode_id": self._optional_str(payload.get("episode_id")),
            "runtime_mode": self.runtime_mode,
            "robot_state": dict(state_response.get("robot_state", {})),
            "telemetry": {
                "bridge": "groot_vigil_bridge",
                "phase": self._bridge_phase(),
                "dry_run": self._bridge_dry_run(),
                "health": self._runtime_health(),
            },
        }

    def execute_action(self, payload: Mapping[str, Any]) -> ExecuteActionResponse:
        if self._closed:
            return self._action_error("bridge service is closed")

        self._set_runtime_mode(str(payload.get("runtime_mode", self.runtime_mode)))
        assert self.executor is not None
        assert self.sensor_provider is not None

        before_response = self.sensor_provider.get_robot_state()
        robot_state_before = dict(before_response.get("robot_state", {}))
        try:
            action_response = self.executor.execute_action(
                skill_name=str(payload.get("skill_name", "")),
                arguments=self._mapping_or_empty(payload.get("arguments")),
                safety=self._mapping_or_empty(payload.get("safety")),
            )
        except Exception as exc:
            halt_health = self.executor.halt()
            return {
                "ok": False,
                "error_message": str(exc),
                "action_status": "failed",
                "executed_arguments": {},
                "robot_state_before": robot_state_before,
                "robot_state_after": robot_state_before,
                "telemetry": {
                    "bridge": "groot_vigil_bridge",
                    "phase": self._bridge_phase(),
                    "halt_called": True,
                    "halt_health": halt_health,
                    "completion": {
                        "motion_commanded": False,
                        "completion_source": "exception",
                        "capture_timing": "after_halt",
                        "settled": False,
                        "duration_s": 0.0,
                    },
                },
            }

        after_response = self.sensor_provider.get_robot_state()
        executed_arguments = dict(action_response.get("executed_arguments", {}))
        if executed_arguments:
            executed_arguments.setdefault("skill_name", str(payload.get("skill_name", "")))
        telemetry = {
            "bridge": "groot_vigil_bridge",
            "phase": self._bridge_phase(),
            "episode_id": self._optional_str(payload.get("episode_id")),
            "step_id": payload.get("step_id"),
        }
        telemetry.update(dict(action_response.get("telemetry", {})))
        return {
            "ok": bool(action_response.get("ok", False)),
            "error_message": action_response.get("error_message"),
            "action_status": self._action_status(action_response),
            "executed_arguments": executed_arguments,
            "robot_state_before": robot_state_before,
            "robot_state_after": dict(after_response.get("robot_state", {})),
            "telemetry": telemetry,
        }

    def get_observation(self, payload: Mapping[str, Any] | None = None) -> ObservationResponse:
        if payload is not None:
            self._set_runtime_mode(str(payload.get("runtime_mode", self.runtime_mode)))
        assert self.sensor_provider is not None
        observation = self.sensor_provider.get_observation()
        telemetry = {
            "bridge": "groot_vigil_bridge",
            "phase": self._bridge_phase(),
        }
        telemetry.update(dict(observation.get("telemetry", {})))
        observation["telemetry"] = telemetry
        return observation

    def get_robot_state(self) -> RobotStateResponse:
        assert self.sensor_provider is not None
        state_response = self.sensor_provider.get_robot_state()
        telemetry = {
            "bridge": "groot_vigil_bridge",
            "phase": self._bridge_phase(),
        }
        telemetry.update(dict(state_response.get("telemetry", {})))
        state_response["telemetry"] = telemetry
        return state_response

    def halt(self) -> RuntimeHealth:
        assert self.executor is not None
        return self.executor.halt()

    def close(self) -> None:
        if not self._closed:
            assert self.executor is not None
            executor_close = getattr(self.executor, "close", None)
            if callable(executor_close):
                executor_close()
            else:
                self.halt()
        self._closed = True

    def _set_runtime_mode(self, runtime_mode: str) -> None:
        self.runtime_mode = runtime_mode
        if self.executor is not None:
            self.executor.runtime_mode = runtime_mode
        if self.sensor_provider is not None:
            self.sensor_provider.runtime_mode = runtime_mode

    def _runtime_health(self) -> RuntimeHealth:
        assert self.executor is not None
        assert self.sensor_provider is not None
        return self.executor.get_health(sensor_connected=self.sensor_provider.connected)

    def _reset_error(self, error_message: str) -> ResetEpisodeResponse:
        return {
            "ok": False,
            "error_message": error_message,
            "episode_id": None,
            "runtime_mode": self.runtime_mode,
            "robot_state": {},
            "telemetry": {
                "bridge": "groot_vigil_bridge",
                "phase": self._bridge_phase(),
                "dry_run": self._bridge_dry_run(),
            },
        }

    def _action_error(self, error_message: str) -> ExecuteActionResponse:
        return {
            "ok": False,
            "error_message": error_message,
            "action_status": "failed",
            "executed_arguments": {},
            "robot_state_before": {},
            "robot_state_after": {},
            "telemetry": {
                "bridge": "groot_vigil_bridge",
                "phase": self._bridge_phase(),
                "dry_run": self._bridge_dry_run(),
                "completion": {
                    "motion_commanded": False,
                    "completion_source": "bridge_service",
                    "capture_timing": "not_started",
                    "settled": False,
                    "duration_s": 0.0,
                },
            },
        }

    def _bridge_phase(self) -> str:
        executor_name = type(self.executor).__name__.lower() if self.executor is not None else ""
        if "mujoco" in executor_name:
            return "mujoco_adapter_phase3"
        if "real" in executor_name:
            return "real_adapter_phase1"
        return "dry_run_skeleton"

    def _bridge_dry_run(self) -> bool:
        executor_name = type(self.executor).__name__.lower() if self.executor is not None else ""
        return "mujoco" not in executor_name and "real" not in executor_name

    @staticmethod
    def _action_status(action_response: Mapping[str, Any]) -> str:
        status = action_response.get("action_status")
        if isinstance(status, str) and status:
            return status
        return "completed" if bool(action_response.get("ok", False)) else "failed"

    @staticmethod
    def _mapping_or_empty(value: Any) -> Mapping[str, Any]:
        return value if isinstance(value, Mapping) else {}

    @staticmethod
    def _optional_str(value: Any) -> str | None:
        return None if value is None else str(value)
