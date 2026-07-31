"""Service facade for the GR00T-side Vigil bridge skeleton."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from gear_sonic.vigil_bridge.audio import AudioSessionManager
from gear_sonic.vigil_bridge.primitive_executor import DryRunPrimitiveExecutor
from gear_sonic.vigil_bridge.protocol import (
    AUDIO_CAPABILITIES,
    BRIDGE_NAME,
    BRIDGE_VERSION,
    AudioResponse,
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
from gear_sonic.vigil_bridge.rollout_recorder import RolloutRecorder
from gear_sonic.vigil_bridge.sensors import FakeSensorProvider
from gear_sonic.vigil_bridge.sit_chair_diagnostics import (
    analyze_rollout,
    diagnose_preflight,
    select_rollout_session,
)


@dataclass
class VigilBridgeService:
    """Composes a primitive executor and sensor provider behind the bridge API."""

    executor: DryRunPrimitiveExecutor | None = None
    sensor_provider: FakeSensorProvider | None = None
    audio_manager: AudioSessionManager | None = None
    rollout_recorder: RolloutRecorder | None = None
    audio_advertise_always: bool = False
    runtime_mode: str = "dry_run"
    _closed: bool = False

    def __post_init__(self) -> None:
        if self.executor is None:
            self.executor = DryRunPrimitiveExecutor(runtime_mode=self.runtime_mode)
        if self.sensor_provider is None:
            self.sensor_provider = FakeSensorProvider(runtime_mode=self.runtime_mode)

    def handshake(self, payload: dict[str, Any]) -> HandshakeResponse:
        self._set_runtime_mode(str(payload["runtime_mode"]))
        capabilities = {
            "actions": list(SUPPORTED_ACTIONS),
            "observation": list(SUPPORTED_OBSERVATIONS),
            "oracle_source": ORACLE_SOURCE,
        }
        if self._should_advertise_audio(payload):
            capabilities["audio"] = self._audio_capabilities()
        if self.rollout_recorder is not None:
            capabilities["rollout_recording"] = {
                "source": "g1_debug",
                "measured_joints": 29,
                "external_base_localization": True,
                "default_capture": {
                    "skill": "sonic.sit_chair",
                    "pre_roll_max_s": 3.0,
                    "post_roll_max_s": 3.0,
                    "auto_export": True,
                },
                "exports": [
                    "raw_real_rollout.npz",
                    "3dgs_replay.npz",
                    "gear_sonic_reference.npz",
                ],
            }
        return {
            "ok": True,
            "error_message": None,
            "protocol_version": PROTOCOL_VERSION,
            "runtime_mode": self.runtime_mode,
            "capabilities": capabilities,
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
        self._update_rollout_action_context(payload)
        self._begin_rollout_action(payload)
        try:
            action_response = self.executor.execute_action(
                skill_name=str(payload.get("skill_name", "")),
                arguments=self._mapping_or_empty(payload.get("arguments")),
                safety=self._mapping_or_empty(payload.get("safety")),
            )
        except Exception as exc:
            self._finish_rollout_action(
                payload,
                action_status="failed",
                motion_commanded=None,
            )
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

        executed_arguments = dict(action_response.get("executed_arguments", {}))
        if executed_arguments:
            executed_arguments.setdefault("skill_name", str(payload.get("skill_name", "")))
            self._update_rollout_action_context(
                {
                    **dict(payload),
                    "arguments": executed_arguments,
                }
            )
        completion = self._mapping_or_empty(
            self._mapping_or_empty(action_response.get("telemetry")).get("completion")
        )
        self._finish_rollout_action(
            payload,
            action_status=self._action_status(action_response),
            motion_commanded=completion.get("motion_commanded"),
            executed_arguments=executed_arguments,
        )
        after_response = self.sensor_provider.get_robot_state()
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

    def diagnose_sit_chair_preflight(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Read current state, validate a reference, and optionally arm capture."""
        if self._closed:
            return {"ok": False, "error_message": "bridge service is closed"}
        self._set_runtime_mode(str(payload.get("runtime_mode", self.runtime_mode)))
        assert self.executor is not None
        assert self.sensor_provider is not None
        try:
            state_response = self.sensor_provider.get_robot_state()
            response = diagnose_preflight(
                executor=self.executor,
                state_response=state_response,
                request=payload,
            )
            if bool(payload.get("arm_rollout", False)):
                if self.rollout_recorder is None:
                    raise RuntimeError("rollout recorder is not configured")
                selected = response["selected_reference"]
                session_name = str(
                    payload.get(
                        "session_name",
                        f"facee_diagnostic_{selected['tag']}",
                    )
                )
                response["rollout"] = self.rollout_recorder.start(
                    {
                        "session_name": session_name,
                        "capture_mode": "skill_window",
                        "capture_skill": "sonic.sit_chair",
                        "pre_roll_s": float(payload.get("pre_roll_s", 3.0)),
                        "post_roll_s": float(payload.get("post_roll_s", 3.0)),
                        "target_fps": float(payload.get("target_fps", 50.0)),
                        "checkpoint": str(
                            payload.get(
                                "checkpoint",
                                "policy/facee_v73_noheight/model",
                            )
                        ),
                        "observation_config": str(
                            payload.get(
                                "observation_config",
                                "policy/facee_v73_noheight/observation_config.yaml",
                            )
                        ),
                    }
                )
            return response
        except Exception as exc:
            return {
                "ok": False,
                "error_message": str(exc),
                "diagnostic": "sonic.sit_chair_preflight",
                "motion_commanded": False,
            }

    def analyze_sit_chair_rollout(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Analyze an exported rollout without commanding the runtime."""
        if self.rollout_recorder is None:
            return self._rollout_unavailable()
        try:
            session_dir = select_rollout_session(
                self.rollout_recorder.output_root,
                payload.get("session_id"),
            )
            return analyze_rollout(session_dir)
        except Exception as exc:
            return {
                "ok": False,
                "error_message": str(exc),
                "diagnostic": "sonic.sit_chair_rollout_analysis",
            }

    def halt(self) -> RuntimeHealth:
        assert self.executor is not None
        if self.audio_manager is not None:
            self.audio_manager.stop_output()
        return self.executor.halt()

    def pause(self) -> RuntimeHealth:
        assert self.executor is not None
        pause = getattr(self.executor, "pause", None)
        if callable(pause):
            return pause()
        return self.executor.halt()

    def resume(self) -> RuntimeHealth:
        assert self.executor is not None
        resume = getattr(self.executor, "resume", None)
        if callable(resume):
            return resume()
        return self.executor.start()

    def start_rollout(self, payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if self.rollout_recorder is None:
            return self._rollout_unavailable()
        try:
            return self.rollout_recorder.start(payload)
        except Exception as exc:
            return {"ok": False, "error_message": str(exc), "active": self.rollout_recorder.active}

    def stop_rollout(self, payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if self.rollout_recorder is None:
            return self._rollout_unavailable()
        try:
            return self.rollout_recorder.stop(payload)
        except Exception as exc:
            return {"ok": False, "error_message": str(exc), "active": self.rollout_recorder.active}

    def get_rollout_status(self) -> dict[str, Any]:
        if self.rollout_recorder is None:
            return self._rollout_unavailable()
        return self.rollout_recorder.status()

    def update_rollout_localization(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        if self.rollout_recorder is None:
            return self._rollout_unavailable()
        try:
            return self.rollout_recorder.update_localization(payload)
        except Exception as exc:
            return {"ok": False, "error_message": str(exc), "active": self.rollout_recorder.active}

    def update_rollout_context(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        if self.rollout_recorder is None:
            return self._rollout_unavailable()
        try:
            return self.rollout_recorder.set_motion_context(payload)
        except Exception as exc:
            return {"ok": False, "error_message": str(exc), "active": self.rollout_recorder.active}

    def close(self) -> None:
        if not self._closed:
            if self.rollout_recorder is not None:
                self.rollout_recorder.close()
            if self.audio_manager is not None:
                self.audio_manager.close()
            assert self.executor is not None
            executor_close = getattr(self.executor, "close", None)
            if callable(executor_close):
                executor_close()
            else:
                self.halt()
        self._closed = True

    def get_audio_health(self) -> AudioResponse:
        if self.audio_manager is None:
            return self._audio_unavailable()
        return self.audio_manager.health()

    def start_audio_session(self, payload: Mapping[str, Any] | None = None) -> AudioResponse:
        if self.audio_manager is None:
            return self._audio_unavailable()
        return self.audio_manager.start_session(payload)

    def stop_audio_session(self, payload: Mapping[str, Any] | None = None) -> AudioResponse:
        if self.audio_manager is None:
            return self._audio_unavailable()
        return self.audio_manager.stop_session(payload)

    def get_audio_input_segment(self, payload: Mapping[str, Any] | None = None) -> AudioResponse:
        if self.audio_manager is None:
            return self._audio_unavailable()
        return self.audio_manager.input_segment(payload)

    def play_audio_output_segment(self, payload: Mapping[str, Any] | None = None) -> AudioResponse:
        if self.audio_manager is None:
            return self._audio_unavailable()
        return self.audio_manager.output_segment(payload)

    def play_audio_tts(self, payload: Mapping[str, Any] | None = None) -> AudioResponse:
        if self.audio_manager is None:
            return self._audio_unavailable()
        return self.audio_manager.tts(payload)

    def stop_audio_output(self) -> AudioResponse:
        if self.audio_manager is None:
            return self._audio_unavailable()
        return self.audio_manager.stop_output()

    def _set_runtime_mode(self, runtime_mode: str) -> None:
        self.runtime_mode = runtime_mode
        if self.executor is not None:
            self.executor.runtime_mode = runtime_mode
        if self.sensor_provider is not None:
            self.sensor_provider.runtime_mode = runtime_mode

    def _runtime_health(self) -> RuntimeHealth:
        assert self.executor is not None
        assert self.sensor_provider is not None
        health = self.executor.get_health(sensor_connected=self.sensor_provider.connected)
        telemetry = dict(health.get("telemetry", {}))
        if self.audio_manager is not None:
            telemetry["audio"] = self._audio_capabilities()
        health["telemetry"] = telemetry
        return health

    def _audio_capabilities(self) -> dict[str, Any]:
        if self.audio_manager is None:
            return dict(AUDIO_CAPABILITIES)
        return self.audio_manager.capabilities()

    def _update_rollout_action_context(self, payload: Mapping[str, Any]) -> None:
        if self.rollout_recorder is None:
            return
        arguments = self._mapping_or_empty(payload.get("arguments"))
        context: dict[str, Any] = {
            "skill_name": str(payload.get("skill_name", "")),
        }
        for key in (
            "episode_id",
            "step_id",
            "motion_name",
            "reference_distance_m",
            "chair_distance_m",
            "tag",
        ):
            if key in payload:
                context[key] = payload[key]
            elif key in arguments:
                context[key] = arguments[key]
        try:
            self.rollout_recorder.set_motion_context(context)
        except (TypeError, ValueError):
            # Recording metadata must never break execution or trigger robot halt.
            return

    def _begin_rollout_action(self, payload: Mapping[str, Any]) -> None:
        if self.rollout_recorder is None:
            return
        try:
            self.rollout_recorder.begin_action(payload)
        except (RuntimeError, TypeError, ValueError):
            # Recording must never block or alter robot action execution.
            return

    def _finish_rollout_action(
        self,
        payload: Mapping[str, Any],
        *,
        action_status: str,
        motion_commanded: Any,
        executed_arguments: Mapping[str, Any] | None = None,
    ) -> None:
        if self.rollout_recorder is None:
            return
        try:
            finish_payload = dict(payload)
            for key in (
                "duration_s",
                "motion_name",
                "reference_distance_m",
                "chair_distance_m",
                "tag",
            ):
                if executed_arguments is not None and key in executed_arguments:
                    finish_payload[key] = executed_arguments[key]
            self.rollout_recorder.finish_action(
                {
                    **finish_payload,
                    "action_status": action_status,
                    "motion_commanded": motion_commanded,
                }
            )
        except (RuntimeError, TypeError, ValueError):
            # Recording must never block or alter robot action execution.
            return

    def _should_advertise_audio(self, payload: Mapping[str, Any]) -> bool:
        if self.audio_advertise_always:
            return True
        if bool(payload.get("include_audio_capabilities", False)):
            return True
        required_capabilities = payload.get("required_capabilities")
        return isinstance(required_capabilities, Mapping) and "audio" in required_capabilities

    @staticmethod
    def _audio_unavailable() -> AudioResponse:
        return {
            "ok": False,
            "error_message": "audio manager is not configured",
            "telemetry": {
                "audio": dict(AUDIO_CAPABILITIES),
            },
        }

    @staticmethod
    def _rollout_unavailable() -> dict[str, Any]:
        return {
            "ok": False,
            "error_message": "rollout recorder is not configured for this backend",
            "active": False,
        }

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
