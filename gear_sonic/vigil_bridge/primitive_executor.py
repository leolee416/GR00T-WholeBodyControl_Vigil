"""Dry-run primitive execution for the GR00T-side Vigil bridge skeleton."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from gear_sonic.vigil_bridge.protocol import ExecuteActionResponse, JSONDict, RuntimeHealth


@dataclass
class DryRunPrimitiveExecutor:
    """Primitive executor that never calls real WBC or robot deployment code."""

    runtime_mode: str = "dry_run"
    default_distance_m: float = 0.25
    default_turn_degrees: float = 30.0
    default_speed_mps: float = 0.25
    default_rate_deg_s: float = 30.0
    started: bool = False
    _command_index: int = 0
    _last_telemetry: JSONDict = field(default_factory=dict)

    def start(self) -> RuntimeHealth:
        self.started = True
        self._last_telemetry = {"event": "start", "dry_run": True}
        return self.get_health()

    def halt(self) -> RuntimeHealth:
        self.started = False
        self._last_telemetry = {"event": "halt", "dry_run": True}
        return self.get_health()

    def move(
        self,
        distance_m: float,
        speed_mps: float | None = None,
        timeout_s: float | None = None,
    ) -> ExecuteActionResponse:
        speed = self._positive_float_or_default(speed_mps, self.default_speed_mps, "speed_mps")
        timeout = self._optional_positive_float(timeout_s, "timeout_s")
        distance = self._float(distance_m, "distance_m")
        estimated_duration_s = abs(distance) / speed if speed > 0.0 else None

        return self._success(
            executed_arguments={
                "primitive": "move_model",
                "distance_m": distance,
                "speed_mps": speed,
                "timeout_s": timeout,
            },
            telemetry={
                "motion": "move",
                "estimated_duration_s": estimated_duration_s,
            },
        )

    def rotate(
        self,
        degrees: float,
        rate_deg_s: float | None = None,
        timeout_s: float | None = None,
    ) -> ExecuteActionResponse:
        rate = self._positive_float_or_default(rate_deg_s, self.default_rate_deg_s, "rate_deg_s")
        timeout = self._optional_positive_float(timeout_s, "timeout_s")
        angle = self._float(degrees, "degrees")
        estimated_duration_s = abs(angle) / rate if rate > 0.0 else None

        return self._success(
            executed_arguments={
                "primitive": "rotate",
                "degrees": angle,
                "rate_deg_s": rate,
                "timeout_s": timeout,
            },
            telemetry={
                "motion": "rotate",
                "estimated_duration_s": estimated_duration_s,
            },
        )

    def execute_action(
        self,
        skill_name: str,
        arguments: Mapping[str, Any] | None,
        safety: Mapping[str, Any] | None,
    ) -> ExecuteActionResponse:
        args = dict(arguments or {})
        safety_args = dict(safety or {})
        skill = str(skill_name or "").strip().lower()

        try:
            if skill == "navigate.forward":
                distance = self._distance_from_arguments(args, sign=1.0)
                return self.move(
                    distance_m=distance,
                    speed_mps=self._safety_value(safety_args, "max_speed_mps", "speed_mps"),
                    timeout_s=self._safety_value(safety_args, "timeout_s"),
                )
            if skill == "navigate.backward":
                distance = self._distance_from_arguments(args, sign=-1.0)
                return self.move(
                    distance_m=distance,
                    speed_mps=self._safety_value(safety_args, "max_speed_mps", "speed_mps"),
                    timeout_s=self._safety_value(safety_args, "timeout_s"),
                )
            if skill == "navigate.turn_left":
                degrees = self._degrees_from_arguments(args, sign=1.0)
                return self.rotate(
                    degrees=degrees,
                    rate_deg_s=self._safety_value(safety_args, "max_rate_deg_s", "rate_deg_s"),
                    timeout_s=self._safety_value(safety_args, "timeout_s"),
                )
            if skill == "navigate.turn_right":
                degrees = self._degrees_from_arguments(args, sign=-1.0)
                return self.rotate(
                    degrees=degrees,
                    rate_deg_s=self._safety_value(safety_args, "max_rate_deg_s", "rate_deg_s"),
                    timeout_s=self._safety_value(safety_args, "timeout_s"),
                )
            if skill == "report":
                return self._success(
                    executed_arguments={"primitive": "report", "motion": "none"},
                    telemetry={"motion": "none", "report_handled_by": "caller"},
                )
        except ValueError as exc:
            return self._failure(str(exc), {"skill_name": skill})

        return self._failure(
            f"unsupported skill_name: {skill_name}",
            {"skill_name": str(skill_name or "")},
        )

    def get_health(self, sensor_connected: bool = True) -> RuntimeHealth:
        return {
            "ok": True,
            "runtime_mode": self.runtime_mode,
            "executor_started": self.started,
            "sensor_connected": sensor_connected,
            "error_message": None,
            "telemetry": {
                "executor": "dry_run",
                "dry_run": True,
                "last_event": self._last_telemetry,
            },
        }

    def _success(self, executed_arguments: JSONDict, telemetry: JSONDict) -> ExecuteActionResponse:
        command_id = self._next_command_id()
        motion = str(telemetry.get("motion", "none"))
        motion_commanded = motion not in {"", "none"}
        completion: JSONDict = {
            "motion_commanded": motion_commanded,
            "completion_source": "dry_run",
            "capture_timing": "after_completion",
            "settled": True,
            "duration_s": 0.0,
        }
        estimated_duration_s = telemetry.get("estimated_duration_s")
        if isinstance(estimated_duration_s, int | float):
            completion["command_duration_s"] = float(estimated_duration_s)
        response_telemetry: JSONDict = {
            "command_id": command_id,
            "runtime_mode": self.runtime_mode,
            "controller": "dry_run",
            "dry_run": True,
            "executor_started": self.started,
            "completion": completion,
        }
        response_telemetry.update(telemetry)
        self._last_telemetry = response_telemetry
        return {
            "ok": True,
            "error_message": None,
            "action_status": "completed",
            "executed_arguments": executed_arguments,
            "telemetry": response_telemetry,
        }

    def _failure(self, error_message: str, telemetry: JSONDict | None = None) -> ExecuteActionResponse:
        response_telemetry: JSONDict = {
            "runtime_mode": self.runtime_mode,
            "controller": "dry_run",
            "dry_run": True,
            "executor_started": self.started,
            "completion": {
                "motion_commanded": False,
                "completion_source": "validation",
                "capture_timing": "not_started",
                "settled": False,
                "duration_s": 0.0,
            },
        }
        if telemetry:
            response_telemetry.update(telemetry)
        self._last_telemetry = response_telemetry
        return {
            "ok": False,
            "error_message": error_message,
            "action_status": "rejected",
            "executed_arguments": {},
            "telemetry": response_telemetry,
        }

    def _next_command_id(self) -> str:
        self._command_index += 1
        return f"dry_run_{self._command_index:04d}"

    def _distance_from_arguments(self, arguments: Mapping[str, Any], sign: float) -> float:
        if "distance_m" in arguments:
            magnitude = abs(self._float(arguments["distance_m"], "distance_m"))
        elif "magnitude" in arguments:
            magnitude = abs(self._float(arguments["magnitude"], "magnitude")) * self.default_distance_m
        else:
            magnitude = self.default_distance_m
        return sign * magnitude

    def _degrees_from_arguments(self, arguments: Mapping[str, Any], sign: float) -> float:
        if "degrees" in arguments:
            magnitude = abs(self._float(arguments["degrees"], "degrees"))
        elif "angle_deg" in arguments:
            magnitude = abs(self._float(arguments["angle_deg"], "angle_deg"))
        elif "magnitude" in arguments:
            magnitude = abs(self._float(arguments["magnitude"], "magnitude")) * self.default_turn_degrees
        else:
            magnitude = self.default_turn_degrees
        return sign * magnitude

    def _safety_value(self, safety: Mapping[str, Any], *keys: str) -> float | None:
        for key in keys:
            if key in safety:
                value = safety[key]
                return None if value is None else self._float(value, key)
        return None

    def _positive_float_or_default(
        self,
        value: float | None,
        default: float,
        field_name: str,
    ) -> float:
        candidate = default if value is None else self._float(value, field_name)
        if candidate <= 0.0:
            raise ValueError(f"{field_name} must be positive")
        return candidate

    def _optional_positive_float(self, value: float | None, field_name: str) -> float | None:
        if value is None:
            return None
        candidate = self._float(value, field_name)
        if candidate <= 0.0:
            raise ValueError(f"{field_name} must be positive")
        return candidate

    @staticmethod
    def _float(value: Any, field_name: str) -> float:
        try:
            return float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field_name} must be numeric") from exc


class FakePrimitiveExecutor(DryRunPrimitiveExecutor):
    """Backward-compatible name for tests and fake clients."""
