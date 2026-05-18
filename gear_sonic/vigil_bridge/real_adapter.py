"""Real-robot runtime adapter for the GR00T-side Vigil bridge.

This backend intentionally uses the same public runtime transports as the
MuJoCo adapter where possible:

* ZMQ planner/command PUB consumed by deploy when started with zmq_manager.
* ZMQ g1_debug SUB published by deploy when started with output type all/zmq.
* Optional ZMQ camera stream supplied by the real-robot camera stack.

It does not import deploy internals, policy inference, WBC control code, or
Vigil. Real motion is disabled by default and must be enabled explicitly by
configuration.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Mapping

from gear_sonic.vigil_bridge.mujoco_adapter import (
    LOCO_IDLE,
    LOCO_SLOW_WALK,
    LOCO_WALK,
    MoveModel,
    MoveModelSample,
    PackedPublisher,
    REPO_ROOT,
    StateSubscriber,
    ZMQImageSubscriber,
    facing_from_yaw,
    interp_linear,
    resolve_repo_path,
    wrap_pi,
)
from gear_sonic.vigil_bridge.primitive_executor import DryRunPrimitiveExecutor
from gear_sonic.vigil_bridge.protocol import (
    ExecuteActionResponse,
    JSONDict,
    ObservationResponse,
    RobotStateResponse,
    RuntimeHealth,
)
from gear_sonic.vigil_bridge.service import VigilBridgeService


@dataclass(frozen=True)
class RealBridgeConfig:
    """Configuration for the real-robot bridge adapter."""

    runtime_mode: str = "real"
    command_bind_host: str = "*"
    command_port: int = 5556
    state_host: str = "localhost"
    state_port: int = 5557
    state_topic: str = "g1_debug"
    rate_hz: float = 10.0
    default_distance_m: float = 0.10
    default_turn_degrees: float = 15.0
    default_move_speed_mps: float = 0.15
    min_move_speed_mps: float = 0.05
    max_move_speed_mps: float = 2.00
    use_move_model: bool = True
    move_model_file: str = "auto"
    model_chunk_pause_s: float = 0.0
    move_settle_time_s: float = 1.0
    default_rotate_rate_deg_s: float = 20.0
    min_rotate_rate_deg_s: float = 5.0
    max_rotate_rate_deg_s: float = 30.0
    rotate_tolerance_deg: float = 5.0
    rotate_timeout_s: float = 6.0
    rotate_extra_time_s: float = 8.0
    rotate_settle_time_s: float = 0.5
    rotate_yaw_rate_tolerance_deg: float = 10.0
    state_timeout_s: float = 3.0
    startup_command_burst_s: float = 0.5
    startup_command_period_s: float = 0.05
    camera_enabled: bool = True
    camera_required: bool = True
    camera_host: str = "localhost"
    camera_port: int = 5555
    camera_timeout_s: float = 3.0
    motion_enabled: bool = False
    auto_start_control: bool = False
    stop_on_halt: bool = False
    verbose: bool = False


class RealRuntimeClient:
    """Runtime client shared by the real executor and sensor provider."""

    def __init__(self, config: RealBridgeConfig) -> None:
        self.config = config
        self.started = False
        self._startup_error: str | None = None
        self._publisher: PackedPublisher | None = None
        self._state_sub: StateSubscriber | None = None
        self._image_sub: ZMQImageSubscriber | None = None
        self._yaw_origin: float | None = None
        self._last_facing_yaw = 0.0

    def start(self) -> RuntimeHealth:
        if self.started:
            return self.get_health()

        try:
            if self._publisher is None:
                self._publisher = PackedPublisher(
                    self.config.command_bind_host,
                    self.config.command_port,
                    verbose=self.config.verbose,
                )
            if self._state_sub is None:
                self._state_sub = StateSubscriber(
                    self.config.state_host,
                    self.config.state_port,
                    self.config.state_topic,
                )
                self._state_sub.start()
            if self.config.camera_enabled and self._image_sub is None:
                self._image_sub = ZMQImageSubscriber(
                    host=self.config.camera_host,
                    port=self.config.camera_port,
                )

            if self.config.auto_start_control:
                if not self.config.motion_enabled:
                    raise RuntimeError("auto_start_control requires motion_enabled")
                self.send_start_control()

            state = self.wait_for_state(timeout=self.config.state_timeout_s)
            if state is None:
                raise RuntimeError("no real-robot g1_debug state received")
            if self.config.camera_required and self._wait_for_camera(self.config.camera_timeout_s) is None:
                raise RuntimeError("no real-robot camera payload received")

            self.started = True
            self._startup_error = None
        except Exception as exc:  # noqa: BLE001 - return structured health.
            self._startup_error = str(exc)
            self.halt()
        return self.get_health()

    def halt(self) -> RuntimeHealth:
        if self._publisher is not None:
            try:
                self.send_idle_burst(duration=0.4, preserve_facing=True)
                if self.config.stop_on_halt:
                    self._publisher.send_command(start=False, stop=True, planner=True)
            except Exception as exc:  # noqa: BLE001 - surface through health.
                self._startup_error = str(exc)
        self.started = False
        return self.get_health()

    def close(self) -> None:
        self.started = False
        for closeable in (self._image_sub, self._state_sub, self._publisher):
            if closeable is not None:
                closeable.close()
        self._image_sub = None
        self._state_sub = None
        self._publisher = None

    def send_start_control(self) -> None:
        publisher = self._require_publisher()
        deadline = time.monotonic() + max(self.config.startup_command_burst_s, 0.0)
        period_s = max(self.config.startup_command_period_s, 0.01)
        while True:
            publisher.send_command(start=True, stop=False, planner=True)
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                break
            time.sleep(min(period_s, remaining_s))
        self.send_idle_burst(duration=0.5, preserve_facing=False)

    def send_idle_burst(self, duration: float, preserve_facing: bool = False) -> None:
        publisher = self._require_publisher()
        if not preserve_facing:
            state = self.latest_state()
            if state is not None:
                self._ensure_yaw_origin(state)
                self._last_facing_yaw = self._relative_yaw(state)
        facing = facing_from_yaw(self._last_facing_yaw)
        deadline = time.monotonic() + max(duration, 0.0)
        while time.monotonic() < deadline:
            publisher.send_planner(LOCO_IDLE, [0.0, 0.0, 0.0], facing, -1.0, -1.0)
            time.sleep(1.0 / max(self.config.rate_hz, 1.0))

    def move(self, distance_m: float, speed_mps: float, duration_s: float) -> JSONDict:
        publisher = self._require_publisher()
        if speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive")

        state = self.wait_for_state(timeout=self.config.state_timeout_s)
        if state is None:
            raise RuntimeError("no real-robot state available before move")
        self._ensure_yaw_origin(state)
        self._last_facing_yaw = self._relative_yaw(state)

        sign = 1.0 if distance_m >= 0.0 else -1.0
        facing = facing_from_yaw(self._last_facing_yaw)
        movement = [sign * facing[0], sign * facing[1], 0.0]
        mode = LOCO_SLOW_WALK if speed_mps <= 0.20 else LOCO_WALK
        started_at = time.monotonic()

        try:
            deadline = time.monotonic() + max(duration_s, 0.0)
            while time.monotonic() < deadline:
                publisher.send_planner(mode, movement, facing, speed_mps, -1.0)
                time.sleep(1.0 / max(self.config.rate_hz, 1.0))
        finally:
            self.send_idle_burst(
                duration=self.config.move_settle_time_s,
                preserve_facing=True,
            )

        return {
            "motion": "move",
            "completion": {
                "motion_commanded": True,
                "completion_source": "duration_and_settle",
                "capture_timing": "after_settle",
                "settled": True,
                "duration_s": time.monotonic() - started_at,
                "command_duration_s": duration_s,
                "settle_duration_s": self.config.move_settle_time_s,
            },
        }

    def rotate(self, degrees: float, rate_deg_s: float, timeout_s: float) -> JSONDict:
        publisher = self._require_publisher()
        state = self.wait_for_state(timeout=self.config.state_timeout_s)
        if state is None:
            raise RuntimeError("no real-robot state available before rotate")

        self._ensure_yaw_origin(state)
        start_yaw = self._relative_yaw(state)
        target_yaw = wrap_pi(start_yaw + math.radians(degrees))
        tolerance = math.radians(self.config.rotate_tolerance_deg)
        yaw_rate_tol = math.radians(self.config.rotate_yaw_rate_tolerance_deg)
        deadline = time.monotonic() + timeout_s
        settle_since: float | None = None
        commanded_yaw = start_yaw
        last_command_time = time.monotonic()
        last_error = wrap_pi(target_yaw - start_yaw)
        completed = False
        started_at = time.monotonic()

        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                dt = max(now - last_command_time, 1e-3)
                last_command_time = now

                remaining_command = wrap_pi(target_yaw - commanded_yaw)
                max_step = math.radians(rate_deg_s) * dt
                if abs(remaining_command) <= max_step:
                    commanded_yaw = target_yaw
                else:
                    commanded_yaw = wrap_pi(commanded_yaw + math.copysign(max_step, remaining_command))

                state = self.latest_state()
                yaw_rate = None
                if state is not None:
                    self._ensure_yaw_origin(state)
                    last_error = wrap_pi(target_yaw - self._relative_yaw(state))
                    yaw_rate = state.yaw_rate
                    yaw_is_stable = yaw_rate is None or abs(yaw_rate) <= yaw_rate_tol
                    if abs(last_error) <= tolerance and yaw_is_stable:
                        if settle_since is None:
                            settle_since = now
                    else:
                        settle_since = None
                    if settle_since is not None and now - settle_since >= self.config.rotate_settle_time_s:
                        completed = True
                        break

                publisher.send_planner(
                    LOCO_IDLE,
                    [0.0, 0.0, 0.0],
                    facing_from_yaw(commanded_yaw),
                    -1.0,
                    -1.0,
                )
                time.sleep(1.0 / max(self.config.rate_hz, 1.0))
        finally:
            self._last_facing_yaw = target_yaw
            self.send_idle_burst(duration=0.3, preserve_facing=True)

        final_state = self.latest_state()
        actual_degrees: float | None = None
        if final_state is not None:
            self._ensure_yaw_origin(final_state)
            actual_degrees = math.degrees(wrap_pi(self._relative_yaw(final_state) - start_yaw))

        motion_result: JSONDict = {
            "target_degrees": degrees,
            "final_error_deg": math.degrees(last_error),
        }
        if actual_degrees is not None:
            motion_result["estimated_degrees"] = actual_degrees
            motion_result["estimated_degrees_source"] = "g1_debug_heading"
        return {
            "motion": "rotate",
            "completed": completed,
            "completion": {
                "motion_commanded": True,
                "completion_source": "yaw_closed_loop",
                "capture_timing": "after_settle" if completed else "after_timeout",
                "settled": completed,
                "duration_s": time.monotonic() - started_at,
            },
            "motion_result": motion_result,
        }

    def latest_state(self) -> Any | None:
        return None if self._state_sub is None else self._state_sub.latest()

    def wait_for_state(self, timeout: float) -> Any | None:
        return None if self._state_sub is None else self._state_sub.wait_for_state(timeout)

    def latest_camera_payload(self) -> Mapping[str, Any] | None:
        if self._image_sub is None:
            return None
        return self._image_sub.read_latest()

    def get_robot_state_payload(self) -> JSONDict | None:
        state = self.latest_state()
        if state is None:
            return None
        self._ensure_yaw_origin(state)
        now = time.monotonic()
        return {
            "state_id": f"real_state_{int(now * 1000)}",
            "base_pose": {
                "x_m": None,
                "y_m": None,
                "z_m": None,
                "yaw_deg": math.degrees(self._relative_yaw(state)),
            },
            "base_velocity": {
                "linear_mps": None,
                "angular_rad_s": [None, None, state.yaw_rate],
                "angular_deg_s": [None, None, math.degrees(state.yaw_rate)] if state.yaw_rate is not None else None,
            },
            "joint_positions": {},
            "estimated": True,
            "source": "g1_debug_heading",
            "heading_state": {
                "base_quat_wxyz": state.base_quat,
                "delta_heading_rad": state.delta_heading,
                "yaw_rate_rad_s": state.yaw_rate,
                "age_s": now - state.timestamp,
            },
        }

    def get_health(self) -> RuntimeHealth:
        state_connected = self.latest_state() is not None
        camera_payload = self.latest_camera_payload() if self.config.camera_enabled else None
        camera_connected = camera_payload is not None
        command_connected = self._publisher is not None
        ready_for_motion = (
            self._startup_error is None
            and self.config.motion_enabled
            and command_connected
            and state_connected
            and (camera_connected or not self.config.camera_required)
        )
        return {
            "ok": self._startup_error is None,
            "runtime_mode": self.config.runtime_mode,
            "executor_started": self.started,
            "sensor_connected": state_connected and (camera_connected or not self.config.camera_required),
            "error_message": self._startup_error,
            "telemetry": {
                "executor": "real_zmq",
                "controller": "groot_wbc",
                "hardware": True,
                "command_endpoint": f"tcp://{self.config.command_bind_host}:{self.config.command_port}",
                "command_connected": command_connected,
                "state_endpoint": f"tcp://{self.config.state_host}:{self.config.state_port}",
                "state_topic": self.config.state_topic,
                "state_connected": state_connected,
                "camera_enabled": self.config.camera_enabled,
                "camera_required": self.config.camera_required,
                "camera_connected": camera_connected,
                "camera_endpoint": f"tcp://{self.config.camera_host}:{self.config.camera_port}",
                "motion_enabled": self.config.motion_enabled,
                "ready_for_motion": ready_for_motion,
                "auto_start_control": self.config.auto_start_control,
                "max_speed_mps": self.config.max_move_speed_mps,
            },
        }

    def _wait_for_camera(self, timeout: float) -> Mapping[str, Any] | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            payload = self.latest_camera_payload()
            if payload is not None:
                return payload
            time.sleep(0.05)
        return self.latest_camera_payload()

    def _require_publisher(self) -> PackedPublisher:
        if self._publisher is None:
            raise RuntimeError("real-robot command publisher is not started")
        return self._publisher

    def _ensure_yaw_origin(self, state: Any) -> None:
        if self._yaw_origin is None:
            self._yaw_origin = state.yaw

    def _relative_yaw(self, state: Any) -> float:
        if self._yaw_origin is None:
            return state.yaw
        return wrap_pi(state.yaw - self._yaw_origin)


@dataclass
class RealPrimitiveExecutor(DryRunPrimitiveExecutor):
    """Primitive executor that commands the existing real-robot runtime."""

    config: RealBridgeConfig = field(default_factory=RealBridgeConfig)
    runtime: RealRuntimeClient | None = None
    _motion_lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _move_model: MoveModel | None = field(default=None, init=False)
    _move_model_error: str | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        self.runtime_mode = self.config.runtime_mode
        self.default_distance_m = self.config.default_distance_m
        self.default_turn_degrees = self.config.default_turn_degrees
        self.default_speed_mps = self.config.default_move_speed_mps
        self.default_rate_deg_s = self.config.default_rotate_rate_deg_s
        if self.runtime is None:
            self.runtime = RealRuntimeClient(self.config)
        if self.config.use_move_model:
            self._move_model, self._move_model_error = self._load_move_model(self.config.move_model_file)

    def start(self) -> RuntimeHealth:
        assert self.runtime is not None
        health = self.runtime.start()
        self.started = bool(health.get("ok", False)) and bool(health.get("executor_started", False))
        self._last_telemetry = dict(health.get("telemetry", {}))
        return health

    def halt(self) -> RuntimeHealth:
        assert self.runtime is not None
        health = self.runtime.halt()
        self.started = False
        self._last_telemetry = dict(health.get("telemetry", {}))
        return health

    def close(self) -> None:
        assert self.runtime is not None
        self.runtime.close()
        self.started = False

    def move(
        self,
        distance_m: float,
        speed_mps: float | None = None,
        timeout_s: float | None = None,
    ) -> ExecuteActionResponse:
        with self._motion_lock:
            if not self.config.motion_enabled:
                return self._real_failure(
                    "real motion is disabled; start bridge with --enable-real-motion to command hardware",
                    {"motion": "move"},
                    action_status="rejected",
                )
            try:
                distance = self._float(distance_m, "distance_m")
                max_speed = self._clamped_max_speed(speed_mps)
                timeout = self._optional_positive_float(timeout_s, "timeout_s")
                commands = self._move_commands(distance, max_speed)
                command_duration_s = sum(float(command["duration_s"]) for command in commands)
                if timeout is not None and command_duration_s > timeout:
                    return self._real_failure(
                        "move command duration exceeds timeout_s",
                        {
                            "distance_m": distance,
                            "max_speed_mps": max_speed,
                            "estimated_duration_s": command_duration_s,
                            "timeout_s": timeout,
                        },
                        action_status="rejected",
                    )
                health = self.start()
                if not health.get("ok", False):
                    return self._real_failure(str(health.get("error_message")), dict(health.get("telemetry", {})))
                assert self.runtime is not None
                telemetry = self._run_move_commands(commands)
            except Exception as exc:  # noqa: BLE001 - fail closed for hardware mode.
                halt_health = self.halt()
                return self._real_failure(str(exc), {"motion": "move", "halt_called": True, "halt_health": halt_health})

            return self._real_success(
                executed_arguments={
                    "primitive": "move_model",
                    "distance_m": distance,
                    "max_speed_mps": max_speed,
                    "timeout_s": timeout,
                    "move_model_file": str(self._move_model.path) if self._move_model is not None else None,
                },
                telemetry=telemetry,
            )

    def rotate(
        self,
        degrees: float,
        rate_deg_s: float | None = None,
        timeout_s: float | None = None,
    ) -> ExecuteActionResponse:
        with self._motion_lock:
            if not self.config.motion_enabled:
                return self._real_failure(
                    "real motion is disabled; start bridge with --enable-real-motion to command hardware",
                    {"motion": "rotate"},
                    action_status="rejected",
                )
            try:
                angle = self._float(degrees, "degrees")
                rate = self._clamped_rotate_rate(rate_deg_s)
                timeout = self._optional_positive_float(timeout_s, "timeout_s")
                if timeout is None:
                    timeout = max(
                        self.config.rotate_timeout_s,
                        abs(angle) / max(rate, 1.0) + self.config.rotate_extra_time_s,
                    )
                health = self.start()
                if not health.get("ok", False):
                    return self._real_failure(str(health.get("error_message")), dict(health.get("telemetry", {})))
                assert self.runtime is not None
                telemetry = self.runtime.rotate(angle, rate, timeout)
            except Exception as exc:  # noqa: BLE001 - fail closed for hardware mode.
                halt_health = self.halt()
                return self._real_failure(str(exc), {"motion": "rotate", "halt_called": True, "halt_health": halt_health})

            if telemetry.get("completed") is not True:
                return self._real_failure(
                    "rotate command timed out before reaching target yaw",
                    telemetry,
                    action_status="timeout",
                )
            return self._real_success(
                executed_arguments={
                    "primitive": "rotate",
                    "degrees": angle,
                    "rate_deg_s": rate,
                    "timeout_s": timeout,
                },
                telemetry=telemetry,
            )

    def get_health(self, sensor_connected: bool = True) -> RuntimeHealth:
        assert self.runtime is not None
        health = self.runtime.get_health()
        telemetry = dict(health.get("telemetry", {}))
        telemetry.update(
            {
                "move_model_enabled": self.config.use_move_model,
                "move_model_loaded": self._move_model is not None,
                "move_model_path": str(self._move_model.path) if self._move_model is not None else None,
                "move_model_error": self._move_model_error,
            }
        )
        health["telemetry"] = telemetry
        return health

    def _real_success(self, executed_arguments: JSONDict, telemetry: JSONDict) -> ExecuteActionResponse:
        command_id = self._next_command_id()
        response_telemetry: JSONDict = {
            "command_id": command_id,
            "runtime_mode": self.runtime_mode,
            "controller": "real_zmq_wbc",
            "dry_run": False,
            "hardware": True,
            "executor_started": self.started,
            "completion": self._completion_or_default(telemetry, action_status="completed"),
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

    def _real_failure(
        self,
        error_message: str,
        telemetry: JSONDict | None = None,
        action_status: str = "failed",
    ) -> ExecuteActionResponse:
        action_telemetry = dict(telemetry or {})
        response_telemetry: JSONDict = {
            "runtime_mode": self.runtime_mode,
            "controller": "real_zmq_wbc",
            "dry_run": False,
            "hardware": True,
            "executor_started": self.started,
            "completion": self._completion_or_default(action_telemetry, action_status=action_status),
        }
        response_telemetry.update(action_telemetry)
        self._last_telemetry = response_telemetry
        return {
            "ok": False,
            "error_message": error_message,
            "action_status": action_status,
            "executed_arguments": {},
            "telemetry": response_telemetry,
        }

    @staticmethod
    def _completion_or_default(telemetry: Mapping[str, Any], action_status: str) -> JSONDict:
        completion = telemetry.get("completion")
        if isinstance(completion, Mapping):
            return dict(completion)
        if action_status == "timeout":
            return {
                "motion_commanded": True,
                "completion_source": "runtime_timeout",
                "capture_timing": "after_timeout",
                "settled": False,
            }
        if action_status == "rejected":
            return {
                "motion_commanded": False,
                "completion_source": "validation",
                "capture_timing": "not_started",
                "settled": False,
                "duration_s": 0.0,
            }
        return {
            "motion_commanded": action_status == "completed",
            "completion_source": "runtime",
            "capture_timing": "after_completion" if action_status == "completed" else "unknown",
            "settled": action_status == "completed",
        }

    def _clamped_speed(self, value: float | None) -> float:
        speed = self._positive_float_or_default(value, self.config.default_move_speed_mps, "speed_mps")
        return min(max(speed, self.config.min_move_speed_mps), self.config.max_move_speed_mps)

    def _clamped_max_speed(self, value: float | None) -> float:
        speed = self._positive_float_or_default(value, self.config.max_move_speed_mps, "max_speed_mps")
        return min(max(speed, self.config.min_move_speed_mps), self.config.max_move_speed_mps)

    def _clamped_rotate_rate(self, value: float | None) -> float:
        rate = self._positive_float_or_default(value, self.config.default_rotate_rate_deg_s, "rate_deg_s")
        return min(max(rate, self.config.min_rotate_rate_deg_s), self.config.max_rotate_rate_deg_s)

    def _move_commands(self, distance_m: float, max_speed_mps: float) -> list[JSONDict]:
        if not self.config.use_move_model:
            duration = abs(distance_m) / max_speed_mps if max_speed_mps > 0.0 else 0.0
            return [
                {
                    "distance_m": distance_m,
                    "speed_mps": max_speed_mps,
                    "duration_s": duration,
                    "source": "open_loop",
                }
            ]
        if self._move_model is None:
            detail = f": {self._move_model_error}" if self._move_model_error else ""
            raise RuntimeError(f"move_model is enabled but no move model is loaded{detail}")

        return [
            self._predict_model_command(chunk, max_speed_mps)
            for chunk in self._split_model_move(distance_m)
        ]

    def _run_move_commands(self, commands: list[JSONDict]) -> JSONDict:
        assert self.runtime is not None
        if not commands:
            return {
                "motion": "move",
                "completion": {
                    "motion_commanded": False,
                    "completion_source": "move_model",
                    "capture_timing": "after_completion",
                    "settled": True,
                    "duration_s": 0.0,
                    "command_duration_s": 0.0,
                },
                "move_model": self._move_model_telemetry(commands),
            }

        chunk_telemetry: list[JSONDict] = []
        for index, command in enumerate(commands, start=1):
            telemetry = self.runtime.move(
                float(command["distance_m"]),
                float(command["speed_mps"]),
                float(command["duration_s"]),
            )
            chunk_telemetry.append(telemetry)
            if index < len(commands) and self.config.model_chunk_pause_s > 0.0:
                time.sleep(self.config.model_chunk_pause_s)

        if len(commands) == 1:
            response = dict(chunk_telemetry[0])
            response["move_model"] = self._move_model_telemetry(commands)
            return response

        command_duration_s = sum(float(command["duration_s"]) for command in commands)
        completion_duration_s = 0.0
        settled = True
        capture_timing = "after_settle"
        for telemetry in chunk_telemetry:
            completion = telemetry.get("completion")
            if isinstance(completion, Mapping):
                completion_duration_s += float(completion.get("duration_s", 0.0))
                settled = settled and bool(completion.get("settled", True))
                capture_timing = str(completion.get("capture_timing", capture_timing))

        return {
            "motion": "move",
            "completion": {
                "motion_commanded": True,
                "completion_source": "move_model_chunks",
                "capture_timing": capture_timing,
                "settled": settled,
                "duration_s": completion_duration_s,
                "command_duration_s": command_duration_s,
                "chunk_count": len(commands),
            },
            "move_model": self._move_model_telemetry(commands),
        }

    def _move_model_telemetry(self, commands: list[JSONDict]) -> JSONDict:
        return {
            "enabled": self.config.use_move_model,
            "model_file": str(self._move_model.path) if self._move_model is not None else None,
            "chunk_count": len(commands),
            "chunks": commands,
        }

    def _split_model_move(self, distance_m: float) -> list[float]:
        if abs(distance_m) < 1e-9:
            return []
        if self._move_model is None:
            return [distance_m]
        sign = 1.0 if distance_m > 0.0 else -1.0
        limit = (
            self._move_model.max_forward_magnitude
            if sign > 0.0
            else self._move_model.max_backward_magnitude
        )
        if limit <= 1e-9:
            return [distance_m]

        remaining = abs(distance_m)
        chunks: list[float] = []
        while remaining > limit + 1e-9:
            chunks.append(sign * limit)
            remaining -= limit
        if remaining > 1e-9:
            chunks.append(sign * remaining)
        return chunks

    def _predict_model_command(self, distance_m: float, max_speed_mps: float) -> JSONDict:
        if self._move_model is None:
            raise RuntimeError("No move model loaded.")
        samples = self._move_model.forward if distance_m >= 0.0 else self._move_model.backward
        target = abs(distance_m)
        speed = interp_linear(target, [(sample.magnitude_abs, sample.rate) for sample in samples])
        execute_time = interp_linear(
            target,
            [(sample.magnitude_abs, sample.execute_time) for sample in samples],
        )
        clamped_speed = min(max(abs(speed), self.config.min_move_speed_mps), max_speed_mps)
        return {
            "distance_m": distance_m,
            "speed_mps": clamped_speed,
            "duration_s": max(execute_time, 0.0),
            "model_speed_mps": speed,
            "model_execute_time_s": execute_time,
            "source": "move_model",
        }

    def _load_move_model(self, model_file: str) -> tuple[MoveModel | None, str | None]:
        try:
            path = self._resolve_move_model_path(model_file)
            if path is None:
                return None, "no JSON model found under outputs/vigil_move_models"
            payload = json.loads(path.read_text(encoding="utf-8"))
            models = payload["models"]
            forward = self._parse_move_model_samples(models["forward"])
            backward = self._parse_move_model_samples(models["backward"])
            if not forward or not backward:
                raise ValueError("forward/backward samples are required")
            return MoveModel(path=path, forward=forward, backward=backward), None
        except Exception as exc:  # noqa: BLE001 - report as bridge validation error.
            return None, str(exc)

    def _resolve_move_model_path(self, model_file: str) -> Path | None:
        if model_file.strip().lower() == "auto":
            candidates = sorted((REPO_ROOT / "outputs" / "vigil_move_models").glob("vigil_move_model_*.json"))
            return candidates[-1] if candidates else None
        return resolve_repo_path(model_file)

    @staticmethod
    def _parse_move_model_samples(rows: list[dict[str, Any]]) -> list[MoveModelSample]:
        samples = [
            MoveModelSample(
                magnitude_abs=float(row["magnitude_abs"]),
                rate=abs(float(row["rate"])),
                execute_time=float(row["execute_time"]),
            )
            for row in rows
        ]
        return sorted(samples, key=lambda row: row.magnitude_abs)


@dataclass
class RealSensorProvider:
    """Sensor provider backed by real-robot runtime transports."""

    runtime: RealRuntimeClient
    runtime_mode: str = "real"
    _observation_index: int = 0

    @property
    def connected(self) -> bool:
        health = self.runtime.get_health()
        return bool(health.get("sensor_connected", False))

    def get_observation(self) -> ObservationResponse:
        self._observation_index += 1
        state_response = self.get_robot_state()
        camera_payload = self.runtime.latest_camera_payload()
        images, timestamps, camera_error = self._normalize_camera_payload(camera_payload)
        return {
            "observation_id": f"real_obs_{self._observation_index:04d}",
            "runtime_mode": self.runtime_mode,
            "images": images,
            "camera_timestamps": timestamps,
            "robot_state": dict(state_response.get("robot_state", {})),
            "telemetry": {
                "sensor_provider": "real",
                "hardware": True,
                "connected": self.connected,
                "camera_enabled": self.runtime.config.camera_enabled,
                "camera_required": self.runtime.config.camera_required,
                "camera_connected": bool(images),
                "camera_error": camera_error,
            },
            "perception": {
                "source": "none",
                "detections": [],
            },
        }

    def get_robot_state(self) -> RobotStateResponse:
        robot_state = self.runtime.get_robot_state_payload()
        health = self.runtime.get_health()
        if robot_state is None:
            return {
                "ok": False,
                "error_message": "no real-robot state received from g1_debug",
                "runtime_mode": self.runtime_mode,
                "robot_state": {},
                "telemetry": dict(health.get("telemetry", {})),
            }
        return {
            "ok": True,
            "error_message": None,
            "runtime_mode": self.runtime_mode,
            "robot_state": robot_state,
            "telemetry": dict(health.get("telemetry", {})),
        }

    def _normalize_camera_payload(
        self,
        payload: Mapping[str, Any] | None,
    ) -> tuple[dict[str, Any], JSONDict, str | None]:
        if not self.runtime.config.camera_enabled:
            return {}, {}, None
        if payload is None:
            image_sub = getattr(self.runtime, "_image_sub", None)
            error = None if image_sub is None else image_sub.error
            return {}, {}, error or "no camera payload received"

        raw_images = payload.get("images", {})
        raw_timestamps = payload.get("timestamps", {})
        images: dict[str, Any] = {}
        if isinstance(raw_images, Mapping):
            for camera_name, image_value in raw_images.items():
                name = str(camera_name)
                if isinstance(image_value, str):
                    images[name] = {
                        "encoding": "jpeg-base64",
                        "data": image_value,
                    }
                elif isinstance(image_value, bytes | bytearray):
                    import base64

                    images[name] = {
                        "encoding": "jpeg-base64",
                        "data": base64.b64encode(bytes(image_value)).decode("ascii"),
                    }
                else:
                    images[name] = {
                        "encoding": "unsupported",
                        "type": type(image_value).__name__,
                    }

        timestamps: JSONDict = {}
        if isinstance(raw_timestamps, Mapping):
            for camera_name, timestamp in raw_timestamps.items():
                try:
                    timestamps[str(camera_name)] = float(timestamp)
                except (TypeError, ValueError):
                    timestamps[str(camera_name)] = str(timestamp)
        return images, timestamps, None


def create_real_bridge_service(config: RealBridgeConfig | None = None) -> VigilBridgeService:
    """Create a bridge service backed by the real-robot runtime adapter."""

    bridge_config = config or RealBridgeConfig()
    runtime = RealRuntimeClient(bridge_config)
    executor = RealPrimitiveExecutor(config=bridge_config, runtime=runtime)
    sensor_provider = RealSensorProvider(runtime=runtime, runtime_mode=bridge_config.runtime_mode)
    return VigilBridgeService(
        executor=executor,
        sensor_provider=sensor_provider,
        runtime_mode=bridge_config.runtime_mode,
    )
