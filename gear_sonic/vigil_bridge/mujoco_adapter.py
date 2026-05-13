"""MuJoCo runtime adapter for the GR00T-side Vigil bridge.

This adapter talks to the existing simulation/deploy processes over their
public runtime transports:

* ZMQ planner/command PUB consumed by the deploy process.
* ZMQ g1_debug SUB published by deploy when started with output type all.
* Optional DDS rt/odostate from the MuJoCo SDK bridge.
* Optional ZMQ camera stream published by run_sim_loop image publishing.

It intentionally does not import Vigil, policy inference, deploy internals, or
low-level controller code.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import queue
import struct
import threading
import time
from typing import Any, Mapping

from gear_sonic.vigil_bridge.primitive_executor import DryRunPrimitiveExecutor
from gear_sonic.vigil_bridge.protocol import (
    ExecuteActionResponse,
    JSONDict,
    ObservationResponse,
    RobotStateResponse,
    RuntimeHealth,
)
from gear_sonic.vigil_bridge.service import VigilBridgeService


HEADER_SIZE = 1280
REPO_ROOT = Path(__file__).resolve().parents[2]

LOCO_IDLE = 0
LOCO_SLOW_WALK = 1
LOCO_WALK = 2


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat_wxyz(quat: list[float] | tuple[float, ...]) -> float:
    w, x, y, z = quat
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def facing_from_yaw(yaw: float) -> list[float]:
    return [math.cos(yaw), math.sin(yaw), 0.0]


def resolve_repo_path(path_text: str | Path) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


def interp_linear(x: float, points: list[tuple[float, float]]) -> float:
    if not points:
        raise ValueError("empty interpolation points")
    if len(points) == 1:
        return points[0][1]
    if x <= points[0][0]:
        return extrapolate_linear(x, points[0], points[1])
    if x >= points[-1][0]:
        return extrapolate_linear(x, points[-2], points[-1])
    for left, right in zip(points, points[1:]):
        if left[0] <= x <= right[0]:
            return extrapolate_linear(x, left, right)
    return points[-1][1]


def extrapolate_linear(x: float, left: tuple[float, float], right: tuple[float, float]) -> float:
    dx = right[0] - left[0]
    if abs(dx) < 1e-9:
        return left[1]
    alpha = (x - left[0]) / dx
    return left[1] + alpha * (right[1] - left[1])


@dataclass(frozen=True)
class MoveModelSample:
    magnitude_abs: float
    rate: float
    execute_time: float


@dataclass(frozen=True)
class MoveModel:
    path: Path
    forward: list[MoveModelSample]
    backward: list[MoveModelSample]

    @property
    def max_forward_magnitude(self) -> float:
        return self.forward[-1].magnitude_abs if self.forward else 0.0

    @property
    def max_backward_magnitude(self) -> float:
        return self.backward[-1].magnitude_abs if self.backward else 0.0


@dataclass(frozen=True)
class MujocoBridgeConfig:
    """Configuration for the MuJoCo bridge adapter."""

    runtime_mode: str = "mujoco"
    command_bind_host: str = "*"
    command_port: int = 5556
    state_host: str = "localhost"
    state_port: int = 5557
    state_topic: str = "g1_debug"
    rate_hz: float = 20.0
    default_distance_m: float = 0.25
    default_turn_degrees: float = 30.0
    default_move_speed_mps: float = 0.25
    min_move_speed_mps: float = 0.20
    max_move_speed_mps: float = 1.00
    use_move_model: bool = True
    move_model_file: str = "auto"
    model_chunk_pause_s: float = 0.0
    move_settle_time_s: float = 0.8
    default_rotate_rate_deg_s: float = 35.0
    min_rotate_rate_deg_s: float = 10.0
    max_rotate_rate_deg_s: float = 90.0
    rotate_tolerance_deg: float = 3.0
    rotate_timeout_s: float = 4.0
    rotate_extra_time_s: float = 8.0
    rotate_settle_time_s: float = 0.35
    rotate_yaw_rate_tolerance_deg: float = 8.0
    state_timeout_s: float = 3.0
    odom_source: str = "auto"
    dds_interface: str = "lo"
    dds_domain: int = 0
    camera_enabled: bool = False
    camera_host: str = "localhost"
    camera_port: int = 5555
    auto_start_control: bool = False
    stop_on_halt: bool = False
    verbose: bool = False


@dataclass
class MujocoRobotState:
    yaw: float
    base_quat: list[float]
    delta_heading: float
    timestamp: float
    yaw_rate: float | None = None


@dataclass
class MujocoOdomState:
    position: list[float]
    yaw: float
    timestamp: float
    linear_velocity: list[float] | None = None
    angular_velocity: list[float] | None = None


class PackedPublisher:
    """Publishes packed command/planner messages accepted by ZMQManager."""

    def __init__(self, bind_host: str, port: int, verbose: bool = False) -> None:
        try:
            import zmq
        except ImportError as exc:
            raise RuntimeError("pyzmq is required for MuJoCo bridge command transport") from exc

        self._zmq = zmq
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.endpoint = f"tcp://{bind_host}:{port}"
        self.socket.bind(self.endpoint)
        self.verbose = verbose
        time.sleep(0.5)

    def close(self) -> None:
        self.socket.close(0)
        self.context.term()

    def send_command(self, start: bool, stop: bool, planner: bool = True) -> None:
        fields = [
            {"name": "start", "dtype": "u8", "shape": [1]},
            {"name": "stop", "dtype": "u8", "shape": [1]},
            {"name": "planner", "dtype": "u8", "shape": [1]},
        ]
        header = {"v": 1, "endian": "le", "count": 1, "fields": fields}
        data = struct.pack("BBB", int(start), int(stop), int(planner))
        self._send_packed("command", header, data)

    def send_planner(
        self,
        mode: int,
        movement: list[float],
        facing: list[float],
        speed: float = -1.0,
        height: float = -1.0,
    ) -> None:
        fields = [
            {"name": "mode", "dtype": "i32", "shape": [1]},
            {"name": "movement", "dtype": "f32", "shape": [3]},
            {"name": "facing", "dtype": "f32", "shape": [3]},
            {"name": "speed", "dtype": "f32", "shape": [1]},
            {"name": "height", "dtype": "f32", "shape": [1]},
        ]
        header = {"v": 1, "endian": "le", "count": 1, "fields": fields}
        data = b"".join(
            [
                struct.pack("<i", int(mode)),
                struct.pack("<fff", *[float(v) for v in movement]),
                struct.pack("<fff", *[float(v) for v in facing]),
                struct.pack("<f", float(speed)),
                struct.pack("<f", float(height)),
            ]
        )
        self._send_packed("planner", header, data)

    def _send_packed(self, topic: str, header: dict[str, Any], data: bytes) -> None:
        import json

        header_json = json.dumps(header, separators=(",", ":")).encode("utf-8")
        if len(header_json) > HEADER_SIZE:
            raise ValueError(f"Header too large: {len(header_json)} > {HEADER_SIZE}")
        header_bytes = header_json + b"\x00" * (HEADER_SIZE - len(header_json))
        self.socket.send(topic.encode("utf-8") + header_bytes + data)
        if self.verbose:
            print(f"[mujoco_bridge] sent {topic}")


class StateSubscriber(threading.Thread):
    """Subscribes to deploy g1_debug state over ZMQ."""

    def __init__(self, host: str, port: int, topic: str) -> None:
        super().__init__(daemon=True)
        try:
            import msgpack
            import zmq
        except ImportError as exc:
            raise RuntimeError("pyzmq and msgpack are required for MuJoCo bridge state transport") from exc

        self._msgpack = msgpack
        self._zmq = zmq
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVTIMEO, 100)
        self.socket.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))
        self.socket.connect(f"tcp://{host}:{port}")
        self.topic = topic
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._latest: MujocoRobotState | None = None
        self.errors: queue.Queue[str] = queue.Queue(maxsize=5)

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                message = self.socket.recv()
            except self._zmq.Again:
                continue
            except self._zmq.ZMQError as exc:
                self._remember_error(str(exc))
                continue

            try:
                data = self._topic_msgpack_payload(message)
                if not data:
                    continue
                base_quat = data.get("base_quat")
                if base_quat is None or len(base_quat) < 4:
                    continue
                base_ang_vel = data.get("base_ang_vel")
                yaw_rate = float(base_ang_vel[2]) if base_ang_vel and len(base_ang_vel) >= 3 else None
                state = MujocoRobotState(
                    yaw=yaw_from_quat_wxyz([float(v) for v in base_quat[:4]]),
                    base_quat=[float(v) for v in base_quat[:4]],
                    delta_heading=float(data.get("delta_heading", 0.0)),
                    timestamp=time.monotonic(),
                    yaw_rate=yaw_rate,
                )
                with self._lock:
                    self._latest = state
            except Exception as exc:  # noqa: BLE001 - keep subscriber alive.
                self._remember_error(str(exc))

    def close(self) -> None:
        self._stop_event.set()
        self.join(timeout=1.0)
        self.socket.close(0)
        self.context.term()

    def latest(self) -> MujocoRobotState | None:
        with self._lock:
            return self._latest

    def wait_for_state(self, timeout: float) -> MujocoRobotState | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.latest()
            if state is not None:
                return state
            time.sleep(0.02)
        return self.latest()

    def _topic_msgpack_payload(self, message: bytes) -> Mapping[str, Any] | None:
        topic_bytes = self.topic.encode("utf-8")
        if not message.startswith(topic_bytes):
            return None
        payload = message[len(topic_bytes) :]
        data = self._msgpack.unpackb(payload, raw=False)
        return data if isinstance(data, Mapping) else None

    def _remember_error(self, text: str) -> None:
        try:
            self.errors.put_nowait(text)
        except queue.Full:
            pass


class DDSOdomSubscriber(threading.Thread):
    """Optional sim-only odometry subscriber for MuJoCo ground-truth odom."""

    def __init__(self, interface: str, domain_id: int) -> None:
        super().__init__(daemon=True)
        self.interface = interface
        self.domain_id = domain_id
        self.available = False
        self.error: str | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._latest: MujocoOdomState | None = None

        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import OdoState_

            ChannelFactoryInitialize(domain_id, interface)
            self._subscriber = ChannelSubscriber("rt/odostate", OdoState_)
            self._subscriber.Init(None, 0)
            self.available = True
        except Exception as exc:  # noqa: BLE001 - optional path.
            self.error = str(exc)
            self._subscriber = None

    def run(self) -> None:
        if not self.available or self._subscriber is None:
            return
        while not self._stop_event.is_set():
            try:
                msg = self._subscriber.Read()
                if msg is None:
                    time.sleep(0.01)
                    continue

                quat = [float(v) for v in msg.orientation]
                state = MujocoOdomState(
                    position=[float(v) for v in msg.position],
                    yaw=yaw_from_quat_wxyz(quat),
                    timestamp=time.monotonic(),
                    linear_velocity=[float(v) for v in msg.linear_velocity],
                    angular_velocity=[float(v) for v in msg.angular_velocity],
                )
                with self._lock:
                    self._latest = state
            except Exception as exc:  # noqa: BLE001 - keep optional subscriber alive.
                self.error = str(exc)
                time.sleep(0.05)

    def close(self) -> None:
        self._stop_event.set()
        self.join(timeout=1.0)

    def latest(self) -> MujocoOdomState | None:
        with self._lock:
            return self._latest


class ZMQImageSubscriber:
    """Reads the latest camera frame payload from a MuJoCo camera ZMQ server."""

    def __init__(self, host: str, port: int) -> None:
        try:
            import msgpack
            import zmq
        except ImportError as exc:
            raise RuntimeError("pyzmq and msgpack are required for MuJoCo bridge camera transport") from exc

        self._msgpack = msgpack
        self._zmq = zmq
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.CONFLATE, True)
        self.socket.setsockopt(zmq.RCVHWM, 3)
        self.socket.connect(f"tcp://{host}:{port}")
        self._latest: Mapping[str, Any] | None = None
        self.error: str | None = None

    def read_latest(self) -> Mapping[str, Any] | None:
        try:
            while self.socket.poll(0):
                packed = self.socket.recv()
                data = self._msgpack.unpackb(packed, raw=False)
                if isinstance(data, Mapping):
                    self._latest = data
        except Exception as exc:  # noqa: BLE001 - keep bridge alive.
            self.error = str(exc)
        return self._latest

    def close(self) -> None:
        self.socket.close(0)
        self.context.term()


class MujocoRuntimeClient:
    """Runtime client shared by the MuJoCo executor and sensor provider."""

    def __init__(self, config: MujocoBridgeConfig) -> None:
        self.config = config
        self.started = False
        self._startup_error: str | None = None
        self._publisher: PackedPublisher | None = None
        self._state_sub: StateSubscriber | None = None
        self._odom_sub: DDSOdomSubscriber | None = None
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
            if self.config.odom_source in {"auto", "dds"} and self._odom_sub is None:
                self._odom_sub = DDSOdomSubscriber(
                    interface=self.config.dds_interface,
                    domain_id=self.config.dds_domain,
                )
                if self._odom_sub.available:
                    self._odom_sub.start()
            if self.config.camera_enabled and self._image_sub is None:
                self._image_sub = ZMQImageSubscriber(
                    host=self.config.camera_host,
                    port=self.config.camera_port,
                )

            self.started = True
            self._startup_error = None
            if self.config.auto_start_control:
                self.send_start_control()
        except Exception as exc:  # noqa: BLE001 - return structured health.
            self._startup_error = str(exc)
            self.close(stop_control=False)
        return self.get_health()

    def halt(self) -> RuntimeHealth:
        if self._publisher is not None:
            try:
                self.send_idle_burst(duration=0.3, preserve_facing=True)
                if self.config.stop_on_halt:
                    self._publisher.send_command(start=False, stop=True, planner=True)
            except Exception as exc:  # noqa: BLE001 - surface through health.
                self._startup_error = str(exc)
        self.started = False
        return self.get_health()

    def close(self, stop_control: bool = False) -> None:
        if stop_control:
            self.halt()
        self.started = False
        for closeable in (self._image_sub, self._odom_sub, self._state_sub, self._publisher):
            if closeable is not None:
                closeable.close()
        self._image_sub = None
        self._odom_sub = None
        self._state_sub = None
        self._publisher = None

    def send_start_control(self) -> None:
        self._require_publisher().send_command(start=True, stop=False, planner=True)
        self.send_idle_burst(duration=0.4, preserve_facing=False)

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

        state = self.latest_state()
        if state is not None:
            self._ensure_yaw_origin(state)
            self._last_facing_yaw = self._relative_yaw(state)

        sign = 1.0 if distance_m >= 0.0 else -1.0
        facing = facing_from_yaw(self._last_facing_yaw)
        movement = [sign * facing[0], sign * facing[1], 0.0]
        mode = LOCO_SLOW_WALK if speed_mps <= 0.8 else LOCO_WALK
        start_odom = self.latest_odom()
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

        elapsed_s = time.monotonic() - started_at
        actual_distance = self._signed_odom_delta(start_odom, distance_m)
        telemetry: JSONDict = {
            "motion": "move",
            "completion": {
                "motion_commanded": True,
                "completion_source": "duration_and_settle",
                "capture_timing": "after_settle",
                "settled": True,
                "duration_s": elapsed_s,
                "command_duration_s": duration_s,
                "settle_duration_s": self.config.move_settle_time_s,
            },
            "planner_mode": mode,
        }
        if actual_distance is not None:
            telemetry["motion_result"] = {
                "actual_distance_m": actual_distance,
                "actual_distance_source": "rt/odostate",
            }
        return telemetry

    def rotate(self, degrees: float, rate_deg_s: float, timeout_s: float) -> JSONDict:
        publisher = self._require_publisher()
        state = self.wait_for_state(timeout=self.config.state_timeout_s)
        if state is None:
            raise RuntimeError(
                "no g1_debug state received; start deploy with --output-type all and ZMQ state output"
            )

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
                    if yaw_rate is None:
                        odom = self.latest_odom()
                        if odom and odom.angular_velocity:
                            yaw_rate = odom.angular_velocity[2]
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
            self.send_idle_burst(duration=0.2, preserve_facing=True)

        final_state = self.latest_state()
        actual_degrees: float | None = None
        if final_state is not None:
            self._ensure_yaw_origin(final_state)
            actual_degrees = math.degrees(wrap_pi(self._relative_yaw(final_state) - start_yaw))

        telemetry: JSONDict = {
            "motion": "rotate",
            "completed": completed,
            "completion": {
                "motion_commanded": True,
                "completion_source": "yaw_closed_loop",
                "capture_timing": "after_settle" if completed else "after_timeout",
                "settled": completed,
                "duration_s": time.monotonic() - started_at,
            },
        }
        motion_result: JSONDict = {
            "target_degrees": degrees,
            "final_error_deg": math.degrees(last_error),
        }
        if actual_degrees is not None:
            motion_result["actual_degrees"] = actual_degrees
            motion_result["actual_degrees_source"] = "g1_debug_heading"
        telemetry["motion_result"] = motion_result
        return telemetry

    def latest_state(self) -> MujocoRobotState | None:
        return None if self._state_sub is None else self._state_sub.latest()

    def wait_for_state(self, timeout: float) -> MujocoRobotState | None:
        return None if self._state_sub is None else self._state_sub.wait_for_state(timeout)

    def latest_odom(self) -> MujocoOdomState | None:
        if self._odom_sub is None or not self._odom_sub.available:
            return None
        return self._odom_sub.latest()

    def latest_camera_payload(self) -> Mapping[str, Any] | None:
        if self._image_sub is None:
            return None
        return self._image_sub.read_latest()

    def get_robot_state_payload(self) -> JSONDict | None:
        odom = self.latest_odom()
        state = self.latest_state()
        now = time.monotonic()

        if odom is None and state is None:
            return None

        yaw = odom.yaw if odom is not None else state.yaw  # type: ignore[union-attr]
        position = odom.position if odom is not None else [0.0, 0.0, 0.0]
        linear_velocity = odom.linear_velocity if odom is not None else None
        angular_velocity = odom.angular_velocity if odom is not None else None

        payload: JSONDict = {
            "state_id": f"mujoco_state_{int(now * 1000)}",
            "base_pose": {
                "x_m": position[0],
                "y_m": position[1],
                "z_m": position[2],
                "yaw_deg": math.degrees(yaw),
            },
            "base_velocity": {
                "linear_mps": linear_velocity,
                "angular_rad_s": angular_velocity,
                "angular_deg_s": (
                    [math.degrees(v) for v in angular_velocity] if angular_velocity is not None else None
                ),
            },
            "joint_positions": {},
            "estimated": odom is None,
            "source": "rt/odostate" if odom is not None else "g1_debug_heading",
        }
        if state is not None:
            payload["heading_state"] = {
                "base_quat_wxyz": state.base_quat,
                "delta_heading_rad": state.delta_heading,
                "yaw_rate_rad_s": state.yaw_rate,
                "age_s": now - state.timestamp,
            }
        if odom is not None:
            payload["odom"] = {
                "position_m": odom.position,
                "yaw_deg": math.degrees(odom.yaw),
                "linear_velocity_mps": odom.linear_velocity,
                "angular_velocity_rad_s": odom.angular_velocity,
                "age_s": now - odom.timestamp,
            }
        return payload

    def get_health(self) -> RuntimeHealth:
        odom_connected = self.latest_odom() is not None
        state_connected = self.latest_state() is not None
        camera_payload = self.latest_camera_payload() if self.config.camera_enabled else None
        camera_connected = camera_payload is not None
        sensor_connected = state_connected or odom_connected
        return {
            "ok": self._startup_error is None,
            "runtime_mode": self.config.runtime_mode,
            "executor_started": self.started,
            "sensor_connected": sensor_connected,
            "error_message": self._startup_error,
            "telemetry": {
                "executor": "mujoco_zmq",
                "controller": "groot_wbc",
                "command_endpoint": f"tcp://{self.config.command_bind_host}:{self.config.command_port}",
                "state_endpoint": f"tcp://{self.config.state_host}:{self.config.state_port}",
                "state_topic": self.config.state_topic,
                "state_connected": state_connected,
                "odom_source": self.config.odom_source,
                "odom_connected": odom_connected,
                "odom_error": None if self._odom_sub is None else self._odom_sub.error,
                "camera_enabled": self.config.camera_enabled,
                "camera_connected": camera_connected,
                "camera_endpoint": f"tcp://{self.config.camera_host}:{self.config.camera_port}",
                "auto_start_control": self.config.auto_start_control,
                "max_speed_mps": self.config.max_move_speed_mps,
                "move_model_enabled": self.config.use_move_model,
                "move_model_file": self.config.move_model_file,
            },
        }

    def _require_publisher(self) -> PackedPublisher:
        if self._publisher is None:
            raise RuntimeError("MuJoCo command publisher is not started")
        return self._publisher

    def _ensure_yaw_origin(self, state: MujocoRobotState) -> None:
        if self._yaw_origin is None:
            self._yaw_origin = state.yaw

    def _relative_yaw(self, state: MujocoRobotState) -> float:
        if self._yaw_origin is None:
            return state.yaw
        return wrap_pi(state.yaw - self._yaw_origin)

    def _signed_odom_delta(
        self,
        start_odom: MujocoOdomState | None,
        target_distance_m: float,
    ) -> float | None:
        end_odom = self.latest_odom()
        if start_odom is None or end_odom is None:
            return None
        dx = end_odom.position[0] - start_odom.position[0]
        dy = end_odom.position[1] - start_odom.position[1]
        traveled = math.hypot(dx, dy)
        return math.copysign(traveled, target_distance_m)


@dataclass
class MujocoPrimitiveExecutor(DryRunPrimitiveExecutor):
    """Primitive executor that commands the existing MuJoCo WBC runtime."""

    config: MujocoBridgeConfig = field(default_factory=MujocoBridgeConfig)
    runtime: MujocoRuntimeClient | None = None
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
            self.runtime = MujocoRuntimeClient(self.config)
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
            try:
                distance = self._float(distance_m, "distance_m")
                max_speed = self._clamped_max_speed(speed_mps)
                timeout = self._optional_positive_float(timeout_s, "timeout_s")
                commands = self._move_commands(distance, max_speed)
                command_duration_s = sum(command["duration_s"] for command in commands)
                if timeout is not None and command_duration_s > timeout:
                    return self._mujoco_failure(
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
                    return self._mujoco_failure(str(health.get("error_message")), dict(health.get("telemetry", {})))
                assert self.runtime is not None
                telemetry = self._run_move_commands(commands)
            except Exception as exc:  # noqa: BLE001 - return structured error.
                return self._mujoco_failure(str(exc), {"motion": "move"})

            return self._mujoco_success(
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
                    return self._mujoco_failure(str(health.get("error_message")), dict(health.get("telemetry", {})))
                assert self.runtime is not None
                telemetry = self.runtime.rotate(angle, rate, timeout)
            except Exception as exc:  # noqa: BLE001 - return structured error.
                return self._mujoco_failure(str(exc), {"motion": "rotate"})

            if telemetry.get("completed") is not True:
                return self._mujoco_failure(
                    "rotate command timed out before reaching target yaw",
                    telemetry,
                    action_status="timeout",
                )
            return self._mujoco_success(
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
                "move_model_loaded": self._move_model is not None,
                "move_model_path": str(self._move_model.path) if self._move_model is not None else None,
                "move_model_error": self._move_model_error,
            }
        )
        health["telemetry"] = telemetry
        return health

    def _mujoco_success(self, executed_arguments: JSONDict, telemetry: JSONDict) -> ExecuteActionResponse:
        command_id = self._next_command_id()
        response_completion = self._completion_or_default(telemetry, action_status="completed")
        response_telemetry: JSONDict = {
            "command_id": command_id,
            "runtime_mode": self.runtime_mode,
            "controller": "mujoco_zmq_wbc",
            "dry_run": False,
            "executor_started": self.started,
            "completion": response_completion,
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

    def _mujoco_failure(
        self,
        error_message: str,
        telemetry: JSONDict | None = None,
        action_status: str = "failed",
    ) -> ExecuteActionResponse:
        action_telemetry = dict(telemetry or {})
        response_completion = self._completion_or_default(action_telemetry, action_status=action_status)
        response_telemetry: JSONDict = {
            "runtime_mode": self.runtime_mode,
            "controller": "mujoco_zmq_wbc",
            "dry_run": False,
            "executor_started": self.started,
            "completion": response_completion,
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
        actual_distance_total = 0.0
        actual_distance_source: str | None = None
        has_actual_distance = False
        for index, command in enumerate(commands, start=1):
            telemetry = self.runtime.move(
                float(command["distance_m"]),
                float(command["speed_mps"]),
                float(command["duration_s"]),
            )
            chunk_telemetry.append(telemetry)
            motion_result = telemetry.get("motion_result")
            if isinstance(motion_result, Mapping) and "actual_distance_m" in motion_result:
                actual_distance_total += float(motion_result["actual_distance_m"])
                actual_distance_source = str(motion_result.get("actual_distance_source", "runtime"))
                has_actual_distance = True
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

        response: JSONDict = {
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
        if has_actual_distance:
            response["motion_result"] = {
                "actual_distance_m": actual_distance_total,
                "actual_distance_source": actual_distance_source,
            }
        return response

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
                rate=float(row["rate"]),
                execute_time=float(row["execute_time"]),
            )
            for row in rows
        ]
        return sorted(samples, key=lambda row: row.magnitude_abs)


@dataclass
class MujocoSensorProvider:
    """Sensor provider backed by the MuJoCo runtime transports."""

    runtime: MujocoRuntimeClient
    runtime_mode: str = "mujoco"
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
            "observation_id": f"mujoco_obs_{self._observation_index:04d}",
            "runtime_mode": self.runtime_mode,
            "images": images,
            "camera_timestamps": timestamps,
            "robot_state": dict(state_response.get("robot_state", {})),
            "telemetry": {
                "sensor_provider": "mujoco",
                "hardware": False,
                "connected": self.connected,
                "camera_enabled": self.runtime.config.camera_enabled,
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
                "error_message": "no MuJoCo robot state received from g1_debug or rt/odostate",
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
            error = None if self.runtime._image_sub is None else self.runtime._image_sub.error
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


def create_mujoco_bridge_service(config: MujocoBridgeConfig | None = None) -> VigilBridgeService:
    """Create a bridge service backed by the MuJoCo runtime adapter."""

    bridge_config = config or MujocoBridgeConfig()
    runtime = MujocoRuntimeClient(bridge_config)
    executor = MujocoPrimitiveExecutor(config=bridge_config, runtime=runtime)
    sensor_provider = MujocoSensorProvider(runtime=runtime, runtime_mode=bridge_config.runtime_mode)
    return VigilBridgeService(
        executor=executor,
        sensor_provider=sensor_provider,
        runtime_mode=bridge_config.runtime_mode,
    )
