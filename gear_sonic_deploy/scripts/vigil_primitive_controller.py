#!/usr/bin/env python3
"""Interactive primitive controller for G1 WBC via ZMQManager.

This tool is intentionally small and conservative:
  - move: open-loop, distance is converted to duration = distance / speed.
  - rotate: closed-loop on yaw estimated from the published base_quat.

It talks to the existing C++ deploy process when it is launched with:
  --input-type zmq_manager --output-type all
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import queue
import struct
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
UNITREE_SDK2_PYTHON = REPO_ROOT / "external_dependencies" / "unitree_sdk2_python"
if UNITREE_SDK2_PYTHON.exists():
    sys.path.insert(0, str(UNITREE_SDK2_PYTHON))

try:
    import msgpack
    import zmq
except ImportError as exc:
    print(f"Missing Python dependency: {exc}", file=sys.stderr)
    print("Install dependencies with: python -m pip install pyzmq msgpack", file=sys.stderr)
    raise


HEADER_SIZE = 1280

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


def topic_msgpack_payload(message: bytes, topic: str) -> Optional[dict]:
    topic_bytes = topic.encode("utf-8")
    if not message.startswith(topic_bytes):
        return None
    payload = message[len(topic_bytes):]
    return msgpack.unpackb(payload, raw=False)


def resolve_repo_path(path_text: str | Path) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


def interp_linear(x: float, points: list[tuple[float, float]]) -> float:
    if not points:
        raise ValueError("Cannot interpolate with no model points.")
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


class PackedPublisher:
    def __init__(self, bind_host: str, port: int, verbose: bool = False) -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
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
        if self.verbose:
            print(f"[command] start={start} stop={stop} planner={planner}")

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
                struct.pack("<i", mode),
                struct.pack("<fff", *movement),
                struct.pack("<fff", *facing),
                struct.pack("<f", float(speed)),
                struct.pack("<f", float(height)),
            ]
        )
        self._send_packed("planner", header, data)
        if self.verbose:
            print(f"[planner] mode={mode} movement={movement} facing={facing} speed={speed:.3f}")

    def _send_packed(self, topic: str, header: dict, data: bytes) -> None:
        header_json = json.dumps(header, separators=(",", ":")).encode("utf-8")
        if len(header_json) > HEADER_SIZE:
            raise ValueError(f"Header too large: {len(header_json)} > {HEADER_SIZE}")
        header_bytes = header_json + b"\x00" * (HEADER_SIZE - len(header_json))
        self.socket.send(topic.encode("utf-8") + header_bytes + data)


class StateSubscriber(threading.Thread):
    def __init__(self, host: str, port: int, topic: str) -> None:
        super().__init__(daemon=True)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVTIMEO, 100)
        self.socket.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))
        self.socket.connect(f"tcp://{host}:{port}")
        self.topic = topic
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[RobotState] = None
        self.errors: queue.Queue[str] = queue.Queue(maxsize=5)

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                message = self.socket.recv()
            except zmq.Again:
                continue
            except zmq.ZMQError as exc:
                self._remember_error(str(exc))
                continue

            try:
                data = topic_msgpack_payload(message, self.topic)
                if not data:
                    continue
                base_quat = data.get("base_quat")
                if base_quat is None:
                    continue
                yaw = yaw_from_quat_wxyz(base_quat)
                delta_heading = float(data.get("delta_heading", 0.0))
                base_ang_vel = data.get("base_ang_vel")
                yaw_rate = float(base_ang_vel[2]) if base_ang_vel and len(base_ang_vel) >= 3 else None
                state = RobotState(
                    yaw=yaw,
                    base_quat=[float(v) for v in base_quat],
                    delta_heading=delta_heading,
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

    def latest(self) -> Optional["RobotState"]:
        with self._lock:
            return self._latest

    def wait_for_state(self, timeout: float) -> Optional["RobotState"]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.latest()
            if state is not None:
                return state
            time.sleep(0.02)
        return None

    def _remember_error(self, text: str) -> None:
        try:
            self.errors.put_nowait(text)
        except queue.Full:
            pass


@dataclass
class RobotState:
    yaw: float
    base_quat: list[float]
    delta_heading: float
    timestamp: float
    yaw_rate: Optional[float] = None


@dataclass
class OdomState:
    position: list[float]
    yaw: float
    timestamp: float
    linear_velocity: Optional[list[float]] = None
    angular_velocity: Optional[list[float]] = None


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


class DDSOdomSubscriber(threading.Thread):
    """Optional sim-only odometry subscriber for MuJoCo ground-truth pose."""

    def __init__(self, interface: str, domain_id: int) -> None:
        super().__init__(daemon=True)
        self.interface = interface
        self.domain_id = domain_id
        self.available = False
        self.error: Optional[str] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[OdomState] = None

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

                quat = list(msg.orientation)
                # The MuJoCo bridge writes qpos[3:7], which is wxyz in this repo.
                yaw = yaw_from_quat_wxyz(quat)
                state = OdomState(
                    position=[float(v) for v in msg.position],
                    yaw=yaw,
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

    def latest(self) -> Optional[OdomState]:
        with self._lock:
            return self._latest


class PrimitiveController:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.publisher = PackedPublisher(args.bind_host, args.port, args.verbose)
        self.state_sub = StateSubscriber(args.state_host, args.state_port, args.state_topic)
        self.state_sub.start()
        self.odom_sub: Optional[DDSOdomSubscriber] = None
        if args.odom_source in {"auto", "dds"}:
            self.odom_sub = DDSOdomSubscriber(args.dds_interface, args.dds_domain)
            if self.odom_sub.available:
                self.odom_sub.start()
                print(f"DDS odom enabled: rt/odostate on interface={args.dds_interface}, domain={args.dds_domain}")
            else:
                print(f"DDS odom unavailable: {self.odom_sub.error}")
        self.yaw_origin: Optional[float] = None
        self.last_facing_yaw = 0.0
        log_file = args.log_file
        if log_file == "auto":
            log_file = f"outputs/vigil_wbc_action_log_{time.strftime('%y%m%d_%H%M%S')}.csv"
        self.log_path = Path(log_file).expanduser()
        if not self.log_path.is_absolute():
            self.log_path = REPO_ROOT / self.log_path
        self._init_log_file()
        self.grid_plan_dir = Path(args.grid_plan_dir).expanduser()
        if not self.grid_plan_dir.is_absolute():
            self.grid_plan_dir = REPO_ROOT / self.grid_plan_dir
        self.grid_plan_dir.mkdir(parents=True, exist_ok=True)
        self.move_model = self._load_move_model(args.move_model_file)

    def close(self) -> None:
        self.send_idle_burst()
        if self.odom_sub is not None:
            self.odom_sub.close()
        self.state_sub.close()
        self.publisher.close()

    def start_control(self) -> None:
        state = self.state_sub.wait_for_state(timeout=0.5)
        if state is not None:
            self.yaw_origin = state.yaw
            self.last_facing_yaw = 0.0
        self.publisher.send_command(start=True, stop=False, planner=True)
        self.send_idle_burst()
        print("Start command sent. WBC should enter planner mode.")

    def stop_control(self) -> None:
        self.publisher.send_command(start=False, stop=True, planner=True)
        print("Stop command sent.")

    def send_idle_burst(self, duration: float = 0.4, preserve_facing: bool = False) -> None:
        if not preserve_facing:
            state = self.state_sub.latest()
            if state is not None:
                self._ensure_yaw_origin(state)
                self.last_facing_yaw = self._relative_yaw(state)
        facing = facing_from_yaw(self.last_facing_yaw)
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            self.publisher.send_planner(LOCO_IDLE, [0.0, 0.0, 0.0], facing, -1.0, -1.0)
            time.sleep(1.0 / self.args.rate)

    def move(
        self,
        distance_m: float,
        speed_mps: Optional[float] = None,
        execute_time_s: Optional[float] = None,
    ) -> Optional[float]:
        speed = abs(speed_mps if speed_mps is not None else self.args.move_speed)
        if speed < self.args.min_move_speed:
            print(f"Requested speed is low; using {self.args.min_move_speed:.2f} m/s.")
            speed = self.args.min_move_speed
        if speed > self.args.max_move_speed:
            print(f"Requested speed is high for this primitive; using {self.args.max_move_speed:.2f} m/s.")
            speed = self.args.max_move_speed

        duration = execute_time_s if execute_time_s is not None else abs(distance_m) / speed if speed > 1e-6 else 0.0
        if duration <= 0.0:
            self.send_idle_burst()
            return None

        state = self.state_sub.latest()
        if state is not None:
            self._ensure_yaw_origin(state)
            self.last_facing_yaw = self._relative_yaw(state)

        sign = 1.0 if distance_m >= 0.0 else -1.0
        facing = facing_from_yaw(self.last_facing_yaw)
        movement = [sign * facing[0], sign * facing[1], 0.0]
        mode = LOCO_SLOW_WALK if speed <= 0.8 else LOCO_WALK
        start_odom = self._wait_for_odom(timeout=0.5)
        start_time = time.monotonic()

        print(f"Open-loop move: distance={distance_m:.3f} m, speed={speed:.3f} m/s, duration={duration:.2f} s")
        deadline = time.monotonic() + duration
        actual_magnitude: Optional[float] = None
        try:
            while time.monotonic() < deadline:
                self.publisher.send_planner(mode, movement, facing, speed, -1.0)
                time.sleep(1.0 / self.args.rate)
        finally:
            self.send_idle_burst(duration=self.args.move_settle_time, preserve_facing=True)
            print("Move complete -> idle.")
            actual_magnitude = self._print_move_calibration(
                distance_m,
                speed,
                start_time,
                start_odom,
                duration,
            )
            self._append_log(
                action="move",
                expected_magnitude=distance_m,
                rate=speed,
                actual_magnitude=actual_magnitude,
                execute_time=duration,
            )
        return actual_magnitude

    def move_with_model(self, distance_m: float) -> Optional[float]:
        if self.move_model is None:
            print("No move model loaded. Run vigil_fit_move_model.py or pass --move-model-file PATH.")
            return None
        if abs(distance_m) < 1e-9:
            self.send_idle_burst()
            return None

        chunks = self._split_model_move(distance_m)
        print(f"Model move: target={distance_m:.3f} m, chunks={len(chunks)}")
        for idx, chunk in enumerate(chunks, start=1):
            speed, execute_time = self._predict_model_command(chunk)
            print(
                f"  chunk {idx}/{len(chunks)}: "
                f"distance={chunk:.3f} m, speed={speed:.3f} m/s, "
                f"execute_time={execute_time:.2f} s"
            )

        actual_total = 0.0
        have_actual = False
        started_at = time.monotonic()
        for idx, chunk in enumerate(chunks, start=1):
            speed, execute_time = self._predict_model_command(chunk)
            print(f"\n[Model move chunk {idx}/{len(chunks)}]")
            actual = self.move(chunk, speed, execute_time)
            if actual is not None:
                actual_total += actual
                have_actual = True
            if idx < len(chunks) and self.args.model_chunk_pause > 0.0:
                time.sleep(self.args.model_chunk_pause)

        execute_time_total = time.monotonic() - started_at
        actual_value = actual_total if have_actual else None
        self._append_log(
            action="move_model",
            expected_magnitude=distance_m,
            rate=0.0,
            actual_magnitude=actual_value,
            execute_time=execute_time_total,
        )
        if have_actual:
            print(
                f"Model move complete: target={distance_m:.3f} m, "
                f"actual_sum={actual_total:.3f} m, error={actual_total - distance_m:.3f} m"
            )
        else:
            print("Model move complete, but no odom-backed actual distance was available.")
        return actual_value

    def _load_move_model(self, model_file: str) -> Optional[MoveModel]:
        if model_file.lower() in {"off", "none", "disabled"}:
            print("Move model: disabled")
            return None

        if model_file == "auto":
            candidates = sorted((REPO_ROOT / "outputs" / "vigil_move_models").glob("vigil_move_model_*.json"))
            if not candidates:
                print("Move model: no JSON model found under outputs/vigil_move_models")
                return None
            path = candidates[-1]
        else:
            path = resolve_repo_path(model_file)

        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            models = payload["models"]
            forward = self._parse_move_model_samples(models["forward"])
            backward = self._parse_move_model_samples(models["backward"])
            if not forward or not backward:
                raise ValueError("forward/backward samples are required")
        except Exception as exc:  # noqa: BLE001 - keep controller usable without model.
            print(f"Move model: failed to load {path}: {exc}")
            return None

        model = MoveModel(path=path, forward=forward, backward=backward)
        print(
            "Move model loaded: "
            f"{path} "
            f"(forward<= {model.max_forward_magnitude:.2f}m, "
            f"backward<= {model.max_backward_magnitude:.2f}m)"
        )
        return model

    @staticmethod
    def _parse_move_model_samples(rows: list[dict]) -> list[MoveModelSample]:
        samples = [
            MoveModelSample(
                magnitude_abs=float(row["magnitude_abs"]),
                rate=abs(float(row["rate"])),
                execute_time=float(row["execute_time"]),
            )
            for row in rows
        ]
        return sorted(samples, key=lambda row: row.magnitude_abs)

    def _split_model_move(self, distance_m: float) -> list[float]:
        if self.move_model is None:
            return [distance_m]
        sign = 1.0 if distance_m > 0.0 else -1.0
        limit = (
            self.move_model.max_forward_magnitude
            if sign > 0.0
            else self.move_model.max_backward_magnitude
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

    def _predict_model_command(self, distance_m: float) -> tuple[float, float]:
        if self.move_model is None:
            raise RuntimeError("No move model loaded.")
        samples = self.move_model.forward if distance_m >= 0.0 else self.move_model.backward
        target = abs(distance_m)
        speed = interp_linear(target, [(sample.magnitude_abs, sample.rate) for sample in samples])
        execute_time = interp_linear(target, [(sample.magnitude_abs, sample.execute_time) for sample in samples])
        speed = min(max(abs(speed), self.args.min_move_speed), self.args.max_move_speed)
        return speed, max(execute_time, 0.0)

    def rotate(
        self,
        degrees: float,
        rate_deg_s: Optional[float] = None,
        timeout_s: Optional[float] = None,
    ) -> None:
        state = self.state_sub.wait_for_state(timeout=self.args.state_timeout)
        if state is None:
            print("No g1_debug state received; cannot run closed-loop rotate.")
            print("Check that deploy is running with --output-type all/zmq and --zmq-out-topic g1_debug.")
            return

        self._ensure_yaw_origin(state)
        start_yaw = self._relative_yaw(state)
        target_yaw = wrap_pi(start_yaw + math.radians(degrees))
        requested_rate = abs(rate_deg_s if rate_deg_s is not None else self.args.rotate_command_rate_deg)
        requested_rate = min(max(requested_rate, self.args.min_rotate_command_rate_deg), self.args.max_rotate_command_rate_deg)
        timeout = timeout_s if timeout_s is not None else max(
            self.args.rotate_timeout,
            abs(degrees) / max(requested_rate, 1.0) + self.args.rotate_extra_time,
        )
        tolerance = math.radians(self.args.rotate_tolerance_deg)
        yaw_rate_tol = math.radians(self.args.rotate_yaw_rate_tolerance_deg)
        deadline = time.monotonic() + timeout
        settle_since: Optional[float] = None
        commanded_yaw = start_yaw
        last_command_time = time.monotonic()
        completed = False
        command_start_time = time.monotonic()

        print(
            "Closed-loop rotate: "
            f"start={math.degrees(start_yaw):.1f} deg, "
            f"target={math.degrees(target_yaw):.1f} deg, "
            f"cmd_rate={requested_rate:.1f} deg/s, "
            f"tol={self.args.rotate_tolerance_deg:.1f} deg, "
            f"settle={self.args.rotate_settle_time:.2f} s, "
            f"timeout={timeout:.1f} s"
        )

        last_error = wrap_pi(target_yaw - start_yaw)
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                dt = max(now - last_command_time, 1e-3)
                last_command_time = now

                remaining_command = wrap_pi(target_yaw - commanded_yaw)
                max_step = math.radians(requested_rate) * dt
                if abs(remaining_command) <= max_step:
                    commanded_yaw = target_yaw
                else:
                    commanded_yaw = wrap_pi(commanded_yaw + math.copysign(max_step, remaining_command))

                state = self.state_sub.latest()
                yaw_rate = None
                if state is not None:
                    self._ensure_yaw_origin(state)
                    last_error = wrap_pi(target_yaw - self._relative_yaw(state))
                    yaw_rate = state.yaw_rate
                    if yaw_rate is None:
                        odom = self._latest_odom()
                        if odom and odom.angular_velocity:
                            yaw_rate = odom.angular_velocity[2]

                    yaw_is_stable = yaw_rate is None or abs(yaw_rate) <= yaw_rate_tol
                    if abs(last_error) <= tolerance and yaw_is_stable:
                        if settle_since is None:
                            settle_since = now
                    else:
                        settle_since = None

                    if settle_since is not None and (now - settle_since) >= self.args.rotate_settle_time:
                        completed = True
                        break
                self.publisher.send_planner(
                    LOCO_IDLE,
                    [0.0, 0.0, 0.0],
                    facing_from_yaw(commanded_yaw),
                    -1.0,
                    -1.0,
                )
                time.sleep(1.0 / self.args.rate)
        finally:
            self.last_facing_yaw = target_yaw
            self.send_idle_burst(preserve_facing=True)

        final_state = self.state_sub.latest()
        actual_degrees: Optional[float] = None
        if final_state is not None:
            self._ensure_yaw_origin(final_state)
            actual_degrees = math.degrees(wrap_pi(self._relative_yaw(final_state) - start_yaw))
        execute_time = time.monotonic() - command_start_time

        if completed:
            print(f"Rotate complete. final_error={math.degrees(last_error):.2f} deg")
        else:
            print(f"Rotate timeout. final_error={math.degrees(last_error):.2f} deg")
            print("Try a longer timeout or lower command rate, e.g. rotate 45 30 12")
        self._append_log(
            action="rotate",
            expected_magnitude=degrees,
            rate=requested_rate,
            actual_magnitude=actual_degrees,
            execute_time=execute_time,
        )

    def status(self) -> None:
        state = self.state_sub.latest()
        if state is None:
            print("No state received yet.")
        else:
            self._ensure_yaw_origin(state)
            age = time.monotonic() - state.timestamp
            yaw_rate_text = (
                f"yaw_rate={math.degrees(state.yaw_rate):.2f} deg/s, "
                if state.yaw_rate is not None
                else ""
            )
            print(
                f"yaw_abs={math.degrees(state.yaw):.2f} deg, "
                f"yaw_rel={math.degrees(self._relative_yaw(state)):.2f} deg, "
                f"{yaw_rate_text}"
                f"delta_heading={state.delta_heading:.3f} rad, "
                f"state_age={age:.2f} s"
            )
        odom = self._latest_odom()
        if odom is not None:
            odom_age = time.monotonic() - odom.timestamp
            print(
                f"odom_pos=[{odom.position[0]:.3f}, {odom.position[1]:.3f}, {odom.position[2]:.3f}], "
                f"odom_yaw={math.degrees(odom.yaw):.2f} deg, "
                f"odom_age={odom_age:.2f} s"
            )
        elif self.odom_sub is not None and self.odom_sub.error:
            print(f"odom unavailable: {self.odom_sub.error}")
        elif self.odom_sub is not None and self.odom_sub.available:
            print("odom subscriber enabled, but no rt/odostate sample received yet.")
        elif self.odom_sub is None:
            print("odom disabled.")

    def _ensure_yaw_origin(self, state: RobotState) -> None:
        if self.yaw_origin is None:
            self.yaw_origin = state.yaw

    def _relative_yaw(self, state: RobotState) -> float:
        if self.yaw_origin is None:
            return state.yaw
        return wrap_pi(state.yaw - self.yaw_origin)

    def _latest_odom(self) -> Optional[OdomState]:
        if self.odom_sub is None or not self.odom_sub.available:
            return None
        return self.odom_sub.latest()

    def _wait_for_odom(self, timeout: float) -> Optional[OdomState]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            odom = self._latest_odom()
            if odom is not None:
                return odom
            time.sleep(0.02)
        return self._latest_odom()

    def _init_log_file(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        if not self.log_path.exists() or self.log_path.stat().st_size == 0:
            with self.log_path.open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "action",
                    "expected_magnitude",
                    "rate",
                    "actual_magnitude",
                    "excute_time",
                ])
        print(f"Experiment log: {self.log_path}")

    def _append_log(
        self,
        action: str,
        expected_magnitude: float,
        rate: float,
        actual_magnitude: Optional[float],
        execute_time: float,
    ) -> None:
        with self.log_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                action,
                f"{expected_magnitude:.6f}",
                f"{rate:.6f}",
                "" if actual_magnitude is None else f"{actual_magnitude:.6f}",
                f"{execute_time:.6f}",
            ])
        print(f"Logged {action} row -> {self.log_path}")

    def _print_move_calibration(
        self,
        target_distance: float,
        command_speed: float,
        start_time: float,
        start_odom: Optional[OdomState],
        command_duration: float,
    ) -> Optional[float]:
        end_odom = self._latest_odom()
        if start_odom is None or end_odom is None:
            print("No sim odom available; cannot report actual move distance.")
            if self.odom_sub is None:
                print("Odom source is disabled. Start with --odom-source auto or --odom-source dds.")
            elif self.odom_sub.error:
                print(f"Odom error: {self.odom_sub.error}")
            elif self.odom_sub.available:
                print("DDS odom subscriber is enabled, but no rt/odostate sample was received.")
            return None

        dx = end_odom.position[0] - start_odom.position[0]
        dy = end_odom.position[1] - start_odom.position[1]
        traveled = math.hypot(dx, dy)
        elapsed = max(time.monotonic() - start_time, 1e-6)
        expected_abs = abs(target_distance)
        effective_speed = traveled / elapsed
        print(
            f"Sim odom: traveled={traveled:.3f} m, elapsed={elapsed:.2f} s, "
            f"effective_speed={effective_speed:.3f} m/s"
        )
        if traveled > 1e-4:
            correction = expected_abs / traveled
            suggested_duration = command_duration * correction
            print(
                f"Calibration: duration_scale={correction:.3f}, "
                f"suggested_t={suggested_duration:.2f} s for {expected_abs:.3f} m "
                f"at command_speed={command_speed:.3f}"
            )
        else:
            print("Calibration: measured travel is near zero; check control start/planner state.")
        return math.copysign(traveled, target_distance)

    def build_move_grid(self, max_extra_time: Optional[float] = None) -> list[tuple[float, float, float]]:
        max_extra = self.args.grid_max_extra_time if max_extra_time is None else max_extra_time
        distances = self._build_grid_distances()

        short_speed_count = int(round(
            (self.args.grid_max_move_speed - self.args.grid_min_move_speed)
            / self.args.grid_speed_step
        ))
        short_speeds = [
            round(self.args.grid_min_move_speed + idx * self.args.grid_speed_step, 6)
            for idx in range(short_speed_count + 1)
        ]
        grid: list[tuple[float, float, float]] = []
        time_offsets = [
            round(idx * self.args.grid_time_step, 6)
            for idx in range(int(round(max_extra / self.args.grid_time_step)) + 1)
        ]
        for distance in distances:
            speeds = short_speeds if abs(distance) <= self.args.grid_short_distance_max else [self.args.grid_max_move_speed]
            for speed in speeds:
                min_time = abs(distance) / speed
                for offset in time_offsets:
                    grid.append((distance, speed, round(min_time + offset, 6)))
        return grid

    def _build_grid_distances(self) -> list[float]:
        min_distance = self.args.grid_min_distance
        max_distance = self.args.grid_max_distance
        if min_distance > max_distance:
            min_distance, max_distance = max_distance, min_distance

        distances: list[float] = []

        def add_range(start: float, stop: float, step: float) -> None:
            current = start
            while current <= stop + 1e-9:
                if min_distance - 1e-9 <= current <= max_distance + 1e-9 and abs(current) > 1e-9:
                    distances.append(round(current, 6))
                current += step

        far_step = self.args.grid_far_distance_step
        near_step = self.args.grid_distance_step
        near_limit = self.args.grid_short_distance_max

        add_range(min_distance, -near_limit - 1e-9, far_step)
        add_range(-near_limit, -near_step, near_step)
        add_range(near_step, near_limit, near_step)
        add_range(near_limit + far_step, max_distance, far_step)

        return sorted(set(distances))

    def grid_move(self, confirm: bool = False, max_extra_time: Optional[float] = None) -> None:
        grid = self.build_move_grid(max_extra_time=max_extra_time)
        trials = self.args.grid_trials
        estimated_active_time = sum(execute_time for _, _, execute_time in grid) * trials
        estimated_total_time = estimated_active_time + len(grid) * trials * (
            self.args.move_settle_time + self.args.grid_trial_pause
        )
        print(
            f"Move grid: combos={len(grid)}, trials={trials}, "
            f"runs={len(grid) * trials}, estimated_time={estimated_total_time / 60.0:.1f} min"
        )
        print(
            f"Distances: {self.args.grid_min_distance:.2f}m .. {self.args.grid_max_distance:.2f}m; "
            f"near_step={self.args.grid_distance_step:.2f}m, "
            f"far_step={self.args.grid_far_distance_step:.2f}m; "
            f"short-distance speeds: {self.args.grid_min_move_speed:.2f} .. "
            f"{self.args.grid_max_move_speed:.2f}; time_step={self.args.grid_time_step:.2f}s"
        )
        if not confirm:
            plan_path = self._create_grid_plan_xml(grid, max_extra_time=max_extra_time)
            print(f"Plan XML: {plan_path}")
            print("Dry run only. Use: grid_move yes")
            return

        plan_path = self._find_latest_incomplete_grid_plan()
        if plan_path is None:
            plan_path = self._create_grid_plan_xml(grid, max_extra_time=max_extra_time)
            print(f"No incomplete plan found. Created new plan: {plan_path}")
        else:
            print(f"Resuming grid plan: {plan_path}")

        try:
            tree = ET.parse(plan_path)
            root = tree.getroot()
            root.set("status", "in_progress")
            self._write_xml(tree, plan_path)
            runs = root.findall("./runs/run")
            total_runs = len(runs)
            completed_before = sum(1 for run in runs if run.get("status") == "completed")
            print(f"Completed before resume: {completed_before}/{total_runs}")

            for run_index, run in enumerate(runs, start=1):
                if run.get("status") == "completed":
                    continue
                distance = float(run.get("expected_magnitude", "0"))
                speed = float(run.get("rate", "0"))
                execute_time = float(run.get("excute_time", "0"))
                trial = int(run.get("trial", "1"))
                print(
                    f"\n[Grid run {run_index}/{total_runs}] "
                    f"distance={distance:.3f}, speed={speed:.3f}, "
                    f"execute_time={execute_time:.2f}, trial={trial}"
                )
                run.set("status", "running")
                run.set("started_at", time.strftime("%y%m%d_%H%M%S"))
                self._write_xml(tree, plan_path)

                actual = self.move(distance, speed, execute_time)
                if actual is not None:
                    run.set("actual_magnitude", f"{actual:.6f}")
                    run.set("abs_error", f"{abs(abs(actual) - abs(distance)):.6f}")
                    run.set("completed_at", time.strftime("%y%m%d_%H%M%S"))
                    run.set("status", "completed")
                else:
                    run.set("status", "pending")
                    run.set("last_error", "missing_odom")
                self._write_xml(tree, plan_path)

                if self.args.grid_trial_pause > 0:
                    time.sleep(self.args.grid_trial_pause)
        except KeyboardInterrupt:
            print("\nGrid interrupted -> idle.")
            self.send_idle_burst()

        results = self._collect_grid_results_from_plan(plan_path)
        if results:
            tree = ET.parse(plan_path)
            root = tree.getroot()
            runs = root.findall("./runs/run")
            if runs and all(run.get("status") == "completed" for run in runs):
                root.set("status", "completed")
                root.set("completed_at", time.strftime("%y%m%d_%H%M%S"))
                self._write_xml(tree, plan_path)
            self._write_grid_summary(results, plan_path=plan_path)
        else:
            print("No valid odom-backed grid results to summarize.")

    def _create_grid_plan_xml(
        self,
        grid: list[tuple[float, float, float]],
        max_extra_time: Optional[float] = None,
    ) -> Path:
        stamp = time.strftime("%y%m%d_%H%M%S")
        plan_path = self.grid_plan_dir / f"vigil_wbc_grid_plan_{stamp}.xml"
        suffix = 1
        while plan_path.exists():
            plan_path = self.grid_plan_dir / f"vigil_wbc_grid_plan_{stamp}_{suffix:02d}.xml"
            suffix += 1

        root = ET.Element(
            "vigil_wbc_grid_plan",
            {
                "id": plan_path.stem,
                "created_at": stamp,
                "status": "planned",
                "log_path": str(self.log_path),
                "move_settle_time": f"{self.args.move_settle_time:.6f}",
                "grid_trials": str(self.args.grid_trials),
                "grid_time_step": f"{self.args.grid_time_step:.6f}",
                "grid_max_extra_time": f"{(self.args.grid_max_extra_time if max_extra_time is None else max_extra_time):.6f}",
            },
        )
        params = ET.SubElement(root, "parameters")
        for key in (
            "grid_min_distance",
            "grid_max_distance",
            "grid_distance_step",
            "grid_far_distance_step",
            "grid_short_distance_max",
            "grid_min_move_speed",
            "grid_max_move_speed",
            "grid_speed_step",
            "grid_trial_pause",
        ):
            ET.SubElement(params, "param", {"name": key, "value": str(getattr(self.args, key))})

        runs_el = ET.SubElement(root, "runs")
        run_id = 1
        for combo_index, (distance, speed, execute_time) in enumerate(grid, start=1):
            for trial in range(1, self.args.grid_trials + 1):
                ET.SubElement(
                    runs_el,
                    "run",
                    {
                        "id": str(run_id),
                        "combo": str(combo_index),
                        "trial": str(trial),
                        "status": "pending",
                        "expected_magnitude": f"{distance:.6f}",
                        "rate": f"{speed:.6f}",
                        "excute_time": f"{execute_time:.6f}",
                    },
                )
                run_id += 1

        tree = ET.ElementTree(root)
        self._write_xml(tree, plan_path)
        return plan_path

    def _find_latest_incomplete_grid_plan(self) -> Optional[Path]:
        candidates = sorted(self.grid_plan_dir.glob("vigil_wbc_grid_plan_*.xml"))
        incomplete: list[Path] = []
        for path in candidates:
            try:
                root = ET.parse(path).getroot()
                if root.tag != "vigil_wbc_grid_plan":
                    continue
                if root.get("status") == "completed":
                    continue
                runs = root.findall("./runs/run")
                if any(run.get("status") != "completed" for run in runs):
                    incomplete.append(path)
            except ET.ParseError:
                continue
        return incomplete[-1] if incomplete else None

    def _collect_grid_results_from_plan(self, plan_path: Path) -> list[dict[str, float]]:
        root = ET.parse(plan_path).getroot()
        results: list[dict[str, float]] = []
        for run in root.findall("./runs/run"):
            if run.get("status") != "completed":
                continue
            actual = run.get("actual_magnitude")
            abs_error = run.get("abs_error")
            if actual is None or abs_error is None:
                continue
            results.append({
                "expected_magnitude": float(run.get("expected_magnitude", "0")),
                "rate": float(run.get("rate", "0")),
                "excute_time": float(run.get("excute_time", "0")),
                "trial": float(run.get("trial", "1")),
                "actual_magnitude": float(actual),
                "abs_error": float(abs_error),
            })
        return results

    @staticmethod
    def _write_xml(tree: ET.ElementTree, path: Path) -> None:
        ET.indent(tree, space="  ")
        tree.write(path, encoding="utf-8", xml_declaration=True)

    def _write_grid_summary(self, results: list[dict[str, float]], plan_path: Optional[Path] = None) -> None:
        grouped: dict[tuple[float, float, float], list[dict[str, float]]] = {}
        for row in results:
            key = (row["expected_magnitude"], row["rate"], row["excute_time"])
            grouped.setdefault(key, []).append(row)

        combo_stats: list[dict[str, float]] = []
        for (distance, rate, execute_time), rows in grouped.items():
            mean_actual = sum(row["actual_magnitude"] for row in rows) / len(rows)
            mean_abs_error = sum(row["abs_error"] for row in rows) / len(rows)
            combo_stats.append({
                "expected_magnitude": distance,
                "rate": rate,
                "excute_time": execute_time,
                "mean_actual_magnitude": mean_actual,
                "mean_abs_error": mean_abs_error,
                "trials": float(len(rows)),
            })

        best_by_distance: dict[float, dict[str, float]] = {}
        for stat in combo_stats:
            distance = stat["expected_magnitude"]
            current = best_by_distance.get(distance)
            if current is None or stat["mean_abs_error"] < current["mean_abs_error"]:
                best_by_distance[distance] = stat

        stamp = time.strftime("%y%m%d_%H%M%S")
        if plan_path is not None:
            summary_csv = plan_path.with_name(f"{plan_path.stem}_summary.csv")
            summary_xml = plan_path.with_name(f"{plan_path.stem}_summary.xml")
        else:
            summary_csv = self.log_path.with_name(f"{self.log_path.stem}_summary_{stamp}.csv")
            summary_xml = self.log_path.with_name(f"{self.log_path.stem}_summary_{stamp}.xml")

        with summary_csv.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "expected_magnitude",
                "best_rate",
                "best_excute_time",
                "mean_actual_magnitude",
                "mean_abs_error",
                "trials",
            ])
            for distance in sorted(best_by_distance):
                stat = best_by_distance[distance]
                writer.writerow([
                    f"{distance:.6f}",
                    f"{stat['rate']:.6f}",
                    f"{stat['excute_time']:.6f}",
                    f"{stat['mean_actual_magnitude']:.6f}",
                    f"{stat['mean_abs_error']:.6f}",
                    int(stat["trials"]),
                ])

        root = ET.Element(
            "vigil_wbc_grid_summary",
            {
                "generated_at": stamp,
                "source_log": str(self.log_path),
                "source_plan": "" if plan_path is None else str(plan_path),
            },
        )
        for distance in sorted(best_by_distance):
            stat = best_by_distance[distance]
            ET.SubElement(
                root,
                "target",
                {
                    "expected_magnitude": f"{distance:.6f}",
                    "best_rate": f"{stat['rate']:.6f}",
                    "best_excute_time": f"{stat['excute_time']:.6f}",
                    "mean_actual_magnitude": f"{stat['mean_actual_magnitude']:.6f}",
                    "mean_abs_error": f"{stat['mean_abs_error']:.6f}",
                    "trials": str(int(stat["trials"])),
                },
            )
        tree = ET.ElementTree(root)
        self._write_xml(tree, summary_xml)

        print(f"Grid summary CSV: {summary_csv}")
        print(f"Grid summary XML: {summary_xml}")

    def repl(self) -> None:
        self.print_help()
        while True:
            try:
                line = input("vigil-wbc> ").strip()
            except EOFError:
                print()
                break
            except KeyboardInterrupt:
                print("\nKeyboardInterrupt -> idle.")
                self.send_idle_burst()
                continue

            if not line:
                continue

            parts = line.split()
            command = parts[0].lower()
            try:
                if command in {"q", "quit", "exit"}:
                    break
                if command in {"help", "h", "?"}:
                    self.print_help()
                elif command in {"start", "s"}:
                    self.start_control()
                elif command in {"stop", "halt"}:
                    self.send_idle_burst()
                    if command == "stop":
                        self.stop_control()
                    else:
                        print("Halt -> idle planner command sent.")
                elif command in {"status", "st"}:
                    self.status()
                elif command in {"move", "m"}:
                    if len(parts) < 2:
                        print("Usage: move <meters> [speed_mps] [execute_time_s]")
                        continue
                    distance = float(parts[1])
                    speed = float(parts[2]) if len(parts) >= 3 else None
                    execute_time = float(parts[3]) if len(parts) >= 4 else None
                    self.move(distance, speed, execute_time)
                elif command in {"move_model", "mm", "move_calib", "mc"}:
                    if len(parts) < 2:
                        print("Usage: move_model <meters>")
                        continue
                    self.move_with_model(float(parts[1]))
                elif command in {"grid_move", "grid"}:
                    confirm = len(parts) >= 2 and parts[1].lower() in {"yes", "y", "run"}
                    max_extra_time = float(parts[2]) if len(parts) >= 3 else None
                    self.grid_move(confirm=confirm, max_extra_time=max_extra_time)
                elif command in {"rotate", "rot", "r"}:
                    if len(parts) < 2:
                        print("Usage: rotate <degrees> [command_rate_deg_s] [timeout_s]")
                        continue
                    rate = float(parts[2]) if len(parts) >= 3 else None
                    timeout = float(parts[3]) if len(parts) >= 4 else None
                    self.rotate(float(parts[1]), rate, timeout)
                else:
                    print(f"Unknown command: {command}")
                    self.print_help()
            except ValueError as exc:
                print(f"Bad argument: {exc}")

    @staticmethod
    def print_help() -> None:
        print("")
        print("Commands:")
        print("  start                 start WBC planner control")
        print("  move <m> [speed] [execute_time]  open-loop move, e.g. move 1 1 1.5")
        print("  move_model <m>        calibrated move; splits targets beyond model range")
        print("  grid_move plan|yes [max_extra_time]  run move calibration grid")
        print("  rotate <deg> [rate] [timeout]  closed-loop yaw rotate, e.g. rotate 45 30 12")
        print("  halt                  send IDLE planner command")
        print("  status                print latest yaw from g1_debug")
        print("  stop                  stop WBC control process state")
        print("  quit                  exit this controller")
        print("")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vigil primitive controller for GR00T WBC.")
    parser.add_argument("--bind-host", default="*", help="PUB bind host for command/planner topics.")
    parser.add_argument("--port", type=int, default=5556, help="PUB port for command/planner topics.")
    parser.add_argument("--state-host", default="localhost", help="Host for g1_debug state subscriber.")
    parser.add_argument("--state-port", type=int, default=5557, help="Port for g1_debug state subscriber.")
    parser.add_argument("--state-topic", default="g1_debug", help="State topic published by deploy.")
    parser.add_argument("--rate", type=float, default=20.0, help="Planner command publish rate.")
    parser.add_argument("--move-speed", type=float, default=0.25, help="Default open-loop move speed.")
    parser.add_argument("--min-move-speed", type=float, default=0.20, help="Minimum commanded speed.")
    parser.add_argument("--max-move-speed", type=float, default=1.00, help="Maximum speed for this primitive.")
    parser.add_argument("--move-settle-time", type=float, default=0.8, help="Idle time before measuring final move distance.")
    parser.add_argument("--rotate-tolerance-deg", type=float, default=3.0, help="Rotate yaw tolerance.")
    parser.add_argument("--rotate-timeout", type=float, default=4.0, help="Minimum rotate timeout.")
    parser.add_argument("--rotate-extra-time", type=float, default=8.0, help="Extra correction time after yaw target ramp.")
    parser.add_argument("--rotate-command-rate-deg", type=float, default=35.0, help="Default yaw target ramp rate.")
    parser.add_argument("--min-rotate-command-rate-deg", type=float, default=10.0, help="Minimum yaw target ramp rate.")
    parser.add_argument("--max-rotate-command-rate-deg", type=float, default=90.0, help="Maximum yaw target ramp rate.")
    parser.add_argument("--rotate-settle-time", type=float, default=0.35, help="Time yaw error must remain small.")
    parser.add_argument("--rotate-yaw-rate-tolerance-deg", type=float, default=8.0, help="Yaw-rate threshold for settle.")
    parser.add_argument("--state-timeout", type=float, default=3.0, help="Seconds to wait for state before rotate.")
    parser.add_argument("--odom-source", choices=["auto", "dds", "off"], default="auto", help="Use sim rt/odostate if available.")
    parser.add_argument("--dds-interface", default="lo", help="DDS interface for sim odometry.")
    parser.add_argument("--dds-domain", type=int, default=0, help="DDS domain id for sim odometry.")
    parser.add_argument("--log-file", default="auto", help="CSV log file path, or 'auto' for timestamped default.")
    parser.add_argument("--move-model-file", default="auto", help="Move model JSON path, 'auto', or 'off'.")
    parser.add_argument("--model-chunk-pause", type=float, default=0.2, help="Pause between calibrated move chunks.")
    parser.add_argument("--grid-distance-step", type=float, default=0.25, help="Distance grid step in meters.")
    parser.add_argument("--grid-far-distance-step", type=float, default=0.5, help="Distance step when abs(distance) > short-distance max.")
    parser.add_argument("--grid-min-distance", type=float, default=-5.0, help="Minimum move target distance in meters.")
    parser.add_argument("--grid-max-distance", type=float, default=5.0, help="Maximum move target distance in meters.")
    parser.add_argument("--grid-short-distance-max", type=float, default=1.0, help="Use speed grid up to this distance.")
    parser.add_argument("--grid-min-move-speed", type=float, default=0.25, help="Minimum speed for short-distance grid.")
    parser.add_argument("--grid-max-move-speed", type=float, default=1.0, help="Maximum/grid speed in m/s.")
    parser.add_argument("--grid-speed-step", type=float, default=0.25, help="Speed step for short-distance grid.")
    parser.add_argument("--grid-time-step", type=float, default=0.1, help="Execute-time offset step in seconds.")
    parser.add_argument("--grid-max-extra-time", type=float, default=0.4, help="Maximum extra time above distance/rate.")
    parser.add_argument("--grid-trials", type=int, default=3, help="Trials per grid point.")
    parser.add_argument("--grid-trial-pause", type=float, default=0.2, help="Pause between grid trials.")
    parser.add_argument("--grid-plan-dir", default="outputs/vigil_grid_plans", help="Directory for resumable grid XML plans.")
    parser.add_argument("--auto-start", action="store_true", help="Send start command on launch.")
    parser.add_argument("--verbose", action="store_true", help="Print every ZMQ command.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    controller = PrimitiveController(args)
    try:
        if args.auto_start:
            controller.start_control()
        controller.repl()
    finally:
        controller.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
