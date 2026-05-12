#!/usr/bin/env python3
"""Lightweight real-robot primitive controller for G1 WBC.

This is the small real-deploy version of the Vigil controller. It only keeps:
  - move: open-loop distance/speed/time command
  - rotate: yaw closed-loop using g1_debug base_quat
  - move_model: calibrated open-loop move using a model JSON

No sim odom, grid search, XML summaries, plots, or analysis are included here.
"""

from __future__ import annotations

import argparse
import json
import math
import msgpack
import queue
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import zmq


REPO_ROOT = Path(__file__).resolve().parents[2]
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


def topic_msgpack_payload(message: bytes, topic: str) -> Optional[dict]:
    topic_bytes = topic.encode("utf-8")
    if not message.startswith(topic_bytes):
        return None
    return msgpack.unpackb(message[len(topic_bytes):], raw=False)


@dataclass
class RobotState:
    yaw: float
    delta_heading: float
    timestamp: float
    yaw_rate: Optional[float] = None


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


class PackedPublisher:
    def __init__(self, bind_host: str, port: int, verbose: bool = False) -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{bind_host}:{port}")
        self.verbose = verbose
        time.sleep(0.5)

    def close(self) -> None:
        self.socket.close(0)
        self.context.term()

    def send_command(self, start: bool, stop: bool, planner: bool = True) -> None:
        header = {
            "v": 1,
            "endian": "le",
            "count": 1,
            "fields": [
                {"name": "start", "dtype": "u8", "shape": [1]},
                {"name": "stop", "dtype": "u8", "shape": [1]},
                {"name": "planner", "dtype": "u8", "shape": [1]},
            ],
        }
        self._send_packed("command", header, struct.pack("BBB", int(start), int(stop), int(planner)))
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
        header = {
            "v": 1,
            "endian": "le",
            "count": 1,
            "fields": [
                {"name": "mode", "dtype": "i32", "shape": [1]},
                {"name": "movement", "dtype": "f32", "shape": [3]},
                {"name": "facing", "dtype": "f32", "shape": [3]},
                {"name": "speed", "dtype": "f32", "shape": [1]},
                {"name": "height", "dtype": "f32", "shape": [1]},
            ],
        }
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
                if not data or data.get("base_quat") is None:
                    continue
                base_quat = data["base_quat"]
                base_ang_vel = data.get("base_ang_vel")
                yaw_rate = float(base_ang_vel[2]) if base_ang_vel and len(base_ang_vel) >= 3 else None
                state = RobotState(
                    yaw=yaw_from_quat_wxyz(base_quat),
                    delta_heading=float(data.get("delta_heading", 0.0)),
                    timestamp=time.monotonic(),
                    yaw_rate=yaw_rate,
                )
                with self._lock:
                    self._latest = state
            except Exception as exc:  # noqa: BLE001 - keep subscriber alive on real robot.
                self._remember_error(str(exc))

    def latest(self) -> Optional[RobotState]:
        with self._lock:
            return self._latest

    def wait_for_state(self, timeout: float) -> Optional[RobotState]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.latest()
            if state is not None:
                return state
            time.sleep(0.02)
        return None

    def close(self) -> None:
        self._stop_event.set()
        self.join(timeout=1.0)
        self.socket.close(0)
        self.context.term()

    def _remember_error(self, text: str) -> None:
        try:
            self.errors.put_nowait(text)
        except queue.Full:
            pass


class RealPrimitiveController:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.publisher = PackedPublisher(args.bind_host, args.port, args.verbose)
        self.state_sub = StateSubscriber(args.state_host, args.state_port, args.state_topic)
        self.state_sub.start()
        self.yaw_origin: Optional[float] = None
        self.last_facing_yaw = 0.0
        self.move_model = self._load_move_model(args.move_model_file)

    def close(self) -> None:
        self.send_idle_burst(duration=0.4)
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

    def move(self, distance_m: float, speed_mps: Optional[float] = None, execute_time_s: Optional[float] = None) -> None:
        speed = abs(speed_mps if speed_mps is not None else self.args.move_speed)
        speed = min(max(speed, self.args.min_move_speed), self.args.max_move_speed)
        duration = execute_time_s if execute_time_s is not None else abs(distance_m) / speed if speed > 1e-6 else 0.0
        if duration <= 0.0:
            self.send_idle_burst()
            return

        state = self.state_sub.latest()
        if state is not None:
            self._ensure_yaw_origin(state)
            self.last_facing_yaw = self._relative_yaw(state)

        sign = 1.0 if distance_m >= 0.0 else -1.0
        facing = facing_from_yaw(self.last_facing_yaw)
        movement = [sign * facing[0], sign * facing[1], 0.0]
        mode = LOCO_SLOW_WALK if speed <= self.args.slow_walk_speed_threshold else LOCO_WALK

        print(f"Move: distance={distance_m:.3f} m, speed={speed:.3f} m/s, duration={duration:.2f} s")
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline:
                self.publisher.send_planner(mode, movement, facing, speed, -1.0)
                time.sleep(1.0 / self.args.rate)
        finally:
            self.send_idle_burst(duration=self.args.move_settle_time, preserve_facing=True)
            print("Move complete -> idle.")

    def move_with_model(self, distance_m: float) -> None:
        if self.move_model is None:
            print("No move model loaded. Pass --move-model-file PATH, or generate one from sim first.")
            return
        chunks = self._split_model_move(distance_m)
        print(f"Model move: target={distance_m:.3f} m, chunks={len(chunks)}")
        for idx, chunk in enumerate(chunks, start=1):
            speed, execute_time = self._predict_model_command(chunk)
            print(
                f"  chunk {idx}/{len(chunks)}: "
                f"distance={chunk:.3f} m, speed={speed:.3f} m/s, execute_time={execute_time:.2f} s"
            )

        for idx, chunk in enumerate(chunks, start=1):
            speed, execute_time = self._predict_model_command(chunk)
            print(f"\n[Model move chunk {idx}/{len(chunks)}]")
            self.move(chunk, speed, execute_time)
            if idx < len(chunks) and self.args.model_chunk_pause > 0.0:
                time.sleep(self.args.model_chunk_pause)
        print("Model move complete.")

    def rotate(self, degrees: float, rate_deg_s: Optional[float] = None, timeout_s: Optional[float] = None) -> None:
        state = self.state_sub.wait_for_state(timeout=self.args.state_timeout)
        if state is None:
            print("No g1_debug state received; cannot run rotate.")
            print("Start deploy with: ./deploy.sh real --input-type zmq_manager --output-type all")
            return

        self._ensure_yaw_origin(state)
        start_yaw = self._relative_yaw(state)
        target_yaw = wrap_pi(start_yaw + math.radians(degrees))
        command_rate = abs(rate_deg_s if rate_deg_s is not None else self.args.rotate_command_rate_deg)
        command_rate = min(max(command_rate, self.args.min_rotate_command_rate_deg), self.args.max_rotate_command_rate_deg)
        timeout = timeout_s if timeout_s is not None else max(
            self.args.rotate_timeout,
            abs(degrees) / max(command_rate, 1.0) + self.args.rotate_extra_time,
        )
        tolerance = math.radians(self.args.rotate_tolerance_deg)
        yaw_rate_tol = math.radians(self.args.rotate_yaw_rate_tolerance_deg)
        last_error = wrap_pi(target_yaw - start_yaw)
        completed = False

        print(
            "Rotate: "
            f"start={math.degrees(start_yaw):.1f} deg, "
            f"target={math.degrees(target_yaw):.1f} deg, "
            f"cmd_rate={command_rate:.1f} deg/s, "
            f"tol={self.args.rotate_tolerance_deg:.1f} deg, "
            f"timeout={timeout:.1f} s, "
            f"fb_gain={self.args.rotate_feedback_gain:.2f}, "
            f"fb_limit={self.args.rotate_feedback_limit_deg:.1f} deg"
        )

        attempts = 1 + max(0, self.args.rotate_correction_retries)
        for attempt in range(1, attempts + 1):
            attempt_start_yaw = start_yaw
            if attempt == 1:
                command_target_yaw = target_yaw
                attempt_timeout = timeout
            else:
                state = self.state_sub.latest()
                if state is not None:
                    self._ensure_yaw_origin(state)
                    attempt_start_yaw = self._relative_yaw(state)
                    last_error = wrap_pi(target_yaw - attempt_start_yaw)
                if abs(last_error) <= tolerance:
                    break
                boost = math.radians(self.args.rotate_correction_boost_deg)
                command_target_yaw = wrap_pi(target_yaw + math.copysign(boost, last_error))
                attempt_timeout = max(
                    self.args.rotate_correction_min_time,
                    abs(math.degrees(last_error)) / max(command_rate, 1.0) + self.args.rotate_correction_extra_time,
                )
                print(
                    f"Rotate correction {attempt - 1}/{attempts - 1}: "
                    f"remaining_error={math.degrees(last_error):.2f} deg, "
                    f"command_target={math.degrees(command_target_yaw):.1f} deg, "
                    f"timeout={attempt_timeout:.1f} s"
                )

            completed, last_error = self._run_rotate_attempt(
                desired_yaw=target_yaw,
                command_target_yaw=command_target_yaw,
                start_command_yaw=attempt_start_yaw,
                command_rate=command_rate,
                timeout=attempt_timeout,
                tolerance=tolerance,
                yaw_rate_tol=yaw_rate_tol,
            )
            if completed:
                break

        self.last_facing_yaw = target_yaw
        self.send_idle_burst(preserve_facing=True)

        if completed:
            print(f"Rotate complete. final_error={math.degrees(last_error):.2f} deg")
        else:
            print(f"Rotate timeout. final_error={math.degrees(last_error):.2f} deg")

    def _run_rotate_attempt(
        self,
        desired_yaw: float,
        command_target_yaw: float,
        start_command_yaw: float,
        command_rate: float,
        timeout: float,
        tolerance: float,
        yaw_rate_tol: float,
    ) -> tuple[bool, float]:
        deadline = time.monotonic() + timeout
        ramp_yaw = start_command_yaw
        last_command_time = time.monotonic()
        settle_since: Optional[float] = None
        last_error = wrap_pi(desired_yaw - start_command_yaw)

        while time.monotonic() < deadline:
            now = time.monotonic()
            dt = max(now - last_command_time, 1e-3)
            last_command_time = now
            remaining_command = wrap_pi(command_target_yaw - ramp_yaw)
            max_step = math.radians(command_rate) * dt
            if abs(remaining_command) <= max_step:
                ramp_yaw = command_target_yaw
            else:
                ramp_yaw = wrap_pi(ramp_yaw + math.copysign(max_step, remaining_command))

            state = self.state_sub.latest()
            yaw_rate = None
            command_yaw = ramp_yaw
            if state is not None:
                self._ensure_yaw_origin(state)
                last_error = wrap_pi(desired_yaw - self._relative_yaw(state))
                yaw_rate = state.yaw_rate
                feedback_limit = math.radians(self.args.rotate_feedback_limit_deg)
                feedback = self.args.rotate_feedback_gain * last_error
                feedback = min(max(feedback, -feedback_limit), feedback_limit)
                command_yaw = wrap_pi(ramp_yaw + feedback)

                yaw_is_stable = yaw_rate is None or abs(yaw_rate) <= yaw_rate_tol
                if abs(last_error) <= tolerance and yaw_is_stable:
                    if settle_since is None:
                        settle_since = now
                else:
                    settle_since = None

                if settle_since is not None and (now - settle_since) >= self.args.rotate_settle_time:
                    return True, last_error

            self.publisher.send_planner(LOCO_IDLE, [0.0, 0.0, 0.0], facing_from_yaw(command_yaw), -1.0, -1.0)
            time.sleep(1.0 / self.args.rate)

        return False, last_error

    def status(self) -> None:
        state = self.state_sub.latest()
        if state is None:
            print("No g1_debug state received yet.")
            return
        self._ensure_yaw_origin(state)
        yaw_rate = "" if state.yaw_rate is None else f", yaw_rate={math.degrees(state.yaw_rate):.2f} deg/s"
        print(
            f"yaw_abs={math.degrees(state.yaw):.2f} deg, "
            f"yaw_rel={math.degrees(self._relative_yaw(state)):.2f} deg"
            f"{yaw_rate}, delta_heading={state.delta_heading:.3f} rad, "
            f"state_age={time.monotonic() - state.timestamp:.2f} s"
        )

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
            forward = self._parse_move_model_samples(payload["models"]["forward"])
            backward = self._parse_move_model_samples(payload["models"]["backward"])
            if not forward or not backward:
                raise ValueError("forward/backward samples are required")
        except Exception as exc:  # noqa: BLE001
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
        if self.move_model is None or abs(distance_m) < 1e-9:
            return []
        sign = 1.0 if distance_m > 0.0 else -1.0
        limit = self.move_model.max_forward_magnitude if sign > 0.0 else self.move_model.max_backward_magnitude
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

    def _ensure_yaw_origin(self, state: RobotState) -> None:
        if self.yaw_origin is None:
            self.yaw_origin = state.yaw

    def _relative_yaw(self, state: RobotState) -> float:
        if self.yaw_origin is None:
            return state.yaw
        return wrap_pi(state.yaw - self.yaw_origin)

    def repl(self) -> None:
        self.print_help()
        while True:
            try:
                line = input("vigil-real> ").strip()
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
                elif command == "halt":
                    self.send_idle_burst()
                    print("Halt -> idle planner command sent.")
                elif command == "stop":
                    self.send_idle_burst()
                    self.stop_control()
                elif command in {"status", "st"}:
                    self.status()
                elif command in {"move", "m"}:
                    if len(parts) < 2:
                        print("Usage: move <meters> [speed_mps] [execute_time_s]")
                        continue
                    self.move(
                        float(parts[1]),
                        float(parts[2]) if len(parts) >= 3 else None,
                        float(parts[3]) if len(parts) >= 4 else None,
                    )
                elif command in {"move_model", "mm", "move_calib", "mc"}:
                    if len(parts) < 2:
                        print("Usage: move_model <meters>")
                        continue
                    self.move_with_model(float(parts[1]))
                elif command in {"rotate", "rot", "r"}:
                    if len(parts) < 2:
                        print("Usage: rotate <degrees> [command_rate_deg_s] [timeout_s]")
                        continue
                    self.rotate(
                        float(parts[1]),
                        float(parts[2]) if len(parts) >= 3 else None,
                        float(parts[3]) if len(parts) >= 4 else None,
                    )
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
        print("  move <m> [speed] [execute_time]  open-loop move, e.g. move 0.2 0.25 0.8")
        print("  move_model <m>        calibrated move; splits targets beyond model range")
        print("  rotate <deg> [rate] [timeout]  yaw rotate, e.g. rotate 30 20 10")
        print("  halt                  send IDLE planner command")
        print("  status                print latest yaw from g1_debug")
        print("  stop                  stop WBC control process state")
        print("  quit                  exit this controller")
        print("")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Lightweight Vigil real-robot primitive controller.")
    parser.add_argument("--bind-host", default="*", help="PUB bind host for command/planner topics.")
    parser.add_argument("--port", type=int, default=5556, help="PUB port for command/planner topics.")
    parser.add_argument("--state-host", default="localhost", help="Host for g1_debug state subscriber.")
    parser.add_argument("--state-port", type=int, default=5557, help="Port for g1_debug state subscriber.")
    parser.add_argument("--state-topic", default="g1_debug", help="State topic published by deploy.")
    parser.add_argument("--rate", type=float, default=20.0, help="Planner command publish rate.")
    parser.add_argument("--move-speed", type=float, default=0.25, help="Default open-loop move speed.")
    parser.add_argument("--min-move-speed", type=float, default=0.10, help="Minimum commanded speed.")
    parser.add_argument("--max-move-speed", type=float, default=0.50, help="Real-robot max commanded move speed.")
    parser.add_argument("--slow-walk-speed-threshold", type=float, default=0.8, help="Use SLOW_WALK up to this speed.")
    parser.add_argument("--move-settle-time", type=float, default=0.8, help="Idle time after each move.")
    parser.add_argument("--move-model-file", default="auto", help="Move model JSON path, 'auto', or 'off'.")
    parser.add_argument("--model-chunk-pause", type=float, default=0.2, help="Pause between model-move chunks.")
    parser.add_argument("--rotate-tolerance-deg", type=float, default=3.0, help="Rotate yaw tolerance.")
    parser.add_argument("--rotate-timeout", type=float, default=4.0, help="Minimum rotate timeout.")
    parser.add_argument("--rotate-extra-time", type=float, default=8.0, help="Extra correction time after yaw target ramp.")
    parser.add_argument("--rotate-command-rate-deg", type=float, default=25.0, help="Default yaw target ramp rate.")
    parser.add_argument("--min-rotate-command-rate-deg", type=float, default=5.0, help="Minimum yaw target ramp rate.")
    parser.add_argument("--max-rotate-command-rate-deg", type=float, default=60.0, help="Maximum yaw target ramp rate.")
    parser.add_argument("--rotate-correction-retries", type=int, default=2, help="Residual correction attempts after rotate timeout.")
    parser.add_argument("--rotate-correction-boost-deg", type=float, default=10.0, help="Extra facing target angle during residual correction.")
    parser.add_argument("--rotate-correction-min-time", type=float, default=3.0, help="Minimum timeout for each residual correction.")
    parser.add_argument("--rotate-correction-extra-time", type=float, default=3.0, help="Extra timeout for residual correction.")
    parser.add_argument("--rotate-feedback-gain", type=float, default=1.0, help="Yaw error feedback gain for facing command.")
    parser.add_argument("--rotate-feedback-limit-deg", type=float, default=20.0, help="Max extra facing angle from yaw feedback.")
    parser.add_argument("--rotate-settle-time", type=float, default=0.35, help="Time yaw error must remain small.")
    parser.add_argument("--rotate-yaw-rate-tolerance-deg", type=float, default=8.0, help="Yaw-rate threshold for settle.")
    parser.add_argument("--state-timeout", type=float, default=3.0, help="Seconds to wait for state before rotate.")
    parser.add_argument("--auto-start", action="store_true", help="Send start command on launch.")
    parser.add_argument("--verbose", action="store_true", help="Print every ZMQ command.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    controller = RealPrimitiveController(args)
    try:
        if args.auto_start:
            controller.start_control()
        controller.repl()
    finally:
        controller.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
