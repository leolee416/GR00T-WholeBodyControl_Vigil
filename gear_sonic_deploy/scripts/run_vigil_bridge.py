#!/usr/bin/env python3
"""Run the GR00T-side Vigil bridge HTTP service."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from gear_sonic.vigil_bridge import VigilBridgeService
from gear_sonic.vigil_bridge.mujoco_adapter import (
    MujocoBridgeConfig,
    create_mujoco_bridge_service,
)
from gear_sonic.vigil_bridge.real_adapter import (
    RealBridgeConfig,
    create_real_bridge_service,
)
from gear_sonic.vigil_bridge.transport import serve_http


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the GR00T-side Vigil bridge HTTP service.")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host.")
    parser.add_argument("--port", type=int, default=8765, help="HTTP bind port.")
    parser.add_argument(
        "--backend",
        default="dry_run",
        choices=("dry_run", "mujoco", "real"),
        help="Bridge backend implementation. Use mujoco/real only with runtime/deploy already running.",
    )
    parser.add_argument(
        "--runtime-mode",
        default=None,
        choices=("dry_run", "mujoco", "real"),
        help="Runtime mode reported through the bridge protocol. Defaults to the selected backend.",
    )
    parser.add_argument("--command-bind-host", default="*", help="Runtime ZMQ command/planner PUB bind host.")
    parser.add_argument("--command-port", type=int, default=5556, help="Runtime ZMQ command/planner PUB port.")
    parser.add_argument("--state-host", default="localhost", help="Runtime g1_debug state ZMQ host.")
    parser.add_argument("--state-port", type=int, default=5557, help="Runtime g1_debug state ZMQ port.")
    parser.add_argument("--state-topic", default="g1_debug", help="Runtime state topic published by deploy.")
    parser.add_argument("--rate", type=float, default=20.0, help="Planner command publish rate.")
    parser.add_argument("--move-speed", type=float, default=0.25, help="Default move speed in m/s.")
    parser.add_argument("--min-move-speed", type=float, default=0.20, help="Minimum move speed in m/s.")
    parser.add_argument(
        "--max-speed-mps",
        "--max-move-speed",
        dest="max_move_speed",
        type=float,
        default=1.00,
        help="Maximum move speed exposed as safety.max_speed_mps default, in m/s.",
    )
    parser.add_argument(
        "--move-model-file",
        default="auto",
        help="Move model JSON path, or 'auto' for latest outputs/vigil_move_models/vigil_move_model_*.json.",
    )
    parser.add_argument(
        "--disable-move-model",
        action="store_true",
        help="Use direct open-loop move(distance, speed, duration) instead of calibrated move_model.",
    )
    parser.add_argument("--model-chunk-pause", type=float, default=0.0, help="Pause between move_model chunks.")
    parser.add_argument("--move-settle-time", type=float, default=0.8, help="Idle settle time after move.")
    parser.add_argument("--rotate-rate", type=float, default=35.0, help="Default rotate command rate in deg/s.")
    parser.add_argument("--rotate-timeout", type=float, default=4.0, help="Minimum rotate timeout in seconds.")
    parser.add_argument("--rotate-extra-time", type=float, default=8.0, help="Extra rotate correction time.")
    parser.add_argument("--rotate-tolerance-deg", type=float, default=3.0, help="Rotate yaw tolerance.")
    parser.add_argument("--state-timeout", type=float, default=3.0, help="Seconds to wait for g1_debug state.")
    parser.add_argument(
        "--odom-source",
        default="auto",
        choices=("auto", "dds", "off"),
        help="Use MuJoCo rt/odostate if available.",
    )
    parser.add_argument("--dds-interface", default="lo", help="DDS interface for sim odometry.")
    parser.add_argument("--dds-domain", type=int, default=0, help="DDS domain id for sim odometry.")
    parser.add_argument("--mujoco-camera", action="store_true", help="Read MuJoCo camera stream over ZMQ.")
    parser.add_argument("--camera-host", default="localhost", help="MuJoCo camera ZMQ host.")
    parser.add_argument("--camera-port", type=int, default=5555, help="MuJoCo camera ZMQ port.")
    parser.add_argument("--real-camera", action="store_true", help="Read real-robot camera stream over ZMQ.")
    parser.add_argument(
        "--real-camera-optional",
        action="store_true",
        help="Allow real mode to run without a camera payload. Default real mode requires camera.",
    )
    parser.add_argument("--camera-timeout", type=float, default=3.0, help="Seconds to wait for real camera payload.")
    parser.add_argument(
        "--enable-real-motion",
        action="store_true",
        help="Allow the real backend to send motion commands. Off by default.",
    )
    parser.add_argument(
        "--auto-start-control",
        action="store_true",
        help="Send WBC start/planner command on bridge reset/start.",
    )
    parser.add_argument(
        "--send-stop-on-halt",
        action="store_true",
        help="Also send deploy stop=True on /halt. Off by default because it can terminate deploy.",
    )
    parser.add_argument("--verbose", action="store_true", help="Print MuJoCo bridge transport details.")
    args = parser.parse_args()
    serve_http(
        host=args.host,
        port=args.port,
        runtime_mode=_runtime_mode(args),
        service_factory=lambda: _create_service(args),
    )


def _create_service(args: argparse.Namespace) -> VigilBridgeService:
    if args.backend == "mujoco":
        return create_mujoco_bridge_service(
            MujocoBridgeConfig(
                runtime_mode=_runtime_mode(args),
                command_bind_host=args.command_bind_host,
                command_port=args.command_port,
                state_host=args.state_host,
                state_port=args.state_port,
                state_topic=args.state_topic,
                rate_hz=args.rate,
                default_move_speed_mps=args.move_speed,
                min_move_speed_mps=args.min_move_speed,
                max_move_speed_mps=args.max_move_speed,
                use_move_model=not args.disable_move_model,
                move_model_file=args.move_model_file,
                model_chunk_pause_s=args.model_chunk_pause,
                move_settle_time_s=args.move_settle_time,
                default_rotate_rate_deg_s=args.rotate_rate,
                rotate_timeout_s=args.rotate_timeout,
                rotate_extra_time_s=args.rotate_extra_time,
                rotate_tolerance_deg=args.rotate_tolerance_deg,
                state_timeout_s=args.state_timeout,
                odom_source=args.odom_source,
                dds_interface=args.dds_interface,
                dds_domain=args.dds_domain,
                camera_enabled=args.mujoco_camera,
                camera_host=args.camera_host,
                camera_port=args.camera_port,
                auto_start_control=args.auto_start_control,
                stop_on_halt=args.send_stop_on_halt,
                verbose=args.verbose,
            )
        )
    if args.backend == "real":
        return create_real_bridge_service(
            RealBridgeConfig(
                runtime_mode=_runtime_mode(args),
                command_bind_host=args.command_bind_host,
                command_port=args.command_port,
                state_host=args.state_host,
                state_port=args.state_port,
                state_topic=args.state_topic,
                rate_hz=args.rate,
                default_move_speed_mps=min(args.move_speed, 0.15),
                min_move_speed_mps=min(args.min_move_speed, 0.05),
                max_move_speed_mps=min(args.max_move_speed, 0.30),
                move_settle_time_s=max(args.move_settle_time, 1.0),
                default_rotate_rate_deg_s=min(args.rotate_rate, 20.0),
                rotate_timeout_s=max(args.rotate_timeout, 6.0),
                rotate_extra_time_s=max(args.rotate_extra_time, 8.0),
                rotate_tolerance_deg=max(args.rotate_tolerance_deg, 5.0),
                state_timeout_s=args.state_timeout,
                camera_enabled=args.real_camera,
                camera_required=not args.real_camera_optional,
                camera_host=args.camera_host,
                camera_port=args.camera_port,
                camera_timeout_s=args.camera_timeout,
                motion_enabled=args.enable_real_motion,
                auto_start_control=args.auto_start_control,
                stop_on_halt=args.send_stop_on_halt,
                verbose=args.verbose,
            )
        )
    return VigilBridgeService(runtime_mode=_runtime_mode(args))


def _runtime_mode(args: argparse.Namespace) -> str:
    if args.runtime_mode is not None:
        return args.runtime_mode
    if args.backend in {"mujoco", "real"}:
        return args.backend
    return "dry_run"


if __name__ == "__main__":
    main()
