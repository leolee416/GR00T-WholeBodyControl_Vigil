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
from gear_sonic.vigil_bridge.chair_motion_catalog import (
    ChairMotionCatalog,
    DEFAULT_CATALOG,
)
from gear_sonic.vigil_bridge.audio import AudioBridgeConfig, AudioSessionManager
from gear_sonic.vigil_bridge.audio_ws import AudioWebSocketServer
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
    parser.add_argument(
        "--rollout-output-dir",
        default="outputs/vigil_rollouts",
        help="Directory for bridge rollout sessions and NPZ exports.",
    )
    parser.add_argument(
        "--rollout-localization-max-age",
        type=float,
        default=0.5,
        help="Maximum age in seconds for externally supplied base_xyz localization.",
    )
    parser.add_argument("--rate", type=float, default=20.0, help="Planner command publish rate.")
    parser.add_argument("--move-speed", type=float, default=0.25, help="Default move speed in m/s.")
    parser.add_argument("--min-move-speed", type=float, default=0.20, help="Minimum move speed in m/s.")
    parser.add_argument(
        "--max-speed-mps",
        "--max-move-speed",
        dest="max_move_speed",
        type=float,
        default=2.00,
        help="Maximum move speed threshold used when action omits safety.max_speed_mps, in m/s.",
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
    parser.add_argument("--pause-settle-time", type=float, default=3.2, help="Seconds to wait after pause command.")
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
    parser.add_argument(
        "--chair-motion-catalog",
        default=str(DEFAULT_CATALOG),
        help="Manifest for exact-distance sonic.sit_chair references.",
    )
    parser.add_argument("--audio-enabled", action="store_true", help="Enable bridge audio I/O endpoints.")
    parser.add_argument(
        "--audio-advertise-always",
        action="store_true",
        help="Advertise audio capabilities to all clients. Default only advertises when requested in handshake.",
    )
    parser.add_argument("--audio-mic-group", default="239.168.123.161", help="G1 microphone multicast group.")
    parser.add_argument("--audio-mic-port", type=int, default=5555, help="G1 microphone multicast UDP port.")
    parser.add_argument(
        "--audio-mic-interface-ip",
        default=None,
        help="Local 192.168.123.x IP to join microphone multicast. Defaults to INADDR_ANY.",
    )
    parser.add_argument(
        "--audio-segment-max-s",
        type=float,
        default=10.0,
        help="Maximum HTTP audio segment duration in seconds.",
    )
    parser.add_argument("--audio-speaker-volume", type=int, default=100, help="G1 speaker API volume.")
    parser.add_argument(
        "--audio-speaker-peak-target",
        type=int,
        default=27800,
        help="PCM16 peak target used before speaker PlayStream at volume 100.",
    )
    parser.add_argument(
        "--audio-speaker-runner",
        default=None,
        help="Optional external G1 PlayStream runner path for speaker output.",
    )
    parser.add_argument(
        "--audio-speaker-iface",
        default=None,
        help="Optional Unitree DDS interface for the external speaker runner.",
    )
    parser.add_argument(
        "--audio-fake-speaker",
        action="store_true",
        help="Use a fake speaker client for bridge tests instead of hardware playback.",
    )
    parser.add_argument("--audio-ws", action="store_true", help="Start optional WebSocket audio stream server.")
    parser.add_argument("--audio-ws-host", default=None, help="Audio WebSocket bind host. Defaults to --host.")
    parser.add_argument("--audio-ws-port", type=int, default=8766, help="Audio WebSocket bind port.")
    parser.add_argument("--verbose", action="store_true", help="Print MuJoCo bridge transport details.")
    args = parser.parse_args()
    service = _create_service(args)
    audio_ws_server = _start_audio_ws_if_requested(args, service)
    try:
        serve_http(
            host=args.host,
            port=args.port,
            runtime_mode=_runtime_mode(args),
            service_factory=lambda: service,
        )
    finally:
        if audio_ws_server is not None:
            audio_ws_server.stop()


def _create_service(args: argparse.Namespace) -> VigilBridgeService:
    if args.backend == "mujoco":
        service = create_mujoco_bridge_service(
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
                chair_motion_catalog=args.chair_motion_catalog,
                rollout_output_dir=args.rollout_output_dir,
                rollout_localization_max_age_s=args.rollout_localization_max_age,
                verbose=args.verbose,
            )
        )
        return _attach_audio(args, service)
    if args.backend == "real":
        service = create_real_bridge_service(
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
                max_move_speed_mps=min(args.max_move_speed, 2.00),
                use_move_model=not args.disable_move_model,
                move_model_file=args.move_model_file,
                model_chunk_pause_s=args.model_chunk_pause,
                move_settle_time_s=max(args.move_settle_time, 1.0),
                pause_settle_time_s=max(args.pause_settle_time, 0.0),
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
                chair_motion_catalog=args.chair_motion_catalog,
                rollout_output_dir=args.rollout_output_dir,
                rollout_localization_max_age_s=args.rollout_localization_max_age,
                verbose=args.verbose,
            )
        )
        return _attach_audio(args, service)
    service = VigilBridgeService(runtime_mode=_runtime_mode(args))
    assert service.executor is not None
    service.executor.chair_motion_catalog = ChairMotionCatalog(
        args.chair_motion_catalog
    )
    return _attach_audio(args, service)


def _runtime_mode(args: argparse.Namespace) -> str:
    if args.runtime_mode is not None:
        return args.runtime_mode
    if args.backend in {"mujoco", "real"}:
        return args.backend
    return "dry_run"


def _attach_audio(args: argparse.Namespace, service: VigilBridgeService) -> VigilBridgeService:
    if args.audio_enabled:
        service.audio_manager = AudioSessionManager(
            AudioBridgeConfig(
                enabled=True,
                runtime_mode=_runtime_mode(args),
                mic_group=args.audio_mic_group,
                mic_port=args.audio_mic_port,
                mic_interface_ip=args.audio_mic_interface_ip,
                segment_max_s=args.audio_segment_max_s,
                speaker_volume=args.audio_speaker_volume,
                speaker_peak_target=args.audio_speaker_peak_target,
                speaker_runner=args.audio_speaker_runner,
                speaker_iface=args.audio_speaker_iface,
                fake_speaker=args.audio_fake_speaker or (
                    args.backend == "dry_run" and not args.audio_speaker_runner
                ),
            )
        )
    service.audio_advertise_always = bool(args.audio_advertise_always)
    return service


def _start_audio_ws_if_requested(
    args: argparse.Namespace,
    service: VigilBridgeService,
) -> AudioWebSocketServer | None:
    if not args.audio_ws:
        return None
    if service.audio_manager is None:
        print("Audio WebSocket requested but --audio-enabled is not set; skipping.", file=sys.stderr)
        return None
    server = AudioWebSocketServer(
        manager=service.audio_manager,
        host=args.audio_ws_host or args.host,
        port=args.audio_ws_port,
    )
    health = server.start()
    if not health.get("ok", False):
        print(f"Audio WebSocket unavailable: {health.get('error_message')}", file=sys.stderr)
    return server


if __name__ == "__main__":
    main()
