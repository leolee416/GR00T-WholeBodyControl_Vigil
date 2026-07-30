"""Robot-side launcher for GR00T deploy + Vigil bridge.

The launcher intentionally stays in the bridge layer.  It starts the existing
Docker development container, runs the existing deploy script inside it, then
starts the GR00T-side Vigil HTTP bridge on the robot host.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import socket
import subprocess
import sys
import textwrap
import time
from urllib import request


REPO_ROOT = Path(__file__).resolve().parents[2]
DEPLOY_DIR = REPO_ROOT / "gear_sonic_deploy"
DEFAULT_SESSION = "vigil_bridge"
DEFAULT_CONTAINER = "g1-deploy-dev"
DEFAULT_TENSORRT_ROOT = "/home/unitree/TensorRT-10.7.0.23"
DEFAULT_CAMERA_SERVICE = "composed_camera_server_vigil.service"
DEFAULT_LEGACY_CAMERA_SERVICE = "composed_camera_server.service"
RUNTIME_ROOT = Path("/tmp/vigil_bridge_launcher")


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.command == "start":
        return start(args)
    if args.command == "pause":
        return pause(args)
    if args.command == "resume":
        return resume(args)
    if args.command == "stop":
        return stop(args)
    if args.command == "status":
        return status(args)
    if args.command == "attach":
        return attach(args)
    if args.command == "logs":
        return logs(args)
    parser.print_help()
    return 1


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="vigil_bridge",
        description="Start/stop the robot-side GR00T deploy process and Vigil bridge.",
    )
    parser.add_argument("--session", default=DEFAULT_SESSION, help="tmux session name.")
    parser.add_argument("--container-name", default=DEFAULT_CONTAINER, help="Docker container name.")
    subparsers = parser.add_subparsers(dest="command")

    start_parser = subparsers.add_parser("start", help="Start Docker, deploy, and bridge in tmux windows.")
    start_parser.add_argument("--tensorrt-root", default=os.environ.get("TensorRT_ROOT", DEFAULT_TENSORRT_ROOT))
    start_parser.add_argument("--docker-rebuild", action="store_true", help="Pass --rebuild to docker/run-ros2-dev.sh.")
    start_parser.add_argument("--robot-interface", default="real", help="deploy.sh interface argument.")
    start_parser.add_argument("--input-type", default="zmq_manager", help="deploy input type.")
    start_parser.add_argument("--output-type", default="zmq", help="deploy output type.")
    start_parser.add_argument("--zmq-host", default="127.0.0.1", help="deploy ZMQ command host.")
    start_parser.add_argument("--bridge-host", default="0.0.0.0", help="HTTP bridge bind host.")
    start_parser.add_argument("--bridge-port", type=int, default=8765, help="HTTP bridge bind port.")
    start_parser.add_argument("--command-bind-host", default="127.0.0.1", help="Bridge ZMQ command PUB bind host.")
    start_parser.add_argument("--command-port", type=int, default=5556, help="Bridge ZMQ command PUB port.")
    start_parser.add_argument("--state-host", default="127.0.0.1", help="Bridge ZMQ state host.")
    start_parser.add_argument("--state-port", type=int, default=5557, help="Bridge ZMQ state port.")
    start_parser.add_argument("--state-topic", default="g1_debug", help="Bridge ZMQ state topic.")
    start_parser.add_argument("--state-timeout", type=float, default=10.0, help="Seconds bridge waits for g1_debug state.")
    start_parser.add_argument(
        "--rollout-output-dir",
        default="outputs/vigil_rollouts",
        help="Directory for measured rollout sessions and NPZ exports.",
    )
    start_parser.add_argument(
        "--rollout-localization-max-age",
        type=float,
        default=0.5,
        help="Maximum external base localization age in seconds.",
    )
    start_parser.add_argument("--max-speed-mps", type=float, default=2.0, help="Maximum real-robot move speed threshold in m/s.")
    start_parser.add_argument("--move-model-file", default="auto", help="Move model JSON path, or 'auto' for latest output.")
    start_parser.add_argument("--disable-move-model", action="store_true", help="Use direct open-loop move instead of move_model.")
    start_parser.add_argument("--model-chunk-pause", type=float, default=0.0, help="Pause between move_model chunks.")
    start_parser.add_argument("--pause-settle-time", type=float, default=3.2, help="Seconds pause waits for default-pose settle.")
    start_parser.add_argument("--camera-host", default="localhost", help="Real camera ZMQ host.")
    start_parser.add_argument("--camera-port", type=int, default=5555, help="Real camera ZMQ port.")
    start_parser.add_argument(
        "--chair-motion-catalog",
        default=str(
            REPO_ROOT
            / "gear_sonic/vigil_bridge/data/facee_chair_13s/manifest.json"
        ),
        help="Exact-distance FaceE chair reference manifest.",
    )
    start_parser.add_argument(
        "--camera-service",
        default=DEFAULT_CAMERA_SERVICE,
        help="Systemd camera service to start for this Vigil launcher session.",
    )
    start_parser.add_argument(
        "--legacy-camera-service",
        default=DEFAULT_LEGACY_CAMERA_SERVICE,
        help="Legacy camera service whose active state is restored by stop.",
    )
    start_parser.add_argument(
        "--no-camera-service",
        action="store_true",
        help="Do not manage a systemd camera service before starting the bridge.",
    )
    start_parser.add_argument(
        "--camera-service-timeout",
        type=float,
        default=15.0,
        help="Seconds to wait for the managed camera service port.",
    )
    start_parser.add_argument("--no-real-motion", action="store_true", help="Do not pass --enable-real-motion.")
    start_parser.add_argument("--no-auto-start-control", action="store_true", help="Do not pass --auto-start-control.")
    start_parser.add_argument(
        "--no-reset-after-start",
        action="store_true",
        help="Do not call /reset_episode after the bridge HTTP port is up.",
    )
    start_parser.add_argument("--camera-required", action="store_true", help="Require real camera payload.")
    audio_group = start_parser.add_argument_group("audio/TTS")
    audio_group.add_argument(
        "--audio-enabled",
        "--with-audio",
        "--with-tts",
        dest="audio_enabled",
        action="store_true",
        help="Enable bridge audio I/O and native TTS endpoints.",
    )
    audio_group.add_argument(
        "--audio-advertise-always",
        action="store_true",
        help="Advertise audio capabilities to all clients, not only clients that request audio.",
    )
    audio_group.add_argument("--audio-mic-group", default="239.168.123.161", help="G1 microphone multicast group.")
    audio_group.add_argument("--audio-mic-port", type=int, default=5555, help="G1 microphone multicast UDP port.")
    audio_group.add_argument(
        "--audio-mic-interface-ip",
        default=None,
        help="Local 192.168.123.x IP used to join microphone multicast. Defaults to INADDR_ANY.",
    )
    audio_group.add_argument(
        "--audio-segment-max-s",
        type=float,
        default=10.0,
        help="Maximum HTTP audio segment duration in seconds.",
    )
    audio_group.add_argument("--audio-speaker-volume", type=int, default=100, help="G1 speaker API volume.")
    audio_group.add_argument(
        "--audio-speaker-peak-target",
        type=int,
        default=27800,
        help="PCM16 peak target used before G1 PlayStream at volume 100.",
    )
    audio_group.add_argument(
        "--audio-speaker-runner",
        default=None,
        help="External G1 speaker/TTS runner executable used for real speaker output.",
    )
    audio_group.add_argument(
        "--audio-speaker-iface",
        default=None,
        help="Optional Unitree DDS interface for the external speaker runner.",
    )
    audio_group.add_argument(
        "--audio-fake-speaker",
        action="store_true",
        help="Use a fake speaker client for dry bridge tests instead of hardware playback.",
    )
    audio_group.add_argument("--audio-ws", action="store_true", help="Start optional full-duplex audio WebSocket server.")
    audio_group.add_argument("--audio-ws-host", default=None, help="Audio WebSocket bind host. Defaults to --bridge-host.")
    audio_group.add_argument("--audio-ws-port", type=int, default=8766, help="Audio WebSocket bind port.")
    start_parser.add_argument("--attach", action="store_true", help="Attach to the tmux session after starting.")

    stop_parser = subparsers.add_parser("stop", help="Halt bridge/deploy and stop the tmux session.")
    stop_parser.add_argument("--bridge-host", default="127.0.0.1", help="Local bridge HTTP host for halt.")
    stop_parser.add_argument("--bridge-port", type=int, default=8765, help="Local bridge HTTP port for halt.")
    stop_parser.add_argument(
        "--no-camera-service",
        action="store_true",
        help="Do not stop the Vigil camera service or restore the previous camera service.",
    )

    pause_parser = subparsers.add_parser(
        "pause",
        help="Pause deploy in default pose while keeping policy, bridge, container, and tmux alive.",
    )
    pause_parser.add_argument("--bridge-host", default="127.0.0.1", help="Local bridge HTTP host for pause.")
    pause_parser.add_argument("--bridge-port", type=int, default=8765, help="Local bridge HTTP port for pause.")

    resume_parser = subparsers.add_parser(
        "resume",
        help="Resume a paused bridge/deploy session without redeploying policy.",
    )
    resume_parser.add_argument("--bridge-host", default="127.0.0.1", help="Local bridge HTTP host for resume.")
    resume_parser.add_argument("--bridge-port", type=int, default=8765, help="Local bridge HTTP port for resume.")

    status_parser = subparsers.add_parser("status", help="Show tmux, Docker, and bridge status.")
    status_parser.add_argument("--bridge-host", default="127.0.0.1", help="Local bridge HTTP host for health.")
    status_parser.add_argument("--bridge-port", type=int, default=8765, help="Local bridge HTTP port for health.")
    status_parser.add_argument(
        "--camera-service",
        default=DEFAULT_CAMERA_SERVICE,
        help="Vigil camera service name to report.",
    )
    status_parser.add_argument(
        "--legacy-camera-service",
        default=DEFAULT_LEGACY_CAMERA_SERVICE,
        help="Legacy camera service name to report.",
    )
    subparsers.add_parser("attach", help="Attach to the tmux session.")
    subparsers.add_parser("logs", help="Print launcher log paths.")
    return parser


def start(args: argparse.Namespace) -> int:
    _require_command("tmux")
    _require_command("docker")

    if _tmux_session_exists(args.session):
        print(f"tmux session already exists: {args.session}; requesting bridge resume instead of redeploying.")
        return resume(args)

    _validate_start_inputs(args)
    _stop_stale_bridge_service(args.bridge_port)

    runtime_dir = _runtime_dir(args.session)
    runtime_dir.mkdir(parents=True, exist_ok=True)
    if not args.no_camera_service:
        _activate_camera_service(args, runtime_dir)

    scripts_dir = runtime_dir / "scripts"
    scripts_dir.mkdir(parents=True, exist_ok=True)

    container_script = scripts_dir / "container.sh"
    policy_script = scripts_dir / "policy.sh"
    bridge_script = scripts_dir / "bridge.sh"
    container_log = runtime_dir / "container.log"
    policy_log = runtime_dir / "policy.log"
    bridge_log = runtime_dir / "bridge.log"

    _write_script(container_script, _container_script(args, container_log))
    _write_script(policy_script, _policy_script(args, policy_log))
    _write_script(bridge_script, _bridge_script(args, policy_log, bridge_log))

    _run(["tmux", "new-session", "-d", "-s", args.session, "-n", "container", str(container_script)])
    _run(["tmux", "set-option", "-t", args.session, "remain-on-exit", "on"])
    _run(["tmux", "new-window", "-t", args.session, "-n", "policy", str(policy_script)])
    _run(["tmux", "new-window", "-t", args.session, "-n", "bridge", str(bridge_script)])

    print(f"Started tmux session: {args.session}")
    print(f"Windows: container, policy, bridge")
    print(f"Logs: {runtime_dir}")
    print(f"Attach: ./vigil_bridge attach")
    if args.attach:
        return attach(args)
    return 0


def stop(args: argparse.Namespace) -> int:
    # Halt first. This is intentionally best-effort because the bridge may not
    # be running, or the HTTP runtime may already be down.
    bridge_base_url = f"http://{args.bridge_host}:{args.bridge_port}"
    _post_json(f"{bridge_base_url}/halt", b'{"runtime_mode":"real"}', timeout=2.0)
    _post_json(f"{bridge_base_url}/close", b'{"runtime_mode":"real"}', timeout=2.0)
    _terminate_bridge_processes()

    if _docker_container_exists(args.container_name):
        _run(
            [
                "docker",
                "exec",
                args.container_name,
                "bash",
                "-lc",
                "pkill -INT -f g1_deploy_onnx_ref || true",
            ],
            check=False,
        )

    if _tmux_session_exists(args.session):
        _run(["tmux", "kill-session", "-t", args.session], check=False)
    _terminate_bridge_processes()

    if _docker_container_exists(args.container_name):
        _run(["docker", "rm", "-f", args.container_name], check=False)

    if not args.no_camera_service:
        _restore_camera_service(args.session)

    print("Stopped robot-side Vigil bridge launcher.")
    return 0


def pause(args: argparse.Namespace) -> int:
    bridge_base_url = _bridge_base_url(args)
    response = _post_json(f"{bridge_base_url}/pause", b'{"runtime_mode":"real"}', timeout=10.0)
    if response is None:
        print(f"Could not reach bridge pause endpoint at {bridge_base_url}/pause", file=sys.stderr)
        return 1
    print(f"Paused robot-side policy session: {response}")
    print("tmux/container/policy/bridge remain running; use './vigil_bridge start' or './vigil_bridge resume' to reconnect.")
    return 0


def resume(args: argparse.Namespace) -> int:
    bridge_base_url = _bridge_base_url(args)
    response = _post_json(f"{bridge_base_url}/resume", b'{"runtime_mode":"real"}', timeout=30.0)
    if response is None:
        print(f"Could not reach bridge resume endpoint at {bridge_base_url}/resume", file=sys.stderr)
        if _tmux_session_exists(args.session):
            print(f"tmux session exists: {args.session}. Check './vigil_bridge status' or './vigil_bridge attach'.", file=sys.stderr)
        return 1
    print(f"Resumed robot-side policy session: {response}")
    return 0


def status(args: argparse.Namespace) -> int:
    print(f"tmux session {args.session}: {'running' if _tmux_session_exists(args.session) else 'not running'}")
    if _docker_container_exists(args.container_name):
        result = _run(
            ["docker", "inspect", "-f", "{{.State.Status}}", args.container_name],
            check=False,
            capture_output=True,
        )
        print(f"container {args.container_name}: {result.stdout.strip() if result.stdout else 'unknown'}")
    else:
        print(f"container {args.container_name}: not found")
    health = _get_text(f"http://{args.bridge_host}:{args.bridge_port}/health", timeout=1.0)
    print(f"bridge health: {health if health else 'unavailable'}")
    print(
        f"camera service {args.camera_service}: "
        f"{'active' if _systemctl_is_active(args.camera_service) else 'inactive'}"
    )
    print(
        f"legacy camera service {args.legacy_camera_service}: "
        f"{'active' if _systemctl_is_active(args.legacy_camera_service) else 'inactive'}"
    )
    return 0


def attach(args: argparse.Namespace) -> int:
    if not _tmux_session_exists(args.session):
        print(f"tmux session not found: {args.session}", file=sys.stderr)
        return 1
    os.execvp("tmux", ["tmux", "attach", "-t", args.session])
    return 0


def logs(args: argparse.Namespace) -> int:
    runtime_dir = _runtime_dir(args.session)
    print(runtime_dir)
    for name in ("container.log", "policy.log", "bridge.log"):
        print(runtime_dir / name)
    return 0


def _container_script(args: argparse.Namespace, log_path: Path) -> str:
    docker_args = " --rebuild" if args.docker_rebuild else ""
    return f"""\
        #!/usr/bin/env bash
        set -euo pipefail
        cd {shlex.quote(str(DEPLOY_DIR))}
        export TensorRT_ROOT={shlex.quote(args.tensorrt_root)}
        echo "[launcher] starting Docker ROS2 dev container"
        echo "[launcher] TensorRT_ROOT=$TensorRT_ROOT"
        ./docker/run-ros2-dev.sh{docker_args} 2>&1 | tee {shlex.quote(str(log_path))}
    """


def _policy_script(args: argparse.Namespace, log_path: Path) -> str:
    deploy_cmd = " ".join(
        [
            "./deploy.sh",
            shlex.quote(args.robot_interface),
            "--input-type",
            shlex.quote(args.input_type),
            "--output-type",
            shlex.quote(args.output_type),
            "--zmq-host",
            shlex.quote(args.zmq_host),
        ]
    )
    container_command = f"cd /workspace/g1_deploy && source scripts/setup_env.sh && printf '\\n' | {deploy_cmd}"
    return f"""\
        #!/usr/bin/env bash
        set -euo pipefail
        echo "[launcher] waiting for Docker container {args.container_name}"
        until docker inspect -f '{{{{.State.Running}}}}' {shlex.quote(args.container_name)} 2>/dev/null | grep -q true; do
          sleep 1
        done
        echo "[launcher] container is running; starting deploy"
        docker exec -i {shlex.quote(args.container_name)} bash -lc {shlex.quote(container_command)} 2>&1 | tee {shlex.quote(str(log_path))}
    """


def _bridge_script(args: argparse.Namespace, policy_log: Path, bridge_log: Path) -> str:
    bridge_args = [
        "python3",
        "gear_sonic_deploy/scripts/run_vigil_bridge.py",
        "--backend",
        "real",
        "--runtime-mode",
        "real",
        "--host",
        args.bridge_host,
        "--port",
        str(args.bridge_port),
        "--command-bind-host",
        args.command_bind_host,
        "--command-port",
        str(args.command_port),
        "--state-host",
        args.state_host,
        "--state-port",
        str(args.state_port),
        "--state-topic",
        args.state_topic,
        "--state-timeout",
        str(args.state_timeout),
        "--rollout-output-dir",
        args.rollout_output_dir,
        "--rollout-localization-max-age",
        str(args.rollout_localization_max_age),
        "--max-speed-mps",
        str(args.max_speed_mps),
        "--move-model-file",
        args.move_model_file,
        "--model-chunk-pause",
        str(args.model_chunk_pause),
        "--pause-settle-time",
        str(args.pause_settle_time),
        "--real-camera",
        "--camera-host",
        args.camera_host,
        "--camera-port",
        str(args.camera_port),
        "--chair-motion-catalog",
        args.chair_motion_catalog,
        "--verbose",
    ]
    if args.disable_move_model:
        bridge_args.append("--disable-move-model")
    if not args.no_real_motion:
        bridge_args.append("--enable-real-motion")
    if not args.no_auto_start_control:
        bridge_args.append("--auto-start-control")
    if not args.camera_required:
        bridge_args.append("--real-camera-optional")
    bridge_args.extend(_audio_bridge_args(args))

    quoted_bridge_cmd = " ".join(shlex.quote(part) for part in bridge_args)
    reset_block = ""
    if not args.no_reset_after_start:
        reset_block = f"""
        echo "[launcher] waiting for bridge HTTP port {args.bridge_port}"
        for _ in $(seq 1 60); do
          if python3 - <<'PY' >/dev/null 2>&1
import urllib.request
urllib.request.urlopen('http://127.0.0.1:{args.bridge_port}/health', timeout=1)
PY
          then
            break
          fi
          sleep 1
        done
        echo "[launcher] requesting reset_episode to initialize runtime"
        python3 - <<'PY' || true
import json
import time
import urllib.request
url = "http://127.0.0.1:{args.bridge_port}/reset_episode"
data = json.dumps({{"runtime_mode": "real"}}).encode("utf-8")
last_text = ""
for attempt in range(1, 13):
    req = urllib.request.Request(
        url,
        data=data,
        headers={{"Content-Type": "application/json"}},
        method="POST",
    )
    try:
        text = urllib.request.urlopen(req, timeout=20).read().decode("utf-8")
    except Exception as exc:
        text = json.dumps({{"ok": False, "error_message": str(exc)}})
    last_text = text
    print(f"[launcher] reset_episode attempt {{attempt}}: {{text}}", flush=True)
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        payload = {{}}
    if payload.get("ok") is True:
        break
    time.sleep(2)
else:
    print(f"[launcher] reset_episode did not become ready; latest response: {{last_text}}", flush=True)
PY
        """

    return f"""\
        #!/usr/bin/env bash
        set -euo pipefail
        echo "[launcher] waiting for deploy Init Done in {policy_log}"
        until grep -q "Init Done" {shlex.quote(str(policy_log))} 2>/dev/null; do
          sleep 1
        done
        echo "[launcher] deploy initialized; starting bridge"
        cd {shlex.quote(str(REPO_ROOT))}
        {quoted_bridge_cmd} 2>&1 | tee {shlex.quote(str(bridge_log))} &
        bridge_pid=$!
        trap 'kill "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true' INT TERM HUP EXIT
        {textwrap.indent(textwrap.dedent(reset_block), "        ")}
        wait "$bridge_pid"
    """


def _validate_start_inputs(args: argparse.Namespace) -> None:
    if not DEPLOY_DIR.exists():
        raise SystemExit(f"deploy directory not found: {DEPLOY_DIR}")
    trt = Path(args.tensorrt_root).expanduser()
    if not (trt / "include" / "NvInfer.h").exists():
        raise SystemExit(f"TensorRT include not found: {trt / 'include' / 'NvInfer.h'}")
    if not (trt / "lib").exists():
        raise SystemExit(f"TensorRT lib directory not found: {trt / 'lib'}")
    if args.max_speed_mps <= 0.0:
        raise SystemExit("--max-speed-mps must be positive")
    if args.max_speed_mps > 2.0:
        raise SystemExit("--max-speed-mps must be <= 2.0 for real-robot mode")
    if args.model_chunk_pause < 0.0:
        raise SystemExit("--model-chunk-pause must be >= 0")
    if args.pause_settle_time < 0.0:
        raise SystemExit("--pause-settle-time must be >= 0")
    if args.camera_port <= 0:
        raise SystemExit("--camera-port must be positive")
    if not Path(args.chair_motion_catalog).expanduser().is_file():
        raise SystemExit(
            f"chair motion catalog not found: {args.chair_motion_catalog}"
        )
    if not args.audio_enabled and (
        args.audio_advertise_always
        or args.audio_mic_interface_ip
        or args.audio_speaker_runner
        or args.audio_speaker_iface
        or args.audio_fake_speaker
        or args.audio_ws
    ):
        raise SystemExit("audio options require --audio-enabled, --with-audio, or --with-tts")
    if args.audio_mic_port <= 0:
        raise SystemExit("--audio-mic-port must be positive")
    if args.audio_segment_max_s <= 0.0:
        raise SystemExit("--audio-segment-max-s must be positive")
    if not 1 <= args.audio_speaker_volume <= 100:
        raise SystemExit("--audio-speaker-volume must be in 1..100")
    if not 1 <= args.audio_speaker_peak_target <= 32767:
        raise SystemExit("--audio-speaker-peak-target must be in 1..32767")
    if args.audio_ws_port <= 0:
        raise SystemExit("--audio-ws-port must be positive")
    if args.audio_ws_host and not args.audio_ws:
        raise SystemExit("--audio-ws-host requires --audio-ws")


def _audio_bridge_args(args: argparse.Namespace) -> list[str]:
    if not args.audio_enabled:
        return []

    bridge_args = [
        "--audio-enabled",
        "--audio-mic-group",
        args.audio_mic_group,
        "--audio-mic-port",
        str(args.audio_mic_port),
        "--audio-segment-max-s",
        str(args.audio_segment_max_s),
        "--audio-speaker-volume",
        str(args.audio_speaker_volume),
        "--audio-speaker-peak-target",
        str(args.audio_speaker_peak_target),
    ]
    if args.audio_advertise_always:
        bridge_args.append("--audio-advertise-always")
    if args.audio_mic_interface_ip:
        bridge_args.extend(["--audio-mic-interface-ip", args.audio_mic_interface_ip])
    if args.audio_speaker_runner:
        bridge_args.extend(["--audio-speaker-runner", args.audio_speaker_runner])
    if args.audio_speaker_iface:
        bridge_args.extend(["--audio-speaker-iface", args.audio_speaker_iface])
    if args.audio_fake_speaker:
        bridge_args.append("--audio-fake-speaker")
    if args.audio_ws:
        bridge_args.extend(["--audio-ws", "--audio-ws-port", str(args.audio_ws_port)])
        if args.audio_ws_host:
            bridge_args.extend(["--audio-ws-host", args.audio_ws_host])
    return bridge_args


def _write_script(path: Path, content: str) -> None:
    path.write_text(textwrap.dedent(content), encoding="utf-8")
    path.chmod(0o755)


def _runtime_dir(session: str) -> Path:
    return RUNTIME_ROOT / session


def _camera_state_path(session: str) -> Path:
    return _runtime_dir(session) / "camera_service_state.json"


def _bridge_base_url(args: argparse.Namespace) -> str:
    host = str(getattr(args, "bridge_host", "127.0.0.1"))
    if host in {"0.0.0.0", "::", ""}:
        host = "127.0.0.1"
    return f"http://{host}:{args.bridge_port}"


def _require_command(command: str) -> None:
    if shutil.which(command) is None:
        if command == "tmux":
            raise SystemExit("Required command not found: tmux. Install it on the robot host with: sudo apt-get install -y tmux")
        raise SystemExit(f"Required command not found: {command}")


def _tmux_session_exists(session: str) -> bool:
    return _run(["tmux", "has-session", "-t", session], check=False).returncode == 0


def _docker_container_exists(container: str) -> bool:
    return _run(["docker", "inspect", container], check=False, capture_output=True).returncode == 0


def _activate_camera_service(args: argparse.Namespace, runtime_dir: Path) -> None:
    state = {
        "camera_service": args.camera_service,
        "legacy_camera_service": args.legacy_camera_service,
        "legacy_active_before": _systemctl_is_active(args.legacy_camera_service),
        "vigil_active_before": _systemctl_is_active(args.camera_service),
    }
    (runtime_dir / "camera_service_state.json").write_text(
        json.dumps(state, indent=2, sort_keys=True),
        encoding="utf-8",
    )

    print(f"[launcher] starting camera service {args.camera_service}")
    result = _run(["systemctl", "start", args.camera_service], check=False, capture_output=True)
    if result.returncode != 0:
        details = "\n".join(part for part in (result.stdout, result.stderr) if part)
        raise SystemExit(
            textwrap.dedent(
                f"""\
                Failed to start {args.camera_service}.
                This launcher does not modify or disable {args.legacy_camera_service}.
                Install the Vigil service once, then rerun:
                    sudo cp systemd/composed_camera_server_vigil.service /etc/systemd/system/
                    sudo systemctl daemon-reload

                systemctl output:
                {details}
                """
            ).strip()
        )

    if not _wait_for_tcp_port(args.camera_host, args.camera_port, args.camera_service_timeout):
        raise SystemExit(
            f"Camera service {args.camera_service} started, but "
            f"{args.camera_host}:{args.camera_port} did not become reachable."
        )


def _restore_camera_service(session: str) -> None:
    state_path = _camera_state_path(session)
    if not state_path.exists():
        return

    try:
        state = json.loads(state_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return

    camera_service = str(state.get("camera_service", DEFAULT_CAMERA_SERVICE))
    legacy_service = str(state.get("legacy_camera_service", DEFAULT_LEGACY_CAMERA_SERVICE))
    legacy_active_before = bool(state.get("legacy_active_before", False))

    print(f"[launcher] stopping camera service {camera_service}")
    _run(["systemctl", "stop", camera_service], check=False)
    if legacy_active_before:
        print(f"[launcher] restoring legacy camera service {legacy_service}")
        _run(["systemctl", "start", legacy_service], check=False)
    try:
        state_path.unlink()
    except OSError:
        pass


def _systemctl_is_active(service: str) -> bool:
    return _run(["systemctl", "is-active", "--quiet", service], check=False).returncode == 0


def _wait_for_tcp_port(host: str, port: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def _stop_stale_bridge_service(port: int) -> None:
    url = f"http://127.0.0.1:{port}/health"
    health = _get_text(url, timeout=0.5)
    if not health:
        return

    try:
        payload = json.loads(health)
    except json.JSONDecodeError as exc:
        raise SystemExit(f"HTTP port {port} is already in use by a non-bridge service.") from exc

    if payload.get("protocol_version") != "vigil_groot_bridge_v1":
        raise SystemExit(f"HTTP port {port} is already in use by an unknown service.")

    print(f"[launcher] stale bridge service detected on 127.0.0.1:{port}; stopping it first")
    bridge_base_url = f"http://127.0.0.1:{port}"
    _post_json(f"{bridge_base_url}/halt", b'{"runtime_mode":"real"}', timeout=2.0)
    _post_json(f"{bridge_base_url}/close", b'{"runtime_mode":"real"}', timeout=2.0)
    _terminate_bridge_processes()
    if _wait_for_http_down(url, timeout_s=5.0):
        return
    raise SystemExit(f"stale bridge service on 127.0.0.1:{port} did not exit")


def _terminate_bridge_processes() -> None:
    patterns = [
        "gear_sonic_deploy/scripts/run_vigil_bridge.py",
        "run_vigil_bridge.py",
    ]
    for pattern in patterns:
        _run(["pkill", "-TERM", "-f", pattern], check=False)


def _wait_for_http_down(url: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _get_text(url, timeout=0.5) is None:
            return True
        time.sleep(0.2)
    return _get_text(url, timeout=0.5) is None


def _post_json(url: str, data: bytes, timeout: float) -> str | None:
    try:
        req = request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
        with request.urlopen(req, timeout=timeout) as response:
            return response.read().decode("utf-8")
    except Exception:
        return None


def _get_text(url: str, timeout: float) -> str | None:
    try:
        with request.urlopen(url, timeout=timeout) as response:
            return response.read().decode("utf-8")
    except Exception:
        return None


def _run(
    cmd: list[str],
    *,
    check: bool = True,
    capture_output: bool = False,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, check=check, text=True, capture_output=capture_output)


if __name__ == "__main__":
    raise SystemExit(main())
