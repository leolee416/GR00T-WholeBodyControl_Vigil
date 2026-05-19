# Vigil Bridge Integration

This document describes the GR00T-side boundary for integrating
GR00T-WholeBodyControl with Vigil.

## Role

GR00T-WholeBodyControl remains the robot runtime side. This repository provides
whole-body control, deployment, camera, simulator, and robot-state runtime
capabilities.

Vigil integration is a later adapter layer. Its job is to expose selected GR00T
runtime capabilities to Vigil through a stable bridge API. It should not move
Vigil benchmark logic into this repository.

## Ownership Split

Vigil owns:

- benchmark episode specs
- agent prompts and action schema exposed to the model
- trajectory trace format
- judge and scoring
- oracle semantics and score validity
- result aggregation

GR00T owns:

- real robot and MuJoCo execution runtime
- WBC primitive execution
- command/state transport
- camera observation transport
- robot telemetry
- runtime health and structured errors

## Non-Goals

The bridge must not implement:

- benchmark scoring
- `StepJudgeResult`
- hidden task success
- THOR-style object metadata
- agent prompt construction
- Vigil runner logic

If the bridge has perception output, it should label it as estimated perception,
not groundtruth.

## Proposed Code Placement

Add new bridge code under:

```text
gear_sonic/vigil_bridge/
  __init__.py
  protocol.py
  service.py
  primitive_executor.py
  sensors.py
  transport.py
  mujoco_adapter.py
```

Optionally add a small launcher:

```text
gear_sonic_deploy/scripts/run_vigil_bridge.py
```

Do not modify low-level deploy internals or model inference paths for bridge
work unless explicitly requested.

## Current Dry-Run Backend

The dry-run backend is implemented under `gear_sonic/vigil_bridge/` with no
dependency on Vigil and no calls into real WBC, policy inference, MuJoCo,
hardware, or deploy commands.

- `protocol.py` defines JSON/msgpack-compatible `TypedDict` request/response
  shapes.
- `primitive_executor.py` provides `DryRunPrimitiveExecutor` and
  `FakePrimitiveExecutor` for structured telemetry only.
- `sensors.py` provides `FakeSensorProvider` with fake camera and robot-state
  payloads.
- `service.py` composes the executor and sensor provider behind the minimal
  bridge API.
- `tests/vigil_bridge/` covers the dry-run navigation mapping and verifies the
  bridge package does not import Vigil.

The dry-run backend is useful for protocol tests and fake client/service checks.
The `runtime_mode` field is accepted and echoed for API compatibility, but
execution remains dry-run telemetry unless the HTTP bridge is started with a
runtime backend such as `--backend mujoco`.

The public façade handshake currently follows `vigil_groot_bridge_v1`:

```json
{
  "ok": true,
  "error_message": null,
  "protocol_version": "vigil_groot_bridge_v1",
  "runtime_mode": "mujoco",
  "capabilities": {
    "actions": [
      "navigate.backward",
      "navigate.forward",
      "navigate.turn_left",
      "navigate.turn_right"
    ],
    "observation": ["rgb", "depth", "robot_state"],
    "oracle_source": "none"
  },
  "bridge": {
    "name": "gear_sonic_vigil_bridge",
    "version": "dry_run_phase1"
  }
}
```

## Current Phase 2 HTTP Transport

The current transport layer is a small standard-library HTTP façade over
`VigilBridgeService`. The default backend remains dry-run and does not start
MuJoCo, hardware, policy inference, or deploy processes.

Run it from the GR00T repository:

```bash
python gear_sonic_deploy/scripts/run_vigil_bridge.py --host 127.0.0.1 --port 8765 --runtime-mode mujoco
```

Supported HTTP endpoints:

| Method | Path | Service call |
| --- | --- | --- |
| `GET` | `/health` | transport health only |
| `POST` | `/handshake` | `handshake(payload)` |
| `POST` | `/reset_episode` | `reset_episode(payload)` |
| `POST` | `/execute_action` | `execute_action(payload)` |
| `POST` | `/observation` | `get_observation(payload)` |
| `POST` | `/robot_state` | `get_robot_state()` |
| `POST` | `/halt` | `halt()` |
| `POST` | `/close` | `close()` |

All request and response bodies are JSON objects. Bridge-level failures return
structured JSON with `ok=false`; malformed JSON returns HTTP 400, and unknown
routes return HTTP 404.

## Current Phase 3 MuJoCo Adapter

The Phase 3 adapter connects the same HTTP/protocol façade to the existing
MuJoCo sim/deploy transports. It is implemented in
`gear_sonic/vigil_bridge/mujoco_adapter.py`.

The adapter uses:

- ZMQ command/planner PUB messages consumed by deploy `ZMQManager`
- ZMQ `g1_debug` state SUB messages published by deploy
- optional DDS `rt/odostate` for sim odometry
- optional MuJoCo camera ZMQ stream from `run_sim_loop.py`

The bridge still does not import Vigil, does not start MuJoCo, does not start
deploy, does not call policy inference directly, and does not modify WBC
internals. Start the sim/deploy processes separately, then start the bridge
with `--backend mujoco`.

Example sim setup:

```bash
# terminal 1: MuJoCo sim
source .venv_sim/bin/activate
python gear_sonic/scripts/run_sim_loop.py --enable-image-publish --enable-offscreen --camera-port 5555

# terminal 2: WBC/deploy process with ZMQ manager input and debug output
cd gear_sonic_deploy
bash deploy.sh sim --input-type zmq_manager --output-type all

# terminal 3: Vigil HTTP bridge backed by MuJoCo transports
cd /home/lizj/code/GR00T-WholeBodyControl
python gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend mujoco \
  --runtime-mode mujoco \
  --host 127.0.0.1 \
  --port 8765 \
  --command-port 5556 \
  --state-host localhost \
  --state-port 5557 \
  --odom-source auto \
  --move-model-file auto \
  --max-speed-mps 1.0 \
  --mujoco-camera \
  --camera-host localhost \
  --camera-port 5555
```

If deploy is not already in planner mode, pass `--auto-start-control` to send
the start/planner command during bridge reset/start. This is an explicit config
choice; the default does not auto-start control.

`/halt` is the bridge safety stop interface. It sends idle commands by default.
Sending the deploy `stop=True` command is opt-in via `--send-stop-on-halt`
because that command can terminate the policy/deploy process.

`/close` only releases bridge-side sockets/resources in the current
implementation. It does not stop policy/deploy execution by itself. For a
controlled shutdown, call `/halt` first, then `/close`; start the bridge with
`--send-stop-on-halt` when the intended behavior is to stop the deploy process
on halt.

MuJoCo adapter dependencies are optional and local to this backend:

- `pyzmq` and `msgpack` for command/state/camera transport
- `unitree_sdk2py`/DDS dependencies only when `--odom-source auto|dds` is used
- camera transport only when `--mujoco-camera` is set

If these runtime connections are missing, the adapter returns structured
`ok=false` responses or observation telemetry that marks the missing source.
Object-level success, oracle data, benchmark scores, and task completion remain
unavailable on the GR00T side.

## Current Phase 4 Real-Robot Adapter

The Phase 4 real adapter uses the same bridge protocol and the same public
deploy-facing ZMQ style as MuJoCo, but with stricter defaults. It is implemented
in `gear_sonic/vigil_bridge/real_adapter.py`.

The adapter uses:

- ZMQ command/planner PUB messages consumed by real deploy when started with a
  ZMQ-capable input mode such as `zmq_manager` or `manager`
- ZMQ `g1_debug` state SUB messages published by deploy when output is enabled
- optional ZMQ camera frames from the real-robot camera stack

The bridge still does not start deploy, does not start the robot, does not call
policy inference directly, and does not modify WBC internals. Start and verify
the real deploy process separately, then start the bridge with `--backend real`.
For robot-side bring-up, the repository also provides `./vigil_bridge start`,
which is a tmux launcher around Docker, deploy, and the HTTP bridge. That helper
starts processes; the HTTP bridge service itself remains a runtime adapter.

Real motion is disabled by default. The bridge rejects motion commands until it
is started with `--enable-real-motion`. This is intentional: selecting
`--backend real` is not enough to move hardware.

Example one-command robot-side setup:

```bash
cd /home/unitree/GR00T-WholeBodyControl_Vigil
./vigil_bridge start --max-speed-mps 2 --camera-required --attach
```

Example manual real setup:

```bash
# terminal 1: deployment inside the ROS2 Docker container
cd /workspace/g1_deploy
source scripts/setup_env.sh
printf '\n' | ./deploy.sh real \
  --input-type zmq_manager \
  --output-type zmq \
  --zmq-host 127.0.0.1

# terminal 2: Vigil HTTP bridge on the robot host
cd /home/unitree/GR00T-WholeBodyControl_Vigil
python gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend real \
  --runtime-mode real \
  --host 0.0.0.0 \
  --port 8765 \
  --command-bind-host 127.0.0.1 \
  --command-port 5556 \
  --state-host 127.0.0.1 \
  --state-port 5557 \
  --real-camera \
  --camera-host localhost \
  --camera-port 5555 \
  --enable-real-motion \
  --auto-start-control \
  --max-speed-mps 2
```

If no real camera stream is available during early bring-up, use
`--real-camera-optional` and keep test motions disabled or tightly supervised.
The default real mode requires camera/state/command readiness before movement.

Real mode uses conservative defaults:

- default move distance: `0.10` m
- default move speed: `0.15` m/s
- startup speed ceiling: `2.00` m/s
- calibrated move model: enabled by default; use `--disable-move-model` to fall
  back to direct duration-based open-loop move commands
- default turn angle: `15` degrees
- default turn rate ceiling: `30` deg/s

`/halt` is the bridge safety stop interface. It sends idle commands by default.
Sending deploy `stop=True` is still opt-in via `--send-stop-on-halt`.

`/close` only releases bridge-side sockets/resources. For controlled shutdown,
call `/halt` first, then `/close`.

## Runtime Modes

The bridge should support the same API for both modes:

- `runtime_mode=real`
- `runtime_mode=mujoco`

Mode-specific differences should be selected by config:

- command endpoint
- state source
- camera source
- odom source
- speed and timeout limits
- startup and reset policy

## Minimal Bridge API

The first bridge API can be small:

```python
class VigilBridgeService:
    def handshake(self, payload: dict) -> dict: ...
    def reset_episode(self, payload: dict) -> dict: ...
    def execute_action(self, payload: dict) -> dict: ...
    def get_observation(self, payload: dict | None = None) -> dict: ...
    def get_robot_state(self) -> dict: ...
    def halt(self) -> dict: ...
    def close(self) -> None: ...
```

The same conceptual API can be exposed through an in-process client, ZMQ, HTTP,
or another transport. The payloads should stay JSON/msgpack-compatible.

## Action Request

Example request from Vigil:

```json
{
  "episode_id": "groot_real_nav_001",
  "step_id": 3,
  "runtime_mode": "real",
  "skill_name": "navigate.forward",
  "arguments": {
    "magnitude": 1
  },
  "safety": {
    "max_speed_mps": 0.5,
    "timeout_s": 8.0
  }
}
```

## Action Response

`/execute_action` is synchronous at the primitive level. When it returns, the
bridge has either completed the primitive, rejected it before command, timed
out, failed, or been interrupted. This status is separate from benchmark task
success.

Final response shape:

```json
{
  "ok": true,
  "error_message": null,
  "action_status": "completed",
  "executed_arguments": {
    "skill_name": "navigate.forward",
    "primitive": "move_model",
    "distance_m": 0.25,
    "max_speed_mps": 0.5,
    "timeout_s": 8.0
  },
  "robot_state_before": {},
  "robot_state_after": {},
  "telemetry": {
    "bridge": "groot_vigil_bridge",
    "phase": "mujoco_adapter_phase3",
    "runtime_mode": "mujoco",
    "episode_id": "test_001",
    "step_id": 1,
    "controller": "mujoco_zmq_wbc",
    "dry_run": false,
    "completion": {
      "motion_commanded": true,
      "completion_source": "duration_and_settle",
      "capture_timing": "after_settle",
      "settled": true,
      "duration_s": 1.83,
      "command_duration_s": 1.0,
      "settle_duration_s": 0.8
    },
    "motion_result": {
      "actual_distance_m": 0.48,
      "actual_distance_source": "rt/odostate"
    }
  }
}
```

Required stable fields for Vigil:

- top-level `ok`
- top-level `error_message`
- top-level `action_status`
- top-level `executed_arguments`
- top-level `robot_state_before`
- top-level `robot_state_after`
- `telemetry.completion.capture_timing`

`action_status` values:

- `completed`: primitive completed; `robot_state_after` was sampled after
  completion/settle.
- `timeout`: command was issued, but the runtime did not satisfy the completion
  condition before timeout.
- `rejected`: unsupported skill or invalid arguments; no motion command was
  issued.
- `failed`: runtime, transport, or controller error.
- `interrupted`: reserved for halt/close interruption during execution.

Measurement fields are optional. For example, MuJoCo may return
`motion_result.actual_distance_m` from `rt/odostate`, but real mode may omit
that field if no reliable measurement source is available. Do not fabricate
measurement values.

If execution fails, return `ok=false`, `action_status`, and a structured
`error_message`. Prefer safe halt/idle behavior over repeated retries.

## Observation Response

The bridge should provide the latest observation and state metadata:

```json
{
  "observation_id": "obs_0003",
  "runtime_mode": "mujoco",
  "images": {
    "ego_view": {
      "encoding": "jpeg-base64",
      "data": "<base64 jpeg>"
    }
  },
  "camera_timestamps": {},
  "robot_state": {},
  "telemetry": {},
  "perception": {
    "source": "none",
    "detections": []
  }
}
```

Vigil decides how this becomes a Vigil `Observation`.
The image payload is a transport JPEG. Camera publishers keep image arrays in
RGB order internally and convert to OpenCV BGR only at JPEG encode/decode
boundaries so red/blue channels are preserved.

## Initial Primitive Mapping

The bridge should initially focus on navigation primitives:

| Vigil action | GR00T primitive | real | MuJoCo |
| --- | --- | --- | --- |
| `navigate.forward` | `move_model(+distance_m)` | yes | yes |
| `navigate.backward` | `move_model(-distance_m)` | yes | yes |
| `navigate.turn_left` | `rotate(+degrees)` | yes | yes |
| `navigate.turn_right` | `rotate(-degrees)` | yes | yes |
| `report` | no robot motion; terminal report handled by Vigil | yes | yes |

Unsupported interactions should fail clearly rather than silently doing
nothing.

`navigate.forward` and `navigate.backward` use calibrated `move_model` by
default in MuJoCo and real bridge modes. The bridge loads the latest
`outputs/vigil_move_models/vigil_move_model_*.json` file unless
`--move-model-file PATH` is provided. `safety.max_speed_mps` is treated as a
speed ceiling for the model-predicted command; if Vigil omits it, the bridge
uses the startup default `--max-speed-mps 2.0`.

Example MuJoCo bridge startup:

```bash
python gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend mujoco \
  --runtime-mode mujoco \
  --move-model-file auto \
  --max-speed-mps 2
```

## Real Robot Safety

For real mode:

- require explicit start policy
- expose `halt()`
- call `halt()` on exceptions
- use conservative movement defaults
- return timeout errors
- fail closed if camera, state, or command link is missing
- do not auto-run hardware commands in tests

## MuJoCo Mode

MuJoCo mode should use the same bridge API but can expose richer telemetry:

- optional sim odometry
- command timing
- replay/sim camera source
- actual movement estimates when available

MuJoCo object-level success should still be treated as unavailable unless a
deterministic instrumented scene or oracle is explicitly added.

## Testing

Use fake bridge clients and fake sensor payloads for unit tests.

Suggested tests:

- protocol payload validation
- fake `execute_action` request/response
- fake camera observation response
- real/mujoco mode config parsing
- `halt()` is called on simulated exceptions
- bridge code does not import Vigil

Suggested lightweight check:

```bash
python -m compileall gear_sonic/vigil_bridge
```

Current bridge unit checks:

```bash
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider tests/vigil_bridge
# 19 passed
```
