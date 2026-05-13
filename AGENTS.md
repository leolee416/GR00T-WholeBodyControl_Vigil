# GR00T-WholeBodyControl Agent Contract

This repository is a whole-body-control and robot deployment repository.

It is not a benchmark framework.

Vigil integration is an adapter layer that exposes GR00T real-robot and MuJoCo
execution/observation capability to Vigil as an external embodied backend.

Vigil owns:
- benchmark episodes
- agent prompts
- traces
- judge/scoring
- oracle semantics

GR00T owns:
- robot/runtime execution
- primitive execution
- camera/state observations
- runtime health
- telemetry

## Hard Rules

Do not modify low-level control, deployment, policy inference, robot safety, or
runtime model paths unless explicitly requested.

Protected areas include:
- `gear_sonic_deploy/src/`
- `gear_sonic_deploy/CMakeLists.txt`
- `decoupled_wbc/control/`
- `external_dependencies/`
- `gear_sonic/trl/`
- model checkpoints, generated outputs, logs, and runtime assets

Prefer adding bridge code instead of editing deploy/control internals.

Allowed Vigil bridge locations:
- `gear_sonic/vigil_bridge/`
- `gear_sonic_deploy/scripts/run_vigil_bridge.py`
- bridge protocol tests
- fake client/service tests
- docs listed in `docs/INDEX.md`

If a requested change requires touching a protected area, stop and explain the
tradeoff before editing.

## Bridge Contract

The bridge is runtime-facing, not benchmark-facing.

The bridge may return:
- execution status
- structured errors
- executed arguments
- robot state
- camera observation
- runtime health
- telemetry

The bridge must not return:
- benchmark scores
- Vigil `StepJudgeResult`
- hidden task success
- fabricated THOR-style object metadata
- agent prompt logic

Estimated perception facts must be labeled as estimated, not groundtruth.

## Real-Robot Safety

For real robot mode:
- never auto-start motion without explicit user request or config
- expose `halt()` and call it on exceptions
- use conservative speed and timeout defaults
- fail closed when state, camera, or command connection is missing
- do not run hardware/deploy commands unless explicitly requested

For bridge shutdown, treat `/halt` as the runtime safety interface. Do not assume
`/close` stops policy/deploy execution unless the bridge explicitly routes close
through halt/stop behavior.

## Runtime / Verification

For bridge code, prefer lightweight checks first:

```bash
python -m compileall gear_sonic/vigil_bridge
