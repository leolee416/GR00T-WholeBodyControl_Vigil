# Vigil Bridge Contract

The Vigil bridge is runtime-facing, not benchmark-facing.

It exposes GR00T real-robot and MuJoCo execution/observation capability to Vigil
without transferring benchmark ownership to GR00T.

## Dependency Direction

GR00T should not import Vigil.

The bridge protocol should be plain Python / JSON / msgpack-compatible so Vigil
can communicate with this repository without creating a circular dependency.

## Bridge May Return

The bridge may return:

- execution success/failure
- structured error messages
- executed arguments
- robot state before/after
- latest camera observation
- runtime health
- telemetry

## Bridge Must Not Return

The bridge must not return:

- benchmark scores
- Vigil `StepJudgeResult`
- hidden task success
- fabricated THOR-style object metadata
- agent prompt logic

## Groundtruth Rule

If a fact is estimated by perception, label it as estimated.

Do not call estimated perception output groundtruth.

## Real and MuJoCo Modes

Real robot and MuJoCo simulation should share the same bridge API.

Use configuration to choose mode:

- `runtime_mode=real`
- `runtime_mode=mujoco`

Mode-specific differences should live in config or adapter classes, not in separate benchmark logic.