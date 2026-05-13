# Protected Areas Contract

Do not modify model deployment, policy inference, or low-level whole-body
control unless the user explicitly requests it.

## Protected Areas

Protected areas include:

- `gear_sonic_deploy/src/`
- `gear_sonic_deploy/CMakeLists.txt`
- WBC deploy internals
- policy inference code
- ONNX, TensorRT, and runtime model loading paths
- low-level robot command/control loops
- `decoupled_wbc/control/`
- `external_dependencies/`
- `gear_sonic/trl/`
- model checkpoints
- policy assets
- generated outputs
- logs

Do not change:

- robot safety behavior
- controller timing
- motor commands
- policy action dimensions
- model inputs/outputs
- deployment protocol

as part of Vigil bridge work.

## Allowed Bridge Work

Prefer adding a new bridge layer instead of editing existing deploy/control code.

Allowed locations:

- `gear_sonic/vigil_bridge/`
- `gear_sonic/vigil_bridge/protocol.py`
- `gear_sonic/vigil_bridge/service.py`
- `gear_sonic/vigil_bridge/primitive_executor.py`
- `gear_sonic/vigil_bridge/sensors.py`
- `gear_sonic_deploy/scripts/run_vigil_bridge.py`
- bridge protocol tests
- fake client/service tests
- payload validation tests
- docs listed in `docs/INDEX.md`

Existing scripts may be used as references:

- `gear_sonic_deploy/scripts/vigil_real_controller.py`
- `gear_sonic_deploy/scripts/vigil_primitive_controller.py`

Do not make Vigil directly import these REPL/CLI scripts.

## Escalation Rule

If a requested change appears to require touching a protected area, stop and
explain the tradeoff before editing.