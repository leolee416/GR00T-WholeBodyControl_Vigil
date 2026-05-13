# Vigil Bridge Integration

The Vigil bridge connects Vigil to GR00T runtime capability while preserving a
strict ownership boundary.

## Intended Split

Vigil side:

- `GrootWBCEnv`
- `GrootActionBackend`
- oracle handling
- benchmark runner
- trace
- judge
- scoring

GR00T side:

- bridge service/client code
- primitive execution
- camera/state observation
- runtime health
- telemetry

## Design Direction

Prefer a bridge protocol or optional importable client over importing CLI or
REPL scripts from Vigil.

The bridge should support both:

- real robot runtime
- MuJoCo runtime

through the same Vigil-side abstraction.

Current MuJoCo bridge behavior is documented in
[`../vigil_bridge_integration.md`](../vigil_bridge_integration.md). Important
operational points:

- `navigate.forward` and `navigate.backward` use calibrated `move_model` by
  default when a model JSON is available.
- `--max-speed-mps` / `--max-move-speed` sets the startup speed ceiling; default
  is `1.0` m/s.
- `POST /halt` sends idle commands and may send deploy `stop=True` when the
  bridge starts with `--send-stop-on-halt`.
- `POST /close` releases bridge-side resources; it is not a policy/deploy stop
  interface in the current implementation.

Current real-robot bridge behavior is also documented there. Important
operational points:

- `--backend real` connects to already-running real deploy transports; it does
  not start deploy or hardware.
- Real motion remains disabled unless the bridge is started with
  `--enable-real-motion`.
- Real mode uses conservative motion defaults and fails closed if required
  state/camera/command readiness checks are missing.
- `POST /halt` is the safety stop interface; `POST /close` only releases bridge
  resources unless halt/stop behavior is explicitly configured.

## Recommended Locations

Bridge implementation should live in:

- `gear_sonic/vigil_bridge/`
- `gear_sonic_deploy/scripts/run_vigil_bridge.py`

Do not scatter bridge design notes across unrelated deployment, training, or
model documentation.

## Verification

When changing bridge code, prefer:

```bash
python -m compileall gear_sonic/vigil_bridge
```
Then run protocol unit tests or fake service/client smoke tests.

Only run MuJoCo, hardware, camera SDK, or deploy commands when explicitly asked.
