# Real-Robot Safety Rules

These rules apply to real robot mode.

## Motion Safety

Never auto-start motion unless an explicit user request or config says so.

Expose `halt()` and call it on exceptions.

Treat bridge `/halt` as the safety stop interface. Bridge `/close` may only
release sockets/resources; do not assume it stops policy/deploy execution unless
the implementation explicitly routes close through halt/stop behavior.

Use conservative default speed and timeout limits.

Fail closed when any required runtime connection is missing, including:

- state connection
- camera connection
- command connection

Return structured errors instead of retrying unsafe commands indefinitely.

## Hardware Execution

Do not run hardware, camera SDK, MuJoCo, or deploy commands unless explicitly
requested.

Do not change safety behavior, controller timing, motor commands, policy action
dimensions, model inputs/outputs, or deployment protocol as part of ordinary
Vigil bridge work.

## Testing

Do not require real hardware for unit tests.

Use:

- fake bridge clients
- fake services
- fake sensor payloads
- protocol validation tests

Keep bridge dependencies optional and local to bridge code.

Normal GR00T development should not require Vigil to be installed.
