# G1 Audio Bridge Connectivity Record

This document records the two-sided G1 audio bridge connectivity test between
the robot runtime side and the Host/VLT side.

It is runtime-facing only. Do not use this document to start robot motion. The
commands below only exercise audio bridge, microphone multicast, WebSocket, and
G1 speaker `AudioClient.PlayStream` / `AudioClient.TtsMaker`.

## Test Context

- Date: 2026-06-23
- Robot SSH target from Host/VLT: `unitree@192.168.123.164`
- G1 repo: `/home/unitree/GR00T-WholeBodyControl_Vigil`
- G1 DDS/audio interface: `enP8p1s0`
- G1 DDS/audio IP: `192.168.123.164`
- Microphone multicast: `239.168.123.161:5555`
- HTTP bridge port: `8765`
- Audio WebSocket port: `8766`
- Speaker runner:
  `/home/unitree/g1_audio_tests/vigil_led_speaker/build/g1_vigil_led_speaker_runner`
- Standard audio format: 16 kHz, mono, signed PCM16 little-endian
- Speaker calibration: API `SetVolume(100)`, PCM peak target `27800`
- TTS mapping: Chinese `speaker_id=0`, English `speaker_id=1`

## G1 Side Command

For audio-only bridge testing, use `dry_run` backend with `runtime_mode=real`.
This avoids requiring real motion/state/camera readiness while still using the
real G1 audio input/output paths when `--audio-speaker-runner` is provided.

```bash
cd /home/unitree/GR00T-WholeBodyControl_Vigil

RUNNER=/home/unitree/g1_audio_tests/vigil_led_speaker/build/g1_vigil_led_speaker_runner

python3 gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend dry_run \
  --runtime-mode real \
  --host 0.0.0.0 \
  --port 8765 \
  --audio-enabled \
  --audio-ws \
  --audio-ws-host 0.0.0.0 \
  --audio-ws-port 8766 \
  --audio-mic-interface-ip 192.168.123.164 \
  --audio-speaker-iface enP8p1s0 \
  --audio-speaker-runner "$RUNNER" \
  --audio-speaker-reactive-led
```

Important details:

- Keep `--audio-speaker-runner` as one complete executable path.
- Do not split the path after `/home/unitree/g1_audio_tests/`.
- If `--audio-speaker-runner` is omitted in `dry_run`, the bridge uses the fake
  speaker client by design.
- With the runner path present, `dry_run` remains motion-free but uses real
  speaker output through the subprocess runner.

### Reactive Speaker LED

The bridge-owned runner is built from
`gear_sonic/vigil_bridge/native_audio/`. With
`--audio-speaker-reactive-led`, outgoing PCM drives a 50 Hz saturated
orange-to-yellow LED plan. Speech frames always use `B=0`. After audio stops,
the runner transitions to dark blue in 250 ms, brightens to full blue in 500
ms, and holds full blue for 500 ms. Native `TtsMaker` has no synthesized PCM
amplitude, so exact amplitude tracking applies to PCM/WAV output.

```bash
cmake -S gear_sonic/vigil_bridge/native_audio \
  -B /home/unitree/g1_audio_tests/vigil_led_speaker/build
cmake --build /home/unitree/g1_audio_tests/vigil_led_speaker/build -j2
```

## Host/VLT Side Command

If Host/VLT has the repo:

```bash
cd /path/to/GR00T-WholeBodyControl_Vigil
python3 -m pip install --user websockets

PYTHONPATH=$PWD python3 -m gear_sonic.vigil_bridge.audio_ws_client \
  --url ws://192.168.123.164:8766/audio/ws \
  --listen-seconds 5 \
  --record-wav vlt_mic_check.wav \
  --send-tone
```

If Host/VLT only has the copied client script:

```bash
python3 -m pip install --user websockets

python3 audio_ws_client.py \
  --url ws://192.168.123.164:8766/audio/ws \
  --listen-seconds 5 \
  --record-wav vlt_mic_check.wav \
  --send-tone
```

Expected Host/VLT output includes:

```text
{"type": "session.started", ...}
[client] sending tone PCM bytes=32000
[client] received mic PCM bytes=...
{"type": "output.result", ...}
[client] wrote mic recording vlt_mic_check.wav bytes=...
```

## Health Checks

Run these on G1:

```bash
ps -ef | grep run_vigil_bridge | grep -v grep
ss -ltnp | grep -E ':8765|:8766' || true
python3 -c "import websockets; print(websockets.__version__)"
curl -s http://127.0.0.1:8765/audio/health
```

Run these on Host/VLT:

```bash
curl -s http://192.168.123.164:8765/audio/health

curl -s -X POST http://192.168.123.164:8765/audio/tts \
  -H 'Content-Type: application/json' \
  -d '{"text":"你好，这是 G1 运行时 TTS 测试。","language":"zh"}'

curl -s -X POST http://192.168.123.164:8765/audio/tts \
  -H 'Content-Type: application/json' \
  -d '{"text":"Hello, this is a G1 runtime TTS test.","language":"en"}'
```

Or use the repo client:

```bash
cd /path/to/GR00T-WholeBodyControl_Vigil
PYTHONPATH=$PWD python3 -m gear_sonic.vigil_bridge.audio_tts_client \
  --base-url http://192.168.123.164:8765
```

To test the calibrated speaker path with a VLT/OMNI-generated TTS WAV:

```bash
PYTHONPATH=$PWD python3 -m gear_sonic.vigil_bridge.audio_tts_client \
  --base-url http://192.168.123.164:8765 \
  --skip-native-tts \
  --send-wav omni_tts_16k_mono_pcm16.wav
```

Healthy bridge expectations:

- `8765` is listening for HTTP.
- `8766` is listening for audio WebSocket.
- `websockets` imports successfully on G1.
- `/audio/health` returns `enabled=true`.
- Speaker health is `{"available": true, "type": "subprocess"}` for real
  speaker testing.
- Microphone packet counters increase during active sessions.

## Observed Test Results

### WebSocket Connectivity

Host/VLT initially failed with:

```text
ConnectionRefusedError: Connect call failed ('192.168.123.164', 8766)
```

Interpretation:

- G1 was not listening on port `8766` at that moment.
- This is a service/port availability issue, not an audio payload issue.

After restarting the bridge with `--audio-ws --audio-ws-host 0.0.0.0
--audio-ws-port 8766`, Host/VLT WebSocket connection succeeded.

### Bad Speaker Runner Path

An earlier G1 command split the speaker runner path:

```text
--audio-speaker-runner /home/unitree/g1_audio_tests/
speaker_loud_music/build/g1_speaker_loud_music_runner
```

The bridge received `/home/unitree/g1_audio_tests/`, which is a directory, and
speaker playback failed with:

```text
PermissionError: [Errno 13] Permission denied: '/home/unitree/g1_audio_tests/'
```

The bridge now reports this as a structured audio output failure instead of
crashing the WebSocket handler.

### Microphone Input

`/audio/health` showed microphone packets from the multicast stream, for
example:

```json
"mic": {
  "running": false,
  "group": "239.168.123.161",
  "port": 5555,
  "interface_ip": "192.168.123.164",
  "packet_count": 2,
  "byte_count": 10240
}
```

This confirms that the G1-side bridge can receive microphone multicast packets.
During a full Host/VLT test, the client should write `vlt_mic_check.wav`.

### Speaker Output

A direct bridge audio runtime speaker test was executed on G1 with a generated
1 second 440 Hz PCM16 tone.

Observed result:

```text
speaker: subprocess
returncode: 0
AudioClient init=OK
GetVolume ret=0 volume=100
SetVolume requested=100 ret=0
GetVolumeAfterSet ret=0 volume=100
TtsMaker skipped=true
WavInfo sample_rate=16000 channels=1 sample_width=2 pcm_bytes=32000 duration=1
PlayStream offset=0 bytes=32000 ret=0
PlayStop ret=0
```

Bridge normalization telemetry:

```text
target_peak=27800
gain=2.316666666666667
peak_in=12000
clipped_samples=0
peak=27800
```

This verifies the bridge audio output path through the subprocess speaker
runner and G1 `AudioClient.PlayStream`. Human audible confirmation should still
be recorded during onsite tests.

### Native TTS Output

The speaker runner was extended to accept arbitrary native TTS text:

```bash
/home/unitree/g1_audio_tests/speaker_loud_music/build/g1_speaker_loud_music_runner \
  --iface enP8p1s0 \
  --volume 100 \
  --allow-volume-100 \
  --skip-music \
  --tts-text "你好，这是 G1 运行时 TTS 测试。" \
  --tts-speaker-id 0

/home/unitree/g1_audio_tests/speaker_loud_music/build/g1_speaker_loud_music_runner \
  --iface enP8p1s0 \
  --volume 100 \
  --allow-volume-100 \
  --skip-music \
  --tts-text "Hello, this is a G1 runtime TTS test." \
  --tts-speaker-id 1
```

Observed API result for both Chinese and English:

```text
AudioClient init=OK
GetVolume ret=0 volume=100
SetVolume requested=100 ret=0
GetVolumeAfterSet ret=0 volume=100
TtsMaker text ret=0
PlayStream skipped=true
PlayStop skipped=true
```

Important loudness finding: native `AudioClient.TtsMaker` does not use the
runtime PCM calibration path. It has no SDK parameter for PCM gain, and
`speaker_peak_target=27800` is not applied. Onsite listening found native TTS
at `SetVolume(100)` was still much quieter than calibrated `PlayStream`.

For production speech that must match the current speaker calibration, generate
TTS audio on the Host/VLT/OMNI side as 16 kHz mono PCM16 and send it through
`/audio/output_segment` or WebSocket binary output. That path is normalized to
peak `27800` before `AudioClient.PlayStream`.

## Troubleshooting Matrix

| Symptom | Most likely cause | Check/fix |
| --- | --- | --- |
| `ConnectionRefusedError` on Host/VLT | G1 is not listening on `8766` | Run `ss -ltnp | grep 8766` on G1; restart with `--audio-ws` |
| `/audio/health` shows `"speaker":"fake"` | `dry_run` started without runner, or old process still running | Restart with `--audio-speaker-runner "$RUNNER"` |
| WebSocket closes with internal error after sending audio | Speaker subprocess path/config failed | Check G1 bridge logs and `audio/output_segment` response |
| `/audio/tts` is barely audible | Native G1 `TtsMaker` is not PCM gain-calibrated | Use VLT/OMNI TTS -> PCM -> `/audio/output_segment` for calibrated loudness |
| `/audio/tts` returns unknown argument from runner | Old C++ runner is still installed | Rebuild `/home/unitree/g1_audio_tests/speaker_loud_music/build/g1_speaker_loud_music_runner` |
| `Permission denied: /home/unitree/g1_audio_tests/` | Runner path points to directory | Use full runner file path |
| No mic frames on Host/VLT | Mic multicast receiver not started or wrong interface IP | Check `/audio/health` mic packet counters |
| HTTP works but WebSocket does not | `websockets` missing or `--audio-ws` omitted | `python3 -c "import websockets"` and restart bridge |

## Recompile Rule

Audio bridge operation does not require recompilation per run.

No recompile is needed for:

- starting/stopping the Python bridge,
- WebSocket audio streaming,
- receiving microphone PCM,
- sending new PCM/WAV audio,
- changing speaker peak target or bridge CLI parameters.

Recompile is only needed if the C++ speaker runner source changes, for example:

```text
/home/unitree/g1_audio_tests/speaker_loud_music/g1_speaker_loud_music_runner.cpp
```

Normal OMNI audio output and native TTS requests through the rebuilt runner do
not require C++ rebuilds per run.
