# G1 TTS Joint Debug Quickstart

This note is for joint Host/VLT and G1-side TTS debugging. It only exercises
audio bridge endpoints and G1 speaker output. Do not use it to start robot
motion.

## G1 Side: Start Audio Bridge

SSH into G1:

```bash
ssh unitree@192.168.123.164
```

Start the audio bridge from the repo:

```bash
cd /home/unitree/GR00T-WholeBodyControl_Vigil

RUNNER=/home/unitree/g1_audio_tests/speaker_loud_music/build/g1_speaker_loud_music_runner

python3 gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend dry_run \
  --runtime-mode real \
  --host 0.0.0.0 \
  --port 8765 \
  --audio-enabled \
  --audio-advertise-always \
  --audio-ws \
  --audio-ws-host 0.0.0.0 \
  --audio-ws-port 8766 \
  --audio-mic-interface-ip 192.168.123.164 \
  --audio-speaker-iface enP8p1s0 \
  --audio-speaker-volume 100 \
  --audio-speaker-peak-target 27800 \
  --audio-speaker-runner "$RUNNER"
```

If an older bridge process is already running, stop it first and restart with
the command above. Python route changes such as `/audio/tts` are only loaded by
the new process.

Why `dry_run` here:

- It does not send motion commands.
- `runtime-mode real` reports that the test targets the real G1.
- The configured speaker runner still uses the real G1 audio speaker path.

Quick health check on G1:

```bash
curl -s http://127.0.0.1:8765/audio/health
```

## VLT Side: Get The Standalone Client

If VLT does not have this repo, copy the standalone client from G1:

```bash
scp unitree@192.168.123.164:/home/unitree/GR00T-WholeBodyControl_Vigil/gear_sonic/vigil_bridge/audio_tts_client.py .
```

The script only uses Python standard library.

## VLT Side: Native TTS Test

This validates G1 native `AudioClient.TtsMaker` reachability. It is expected to
be quieter than calibrated PCM playback.

```bash
python3 audio_tts_client.py \
  --base-url http://192.168.123.164:8765
```

Single Chinese test:

```bash
python3 audio_tts_client.py \
  --base-url http://192.168.123.164:8765 \
  --text "你好，这是 VLT 侧 TTS 联调测试。" \
  --language zh
```

Single English test:

```bash
python3 audio_tts_client.py \
  --base-url http://192.168.123.164:8765 \
  --text "Hello, this is a VLT side TTS integration test." \
  --language en
```

## VLT Side: Calibrated PCM Speaker Test

This validates the loudness-calibrated path:

```text
TTS audio / PCM -> bridge /audio/output_segment -> peak normalize 27800 -> G1 PlayStream
```

Use a generated tone first:

```bash
python3 audio_tts_client.py \
  --base-url http://192.168.123.164:8765 \
  --skip-native-tts \
  --send-tone
```

Then use a VLT/OMNI-generated TTS WAV. The WAV must be:

- 16 kHz
- mono
- signed 16-bit PCM

```bash
python3 audio_tts_client.py \
  --base-url http://192.168.123.164:8765 \
  --skip-native-tts \
  --send-wav omni_tts_16k_mono_pcm16.wav
```

## Expected Interpretation

- `/audio/tts` success means native G1 TTS API is reachable.
- Native TTS is not loudness calibrated and may be quiet even at API volume 100.
- `/audio/output_segment` success means calibrated speaker playback is working.
- Production VLT/OMNI speech should send synthesized PCM/WAV through
  `/audio/output_segment` or the audio WebSocket binary output path.

## Common Failures

| Symptom | Cause | Fix |
| --- | --- | --- |
| `Connection refused` from VLT | G1 bridge is not running or port blocked | Restart G1 command and check `curl http://127.0.0.1:8765/audio/health` |
| `audio manager is not configured` | Bridge was started without `--audio-enabled` | Restart with the full G1 command above |
| `speaker_runner is not configured` | Missing `--audio-speaker-runner` | Restart with `--audio-speaker-runner "$RUNNER"` |
| Native TTS is barely audible | Native G1 TTS bypasses PCM calibration | Use VLT/OMNI TTS WAV through `/audio/output_segment` |
| WAV rejected | Wrong sample rate/channels/sample width | Convert to 16 kHz mono PCM16 before sending |
