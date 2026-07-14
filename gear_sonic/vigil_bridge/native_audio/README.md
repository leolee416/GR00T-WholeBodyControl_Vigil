# G1 Vigil LED Speaker Runner

This bridge-owned helper wraps the Unitree G1 `AudioClient` speaker path and
applies a precomputed RGB plan while PCM is playing. It does not control robot
motion or policy execution.

The Python bridge generates one packed RGB triple per 20 ms audio window. The
runner enforces a zero blue channel for all speech frames, streams the matching
PCM, and applies the configured post-audio animation:

1. current orange/yellow to dark blue in 250 ms;
2. dark blue to bright blue in 500 ms;
3. hold bright blue for 500 ms and leave it active.

Build on G1:

```bash
cmake -S gear_sonic/vigil_bridge/native_audio \
  -B /home/unitree/g1_audio_tests/vigil_led_speaker/build
cmake --build /home/unitree/g1_audio_tests/vigil_led_speaker/build -j2
```

Enable through the bridge with both the runner path and
`--audio-speaker-reactive-led`. Native `TtsMaker` does not expose synthesized
PCM amplitude, so exact reactive animation applies to PCM/WAV output through
`/audio/output_segment` or the corresponding WebSocket output path.
