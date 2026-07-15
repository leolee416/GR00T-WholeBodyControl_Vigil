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

## Persistent PCM mode

Production OMNI/VLT output uses one long-lived runner instead of launching this
binary once per PCM frame. The Python bridge starts the runner with
`--stream-stdin`, initializes DDS and `AudioClient` once, then uses this framed
stdin protocol:

```text
START <utterance_id>\n
PCM <byte_count>\n<exact PCM16 bytes>
...
END\n
```

`STOP` aborts the active utterance and `QUIT` shuts down the runner. Every
command receives one stdout line beginning with `OK` or `ERR`. All runner logs
go to stderr in this mode so they cannot corrupt framing.

All `PCM` commands between `START` and `END` share the same G1 `stream_id`.
`PlayStop` and the blue LED end animation run only once at `END`; `STOP` skips
the end animation and clears playback immediately. With
`--stream-reactive-led`, the runner keeps its RMS/envelope state across chunks
and drives the LED on a separate 50 Hz worker.

The bridge defaults to 200 ms G1 playback blocks, 400 ms initial prebuffer, a
20 ms send lead, and a bounded 3 second queue. These are onsite tuning values,
not wire-format requirements.
