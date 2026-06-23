# Unitree G1 Audio I/O Notes

This document records the current G1 audio input/output investigation for later
runtime integration work. It is a debugging note, not a benchmark interface.

Do not use these notes to start robot motion. The checks below only touch audio,
VUI discovery, DDS, UDP multicast, and local Linux audio diagnostics.

## Environment Observed

- Host: `unitree-g1-nx`
- OS: Ubuntu 22.04.5 LTS, aarch64, Jetson kernel
- Unitree DDS interface: `enP8p1s0`
- Unitree DDS address: `192.168.123.164/24`
- Microphone multicast source observed: `192.168.123.161`

## SDK Locations

Relevant local SDK paths found during debugging:

- Python SDK source:
  `/home/unitree/GR00T-WholeBodyControl_Vigil/external_dependencies/unitree_sdk2_python`
- C++ SDK in this repo:
  `/home/unitree/GR00T-WholeBodyControl_Vigil/gear_sonic_deploy/thirdparty/unitree_sdk2`
- Standalone C++ SDK:
  `/home/unitree/unitree_sdk2-main`
- Installed C++ SDK:
  `/usr/local/include/unitree`, `/usr/local/lib/libunitree_sdk2.a`

The current global Python environment could not import `unitree_sdk2py`.
With `PYTHONPATH` pointed at the local SDK source, import still failed because
`cyclonedds` is not installed:

```text
ModuleNotFoundError: No module named 'cyclonedds'
```

No Python dependency installation was performed during this investigation.

## Speaker Output

G1 speaker output is exposed by the Unitree G1 `AudioClient` service named
`voice`.

Official/local API definitions:

- `unitree/robot/g1/audio/g1_audio_api.hpp`
- `unitree/robot/g1/audio/g1_audio_client.hpp`
- `example/g1/audio/g1_audio_client_example.cpp`
- `unitree_sdk2py/g1/audio/g1_audio_client.py`

Audio API IDs:

| API | ID |
| --- | --- |
| TTS | `1001` |
| ASR | `1002` |
| Start play / `PlayStream` | `1003` |
| Stop play / `PlayStop` | `1004` |
| Get volume | `1005` |
| Set volume | `1006` |
| RGB LED | `1010` |

The C++ `AudioClient` provides:

- `GetVolume(uint8_t&)`
- `SetVolume(uint8_t)`
- `TtsMaker(const std::string&, int32_t speaker_id)`
- `PlayStream(std::string app_name, std::string stream_id, std::vector<uint8_t>)`
- `PlayStop(std::string app_name)`

### Verified API-Level Speaker Test

A speaker-only C++ test was created outside the repo at:

```text
/home/unitree/g1_audio_tests/test_g1_speaker.cpp
/home/unitree/g1_audio_tests/build/test_g1_speaker
```

It initializes only:

```cpp
unitree::robot::ChannelFactory::Instance()->Init(0, "enP8p1s0");
unitree::robot::g1::AudioClient client;
client.Init();
```

Then it calls `GetVolume`, `SetVolume(50)`, two short `TtsMaker` calls, and a
short 16 kHz mono PCM16 440 Hz `PlayStream`.

Observed API results:

```text
GetVolume ret=0 volume=100
SetVolume ret=0
GetVolume ret=0 volume=50
TtsMaker zh ret=0
TtsMaker en ret=0
PlayStream ret=0
PlayStop ret=0
```

Later onsite tests confirmed audible speaker output through the C++ G1
`AudioClient` path. The useful runtime calibration is:

- Keep API volume at `SetVolume(100)`.
- Treat PCM peak `27800` as the bridge's calibrated "volume 100" output level.
- Use 16 kHz mono PCM16 frames, and pass PCM frame bytes to `PlayStream`.
- Normalize runtime audio to peak `27800` before playback unless the caller
  explicitly opts out.

Native `TtsMaker(text, speaker_id)` is available for Chinese/English text, but
it does not pass through the PCM calibration path. The SDK exposes only `text`
and `speaker_id`; there is no gain parameter and `speaker_peak_target=27800`
cannot be applied to native TTS. Onsite listening found native TTS at
`SetVolume(100)` was still much quieter than calibrated `PlayStream`.

For production speech output, the preferred path is:

```text
text -> Host/VLT/OMNI TTS audio synthesis -> 16 kHz mono PCM16
  -> bridge /audio/output_segment or WebSocket binary output
  -> peak normalize to 27800
  -> G1 AudioClient.PlayStream
```

Use native `TtsMaker` only as a low-loudness fallback or API availability
probe.

Additional debugging showed that trying `SetVolume(150)` did not raise the
actual volume above 100:

```text
SetVolume requested=150 ret=100
GetVolumeAfterSet ret=0 volume=100
```

So practical loudness above the original test level should be achieved by PCM
normalization/gain, not by API volume values above 100. A 300% test reached
PCM clipping, while peak `27800` stayed below clipping and was selected as the
runtime calibration target.

## Microphone Input

The G1 microphone stream is exposed as UDP multicast.

Values from the official/local G1 audio example:

```cpp
#define GROUP_IP "239.168.123.161"
#define PORT 5555
#define WAV_SECOND 5
#define WAV_LEN (16000 * 2 * WAV_SECOND)
```

The example writes the received stream as:

```cpp
WriteWave("record.wav", 16000, pcm_data.data(), pcm_data.size(), 1);
```

That implies:

- 16 kHz sample rate
- mono
- signed 16-bit PCM
- little-endian on this Linux host

### Verified Multicast Stream

`tcpdump` was not installed on the host, so a Python standard-library multicast
receiver was used instead. It joined `239.168.123.161:5555` on
`192.168.123.164`.

Observed background stream:

```text
packets=20
bytes=102400
samples=51200
duration_est_sec=3.200
min=-1630 max=1520 peak=1630
mean_abs=278.12 rms=352.63
zero_fraction=0.0010
```

Observed while a user spoke near the robot:

```text
packets=64
bytes=327680
samples=163840
duration_est_sec=10.240
min=-3965 max=2959 peak=3965
mean_abs=362.37 rms=464.13
zero_fraction=0.0009
```

The RMS and peak rose during speech. The stream is not empty and not all-zero;
it contains real microphone PCM data.

Example receiver skeleton:

```python
import socket

GROUP = "239.168.123.161"
PORT = 5555
IFACE_IP = "192.168.123.164"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", PORT))
sock.setsockopt(
    socket.IPPROTO_IP,
    socket.IP_ADD_MEMBERSHIP,
    socket.inet_aton(GROUP) + socket.inet_aton(IFACE_IP),
)

data, addr = sock.recvfrom(65535)
```

For runtime integration, use a bounded receiver thread or process that:

- joins the multicast group on the `192.168.123.x` interface,
- treats payloads as PCM16 mono at 16 kHz,
- emits timestamped audio frames or short WAV/PCM chunks,
- shuts down cleanly without modifying robot control state.

## VUI / AudioHub Discovery

The following were checked:

```bash
ps -ef | grep -Ei "audio|voice|vui|asr|hub" | grep -v grep
systemctl list-units --type=service | grep -Ei "audio|voice|vui|asr|hub"
systemctl list-unit-files | grep -Ei "audio|voice|vui|asr|hub"
ros2 topic list 2>/dev/null | grep -Ei "audio|voice|vui|asr|hub|api"
```

Findings:

- No running `audiohub`, `vui`, `asr`, or `voice` service process was found.
- Only `pulseaudio` and bluetooth with `--noplugin=audio,a2dp,avrcp` matched.
- `ros2` was not available in the current shell.
- No ROS2 topic information was found for:
  - `/api/audiohub/request`
  - `/api/audiohub/response`
  - `/api/vui/request`
  - `/api/vui/response`
  - `rt/audio_msg`

The SDK source contains Go2/B2 VUI clients:

- `unitree_sdk2py/go2/vui/vui_client.py`
- `unitree_sdk2py/b2/vui/vui_client.py`
- `/usr/local/include/unitree/robot/go2/vui`

The discovered VUI API is service `vui` with:

- `SetSwitch`
- `GetSwitch`
- `SetVolume`
- `GetVolume`
- `SetBrightness`
- `GetBrightness`

No G1-specific `VuiClient` and no `audiohub` client were found in the local SDK
tree. No explicit wake-up conversation mode API was found.

## Current Interpretation

- Microphone multicast is already active without using the phone App or screen
  controller.
- Audio input can be integrated now via UDP multicast.
- Speaker API calls are accepted by the SDK service, but audible output is not
  working on this host/session yet.
- No local, public G1 API was found for enabling Voice Assistant or Wake-up
  Conversation Mode.

If the goal is to enable the built-in G1 voice assistant, the most likely next
step is still the phone App, the screen controller, or a Unitree-provided
private service/tool not present in this SDK checkout.

## Suggested Runtime Extension Shape

Keep audio integration outside low-level control and deployment internals.

Recommended additions:

- `gear_sonic/vigil_bridge/audio.py` for runtime-facing audio I/O glue.
- `gear_sonic/vigil_bridge/audio_ws.py` for optional WebSocket full-duplex
  audio sessions.
- A microphone receiver that exposes timestamped PCM frames and derived
  telemetry only.
- A speaker client wrapper that exposes calibrated `PlayStream` output with
  `SetVolume(100)` and peak normalization to `27800`.
- HTTP segment fallback endpoints for debugging and non-realtime transfer.

Do not put benchmark semantics, task scoring, hidden success checks, or prompt
logic into the audio runtime adapter.

Open items before production use:

- Decide whether Python dependencies should be installed in a controlled venv,
  or keep audio runtime integration in C++.
- Add health checks for multicast packet rate, RMS/peak, and stale stream
  detection.
- Add explicit shutdown behavior for any audio worker threads.
