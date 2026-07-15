# Vigil Bridge Interface

本文档面向需要接入 GR00T bridge 的 agent 或后续维护者。Bridge 只暴露机器人/仿真运行时能力，不承接 Vigil 的 benchmark、prompt、trace、judge、score 或 oracle 语义。

## 设计边界

调用链：

```text
external agent / Vigil client
  -> HTTP JSON endpoints
  -> BridgeRequestRouter
  -> VigilBridgeService
  -> PrimitiveExecutor + SensorProvider
  -> dry_run / MuJoCo / real runtime adapter
```

核心文件：

| 文件 | 职责 |
| --- | --- |
| `gear_sonic/vigil_bridge/protocol.py` | 协议常量、TypedDict 消息形状、当前公开 capability |
| `gear_sonic/vigil_bridge/transport.py` | HTTP endpoint 到 service 方法的路由 |
| `gear_sonic/vigil_bridge/service.py` | bridge facade；组合 executor、sensor provider，补充统一 telemetry |
| `gear_sonic/vigil_bridge/primitive_executor.py` | dry-run 动作映射、参数校验、基础 primitive 语义 |
| `gear_sonic/vigil_bridge/sensors.py` | dry-run observation / robot_state |
| `gear_sonic/vigil_bridge/mujoco_adapter.py` | MuJoCo ZMQ/DDS/camera adapter |
| `gear_sonic/vigil_bridge/real_adapter.py` | real robot ZMQ/camera adapter，默认禁止真实运动 |
| `gear_sonic_deploy/scripts/run_vigil_bridge.py` | HTTP bridge 启动入口和 backend 配置 |
| `tests/vigil_bridge/` | protocol、transport、adapter 的轻量测试 |

## 当前运行模式

| backend | runtime_mode | 用途 | 运行时连接 | 运动安全默认值 |
| --- | --- | --- | --- | --- |
| `dry_run` | `dry_run` 或请求传入值 | 协议测试、假客户端对接 | 无真实 runtime；返回 fake state/image | 不下发任何 WBC/机器人命令 |
| `mujoco` | `mujoco` | 已启动 MuJoCo/deploy 后的仿真控制 | ZMQ command/planner、ZMQ `g1_debug`、可选 DDS `rt/odostate`、可选 camera ZMQ | 不自动启动控制，除非传 `--auto-start-control` |
| `real` | `real` | 已启动 real deploy 后的机器人控制 | ZMQ command/planner、ZMQ `g1_debug`、可选/必需 camera ZMQ | `--enable-real-motion` 未开启时拒绝运动 |

Bridge 不负责启动 MuJoCo、deploy、policy inference 或硬件流程。

## HTTP Protocol

默认启动：

```bash
python gear_sonic_deploy/scripts/run_vigil_bridge.py --host 127.0.0.1 --port 8765 --backend dry_run
```

所有 POST request/response body 都是 JSON object。未知 endpoint 返回 HTTP 404 + `ok=false`；非法 JSON 返回 HTTP 400。

| Method | Path | Request | Response | 说明 |
| --- | --- | --- | --- | --- |
| `GET` | `/health` | none | health object | transport/service 存活检查 |
| `POST` | `/handshake` | `HandshakeRequest` | `HandshakeResponse` | 返回 protocol、runtime_mode、capabilities |
| `POST` | `/reset_episode` | `ResetEpisodeRequest` | `ResetEpisodeResponse` | 启动 executor 并采样初始 robot_state |
| `POST` | `/execute_action` | `ExecuteActionRequest` | `ExecuteActionResponse` | 同步执行一个 primitive-level action |
| `POST` | `/sonic/planner_command` | SONIC planner payload | `ExecuteActionResponse` | 直接下发 backend-neutral planner command；Agent-Sim real facade 使用 |
| `POST` | `/sonic/reference_motion` | SONIC reference payload | `ExecuteActionResponse` | 直接下发 streamed reference frames；Agent-Sim real facade 使用 |
| `POST` | `/observation` | optional object | `ObservationResponse` | 获取最新 image + robot_state |
| `POST` | `/get_observation` | optional object | `ObservationResponse` | `/observation` alias |
| `POST` | `/robot_state` | optional object | `RobotStateResponse` | 获取 robot_state |
| `POST` | `/get_robot_state` | optional object | `RobotStateResponse` | `/robot_state` alias |
| `POST` | `/halt` | optional object | `RuntimeHealth` | 安全停止/idle 接口 |
| `POST` | `/pause` | optional object | `RuntimeHealth` | 保留 policy/deploy；停止推理并让真机保持 `default_angles` |
| `POST` | `/resume` | optional object | `RuntimeHealth` | 从 paused runtime 重新接入 policy |
| `POST` | `/close` | optional object | close response | 释放 bridge 资源；不要假设会停止 deploy/policy |
| `GET/POST` | `/audio/health` | optional object | audio health object | 音频 I/O 健康检查；未启用时返回结构化不可用 |
| `POST` | `/audio/session/start` | audio session options | audio session response | 启动音频 session；HTTP fallback 和 WebSocket 共用 session manager |
| `POST` | `/audio/session/stop` | audio session options | audio session response | 停止音频 session，可选择停止 input/output |
| `POST` | `/audio/input_segment` | `duration_s`, `format` | base64 PCM/WAV audio | HTTP fallback：读取最近一段 mic PCM |
| `POST` | `/audio/output_segment` | base64 PCM/WAV audio | speaker playback result | HTTP fallback：播放一段 PCM/WAV |
| `POST` | `/audio/tts` | `text`, optional `language`/`speaker_id` | native speaker TTS result | HTTP fallback：通过 G1 `AudioClient.TtsMaker` 播放中英文文本；不经过 PCM 峰值校准 |
| `POST` | `/audio/output_stop` | optional object | speaker stop result | 停止当前 speaker 输出 |

协议常量见 `gear_sonic/vigil_bridge/protocol.py`：

| 字段 | 当前值 |
| --- | --- |
| `protocol_version` | `vigil_groot_bridge_v1` |
| `bridge.name` | `gear_sonic_vigil_bridge` |
| `bridge.version` | `dry_run_phase1` |
| `capabilities.actions` | `navigate.backward`, `navigate.forward`, `navigate.turn_left`, `navigate.turn_right` |
| `capabilities.observation` | `rgb`, `depth`, `robot_state` |
| `capabilities.oracle_source` | `none` |

## Message Shapes

### Handshake

当前实现要求 request 中提供 `runtime_mode`。

Request:

```json
{
  "protocol_version": "vigil_groot_bridge_v1",
  "client": {"name": "vigil", "component": "GrootWBCEnv"},
  "episode_id": "episode_001",
  "runtime_mode": "mujoco",
  "required_capabilities": {
    "actions": ["navigate.forward"],
    "observation": ["rgb", "depth", "robot_state"],
    "oracle_source": "none"
  }
}
```

Response stable fields:

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `ok` | bool | handshake 是否成功 |
| `error_message` | string/null | 错误信息 |
| `protocol_version` | string | bridge protocol version |
| `runtime_mode` | string | 当前 runtime mode |
| `capabilities.actions` | list[string] | 公开动作名 |
| `capabilities.observation` | list[string] | 公开观测类型 |
| `capabilities.audio` | object/absent | 仅当客户端请求音频或服务显式配置时出现 |
| `capabilities.oracle_source` | string | 当前为 `none` |
| `bridge.name` / `bridge.version` | string | bridge 标识 |

### Audio Compatibility

音频能力是协商式扩展，保证兼容老 VLT：

- 老 VLT 按旧请求调用 `/handshake`，不包含 `required_capabilities.audio` 时，`capabilities` 不会出现 `audio` 字段。
- 新 VLT 需要音频时，在 `required_capabilities.audio` 中声明需求，bridge 才返回 `capabilities.audio`。
- 运维也可用 `--audio-advertise-always` 让 bridge 对所有客户端主动暴露音频能力。
- 原有 `/execute_action`、`/observation`、`/robot_state` 调用方式不变。

音频能力示例：

```json
{
  "capabilities": {
    "actions": ["navigate.forward"],
    "observation": ["rgb", "depth", "robot_state"],
    "audio": {
      "enabled": true,
      "input": ["pcm16_16k_mono_stream", "pcm16_16k_mono_segment"],
      "output": ["pcm16_16k_mono_stream", "pcm16_16k_mono_segment", "native_tts_text"],
      "transport": ["websocket", "http_segment_fallback"],
      "sample_rate": 16000,
      "channels": 1,
      "sample_width": 2,
      "speaker_volume": 100,
      "speaker_peak_target": 27800,
      "streaming": {
        "protocol": "output.start/binary/output.end",
        "persistent_runner": true,
        "chunk_ms": 200,
        "prebuffer_ms": 400,
        "send_lead_ms": 20,
        "queue_seconds": 3.0,
        "drain_ms": 150,
        "normalization": "fixed_gain_per_utterance"
      },
      "speaker_led": {
        "enabled": true,
        "source": "outgoing_pcm_amplitude",
        "refresh_hz": 50,
        "native_tts_amplitude_available": false
      },
      "tts": {
        "languages": ["zh", "en"],
        "speaker_ids": {"zh": 0, "en": 1},
        "text_max_chars": 500,
        "native_loudness_calibrated": false,
        "calibrated_output_path": "/audio/output_segment"
      }
    },
    "oracle_source": "none"
  }
}
```

### Execute Action

Request:

```json
{
  "episode_id": "episode_001",
  "step_id": 3,
  "runtime_mode": "mujoco",
  "skill_name": "navigate.forward",
  "arguments": {"magnitude": 1},
  "safety": {"max_speed_mps": 0.5, "timeout_s": 8.0}
}
```

Response stable fields:

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `ok` | bool | runtime action 是否成功 |
| `error_message` | string/null | 失败原因 |
| `action_status` | string | `completed`, `timeout`, `rejected`, `failed`; `interrupted` 预留 |
| `executed_arguments` | object | 实际执行参数；失败或拒绝时可为空 |
| `robot_state_before` | object | 执行动作前采样 |
| `robot_state_after` | object | 动作完成/失败后采样 |
| `telemetry.completion.motion_commanded` | bool | 是否下发过运动命令 |
| `telemetry.completion.capture_timing` | string | `after_completion`, `after_settle`, `after_timeout`, `not_started` 等 |
| `telemetry.completion.settled` | bool | 是否认为动作后已稳定 |

`ok=true` 只表示 bridge/runtime primitive 成功，不表示 benchmark task success。

### SONIC Planner / Reference Motion

这两个端点是 Agent-Sim real facade 与 bridge 之间的 runtime contract，不是新的
benchmark action，也不加入 `capabilities.actions`。两者均受 real backend 的
`motion_enabled`、连接状态和异常自动 `/halt` 保护。

`POST /sonic/planner_command` 示例：

```json
{
  "command": {
    "mode": 2,
    "movement_direction": [1.0, 0.0, 0.0],
    "facing_direction": [1.0, 0.0, 0.0],
    "speed": 0.7,
    "height": -1.0,
    "frame": "robot"
  },
  "duration_s": 0.4,
  "stop_after": true,
  "source": "agent_sim"
}
```

`frame="robot"` 表示机器人当前局部坐标意图。real adapter 会使用独立的 planner
heading origin 把 movement/facing 转到 deploy planner 坐标；不能把该字段当 world
方向直接透传。`duration_s` 控制发包时长，`stop_after=true` 会追加保持当前朝向的
idle burst，因此它是定时开环命令，不是精确距离或角度闭环。

`POST /sonic/reference_motion` 顶层字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `motion_name` | string | 诊断名称，例如 `kimodo_wave` |
| `duration_s` | number | 物理播放时长；可小于 transport frame 总时长 |
| `source` | string/absent | 调用来源 |
| `frames.joint_pos` | list[N][29] | IsaacLab/SONIC reference joint order |
| `frames.joint_vel` | list[N][29] | 与 `joint_pos` 等长 |
| `frames.body_quat_w` | list[N][4*K] | 每帧一个或多个 wxyz quaternion |
| `frames.frame_index` | list[N]/absent | 省略时 bridge 使用 `0..N-1` |

bridge 会先连续发送 `start=true, planner=false` 切到 `STREAMED_MOTION`，再原样运输
reference frames；它不会替上游生成入口/出口曲线或 encoder lookahead。当前 Agent-Sim
payload 使用实测关节状态作为入口首帧，并在物理动作后附加 46 帧中立 lookahead，避免
deploy 的 future-observation window 提前截断动作。

成功响应使用 `ExecuteActionResponse` 的 `ok`、`action_status`、
`executed_arguments` 和 `telemetry`。reference motion 的 `telemetry.frame_count` 是运输
帧数；HTTP 成功只表示 frames 已下发，不表示机器人已经完成物理播放。

### Pause / Resume

`/pause` 是保活型暂停，不等同于 `/halt` 或 `/close`：

- HTTP bridge、policy process、deploy process、Docker container 和 tmux session 保持运行。
- real deploy 收到 ZMQ command `pause=true` 后关闭 planner，清空运动状态，不再执行 policy inference。
- 真机以 INIT 同样的默认姿态窗口回到并保持 `default_angles`，用于人工 reset 场景和机器人。

`/resume` 会重新发送正常 start/planner command。普通 pause/resume 在同一 planner
坐标系中保持当前 facing；若此前调用过 `/sonic/reference_motion`，deploy 会在
`STREAMED_MOTION -> planner` 时重置 heading。bridge 此时必须把恢复瞬间的真实 yaw
设为新的 planner heading origin，并以 `[1, 0, 0]` 保持当前朝向，不能把旧坐标系的
relative yaw 再发送一次。

robot-side launcher 也支持复用：

```bash
./vigil_bridge pause
# reset scene / robot
./vigil_bridge start
```

如果 `./vigil_bridge start` 发现既有 tmux session，会请求 `/resume`，不会重新部署 policy。
streamed 恢复成功后，runtime telemetry 应满足：

```text
streamed_motion_active == false
planner_heading_rebase_count 增加 1
ready_for_motion == true
```

诊断用 yaw origin 不会随 planner rebase 改变，因此 `/robot_state` 的 `yaw_deg` 仍连续，
可以用于测量手势前后的真实净偏航。

### Observation

Response:

```json
{
  "observation_id": "mujoco_obs_0001",
  "runtime_mode": "mujoco",
  "images": {
    "ego_view": {"encoding": "jpeg-base64", "data": "..."},
    "ego_view_depth": {"encoding": "jpeg-base64", "data": "..."}
  },
  "camera_timestamps": {"ego_view": 123.0, "ego_view_depth": 123.0},
  "robot_state": {},
  "telemetry": {},
  "perception": {"source": "none", "detections": []}
}
```

Observation 只提供 runtime-facing 数据。若未来加入估计感知，必须在 payload 中标明 `estimated` 或 `source`，不得写成 groundtruth。

Depth 当前作为 camera stream 的 image entry 透出，命名沿用上游 camera key，例如 RealSense driver 输出 `ego_view` 和 `ego_view_depth`。Bridge 不把 depth 改写成单独顶层字段；对接方应从 `images.<camera>_depth` 和 `camera_timestamps.<camera>_depth` 读取。

### Robot State

通用字段：

| 字段 | 说明 |
| --- | --- |
| `state_id` | provider 生成的状态 id |
| `base_pose` | base 位姿；real 模式可能只有 heading |
| `base_velocity` | base 速度；缺失值用 `null` |
| `joint_positions` | real/MuJoCo 优先返回 `{"order":"mujoco","values":[29 values]}`；状态缺失时为空 object |
| `estimated` | 是否为估计值 |
| `source` | 数据来源，例如 `rt/odostate`, `g1_debug_heading`, `fake` |
| `heading_state` | `base_quat_wxyz`、`delta_heading_rad`、`yaw_rate_rad_s`、`age_s`；仅相关 backend 提供 |

real/MuJoCo state subscriber 优先读取 `g1_debug.body_q_measured`，缺失时回退
`body_q`。关节值保持 MuJoCo order，由需要 IsaacLab order 的上游显式转换。

runtime health/状态响应中的关键 telemetry：

| 字段 | 说明 |
| --- | --- |
| `ready_for_motion` | command/state/camera 与 motion gate 均满足 |
| `streamed_motion_active` | 已进入 streamed reference mode，尚未完成 planner 恢复 |
| `planner_heading_rebase_count` | streamed 恢复时成功重建 planner heading origin 的次数 |

### Audio I/O

Bridge 音频设计参考 camera adapter 的 runtime 边界，但不是 snapshot observation：

```text
G1 mic UDP multicast
  -> AudioSessionManager ring buffer
  -> WebSocket PCM chunks / HTTP input_segment
  -> AGENT/VLT OMNI

AGENT/VLT OMNI PCM chunks
  -> WebSocket / HTTP output_segment
  -> AudioSessionManager speaker queue
  -> G1 AudioClient PlayStream
```

标准音频格式：

- 16 kHz
- mono
- signed PCM16 little-endian
- speaker API volume: `100`
- runtime PCM peak target: `27800`

Streaming 是主路径。几十秒整段音频只作为 fallback/debug，因为它至少会增加“录满音频 + 上传/解码 + 播放排队”的延迟，不适合实时对话。

输出 streaming 使用显式 utterance 生命周期。连接建立后先发送：

```json
{"type":"output.start","utterance_id":"utt-123","normalize":true}
```

随后发送任意数量的 binary PCM16 frame，最后发送：

```json
{"type":"output.end","utterance_id":"utt-123"}
```

Bridge 会先做有界预缓冲，再把上游小 frame 聚合成固定播放块。同一 utterance
只启动一次 persistent runner、只创建一个 G1 `stream_id`，并且只在 `output.end`
排空队列后调用一次 `PlayStop`。每个 binary frame 返回 `output.result`；当
`payload.backpressure=true` 时，上游必须暂停发送并重试，bridge 不会静默丢音频。

控制消息语义：

| message | 语义 |
| --- | --- |
| `output.start` | 创建一个输出 utterance；同一时间只允许一个 |
| binary PCM | 加入当前 utterance 的有界队列 |
| `output.end` | 正常排空，`PlayStop`，执行一次 LED 收尾 |
| `output.stop` | 立即清队列并停止，不等待正常收尾 |
| `session.stop` | 停止输入和输出并关闭当前 audio session |

未发送 `output.start` 的单个 binary message 仍走旧 one-shot 兼容路径，不具备跨
message 无缝保证。Streaming 默认使用首个 400 ms prebuffer 计算一次固定 gain，
后续 chunk 沿用该 gain，避免逐 chunk peak normalization 造成音量抽动。

启用 `--audio-speaker-reactive-led` 后，PCM/WAV 播放期间由同一 runner 根据输出振幅
以 50 Hz 驱动橙黄灯效，结束后过渡到蓝色。该功能要求 LED-aware runner；native
`TtsMaker` 不提供合成 PCM，所以 `/audio/tts` 不具备精确的振幅联动。

Native TTS 是独立的 HTTP fallback 输出，不经过 OMNI PCM 流。Host/VLT 发送文本后，
bridge 在 G1 侧调用 `AudioClient.TtsMaker(text, speaker_id)`。默认
`segmentation=auto` 会把短英文 token 保留在相邻中文段里，减少多次 native
TTS 调用造成的停顿；需要强制按中英文切段时，请在 `/audio/tts` payload 中传
`"segmentation":"strict"`。默认 speaker 映射：

| language | speaker_id |
| --- | --- |
| `zh` | `0` |
| `en` | `1` |

示例：

```bash
curl -s -X POST http://<robot-host>:8765/audio/tts \
  -H 'Content-Type: application/json' \
  -d '{"text":"你好，这是 G1 运行时 TTS 测试。","language":"zh"}'

curl -s -X POST http://<robot-host>:8765/audio/tts \
  -H 'Content-Type: application/json' \
  -d '{"text":"Hello, this is a G1 runtime TTS test.","language":"en"}'
```

注意：native TTS 不经过 `speaker_peak_target=27800` 的 PCM 归一化。实测
`SetVolume(100)` 下仍明显小于 `PlayStream` 路径。需要和当前 speaker
校准响度一致时，VLT/OMNI 侧应先把文本合成为 16 kHz mono PCM16，再调用
`/audio/output_segment` 或 WebSocket binary output；bridge 会按 `27800`
峰值归一化后通过 `PlayStream` 播放。

Bridge 侧启动示例：

```bash
python gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend real \
  --host 0.0.0.0 \
  --port 8765 \
  --audio-enabled \
  --audio-ws \
  --audio-ws-port 8766 \
  --audio-mic-interface-ip 192.168.123.164 \
  --audio-speaker-iface enP8p1s0 \
  --audio-speaker-volume 100 \
  --audio-speaker-stream-chunk-ms 200 \
  --audio-speaker-stream-prebuffer-ms 400 \
  --audio-speaker-stream-send-lead-ms 20 \
  --audio-speaker-stream-queue-s 3 \
  --audio-speaker-stream-drain-ms 150 \
  --audio-speaker-runner /home/unitree/g1_audio_tests/vigil_led_speaker/build/g1_vigil_led_speaker_runner \
  --audio-speaker-reactive-led
```

Host/VLT 侧 WebSocket smoke test:

```bash
python -m gear_sonic.vigil_bridge.audio_ws_client \
  --url ws://<robot-host>:8766/audio/ws \
  --listen-seconds 5 \
  --record-wav vlt_mic_check.wav \
  --send-tone
```

也可以发送已准备好的 16 kHz mono PCM16 WAV:

```bash
python -m gear_sonic.vigil_bridge.audio_ws_client \
  --url ws://<robot-host>:8766/audio/ws \
  --listen-seconds 5 \
  --record-wav vlt_mic_check.wav \
  --send-wav output_16k_mono_pcm16.wav
```

## 当前能力表

### Actions

| Public `skill_name` | Arguments | Safety keys | dry_run | MuJoCo | real | 主要实现 |
| --- | --- | --- | --- | --- | --- | --- |
| `navigate.forward` | `distance_m` 或 `magnitude` | `max_speed_mps`/`speed_mps`, `timeout_s` | `move_model(+distance)` | `move_model(+distance)` | `move_open_loop(+distance)` | `primitive_executor.py`, `mujoco_adapter.py`, `real_adapter.py` |
| `navigate.backward` | `distance_m` 或 `magnitude` | `max_speed_mps`/`speed_mps`, `timeout_s` | `move_model(-distance)` | `move_model(-distance)` | `move_open_loop(-distance)` | 同上 |
| `navigate.turn_left` | `degrees`/`angle_deg` 或 `magnitude` | `max_rate_deg_s`/`rate_deg_s`, `timeout_s` | `rotate(+degrees)` | `rotate(+degrees)` | `rotate(+degrees)` | 同上 |
| `navigate.turn_right` | `degrees`/`angle_deg` 或 `magnitude` | `max_rate_deg_s`/`rate_deg_s`, `timeout_s` | `rotate(-degrees)` | `rotate(-degrees)` | `rotate(-degrees)` | 同上 |

备注：`DryRunPrimitiveExecutor.execute_action()` 中存在 `report` 分支，但它没有出现在 `SUPPORTED_ACTIONS`/handshake 中。对外 agent 不应依赖它；如需公开，按新增 action 流程登记并补全 backend telemetry。

### Observations

| Public observation | Payload location | dry_run | MuJoCo | real | 主要实现 |
| --- | --- | --- | --- | --- | --- |
| `rgb` | `ObservationResponse.images` | fake `ego_view` | 可选 camera ZMQ，`jpeg-base64` | camera ZMQ，默认 real camera required | `sensors.py`, `mujoco_adapter.py`, `real_adapter.py` |
| `depth` | `ObservationResponse.images.<camera>_depth` | fake `ego_view_depth` | camera stream 如包含 depth key 则透出 | RealSense/OAK 等 camera stream 如包含 depth key 则透出 | `gear_sonic/camera/drivers/realsense.py`, provider normalize methods |
| `robot_state` | `ObservationResponse.robot_state` / `/robot_state` | fake pose/velocity | DDS odom 优先，否则 `g1_debug` heading | `g1_debug` heading，位置可为 `null` | 同上 |

## 扩展登记表

新增 bridge 能力时，先在表中登记，再改代码和测试。

| 类型 | Public name | Request keys | Response/telemetry keys | 需修改文件 | 测试文件 | 状态 |
| --- | --- | --- | --- | --- | --- | --- |
| action | `navigate.forward` | `arguments.distance_m/magnitude`, `safety.max_speed_mps`, `safety.timeout_s` | `executed_arguments.distance_m`, `telemetry.completion`, optional `motion_result` | `protocol.py`, `primitive_executor.py`, `mujoco_adapter.py`, `real_adapter.py` | `tests/vigil_bridge/test_vigil_bridge_service.py`, adapter tests | current |
| action | `navigate.turn_left` | `arguments.degrees/angle_deg/magnitude`, `safety.max_rate_deg_s`, `safety.timeout_s` | `executed_arguments.degrees`, `telemetry.completion`, optional `motion_result` | 同上 | 同上 | current |
| endpoint | `/sonic/planner_command` | `command`, `duration_s`, `stop_after` | `executed_arguments.command`, planner telemetry | `transport.py`, `service.py`, `mujoco_adapter.py`, `real_adapter.py` | transport + adapter tests | current |
| endpoint | `/sonic/reference_motion` | `motion_name`, `duration_s`, `frames` | `executed_arguments.motion_name`, frame telemetry | 同上 | transport + adapter tests | current |
| observation | `rgb` | none | `images.<camera>.encoding`, `images.<camera>.data`, `camera_timestamps` | `protocol.py`, provider normalize methods | adapter/provider tests | current |
| observation | `depth` | none | `images.<camera>_depth.encoding`, `images.<camera>_depth.data`, `camera_timestamps.<camera>_depth` | `protocol.py`, camera payload normalize methods | adapter/provider tests | current |
| observation | `robot_state` | none | `robot_state.base_pose`, `robot_state.base_velocity`, `robot_state.estimated`, `robot_state.source` | `protocol.py`, provider state methods | adapter/provider tests | current |
| endpoint | `/audio/*` | session/audio payload, `text` for `/audio/tts` | audio health, base64 PCM/WAV, TTS result, speaker telemetry | `audio.py`, `audio_ws.py`, `service.py`, `transport.py`, `run_vigil_bridge.py` | `tests/vigil_bridge/test_vigil_bridge_audio.py` | current |
| action | `<new.skill>` | `<arguments.*>`, `<safety.*>` | `<executed_arguments.*>`, `telemetry.completion`, optional `motion_result` | `protocol.py`, executor/backend files, maybe `run_vigil_bridge.py` | service + backend tests | proposed |
| observation | `<new_observation>` | optional request keys | `ObservationResponse.<new field>` or `telemetry.<source>` | `protocol.py`, sensor providers, maybe backend config | provider tests | proposed |
| endpoint | `/<new_endpoint>` | JSON object | JSON object with `ok/error_message` when applicable | `transport.py`, `service.py`, `protocol.py` | transport tests | proposed |

## 新增 Action 范式

1. 在 `gear_sonic/vigil_bridge/protocol.py` 的 `SUPPORTED_ACTIONS` 增加 public `skill_name`。
2. 在 `DryRunPrimitiveExecutor.execute_action()` 增加参数解析和 fake telemetry；不要调用真实 runtime。
3. 如果 action 对应已有 primitive，复用 `move()`/`rotate()` 这类 executor 方法；否则在 executor 中新增 primitive 方法。
4. 在 `MujocoPrimitiveExecutor` 和 `RealPrimitiveExecutor` 中实现同名 primitive 或覆盖必要逻辑。
5. real mode 必须保持：`motion_enabled` gate、保守速度/超时、异常时 `halt()`、缺 state/camera/command 时 fail closed。
6. 返回统一 `ExecuteActionResponse`：`action_status`、`executed_arguments`、`telemetry.completion` 必须稳定。
7. 在 `tests/vigil_bridge/` 增加 dry-run service 测试、HTTP/transport 测试，以及 MuJoCo/real fake runtime 测试。

## 新增 Observation 范式

1. 在 `SUPPORTED_OBSERVATIONS` 增加 public observation 名称。
2. 在 `FakeSensorProvider`、`MujocoSensorProvider`、`RealSensorProvider` 中补齐 payload 或明确返回空/不可用 telemetry。
3. 如果需要 runtime 连接，在对应 `*BridgeConfig` 中加配置项，并在 `run_vigil_bridge.py` 暴露 CLI 参数。
4. 所有 image/data payload 保持 JSON 兼容；二进制数据使用 base64。
5. 估计值必须标注 `estimated`/`source`；bridge 不输出 benchmark oracle 或 hidden success。
6. 增加 provider normalization 测试和缺连接时的结构化错误测试。

## 新增 Backend 范式

1. 新增 `<Backend>BridgeConfig`、`<Backend>RuntimeClient`、`<Backend>PrimitiveExecutor`、`<Backend>SensorProvider`。
2. 提供 `create_<backend>_bridge_service(config)`，返回 `VigilBridgeService(executor=..., sensor_provider=...)`。
3. 在 `run_vigil_bridge.py` 增加 `--backend` choice 和配置参数。
4. 不导入 Vigil；不修改 low-level control、deploy internals、policy inference 或 model path。
5. 补 fake runtime tests，覆盖 handshake 不启动 runtime、动作映射、halt/close 语义、observation normalization。

## Agent 对接流程

1. `GET /health` 确认服务在线。
2. `POST /handshake` 检查 `protocol_version` 和 `capabilities`。
3. `POST /reset_episode` 初始化 runtime-facing session。
4. 循环调用 `POST /observation` 和 `POST /execute_action`。
5. 出错或结束时先 `POST /halt`，再按需 `POST /close`。

对接方只应使用 handshake 中公开的 capability。unsupported action 应被视为 `rejected`，不要在 agent 侧假设 bridge 会静默忽略。

## 轻量验证

```bash
python -m compileall gear_sonic/vigil_bridge
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider tests/vigil_bridge
```
