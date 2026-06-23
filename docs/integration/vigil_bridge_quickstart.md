# Vigil Bridge 入门指南

这份文档用于在机器人侧快速试运行 GR00T Vigil bridge。

**第一次使用必须由 @lizj18 在场协助运行。不要单独执行真实机器人动作命令。**

## 1. SSH 连接机器人

优先使用机器人当前 Wi-Fi IP：

```bash
ssh unitree@192.168.1.113
```

如果 Wi-Fi 不可用，可尝试默认有线地址：

```bash
ssh unitree@192.168.123.164
```

进入 Vigil 仓库：

```bash
cd ~/GR00T-WholeBodyControl_Vigil
```

## 2. 一键启动 Policy 和 Bridge

启动真实部署 policy、Vigil 专用相机服务和 HTTP bridge：

```bash
./vigil_bridge start --max-speed-mps 2 --camera-required --attach
```

如需在同一次机器人调试会话中一起启动 Audio + TTS bridge，先设置 G1
speaker runner 路径，然后在 `./vigil_bridge start` 上追加 audio 参数：

```bash
RUNNER=/home/unitree/g1_audio_tests/speaker_loud_music/build/g1_speaker_loud_music_runner

./vigil_bridge start \
  --bridge-host 0.0.0.0 \
  --max-speed-mps 2 \
  --camera-required \
  --with-tts \
  --audio-advertise-always \
  --audio-ws \
  --audio-ws-host 0.0.0.0 \
  --audio-ws-port 8766 \
  --audio-mic-interface-ip 192.168.123.164 \
  --audio-speaker-iface enP8p1s0 \
  --audio-speaker-runner "$RUNNER" \
  --attach
```

`--with-tts` 是 `--audio-enabled` 的别名；TTS endpoint 是
`/audio/tts`。真实 G1 喇叭/TTS 输出需要 `--audio-speaker-runner`。

常用管理命令：

```bash
./vigil_bridge status
./vigil_bridge logs
./vigil_bridge attach
./vigil_bridge stop
```

说明：launcher 只会在本次 Vigil 会话中启动 `composed_camera_server_vigil.service`，
不会禁用或替换旧的相机服务。

## 3. Curl 命令集

在客户端机器上设置机器人 bridge 地址：

```bash
ROBOT=http://192.168.1.113:8765
```

健康检查：

```bash
curl -s "$ROBOT/health" | python3 -m json.tool
curl -s "$ROBOT/audio/health" | python3 -m json.tool
```

初始化 runtime：

```bash
curl -s "$ROBOT/reset_episode" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | python3 -m json.tool
```

获取观测：

```bash
curl -s "$ROBOT/observation" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | python3 -m json.tool
```

在 Mac 上保存 `ego_view` 图像：

```bash
mkdir -p /Users/lizj/Pictures/debug && \
curl -s "$ROBOT/observation" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | \
python3 -c 'import sys,json,base64,datetime,os
p="/Users/lizj/Pictures/debug"
payload=json.load(sys.stdin)
entry=payload["images"]["ego_view"]
b64=entry["data"] if isinstance(entry,dict) else entry
out=os.path.join(p,"ego_view_"+datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+".jpg")
open(out,"wb").write(base64.b64decode(b64))
print(out)'
```

获取机器人状态：

```bash
curl -s "$ROBOT/robot_state" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | python3 -m json.tool
```

### Audio / TTS

TTS：Host/VLT 只发送文本，G1 侧负责发声。默认 `segmentation=auto` 会把短
英文 token 保留在中文段里，避免中英文词之间出现多次 runner 启动导致的长停顿。

```bash
curl -s "$ROBOT/audio/tts" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"text":"你好 G1 hello 世界，这是 mixed TTS 测试。"}' \
  | python3 -m json.tool
```

如果需要强制按中英文切段，可显式使用 strict：

```bash
curl -s "$ROBOT/audio/tts" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"text":"你好 G1 hello 世界","segmentation":"strict"}' \
  | python3 -m json.tool
```

停止当前 speaker 输出：

```bash
curl -s "$ROBOT/audio/output_stop" \
  -X POST -H 'Content-Type: application/json' \
  -d '{}' | python3 -m json.tool
```

Speaker calibrated output smoke test：生成 1 秒 440 Hz PCM16 tone，通过
`/audio/output_segment` 播放。执行前确认人员远离机器人扬声器。

```bash
python3 - <<'PY' | curl -s "$ROBOT/audio/output_segment" \
  -X POST -H 'Content-Type: application/json' \
  --data-binary @- | python3 -m json.tool
import base64
import json
import math
import struct

sample_rate = 16000
duration_s = 1.0
frequency_hz = 440.0
peak = 12000
samples = []
for index in range(int(sample_rate * duration_s)):
    phase = 2.0 * math.pi * frequency_hz * index / sample_rate
    samples.append(int(round(math.sin(phase) * peak)))
pcm = struct.pack("<" + "h" * len(samples), *samples)
print(json.dumps({
    "encoding": "pcm16-base64",
    "data": base64.b64encode(pcm).decode("ascii"),
    "normalize": True,
}))
PY
```

Microphone HTTP segment smoke test：启动 audio input session，等待 G1 mic
multicast 数据进入 ring buffer，然后保存最近 2 秒为 WAV。

```bash
curl -s "$ROBOT/audio/session/start" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"input":true,"output":false,"transport":"http"}' \
  | python3 -m json.tool

sleep 2

curl -s "$ROBOT/audio/input_segment" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"duration_s":2.0,"format":"wav"}' \
  | python3 -c 'import sys,json,base64
payload=json.load(sys.stdin)
assert payload["ok"], payload
open("g1_mic_segment.wav","wb").write(base64.b64decode(payload["data"]))
print("g1_mic_segment.wav")'

curl -s "$ROBOT/audio/session/stop" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"stop_input":true,"stop_output":false}' \
  | python3 -m json.tool
```

WebSocket 全双工 smoke test，适合 VLT/OMNI 接入前确认 microphone + speaker
同时工作：

```bash
ROBOT_IP=192.168.1.113

python3 -m gear_sonic.vigil_bridge.audio_ws_client \
  --url ws://$ROBOT_IP:8766/audio/ws \
  --listen-seconds 5 \
  --record-wav vlt_mic_check.wav \
  --send-tone
```

小幅前进，仅限 @lizj18 在场时使用：

```bash
curl -s "$ROBOT/execute_action" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real","skill_name":"navigate.forward","arguments":{"distance_m":0.10},"safety":{"max_speed_mps":0.15,"timeout_s":5}}' \
  | python3 -m json.tool
```

小幅左转，仅限 @lizj18 在场时使用：

```bash
curl -s "$ROBOT/execute_action" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real","skill_name":"navigate.turn_left","arguments":{"degrees":10},"safety":{"rate_deg_s":10,"timeout_s":5}}' \
  | python3 -m json.tool
```

小幅右转，仅限 @lizj18 在场时使用：

```bash
curl -s "$ROBOT/execute_action" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real","skill_name":"navigate.turn_right","arguments":{"degrees":10},"safety":{"rate_deg_s":10,"timeout_s":5}}' \
  | python3 -m json.tool
```

安全停止：

```bash
curl -s "$ROBOT/halt" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | python3 -m json.tool
```

## 4. VLT 语音接入建议阅读顺序

VLT/OMNI 侧接入语音功能时，建议按下面顺序看：

1. `docs/integration/vigil_bridge_quickstart.md`：G1 侧 `./vigil_bridge`
   启动命令和 Host/VLT smoke tests。
2. `docs/integration/vigil_bridge_interface.md`：正式 HTTP/WebSocket
   endpoint、handshake audio capability、TTS payload 字段。
3. `docs/integration/g1_audio_bridge_connectivity.md`：G1/Host-VLT
   双端联调记录、常见失败和端口/runner 检查。
4. `gear_sonic/vigil_bridge/audio_ws_client.py`：最小 WebSocket
   全双工客户端示例。
5. `gear_sonic/vigil_bridge/audio_tts_client.py`：最小 HTTP TTS / speaker
   fallback 客户端示例。
