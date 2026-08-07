# FaceE E0019 真机 sit-chair policy 可复现流程

> 更新时间：2026-08-07
>
> 适用分支：`r2s_ego_exp_fangs`
>
> 固定版本：`1df2f0df0e54675ce520c0e698ea256d558a65ea`
>
> 本文采用标准 `./vigil_bridge start` 路径，适合直接复制到飞书。

## 1. 目的与安全边界

本文记录 2026-08-03 使用 FaceE E0019 exact-v3 policy 进行真机
`sonic.sit_chair` 测试的完整启动、检查、动作和停止流程。

> **真机安全警告**
>
> - 启动 real actor 本身就会发布 LowCmd，并在约 3 秒内进入 INIT 目标姿态；
>   它不是“只把模型加载到显存、机器人完全不动”。
> - 使用 `--initial-chair-reference d1p50` 时，INIT/PAUSE 目标是 d1p50
>   reference 的第 0 帧，不是普通站立姿态。
> - 必须先完成吊装或保护架、固定凳子、物理急停、安全员和机器人周边清场。
> - 紧急情况优先使用现场物理急停；`/halt` 是软件安全接口，但不能替代物理急停。
> - 策略包的 manifest 仍保留 `real_robot_authorized=false`。历史真机实验是在现场
>   明确授权后执行的，不自动构成后续实验的安全放行。

## 2. 上次实际使用的组件

### 2.1 Policy

```text
gear_sonic_deploy/policy/facee_e0019_exact_v3/
├── model_encoder.onnx
├── model_decoder.onnx
├── observation_config.yaml
├── manifest.json
├── encoder_model_encoder.trt
└── policy_model_decoder.trt
```

Policy 接口：

- encoder：`1 × 1751 -> 1 × 64`
- decoder：`1 × 994 -> 1 × 29`
- 单一共享 actor，没有距离 selector
- 不使用 height map、vision 或 chair pose 输入

模型 SHA-256：

```text
model_encoder.onnx
1cf476669348344f9ccda0049fb5b708112bdcada73e9403f227321872e7e459

model_decoder.onnx
5464bd07f7877b9c42d38cb2a9fc73ad601b857941604e8d3e372298e67d7382

observation_config.yaml
a88305b8d12994d831a232619bdc64e58a5378840e4054fd2497d9ad7ebdb33a
```

### 2.2 Sit-chair reference catalog

```text
gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json
```

Catalog manifest SHA-256：

```text
1217669a9457240d65bbe255f296ab9327cf622650f625ad4d9b6b5b8e7ad016
```

Catalog 包含 18 条 reference：

- 距离网格：`1.15, 1.20, ..., 2.00 m`
- 每条：650 帧
- 控制频率：50 Hz
- 动作时长：约 13 秒
- 输入支持范围：`[1.10, 2.00] m`
- 选择规则：向上取最近的 5 cm reference

示例：

```text
1.12 -> 1.15
1.17 -> 1.20
1.50 -> 1.50
1.96 -> 2.00
```

### 2.3 Runtime 和端口

```text
TensorRT root     /home/unitree/TensorRT-10.7.0.23
Robot NIC         enP8p1s0
Robot NIC IP      192.168.123.164/24（上机前重新确认）
Container         g1-deploy-dev
tmux session      vigil_facee_e0019
Bridge HTTP       127.0.0.1:8767
ZMQ command       127.0.0.1:5556
ZMQ state         127.0.0.1:5557
Camera            127.0.0.1:5555
Camera service    composed_camera_server_vigil.service
```

## 3. 完整调用链

```text
./vigil_bridge
  -> host launcher
  -> Docker g1-deploy-dev
  -> deploy.sh real / enP8p1s0
  -> g1_deploy_onnx_ref
  -> TensorRT encoder + decoder + planner
  -> d1p50 frame 0 INIT/PAUSE hold
  -> 等待 Init Done
  -> real Vigil Bridge，HTTP 8767
  -> 自动 /reset_episode
  -> start=True, planner=False, hold=True
  -> 保持 d1p50 frame 0，不推进 reference
  -> /execute_action sonic.sit_chair
  -> planner=false + protocol-v1 pose
  -> 播放选中的 650-frame reference
```

`--initial-chair-reference d1p50` 启用后：

1. launcher 从 exact-v3 catalog 找到 `d1p50.npz`；
2. 自动转换成 C-contiguous、无压缩、C++ `cnpy` 可读取的临时 NPZ；
3. 把临时文件复制进容器；
4. actor 在 INIT 阶段平滑进入 d1p50 第 0 帧；
5. PAUSE 也保持该姿态；
6. 首次 decoder history 缺口使用第 0 帧预填，速度和历史 action 置零；
7. 自动 reset 进入 streamed-motion hold，但不推进 reference；
8. 收到 `sonic.sit_chair` 后才从第 0 帧开始播放。

## 4. 上机前只读检查

### 4.1 仓库版本

```bash
cd /home/unitree/GR00T-WholeBodyControl_Vigil

git branch --show-current
git rev-parse HEAD
```

预期：

```text
r2s_ego_exp_fangs
1df2f0df0e54675ce520c0e698ea256d558a65ea
```

### 4.2 设置 catalog 路径

路径必须放在同一行。不要在引号内部把 `/data/` 和
`facee_chair_13s_exact_v3` 拆到两行。

```bash
FACEE_CHAIR_CATALOG="$PWD/gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json"

test -f "$FACEE_CHAIR_CATALOG" && echo "catalog OK"
```

### 4.3 校验模型和配置

```bash
sha256sum \
  gear_sonic_deploy/policy/facee_e0019_exact_v3/model_encoder.onnx \
  gear_sonic_deploy/policy/facee_e0019_exact_v3/model_decoder.onnx \
  gear_sonic_deploy/policy/facee_e0019_exact_v3/observation_config.yaml \
  "$FACEE_CHAIR_CATALOG"
```

输出应与第 2 节的 SHA-256 完全一致。

### 4.4 校验 TensorRT 和机器人网卡

```bash
test -f /home/unitree/TensorRT-10.7.0.23/include/NvInfer.h && \
  echo "TensorRT include OK"

test -d /home/unitree/TensorRT-10.7.0.23/lib && \
  echo "TensorRT lib OK"

ip -4 addr show dev enP8p1s0
```

必须确认 `enP8p1s0` 是当前机器人控制网卡，不要依赖 `real` 模式的 fallback。

### 4.5 校验相机设备

```bash
lsusb | grep -Ei 'RealSense|Intel'
```

如果 D435i 没有出现在 USB 枚举里，应先恢复设备连接，不能用
`--real-camera-optional` 绕过。

## 5. 停止残留会话

启动前先停止同名 session，防止两套 actor 同时占用 DDS 和 ZMQ：

```bash
cd /home/unitree/GR00T-WholeBodyControl_Vigil

./vigil_bridge \
  --session vigil_facee_e0019 \
  --container-name g1-deploy-dev \
  stop \
  --bridge-port 8767
```

如果输出：

```text
no server running on /tmp/tmux-1000/default
```

通常只表示没有旧 tmux server，不是模型或机器人故障。

## 6. 标准自动 reset 启动命令

> 执行下面命令会启动真机 actor。actor 的 INIT 阶段可能立即产生关节运动。

```bash
cd /home/unitree/GR00T-WholeBodyControl_Vigil

FACEE_CHAIR_CATALOG="$PWD/gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json"

./vigil_bridge \
  --session vigil_facee_e0019 \
  --container-name g1-deploy-dev \
  start \
  --tensorrt-root /home/unitree/TensorRT-10.7.0.23 \
  --robot-interface enP8p1s0 \
  --input-type zmq_manager \
  --output-type all \
  --zmq-host 127.0.0.1 \
  --policy-checkpoint policy/facee_e0019_exact_v3/model \
  --policy-observation-config policy/facee_e0019_exact_v3/observation_config.yaml \
  --bridge-host 127.0.0.1 \
  --bridge-port 8767 \
  --command-bind-host 127.0.0.1 \
  --command-port 5556 \
  --state-host 127.0.0.1 \
  --state-port 5557 \
  --state-timeout 10 \
  --max-speed-mps 0.2 \
  --camera-host 127.0.0.1 \
  --camera-port 5555 \
  --chair-motion-catalog "$FACEE_CHAIR_CATALOG" \
  --camera-required \
  --camera-service-timeout 60 \
  --initial-chair-reference d1p50
```

说明：

- 不要添加 `--no-camera-service`；launcher 应负责启动 Vigil 专用相机服务。
- 保留 `--camera-required`，让相机缺失时 fail closed。
- 上次后期为调试 `navigate.forward` 曾把 `--max-speed-mps` 改成 `1`；坐椅动作不走
  navigation move-model，本文固定使用更保守的 `0.2`。
- 不要 `source tools/setup_facee_cpp_runtime.sh`。该脚本用于仓库 MuJoCo/x86_64
  runtime；真机标准 launcher 使用 Docker 和 `/home/unitree/TensorRT-10.7.0.23`。
- 不要使用 `tools/run_facee_e0019_repo_mujoco.sh`。它强制使用 loopback 和 MuJoCo。

如果只想锁住 Bridge 动作 API，可在启动命令末尾添加：

```text
--no-real-motion --no-auto-start-control --no-reset-after-start
```

但这三个参数只锁 Bridge 的 reset/action 路径，不能保证 C++ actor 的 INIT 阶段
完全没有关节运动。

## 7. 判断 policy 是否真正加载完成

### 7.1 查看 policy 日志

```bash
tail -f /tmp/vigil_bridge_launcher/vigil_facee_e0019/policy.log
```

成功门槛：

```text
decoder input 994 / output 29
encoder input 1751 / output 64
ZMQ command 5556 initialized
ZMQ state 5557 initialized
Init Done
```

首次编译 actor 或首次从 ONNX 构建 TensorRT engine 可能耗时数分钟。Bridge 会一直
等待 `policy.log` 中出现 `Init Done`，在此之前 8767 不会正常提供完整服务。

### 7.2 使用正确端口检查 launcher

```bash
./vigil_bridge \
  --session vigil_facee_e0019 \
  --container-name g1-deploy-dev \
  status \
  --bridge-port 8767
```

不要省略 `--bridge-port 8767`；`status` 默认检查 8765。

预期：

```text
tmux session vigil_facee_e0019: running
container g1-deploy-dev: running
bridge health: { ... }
camera service composed_camera_server_vigil.service: active
```

### 7.3 检查 Bridge health

```bash
curl -sS http://127.0.0.1:8767/health | python3 -m json.tool
```

执行动作前至少确认：

```text
ok = true
executor_started = true
state_connected = true
camera_connected = true
motion_enabled = true
ready_for_motion = true
startup_reference_hold = true
```

### 7.4 检查 RGB 和 Depth

```bash
curl -sS http://127.0.0.1:8767/observation \
  -X POST \
  -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' |
python3 -c 'import json,sys; d=json.load(sys.stdin); print("images:", list((d.get("images") or {}).keys()))'
```

预期至少包含：

```text
ego_view
ego_view_depth
```

## 8. 可选：动作前 sit-chair preflight

该接口只做检查和预置 rollout，不发送坐椅动作：

```bash
curl -sS http://127.0.0.1:8767/diagnostics/sit_chair/preflight \
  -X POST \
  -H 'Content-Type: application/json' \
  -d '{
    "runtime_mode": "real",
    "chair_distance_m": 1.50,
    "arm_rollout": true
  }' | python3 -m json.tool
```

## 9. 执行 d1p50 坐椅动作

> 下面命令会立即下发约 13 秒的真机坐椅 reference。

启动 hold 固定使用 d1p50，因此首次复现优先使用实测 `1.50 m`，确保启动姿态与
实际动作的第 0 帧完全一致。

```bash
curl -sS --max-time 30 \
  http://127.0.0.1:8767/execute_action \
  -X POST \
  -H 'Content-Type: application/json' \
  -d '{
    "episode_id": "facee_real_test",
    "step_id": 1,
    "runtime_mode": "real",
    "skill_name": "sonic.sit_chair",
    "arguments": {
      "chair_distance_m": 1.50
    },
    "safety": {}
  }' | python3 -m json.tool
```

成功的 HTTP 返回至少应包含：

```text
ok = true
action_status = completed
chair_distance_m = 1.50
reference_distance_m = 1.50
tag = d1p50
frame_count = 650
```

HTTP 返回 `completed` 表示命令链和时间窗口完成，不替代现场对机器人接触、稳定性和
落座结果的人工判断。

## 10. 停止与关闭

### 10.1 软件 halt

```bash
curl -sS http://127.0.0.1:8767/halt \
  -X POST \
  -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | python3 -m json.tool
```

### 10.2 关闭 actor、Bridge、tmux 和容器

```bash
cd /home/unitree/GR00T-WholeBodyControl_Vigil

./vigil_bridge \
  --session vigil_facee_e0019 \
  --container-name g1-deploy-dev \
  stop \
  --bridge-port 8767
```

launcher 的 stop 流程会先尝试 `/halt`，再终止 Bridge 和 actor。不要假设单独调用
`/close` 能停止 policy/deploy。

## 11. 上次遇到的问题与处理方式

### 11.1 Catalog 明明存在，但提示 not found

原因：长路径在 `/data/` 后被换行拆开，或双引号中保留了换行和空格。

正确方式：

```bash
FACEE_CHAIR_CATALOG="$PWD/gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json"
test -f "$FACEE_CHAIR_CATALOG" && echo "catalog OK"
```

### 11.2 `status` 显示 bridge unavailable

先确认使用了：

```text
--bridge-port 8767
```

如果 8767 仍不可用，检查：

```bash
tail -n 200 /tmp/vigil_bridge_launcher/vigil_facee_e0019/policy.log
tail -n 200 /tmp/vigil_bridge_launcher/vigil_facee_e0019/bridge.log
```

Bridge 必须等到 policy 日志出现 `Init Done` 才启动。

### 11.3 TensorRT engine 无法反序列化

上次 native TensorRT 10.3 生成的缓存不能在 Docker TensorRT 10.7 中使用。

不要跨 TensorRT/JetPack 环境复制 `.trt`。出现版本不兼容时：

1. 先停止 launcher；
2. 保留 ONNX；
3. 把不兼容的 `.trt` 改名备份，不要删除；
4. 重新启动，让当前 Docker 环境从 ONNX 构建匹配的 engine。

上次保留的备份名：

```text
encoder_model_encoder.trt.incompatible-native-trt103-20260803
policy_model_decoder.trt.incompatible-native-trt103-20260803
```

### 11.4 `load_the_npy_file: failed fread`

原因：原始 d1p50 NPZ 使用 Deflate/ZIP64，且数组为 Fortran-order，C++ `cnpy`
不能直接读取。

修复已包含在 `1df2f0d`：launcher 自动生成 C-contiguous、无压缩临时 NPZ。若再次
看到此错误，优先确认分支/提交和实际使用的 launcher，不要手工改原始 reference。

### 11.5 相机 service active，但 5555 不可用

上次出现过两种情况：

1. D435i 启动超过 launcher 默认 15 秒；
2. RealSense 从 USB 枚举中掉线，systemd 进入重启循环。

检查：

```bash
systemctl status composed_camera_server_vigil.service --no-pager -l
journalctl -u composed_camera_server_vigil.service -n 100 --no-pager
lsusb | grep -Ei 'RealSense|Intel'
ss -lntp | grep ':5555'
```

本文固定使用：

```text
--camera-required --camera-service-timeout 60
```

不要用 `--real-camera-optional` 绕过真机动作的相机门禁。

## 12. 历史真机证据

2026-08-03 的代表性完整 rollout：

```text
outputs/vigil_rollouts/20260803T133200_474161Z_sonic_sit_chair_auto/
```

数据摘要：

```text
runtime_mode      real
hardware          true
action frames     650
pre-roll frames   150
post-roll frames  150
state frames      950
RGB frames        433
Depth frames      433
recording         18.98 s
```

压缩包：

```text
outputs/vigil_rollouts/20260803T133200_474161Z_sonic_sit_chair_auto.zip
```

ZIP SHA-256：

```text
d1dddd2636db96c0ef8d5a340e025aab56c8670a7713f908bcbef13d372c2617
```

相关提交：

```text
9e252d5  FaceE E0019 仓库 MuJoCo/C++/TRT 闭环
ac2eb13  sit-chair rollout 录制与诊断
1df2f0d  d1p50 startup reference hold 与 history 初始化
```

## 13. 最短操作清单

```text
1. 确认分支和提交：r2s_ego_exp_fangs@1df2f0d
2. 确认模型、YAML、catalog SHA-256
3. 确认 enP8p1s0、TensorRT 10.7、D435i、吊装和急停
4. 停止残留 vigil_facee_e0019 session
5. 执行第 6 节标准 start 命令
6. policy.log 等到 Init Done
7. status 必须指定 --bridge-port 8767
8. /health 确认 state/camera/ready/startup hold 全部正常
9. 首次使用 1.50 m / d1p50
10. 异常优先物理急停，然后 /halt 和 launcher stop
```
