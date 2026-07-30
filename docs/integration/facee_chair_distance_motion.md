# FaceE 按椅距选择坐下 reference

## 结论

分支 `er2s_ego_exp2` 在现有 Vigil bridge 上增加了
`sonic.sit_chair`。调用方必须显式提供 `chair_distance_m`，bridge 会向上选择
`1.15, 1.20, ..., 2.00` m 中不小于实测值的最短 reference。例如
`1.12 → 1.15`、`1.17 → 1.20`。

资源清单：

```text
gear_sonic/vigil_bridge/data/facee_chair_13s/manifest.json
```

每条资源为 650 帧、50 Hz、13 秒，包含 29-DoF 位置/速度和 wxyz 根四元数。
清单保存每个 NPZ 和原 GRAIL robot PKL 的 SHA-256。加载时会再次校验 NPZ
哈希、shape、有限值和连续 frame index。

## 距离定义与使用前提

`chair_distance_m` 是 GRAIL task package 中已经验证的目标椅距，不是 bridge
在线视觉估计值。上层 3DGS/感知模块负责按照同一坐标定义算出距离，并只在距离、
椅面高度/朝向、无靠背几何及机器人初始姿态满足实验条件时发动作。

映射不是四舍五入，而是向上取 5 cm 网格，原因是已有实验支持“略长 reference
覆盖略短椅距”，不能反向假设较短 reference 能覆盖更远椅子。支持的实测范围为
`[1.10, 2.00]` m；范围外拒绝。返回结果同时包含原始 `chair_distance_m` 和实际
使用的 `reference_distance_m`，便于记录。

## 启动方式

沿用仓库现有 Docker 启动：

```bash
./vigil_bridge start
```

launcher 会把默认 manifest 绝对路径传给宿主机的
`gear_sonic_deploy/scripts/run_vigil_bridge.py`。Docker 内仍按原方式启动：

```text
deploy.sh real --input-type zmq_manager --output-type zmq
```

没有修改真机环境，也没有修改 `gear_sonic_deploy/src/`、WBC 或 checkpoint。

## 调用

先在 `dry_run` 后端验证解析：

```bash
python3 gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend dry_run --host 127.0.0.1 --port 8765

curl -sS http://127.0.0.1:8765/execute_action \
  -H 'Content-Type: application/json' \
  -d '{
    "runtime_mode": "dry_run",
    "skill_name": "sonic.sit_chair",
    "arguments": {"chair_distance_m": 1.15},
    "safety": {}
  }'
```

真机后端只有在显式启用 `--enable-real-motion` 且 runtime/state/camera readiness
通过后才发送动作。它先重复发送 `planner=false` 完成
`STREAMED_MOTION` 切换，再将选中的完整 reference 通过已有 `pose` topic 发给
`ZMQManager`。任何加载、状态或发送异常都会调用 `halt()` 并返回失败。

## 安全边界

- `/handshake` 会公布 `sonic.sit_chair`，但公布能力不等于自动执行。
- 未启用 real motion 时动作必须返回 `rejected`，且不能启动 runtime。
- 距离只能按上述 ceiling 规则映射，不允许向下选择较短 reference。
- 仿真 18/18 不等于 sim-to-real 18/18；首轮真机必须有固定椅子、机械保护、
  安全员急停及平台既有的限位/倾角/电流监控。
- 当前仓库不跟踪运行所需 ONNX 权重。上机前必须确认机器人现有
  `policy/release/model_{encoder,decoder}.onnx` 与预期 SONIC policy 身份一致；
  本分支不会覆盖它们。

## 离线重建资源

在具有 GRAIL Sonic Python 环境的机器运行：

```bash
/opt/conda/envs/sonic/bin/python tools/build_facee_chair_catalog.py \
  --grail-root /workspace/fangs1@xiaopeng.com/workspace_fs/GRAIL
```

脚本读取
`out/faceE_all_success_hold13_selected_v64/results.json`，使用 Gear-SONIC
`Humanoid_Batch.fk_batch(..., target_fps=50, interpolate_data=True)` 生成输入，
而不是自行实现另一套插值。
