# er2s_ego_exp2：FaceE 多椅距坐下运行说明

## 当前结果

本分支在 `Vigil_WBC_dev@3ec1d9fc` 基础上增加 `sonic.sit_chair`，不修改
真机宿主环境、WBC、checkpoint 或 `gear_sonic_deploy/src/`。

18 条正式 reference 来自：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/GRAIL/
out/faceE_all_success_hold13_selected_v64/results.json
```

部署资源位于：

```text
gear_sonic/vigil_bridge/data/facee_chair_13s/
├── manifest.json
├── d1p15.npz
├── ...
└── d2p00.npz
```

每条 650 帧、50 Hz、13 秒。实测距离按“向上取最近 5 cm reference”映射：

```text
1.12 -> 1.15
1.15 -> 1.15
1.17 -> 1.20
1.951 -> 2.00
```

支持实测范围 `[1.10, 2.00]` m；范围外拒绝。

## 先做离线检查

```bash
cd /workspace/fangs1@xiaopeng.com/workspace_fs/GR00T-WholeBodyControl_Vigil
git branch --show-current
python3 -m pytest -q \
  tests/vigil_bridge \
  tests/test_chair_motion_catalog.py \
  tests/test_reference_motion_packing.py
```

预期分支为 `er2s_ego_exp2`，当前基线结果为 `59 passed`。

dry-run HTTP smoke：

```bash
python3 gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend dry_run --host 127.0.0.1 --port 8765
```

另一个终端：

```bash
curl -sS http://127.0.0.1:8765/execute_action \
  -H 'Content-Type: application/json' \
  -d '{
    "runtime_mode": "dry_run",
    "skill_name": "sonic.sit_chair",
    "arguments": {"chair_distance_m": 1.17},
    "safety": {}
  }'
```

应看到：

```json
{
  "chair_distance_m": 1.17,
  "reference_distance_m": 1.2,
  "tag": "d1p20",
  "frame_count": 650
}
```

## 真机启动链

仍使用仓库原入口：

```bash
./vigil_bridge start
```

调用关系：

```text
./vigil_bridge
  -> host launcher
  -> Docker gear_sonic_deploy
  -> deploy.sh real --input-type zmq_manager --output-type zmq
  -> host run_vigil_bridge.py --backend real
  -> /execute_action sonic.sit_chair
  -> command(planner=false)
  -> pose(protocol v1, selected 650-frame reference)
```

launcher 会自动传默认 manifest；如需显式覆盖：

```bash
./vigil_bridge start \
  --chair-motion-catalog \
  /workspace/fangs1@xiaopeng.com/workspace_fs/GR00T-WholeBodyControl_Vigil/gear_sonic/vigil_bridge/data/facee_chair_13s/manifest.json
```

## 真机动作请求

必须由上层明确请求，不会随 bridge 启动自动播放：

```bash
curl -sS http://127.0.0.1:8765/execute_action \
  -H 'Content-Type: application/json' \
  -d '{
    "runtime_mode": "real",
    "skill_name": "sonic.sit_chair",
    "arguments": {"chair_distance_m": 1.17},
    "safety": {}
  }'
```

real backend 未显式启用 motion、state/camera 未 ready、资源校验失败或发送异常时，
动作返回失败；异常路径调用 `halt()`。

## 上机前必须人工确认

1. `policy/release/model_encoder.onnx`、`model_decoder.onnx` 和 observation config
   是预期 SONIC policy。它们不在当前 Git worktree，不能由本分支替换或证明身份。
2. 3DGS/感知提供的距离与 GRAIL 椅距定义一致；椅面高度、朝向、无靠背几何和
   初始机器人姿态满足实验条件。
3. 固定椅子，准备机械保护、安全员急停及平台已经实机验证的限位、倾角、电流与
   通讯超时保护。
4. 先用 `--no-real-motion --no-auto-start-control` 检查服务和距离映射，再按现场
   安全流程显式启用动作。

仿真 18/18 只证明 GRAIL 条件下可以追踪，不等于真机 18/18 已验证。

更完整的接口与构建说明见
`docs/integration/facee_chair_distance_motion.md`；实施问题和边界见
`/workspace/fangs1@xiaopeng.com/workspace_fs/fs_tasks/task14_process.md`。
