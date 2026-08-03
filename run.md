# er2s_ego_exp2：FaceE 多椅距坐下运行说明

## 当前结果

本分支在 `Vigil_WBC_dev@3ec1d9fc` 基础上增加 `sonic.sit_chair`，不修改
真机宿主环境、WBC、checkpoint 或 `gear_sonic_deploy/src/`。

18 条正式 reference 来自：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/GRAIL/
out/faceE_all_success_hold13_selected_v64/results.json
```

部署资源位于（schema v2，protocol-v1 IsaacLab order）：

```text
gear_sonic/vigil_bridge/data/facee_chair_13s_v2/
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
  tests/test_reference_motion_packing.py \
  tests/test_facee_joint_order_contract.py
```

预期分支为 `r2s_ego_exp_fangs`。上述整组 bridge/joint-order 回归当前为
`85 passed`。

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
  /workspace/fangs1@xiaopeng.com/workspace_fs/GR00T-WholeBodyControl_Vigil/gear_sonic/vigil_bridge/data/facee_chair_13s_v2/manifest.json
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

## FaceE E0019 仓库内 MuJoCo（2026-08-03）

MuJoCo sit 的 catalog 加载和真实 ZMQ reference 下发已经接通，不再落入 dry-run。
同时新增了显式 opt-in 的 E0019 exact-v3 18 条 catalog、配套 E0017 iteration-8
ONNX、0.41 m 无靠背 chair scene，以及场景/reference 共用的距离与位姿契约。

统一命令：

```bash
tools/run_facee_e0019_repo_mujoco.sh scene 1.50
tools/run_facee_e0019_repo_mujoco.sh deploy
tools/run_facee_e0019_repo_mujoco.sh bridge
tools/run_facee_e0019_repo_mujoco.sh request 1.50
```

当前机器已验证 scene、DDS、HTTP 和一个 650 帧 ZMQ pose。C++ deploy 仍因本机缺少
TensorRT/ONNX Runtime C++ 构建环境而未启动，所以这不是 13 秒 closed-loop rollout
成功结论。完整资产路径、哈希、代码和验证边界见：

```text
docs/integration/vigil_mujoco_facee_validation_20260803.md
```

### 2026-08-03 更新：旧 C++ blocker 已解除

上面一段是阶段记录。当前已在持久目录恢复 TensorRT 10.13.3、ONNX Runtime C++
1.16.3 和 CycloneDDS 0.10.2，`g1_deploy_onnx_ref` 已构建并参与真实仓库闭环。

d1p50 已通过 `scene -> C++/TRT deploy -> bridge -> request` 跑满 650 action samples。
任务级结果是坐上并稳定保持；严格结果仍保留两个边界：一个 waist-pitch sample
超限 0.000285 rad，且仓库 recorder 暂未记录物理 substep contact force，不能宣称
strict no-kick pass。

当前唯一应使用的完整仿真说明：

```text
docs/integration/facee_e0019_mujoco_sim_validation_workflow.md
```

代表性证据：

```text
outputs/vigil_rollouts/20260803T082456_135023Z_sonic_sit_chair_auto/
```
