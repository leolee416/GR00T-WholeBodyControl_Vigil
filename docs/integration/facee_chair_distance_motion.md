# FaceE 按椅距选择坐下 reference

## 当前交付状态

分支 `r2s_ego_exp_fangs` 包含 18 条 13 秒 FaceE reference、无高度图
v73 策略的 PyTorch→ONNX 导出物，以及 `sonic.sit_chair` 的距离选择和流式发送
逻辑。

必须区分两层结论：

- Isaac/PhysX：16 条通过严格跟踪验收；1.35 m、1.40 m 两条能够坐上且不摔，
  由人工按任务完成接受，但严格检查在 `anchor_ori_full` 处失败。
- MuJoCo 独立筛选：1.35 m、1.40 m 均完整运行到 13 秒并保持座面接触，但末段
  向后躺倒，最终躯干倾角分别为 76.52°、75.29°，且 `waist_pitch_joint`
  越过模型限位约 0.035/0.032 rad。因此 v73 ONNX **未获真机执行授权**。

仓库保留原有 release policy 作为 launcher 默认值。v73 只作为显式 opt-in
候选，供继续仿真、接口联调和后续安全修复；不可因为 ONNX 可加载就直接上机。

## 资源

Reference 清单：

```text
gear_sonic/vigil_bridge/data/facee_chair_13s/manifest.json
```

每条资源为 650 帧、50 Hz、13 秒，包含 29-DoF 位置/速度和 wxyz 根四元数。
清单保存每个 NPZ 和原 GRAIL robot PKL 的 SHA-256；加载时再次校验哈希、
shape、有限值和连续 frame index。

v73 候选策略：

```text
gear_sonic_deploy/policy/facee_v73_noheight/
├── model_encoder.onnx          # 1×1751 → 1×64
├── model_decoder.onnx          # 1×994  → 1×29
├── observation_config.yaml
├── manifest.json
└── model_{encoder,decoder}.parity.json
```

ONNX SHA-256：

```text
encoder 33c88827b13d0270a1e7f036b882284e6a2429e37f9bdb21b60c706d916ca849
decoder 1bea1e29e8f243cc312e15fe0858c751d48e97ccbaa45ef419e6e4db210c14f7
```

Actor 输入只有 reference motion 和本体状态；没有 `height_map_z_flat`、椅子
相对位置或椅子相对朝向。Encoder observation YAML 使用真机 C++ registry
已有的 observation 名称，启用项总计严格为 1751 维；decoder 为 64 维 token
加 930 维本体历史，总计 994 维。

这里部署的是一个共享 actor：只有一对 encoder/decoder，不包含 v119 的三路
expert bank、距离 selector 或距离条件路由。距离只用于在 actor 外部从 motion
catalog 选择对应的 reference；选定后，所有距离都经过完全相同的 v73 actor。
`encoder_mode_4` 是官方 universal encoder 用于区分 `g1`/`teleop`/`smpl`
输入模态的 token，不是椅距 selector。

同一份无 selector ONNX 包同步到：

```text
oss://xrobot-data/fs/r2s_ego_exp/GR00T-WholeBodyControl_Vigil/policy/facee_v73_noheight/
```

PyTorch/ONNX 同输入校验结果：

- encoder 最大绝对误差 0；
- decoder 最大绝对误差 `1.55e-6`；
- 两者均通过 `rtol=1e-4, atol=1e-5`。

这只证明模型导出一致，不证明物理环境迁移或真机安全性。

## 1.35/1.40 m 的特殊处理

这两档不再使用原先各自的 robot reference，而是逐数组精确复用 1.45 m
reference：

```text
d1p35 joint_pos/joint_vel/body_quat_w == d1p45
d1p40 joint_pos/joint_vel/body_quat_w == d1p45
```

只有椅子相对机器人向前移动：

- 1.35 m：相对 1.45 m 场景靠近 0.10 m；
- 1.40 m：相对 1.45 m 场景靠近 0.05 m。

Isaac 对比视频在持久目录：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/GRAIL/out/
  faceE_sonic_v1_1_noheight_v65/
  eval_v73_iter025_refd1p45_chaircloser_v79/comparison_videos/
  ├── d1p35.mp4
  └── d1p40.mp4
```

这里的 `task_level_status=ACCEPT` 不应解释成严格 tracking success，更不应解释
成真机 safety pass。

## 距离映射

`chair_distance_m` 是与 GRAIL task package 相同定义下的目标椅距，由上层
3DGS/感知模块提供，不是 bridge 内部估计值。

映射采用向上取 5 cm 网格：

```text
1.12 → 1.15
1.17 → 1.20
1.35 → 1.35
1.96 → 2.00
```

原因是实验支持“略长 reference 覆盖略短椅距”，但没有证据支持反向使用更短
reference 去坐更远的椅子。实测输入范围为 `[1.10, 2.00]` m，范围外拒绝。
返回值同时记录输入距离和实际选择的 reference 距离。

上层仍须检查 0.41 m 椅面高度、无靠背几何、椅子朝向和机器人初始姿态；
策略本身看不到椅子 pose。

## 启动与策略选择

默认启动保持原上机行为：

```bash
./vigil_bridge start
```

它继续选择：

```text
policy/release/model
policy/release/observation_config.yaml
```

v73 候选只能显式选择：

```bash
./vigil_bridge start \
  --policy-checkpoint policy/facee_v73_noheight/model \
  --policy-observation-config policy/facee_v73_noheight/observation_config.yaml
```

由于当前 MuJoCo 筛选失败，上述命令仅用于非真机环境或经过独立安全门控的后续
联调。模型加载前 launcher 会检查 encoder、decoder 和 observation config
是否都存在。

Bridge 调用示例：

```bash
python3 gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend dry_run --host 127.0.0.1 --port 8765

curl -sS http://127.0.0.1:8765/execute_action \
  -H 'Content-Type: application/json' \
  -d '{
    "runtime_mode": "dry_run",
    "skill_name": "sonic.sit_chair",
    "arguments": {"chair_distance_m": 1.17},
    "safety": {}
  }'
```

真机 backend 还要求显式 `--enable-real-motion`，并通过 runtime、state、
camera readiness。切换到 `STREAMED_MOTION` 后，bridge 通过已有 `pose` topic
把完整 reference 发给 `ZMQManager`；异常会调用 `halt()`。

## 离线重建

Reference：

```bash
/opt/conda/envs/sonic/bin/python tools/build_facee_chair_catalog.py \
  --grail-root /workspace/fangs1@xiaopeng.com/workspace_fs/GRAIL
```

ONNX：

```bash
tools/export_facee_v73_noheight_onnx.sh \
  /workspace/fangs1@xiaopeng.com/workspace_fs/GRAIL
```

Checkpoint：

```text
GRAIL/out/faceE_sonic_v1_1_noheight_v65/
  dagger_v69iter100_balanced_freezeenc_fixedlr2e6_30iter_v73/
  model_step_000025.pt
```

SHA-256：

```text
610bc1bd21e30ca9dd9690a5096ddf3f54ca7b3fe7fb3ddf3ed73c65ead049c9
```

## MuJoCo 证据

持久目录：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/codex_session_restore/
  facee_v73_noheight_deploy_2026-07-30/
```

每个距离包含视频、contact sheet、逐帧 CSV、NPZ、`metrics.json` 和运行日志。
两条均 `completed_reference=true`、末 2 秒座面接触比例为 1.0，且没有
超过 5 N 的自碰；但由于向后躺倒和腰部限位越界，
`simulation_screen_pass=false`。这是阻止当前模型直接上机的决定性证据。

## 安全边界

- `/handshake` 公布能力不等于允许执行。
- 未启用 real motion 时必须拒绝真实运动。
- Isaac task accept、ONNX parity 和 MuJoCo“坐到椅子上”都不能单独授权真机。
- 在 MuJoCo 末段失稳修复、独立复跑通过，以及真机 dry-run/吊装/急停检查完成
  之前，不得将 v73 设置为默认真机 policy。
