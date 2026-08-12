# FaceE Stage-1 generalist yaw 候选模型

该包对应 E0038 `model_step_000096.pt`。模型本身不接收椅子距离、位置或 yaw；
bridge 必须先根据外部测量或视觉结果选择 `(chair_distance_m,
chair_yaw_deg)` 对应的 reference motion，再由同一 actor 跟踪。

## 资产

重资产已上传到：

```text
oss://xrobot-data/fs/r2s_ego_exp/GR00T-WholeBodyControl_Vigil/policy/facee_stage1_generalist_yaw/
```

真机只需要 ONNX encoder/decoder 和 motion catalog；`.pt` 仅用于复现、重新导出，
不参与 C++ TensorRT 推理。拉取并校验 ONNX：

```bash
tools/fetch_facee_stage1_generalist_yaw_assets.sh \
  --ossutil /path/to/ossutil64 \
  --config /path/to/oss_config \
  --with-full-catalog
```

`hardware18` 已随仓库提交，并同步到 OSS 的 `motion_catalog_hardware18/`；不需要从
原始 PKL 重新生成。它从已审计 full403 精确抽取，18 个 NPZ 的字节和 SHA 均保持
不变。`--with-full-catalog` 只在需要完整 403 条时使用。

### 当前权重位置

训练得到的 PyTorch checkpoint 位于：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/facee_crosssim_dual_agent_v1/rounds/
R0010_task18_sitstand_yaw_vision/E0038_full_sit_8192_ppo/
train_8192_stack256x96/train/model_step_000096.pt
```

OSS 上保存了 checkpoint 和真机使用的 ONNX：

```text
oss://xrobot-data/fs/r2s_ego_exp/GR00T-WholeBodyControl_Vigil/policy/
facee_stage1_generalist_yaw/
├── model_step_000096.pt
├── model_step_000096_encoder.onnx
└── model_step_000096_decoder.onnx
```

拉取后真机以 `model_encoder.onnx`、`model_decoder.onnx` 命名；仓库不提交这两个
重文件。`.pt` 只用于复现和重新导出，不参与 C++ TensorRT 推理。

### 当前 motion 位置和 OSS 范围

原始训练/仿真数据位于：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/facee_crosssim_dual_agent_v1/rounds/
R0010_task18_sitstand_yaw_vision/E0037_extended_range_crosssim/
extended_data/directional/
├── manifest.json
└── training_library/
    ├── robot/
    ├── objects/
    └── object_usd/
```

Robot PKL 是 313 帧、24 Hz、MuJoCo 顺序的源数据。真机不直接读取 PKL；部署 NPZ
已经按合约转换为 650 帧、50 Hz、IsaacLab 顺序。目前 OSS 上传了以下三套
Sit-only catalog，均不包含 Stand：

| OSS 子目录 | 距离 | yaw | 条数 | 说明 |
| --- | --- | --- | ---: | --- |
| `motion_catalog/` | 1.45 m | 0/5/10/15/20/25° | 6 | clean6 首次预检 |
| `motion_catalog_hardware18/` | 1.45 m | 0/5/10/15/20/25° | 6 | clean |
| `motion_catalog_hardware18/` | 2.00 m | -15/-10/-5/0/5/10° | 6 | non-clean，需显式 opt-in |
| `motion_catalog_hardware18/` | 2.40 m | -15/-10/-5/0/5/10° | 6 | non-clean，需显式 opt-in |
| `motion_catalog_full403/` | 0.90–2.40 m，步长 0.05 m | -30–30°，步长 5° | 403 | 完整 Sit 库 |

三套 catalog 有重复条件：clean6 和 hardware18 都是 full403 的子集，因此 OSS 上
唯一的 `(distance, yaw)` 条件仍是 full403 的 403 条。当前上机应选仓库中的：

```text
gear_sonic/vigil_bridge/data/
facee_stage1_generalist_yaw_hardware18/manifest.json
```

## 测试范围

完整 Sit catalog 为 31 个距离（`0.90–2.40 m`，步长 `0.05 m`）× 13 个 yaw
（`-30–30°`，步长 `5°`），共 403 条；包括 2.0 m 和 2.4 m 各 13 条。
不包含 Stand motion。完整二进制只在 OSS，使用 `--with-full-catalog` 拉取。

403 条里只有 6 条同时满足 Isaac strict、MuJoCo action-complete 且小腿碰椅峰值
`<5 N`：`1.45 m × yaw 0/5/10/15/20/25°`。仓库直接提交的 clean6 catalog
用于首次硬件预检。

应本次测试需求，另提交 Sit-only `hardware18`：上述 1.45 m clean6，加上 2.0 m、
2.4 m 各 6 条 yaw `-15/-10/-5/0/5/10°`。后 12 条均为 Isaac strict + MuJoCo
action-complete，但都不是 clean：2.0 m 小腿碰椅峰值为 `164.54–299.70 N`，2.4 m
为 `168.86–229.51 N`。这些条目默认 fail closed，必须显式传
`allow_non_clean_reference=true`。不要把“动作完成”误当成接触安全或真机授权。

视觉 selector 尚未接入 bridge。初测时由上层显式传入距离和角度；连续值会选择最近
reference。clean6 只接受距离 `[1.425, 1.475] m`、yaw `[-2.5, 27.5]°`。
hardware18 只接受 1.45/2.0/2.4 m 各自 ±2.5 cm 的三个离散窗口；1.45 m 接受 yaw
`[-2.5, 27.5]°`，2.0/2.4 m 接受 `[-17.5, 12.5]°`，不会把中间距离误映射过去。

先禁用运动启动，确认模型加载、相机、状态和 reference 初始姿态：

```bash
./vigil_bridge start \
  --policy-checkpoint policy/facee_stage1_generalist_yaw/model \
  --policy-observation-config policy/facee_stage1_generalist_yaw/observation_config.yaml \
  --chair-motion-catalog "$PWD/gear_sonic/vigil_bridge/data/facee_stage1_generalist_yaw_clean6/manifest.json" \
  --no-real-motion \
  --no-auto-start-control

curl -sS http://127.0.0.1:8765/diagnostics/sit_chair/preflight \
  -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real","chair_distance_m":1.45,"chair_yaw_deg":10}'
```

通过预检后需要 `./vigil_bridge stop`，再去掉 `--no-real-motion` 和
`--no-auto-start-control` 重新启动，最后显式发送动作：

```bash
curl -sS http://127.0.0.1:8765/execute_action \
  -H 'Content-Type: application/json' \
  -d '{
    "runtime_mode":"real",
    "skill_name":"sonic.sit_chair",
    "arguments":{"chair_distance_m":1.45,"chair_yaw_deg":10},
    "safety":{}
  }'
```

上机时应使用吊装/保护架、急停和旁站人员，并从 yaw 0° 单次开始逐点放开。双模拟器
结果不等于真机授权；manifest 中仍保持 `real_robot_authorized=false`。

测试 2.0/2.4 m 时，先停止 clean6 实例，再显式切换到 hardware18 catalog：

```bash
./vigil_bridge stop

./vigil_bridge start \
  --policy-checkpoint policy/facee_stage1_generalist_yaw/model \
  --policy-observation-config policy/facee_stage1_generalist_yaw/observation_config.yaml \
  --chair-motion-catalog "$PWD/gear_sonic/vigil_bridge/data/facee_stage1_generalist_yaw_hardware18/manifest.json"
```

例如请求 2.4 m / 0°（非 clean）必须携带显式 opt-in：

```bash
curl -sS http://127.0.0.1:8765/execute_action \
  -H 'Content-Type: application/json' \
  -d '{
    "runtime_mode":"real",
    "skill_name":"sonic.sit_chair",
    "arguments":{
      "chair_distance_m":2.4,
      "chair_yaw_deg":0,
      "allow_non_clean_reference":true
    },
    "safety":{}
  }'
```

完整距离/yaw catalog 的显式切换方式：

```bash
./vigil_bridge start \
  --policy-checkpoint policy/facee_stage1_generalist_yaw/model \
  --policy-observation-config policy/facee_stage1_generalist_yaw/observation_config.yaml \
  --chair-motion-catalog "$PWD/gear_sonic/vigil_bridge/data/facee_stage1_generalist_yaw_full403/manifest.json"
```

例如 2.4 m / 30° 如果不是 clean，请求必须额外携带：

```json
{
  "chair_distance_m": 2.4,
  "chair_yaw_deg": 30,
  "allow_non_clean_reference": true
}
```

## 结构兼容性

与上次实际真机使用的 `facee_e0019_exact_v3` 以及历史 real bridge 的
`facee_v73_noheight` 比较，新模型 encoder/decoder 的 I/O 形状、opset、节点数、
算子直方图和 initializer 形状序列完全相同；只有权重数值不同。详细结果见
`manifest.json`。
