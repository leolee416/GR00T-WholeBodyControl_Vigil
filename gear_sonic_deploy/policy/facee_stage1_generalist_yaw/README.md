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

## 测试范围

完整 Sit catalog 为 31 个距离（`0.90–2.40 m`，步长 `0.05 m`）× 13 个 yaw
（`-30–30°`，步长 `5°`），共 403 条；包括 2.0 m 和 2.4 m 各 13 条。
不包含 Stand motion。完整二进制只在 OSS，使用 `--with-full-catalog` 拉取。

403 条里只有 6 条同时满足 Isaac strict、MuJoCo action-complete 且小腿碰椅峰值
`<5 N`：`1.45 m × yaw 0/5/10/15/20/25°`。仓库直接提交的 clean6 catalog
用于首次硬件预检。完整 catalog 中，任何非 clean 条目默认 fail closed；仅在经过额外
审批的非 clean 测试中显式传 `allow_non_clean_reference=true` 才能选择。不要把
281/403 的“动作完成”误当成接触安全。

视觉 selector 尚未接入 bridge。初测时由上层显式传入角度；连续角度会选择最近的
5° reference，允许范围为 `[-2.5°, 27.5°]`，距离允许范围为
`[1.425 m, 1.475 m]`。

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
