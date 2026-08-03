# FaceE E0019 模型的仓库内 MuJoCo Sim 完整验证流程

> 更新时间：2026-08-03
>
> 适用分支：`r2s_ego_exp_fangs`
>
> 性质：simulation-only。本文所有命令都使用 loopback、MuJoCo 和仿真 DDS，
> 不连接真机，也不构成上机授权。

## 1. 当前结论

仓库内完整闭环已经接通，不再受“缺 C++ deploy/TensorRT”阻塞：

```text
HTTP sonic.sit_chair
  -> Vigil MuJoCo adapter
  -> ZMQ protocol-v1 command + 650-frame pose
  -> C++ g1_deploy_onnx_ref
  -> TensorRT encoder/decoder
  -> Unitree lowcmd over loopback DDS
  -> repository MuJoCo, 4 physics steps / policy target
  -> lowstate + odostate + g1_debug
  -> rollout recorder
```

代表性 `d=1.50 m` 已实际跑满 650 个 action sample/13 秒并坐稳。它不是仅播放
reference，也不是 dry-run。仓库物理状态经 `g1_debug` 和 `rt/odostate` 记录为：

| 项目 | 仓库 MuJoCo 结果 |
| --- | --- |
| action samples | 650 @ 50 Hz |
| 最终 pelvis XYZ | `(-0.3443, 0.0233, 0.5509) m` |
| 最终 pelvis 到椅面中心 XY 距离 | `0.1143 m` |
| 最后 2 秒 pelvis 高度范围 | `[0.550909, 0.550977] m` |
| 最后 2 秒最大 torso tilt | `4.571°` |
| 最终 torso tilt | `4.351°` |
| 倒地 | 没有 |

证据目录：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/GR00T-WholeBodyControl_Vigil/
outputs/vigil_rollouts/20260803T082456_135023Z_sonic_sit_chair_auto
```

其中：

- `raw_real_rollout.npz`：824 个实测状态，含 24 pre + 650 action + 150 post；
- `manifest.json`：采集窗口和来源说明；
- `repository_validation_metrics.json`：离线姿态/稳定性摘要；
- `d1p50_repo_mujoco_measured_replay.mp4`：仓库闭环实测状态的 13 秒可视化；
- `d1p50_reference_isaac_repo_mujoco.mp4`：reference / Isaac / repository MuJoCo
  三栏，325 帧、25 Hz、严格 13.00 秒。

三栏视频 SHA-256：

```text
60d0d6c517364c3db9ba507fc4ca9a07a34b2af446339a4356cf5a3c8a7cd8a5
```

### 证据边界

这次可以判为“任务级坐上并维持平衡”，但还不能判为仓库模式的完整 strict pass：

1. 记录轨迹在 `t=5.40 s` 有一个 `waist_pitch_joint=0.520285 rad` sample，
   比模型上限 `0.52 rad` 高 `0.000285 rad`；
2. 当前仓库 recorder 没有保存每个物理 substep 的 contact force，因此不能仅凭视频
   宣称“落座前小腿/椅子力严格小于 5 N”。

权威 E0019 direct CPU evaluator 的 d1p50 是 strict/no-kick pass，但那是另一条
PyTorch actor 同进程评测链，不能替代仓库 C++/TRT 链的接触力证据。本文不会把两者
混成同一个结论，更不会由仿真结果推导“真机可直接执行”。

## 2. 与此前 MuJoCo 验证相比，本次实现了什么

“MuJoCo 能跑”此前对应过三种不同证据，不能混为一谈：

| 阶段 | 策略执行位置 | reference 如何进入策略 | 物理命令链 | 能证明什么 |
| --- | --- | --- | --- | --- |
| E0019 direct CPU evaluator | Python 进程内 PyTorch actor | evaluator 直接读取 taskset/PKL | actor 直接写同进程 MuJoCo | 证明该权重与 reference 在权威 CPU evaluator 可达 18/18 strict/no-kick；不证明 Vigil/C++ deploy 可用 |
| 早期仓库 `mujoco` mode | 没有实际 actor 闭环 | 未加载 catalog，或手工注入后落入 dry-run | HTTP 可返回 `ok=true`，但 ZMQ command/pose 计数为 0 | 只能证明 HTTP 协议和状态读取，不是 policy rollout |
| 中间仓库实现 | C++ actor 尚未成功启动 | bridge 已真实发送 650 帧 ZMQ pose | scene/DDS/ZMQ 分段可测，但没有 actor `lowcmd` | 证明 dispatch，不证明机器人执行 reference |
| 本文 repository mode | 仓库 `g1_deploy_onnx_ref`，TensorRT encoder/decoder | HTTP 自动选择 exact-v3 catalog，protocol-v1 发送一次 650 帧 pose | ZMQ start/pose → C++ actor → loopback DDS lowcmd → 仓库 MuJoCo → measured state recorder | 证明真实仓库部署链完成 13 秒物理闭环，并可与 reference/Isaac 做三栏比较 |

本次新增或补齐的关键实现是：

1. `MuJoCoExecutor` 正式加载传入的 chair catalog，并覆盖 dry-run 播放函数；
2. `sonic.sit_chair` 根据请求距离同时选择 reference tag 和椅子完整世界位姿；
3. protocol-v1 pose 在 catalog 边界只做一次 MuJoCo→IsaacLab joint-order 重排；
4. ZMQ streamed-motion 的网络 start 被传入 C++ 主状态机，不再停在
   `WAIT_FOR_CONTROL`；
5. 纳入并验证既有 repeat-reset history 修改：CONTROL 起点清空旧 history，首个真实
   本体状态重复填充 10 槽，`last_action` 从零开始；
6. 恢复可迁移的 x86_64 TensorRT/ONNX Runtime/CycloneDDS C++ runtime，并实际构建
   `g1_deploy_onnx_ref`；
7. 场景严格使用 E0019 的 29 个 primitive collider、0.41 m 无靠背椅、MuJoCo
   3.3.4、编译前静态椅位姿、float64 floating-base reset、immutable `model.qpos0`
   和 4-step lockstep；
8. recorder 保存 policy 前/中/后的实测 `g1_debug`/odometry，renderer 从 measured
   state 生成 13 秒 MuJoCo 栏，而不是重新推理或播放 reference。

没有发生的变化也同样重要：actor 权重和网络结构没改，没有 selector，没有重新训练，
没有加入 height map、vision 或椅子 pose 输入，也没有更改默认真机 launcher。

## 3. 策略、reference 与输入契约

### 3.1 共享 actor

仓库 opt-in actor：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/GR00T-WholeBodyControl_Vigil/
gear_sonic_deploy/policy/facee_e0019_exact_v3/
├── model_encoder.onnx
├── model_decoder.onnx
├── observation_config.yaml
└── manifest.json
```

| 文件 | 接口 | SHA-256 |
| --- | --- | --- |
| encoder | `1×1751 -> 1×64` | `1cf476669348344f9ccda0049fb5b708112bdcada73e9403f227321872e7e459` |
| decoder | `1×994 -> 1×29` | `5464bd07f7877b9c42d38cb2a9fc73ad601b857941604e8d3e372298e67d7382` |

来源完整 checkpoint：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/facee_crosssim_dual_agent_v1/
rounds/R0008_mujoco_warp_torch/E0017_mjwarp_shared_ppo/evidence/
s0_iter0008_newbest/full_checkpoint/model_step_mjwarp_s0_iter0008.pt
```

checkpoint SHA-256：

```text
3727209b0340fc31c499d1e069386c7e10297fce80adb4f87e9db10e840e7650
```

18 条 motion 共用一个 encoder/decoder。actor 只有 reference motion +
proprioception，不输入 height map、vision、椅子位置/朝向、distance ID 或 reference
selector。`encoder_mode_4` 是 SONIC 原有 g1/teleop/smpl 模态标识，不是距离 selector。

### 3.2 exact-v3 reference catalog

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/GR00T-WholeBodyControl_Vigil/
gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/
├── manifest.json
├── d1p15.npz
├── ...
└── d2p00.npz
```

共 18 条：`1.15..2.00 m`、间隔 `0.05 m`、每条 650 帧、50 Hz、13 秒。
PKL 源 DOF 是 MuJoCo/MJCF order；打包器在 ZMQ 边界前只做一次
`G1_MUJOCO_TO_ISAACLAB_DOF` 重排，所以 protocol-v1 和 SONIC encoder 收到的是
IsaacLab order。NPZ/manifest 都显式写入：

```text
source_joint_order = mujoco
joint_order = isaaclab
protocol_version = 1
```

距离采用向上选择：`1.12 -> 1.15`、`1.17 -> 1.20`、`1.96 -> 2.00`。
测量支持范围为 `[1.10, 2.00] m`，越界 fail closed。

### 3.3 椅子与机器人物理场景

```text
gear_sonic/data/assets/robot_description/mjcf/
scene_facee_training_primitives.xml
```

SHA-256：

```text
78a08e6ef4b466efb89d759bcb67869f35d4f677b85c08a805211530e24ea7f5
```

物理契约：

- 29-DOF G1、无 dex hands；
- 29 个 Isaac training-URDF primitive colliders；
- self-collision off；
- MuJoCo timestep `0.005 s`、Newton solver、50 iterations；
- actor target 50 Hz，每个 target 严格保持 4 个物理步；
- 无靠背圆角椅，`0.50×0.45 m`，seat top `0.41 m`；
- chair pose 来自每条 manifest 的完整世界 XY/yaw，不把 nominal distance 简化成
  `(distance, 0)`。

## 4. 四个关键数值契约

### 4.1 MuJoCo 必须固定为 3.3.4

权威 E0019 使用 3.3.4。helper 会把下面目录放到 `PYTHONPATH` 最前并硬校验版本：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/sit_chair/.runtime/mujoco_sit_python
```

若迁移机器后目录不同，设置：

```bash
export FACEE_MUJOCO_PYTHONPATH=/absolute/path/to/mujoco_3_3_4_target
```

不能忽略版本门禁；3.7.0 在初始多接触状态下可进入不同 solver branch。

### 4.2 静态椅子必须在编译前放置

不能在 `MjModel.from_xml_path()` 后仅赋值 `model.body_pos/body_quat`。这会绕过
MuJoCo 对静态几何及 broad-phase 派生数据的正常编译路径，后续椅子碰撞和 contact
ordering 不可信。仓库现在用 `MjSpec` 修改椅子 pose 后再 `compile()`。注意
`body_ipos/body_iquat` 表示局部惯性坐标系，本来就不等于 body 世界位姿；测试检查的
是编译后的 `body_pos/body_quat`、运行时 `xpos/xquat` 和椅子 geom 世界坐标。

### 4.3 reference reset 不能修改 model.qpos0

权威 evaluator 的做法是：

```text
mj_resetData -> 写 data.qpos/data.qvel -> mj_forward
```

而不是修改 `model.qpos0`。d1p50 即使 joint stiffness 全为零，改写 qpos0 也会让
首帧接触从 28 个变成 23 个，并立刻进入另一条轨迹。现在
`configure_facee_reference_reset()` 保持 model immutable。

### 4.4 浮动基座 reset 必须保留 float64

旧 catalog 把 root position/quaternion/velocity 降成 float32；初始脚底临界穿透下，
`1e-8` 量级差异也足以改变 contact set。新 catalog 的 reset-only root 数组保持
float64；ZMQ reference 本身仍按 protocol-v1 发送 f32，与 actor 输入一致。

此外，history 使用 `repeat_reset`：首帧 proprio state 填满 10 个 history slots，
与 Isaac Lab circular buffer 首次 push 契约一致，而不是 9 个零帧 + 1 个当前帧。

## 5. 一次性环境准备

### 5.1 Python

主 Python：

```text
/opt/conda/envs/sonic/bin/python
```

MuJoCo 3.3.4 由上一节的 target directory 覆盖。scene helper 启动前会检查两者。

### 5.2 C++/TensorRT/ONNX Runtime

持久化 runtime：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/sonic_cpp_runtime_x86_64_20260803
```

约 5.3 GiB，包含 TensorRT 10.13.3、ONNX Runtime C++ 1.16.3、CycloneDDS
0.10.2 和 planner ONNX。环境脚本：

```bash
source tools/setup_facee_cpp_runtime.sh
```

构建：

```bash
tools/run_facee_e0019_repo_mujoco.sh build
```

生成：

```text
gear_sonic_deploy/target/release/g1_deploy_onnx_ref
```

当前 binary SHA-256：

```text
b856eaf295e7d55ba2bbc73c7df363302077e4b182859f494f01ed198d2b25d5
```

链接门禁：

```bash
source tools/setup_facee_cpp_runtime.sh
ldd gear_sonic_deploy/target/release/g1_deploy_onnx_ref | grep 'not found'
```

正常应无输出。这里的 C++ executable 是策略推理进程；使用 `lo` 和仿真 DDS，并不
依赖或连接真机。

## 6. 完整运行步骤

先进入仓库：

```bash
cd /workspace/fangs1@xiaopeng.com/workspace_fs/GR00T-WholeBodyControl_Vigil
```

### Terminal 1：MuJoCo + DDS scene

```bash
tools/run_facee_e0019_repo_mujoco.sh scene 1.50
```

预期首先看到：

```text
[FaceE] authored reference reset: tag=d1p50 ...
```

scene 在收到 actor policy target 前不推进物理时间；收到 target 后严格按
`4×0.005 s` lockstep 推进。

### Terminal 2：C++ SONIC actor

```bash
tools/run_facee_e0019_repo_mujoco.sh deploy 1.50
```

必须等待完整出现：

```text
Init Done
```

首次 ONNX 转 TensorRT 可能需要几十秒。`Init Done` 前发送 pose 会受到 ZMQ
PUB/SUB slow-join 和模型初始化影响，不算有效运行。

### Terminal 3：Vigil HTTP/ZMQ bridge + recorder

```bash
tools/run_facee_e0019_repo_mujoco.sh bridge 1.50
```

健康检查：

```bash
curl -sS http://127.0.0.1:8765/health
```

### Terminal 4：请求唯一一条 reference

```bash
tools/run_facee_e0019_repo_mujoco.sh request 1.50
```

response 中应有：

```text
ok=true
tag=d1p50
frame_count=650
duration_s=13.0
controller=mujoco_zmq_wbc
dry_run=false
```

注意：HTTP response 的 `completed` 表示 reference 已完整 dispatch，不等于物理
strict pass。物理结论必须来自 scene 状态、recorder 和下述 evaluator。

结束时在三个长驻终端按 `Ctrl-C`。不要用同一个已经坐下的状态连续测试下一距离；
每个距离应新起独立 scene/deploy/bridge 进程，确保 reset/history 独立。

## 7. 找到并验证输出

最新 session：

```bash
find outputs/vigil_rollouts -mindepth 1 -maxdepth 1 -type d \
  -printf '%T@ %p\n' | sort -nr | head
```

检查采集是否完整：

```bash
python - <<'PY'
import numpy as np
p = "outputs/vigil_rollouts/<SESSION>/raw_real_rollout.npz"
with np.load(p) as z:
    phase, count = np.unique(z["capture_phase"], return_counts=True)
    print(dict(zip(phase.tolist(), count.tolist())))
PY
```

预期至少包含：

```text
action=650, pre=24 左右, post=150 左右
```

### 生成仓库 MuJoCo 实测回放

```bash
PYTHONPATH=/workspace/fangs1@xiaopeng.com/workspace_fs/sit_chair/.runtime/mujoco_sit_python:. \
/opt/conda/envs/sonic/bin/python tools/render_facee_repo_rollout.py \
  outputs/vigil_rollouts/<SESSION>/raw_real_rollout.npz \
  --distance-m 1.50 \
  --output outputs/vigil_rollouts/<SESSION>/d1p50_repo_mujoco_measured_replay.mp4 \
  --metrics outputs/vigil_rollouts/<SESSION>/repository_validation_metrics.json
```

该 MP4 是已完成闭环的 measured-state replay，不是重新推理，也不是把 reference
当 rollout。它适合人工看动作；接触力必须在原物理 loop 采集，不能从 replay 伪造。

视频门禁：

```bash
ffprobe -v error \
  -show_entries stream=width,height,r_frame_rate,nb_frames \
  -show_entries format=duration -of json \
  outputs/vigil_rollouts/<SESSION>/d1p50_repo_mujoco_measured_replay.mp4
```

应为 1280×720、25 Hz、325 帧、13.00 秒。

## 8. 正式验收规则

每条距离都必须同时保存机器可读 JSON 和视频，且至少检查：

1. reference 确实是所选 tag，650 帧/13 秒；
2. actor 是本文哈希对应的共享 actor，无 selector；
3. 物理 rollout 跑满 13 秒，pelvis 未跌到 `0.2 m` 以下；
4. 最后 2 秒持续椅面接触比例 `>=0.5`；
5. 最终 pelvis z 位于 `[0.45,0.65] m`；
6. 最终 torso tilt `<30°`；
7. 所有关节不越限；
8. raw torque saturation fraction `<0.5%`；
9. 高风险 self-contact `<5 N`；
10. 首次 pelvis-chair contact 之前，ankle/knee-chair peak normal force `<5 N`。

第 4、8、9、10 项需要 physics-step force/torque instrumentation。没有这些字段时，
结果只能标为 task-level/partial evidence，不能填成 strict PASS。

## 9. 自动化回归

```bash
/opt/conda/envs/sonic/bin/python -m pytest -q \
  tests/test_facee_mujoco_scene.py \
  tests/test_mujoco_facee_runtime_config.py \
  tests/vigil_bridge/test_vigil_bridge_mujoco_adapter.py \
  tests/test_reference_motion_packing.py \
  tests/test_chair_motion_catalog.py \
  tests/test_facee_e0019_policy_package.py
```

当前结果：上述定向套件在本文环境入口下为 `52 passed`。

C++ build：

```bash
tools/run_facee_e0019_repo_mujoco.sh build
```

StateLogger 的 repeat-reset smoke 也应单独通过；完整 C++ 单测仍可能因仓库既有
`reference/bones_072925_test` 资产缺失而失败，这与 FaceE runtime 链不同。

## 10. 常见错误

### `No XML model loaded`

`MjSpec.compile()` 不会填 MuJoCo process-global last-XML。FaceE path 已跳过未使用的
legacy sysid class cache；不要重新引入 `mj_saveLastXML()` 依赖。

### 首步状态明显不同、机器人很快倒地

依次检查：

1. `mujoco.__version__ == 3.3.4`；
2. catalog 是否为 `facee_chair_13s_exact_v3`；
3. reset root 是否 float64；
4. 是否错误修改了 `model.qpos0`；
5. chair 是否在 MjSpec compile 前放好；
6. 是否启用 `repeat_reset` 和 4-step lockstep；
7. joint order 是否只做一次 MuJoCo→IsaacLab 重排。

### request 成功但机器人不动

检查 Terminal 2 是否已出现 `Init Done`，deploy 是否订阅 `localhost:5556 pose`，
以及 scene 是否已发布 `rt/lowstate`。reference dispatch 成功不能代替 lowcmd 证据。

### 把本流程用于真机

不可以直接照搬。本文没有验证实机电机、延迟、状态估计、摩擦、软包、线缆、椅子
顺应性或现场安全系统；helper 也故意固定仿真入口。真机必须另走硬件安全评审与
逐级放权流程。
