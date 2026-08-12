# FaceE 真机 reference joint order 错位：根因、源码证据与复现

> 状态：**根因已确认；系统修复第 1–5 层已实现（FIXED_IN_V2）**
>
> 记录日期：2026-07-31
>
> 证据基线：Git commit `0283feed7283a891d277653e5280f1546ecf4fe3`
>
> 影响对象：旧 `facee_chair_13s/*.npz` 经 Vigil Bridge、ZMQ protocol v1 进入
> `g1_deploy_onnx_ref` 的部署链路
>
> 安全结论：新 v2 资产已修复语义契约，但部署等价 MuJoCo
> 中 `d1p70` 仍末段后仰；因此 **修复 joint order 不等于获得真机执行许可**。

## 0. 2026-07-31 修复落地状态

本文后半部保留了修复前的证据和设计推导。对应的前五层已经
落地：

| 层 | 已实现内容 | 主要代码/证据 |
|---:|---|---|
| 1 | 唯一顺序契约：源为 MuJoCo，protocol-v1 资产为 IsaacLab | `g1.py` 官方映射；打包器通过 AST 读取同一定义 |
| 2 | 在打包边界对 q/dq 各做一次 `MJ -> IL` | `tools/build_facee_chair_catalog.py` |
| 3 | 生成 18 条 schema-v2 资产，写入 order/protocol/mapping 和新 checksum；旧 v1 不再是默认 | `gear_sonic/vigil_bridge/data/facee_chair_13s_v2/` |
| 4 | catalog 和 sender fail-closed；增加 mapping、asset→wire、C++ `MotionSequence` 580 值 parity | `chair_motion_catalog.py`、`mujoco_adapter.py`、`tests/test_*joint_order*`、`tests/cpp/facee_motion_sequence_parity.cpp` |
| 5 | Agent-Sim MuJoCo 改为直接消费同一 v2 NPZ；只在 simulator reset 边界转回 MuJoCo order | `agent_sim_r2s_ego_exp_real` 的 `deployment_reference.py` 和 FaceE runner |

前四层连同 Vigil bridge 回归为 `85 passed`。第 5 层对两条代表距离的 fresh run 结果是：

- `d2p00`：650 个 control samples，13.0 s，末 2 s 座面接触比例
  1.0，最终 torso 倾角 `11.86°`，安全筛选 PASS；
- `d1p70`：650 个 control samples，13.0 s，末 2 s 座面接触比例
  1.0，但最终 torso 倾角 `74.92°`，安全筛选 FAIL。

旧 MuJoCo 报告从 GRAIL PKL 在 runner 内部重建 reference，不是真机会收到的
NPZ 字节，因而旧的 `d1p70 PASS` 不能再当作部署 parity 证据。对比表明：
新旧 joint position 最大只差约 `0.00103 rad`，joint velocity 最大差约
`0.05358 rad/s`，但闭环在接触前的转身段已逐渐分叉，说明 `d1p70` 是一条
对 reference 数值细节敏感的边界成功样本。根四元数在第 260 帧存在等价的
`q/-q` 表示切换，但 Python 编码器显式统一符号，C++ 编码器转 rotation
matrix 且 slerp 采用短弧，所以这不是本次后仰的解释。

部署等价产物保存于：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/codex_session_restore/
  facee_joint_order_v2_deployment_parity_2026-07-31/
```

## 1. 一句话结论

FaceE catalog 打包器把 `Humanoid_Batch.fk_batch()` 产生的 **MuJoCo/MJCF
关节顺序**直接保存成 NPZ；但 ZMQ protocol v1 和 SONIC encoder 的 motion
reference 槽位契约是 **IsaacLab 关节顺序**。从 NPZ 加载、Python 发布、C++
解码、motion 合并到 encoder observation 的整条链路都没有重排，因此 29 个
槽位中只有 2 个碰巧仍是同一个关节，另外 27 个都被错误解释。

这不是“小误差”：在 `d1p70` 的受控 ONNX 反事实实验中，只改变 reference
的关节排列、固定其余所有输入后：

- 10 帧位置 + 10 帧速度共 580 个 reference 槽位中，540 个槽位的**关节语义**改变；
- 当前这段 motion 因若干值恰好相同或为零，实际数值不相等的是 490/580；
- 64 维 encoder token 中 56 维改变，最大绝对差 `0.4375`；
- 29 维 raw action 全部改变，最大绝对差 `5.164087`；
- `waist_pitch_joint` raw action 从 `+0.943958` 变为 `-1.065753`，差值
  `-2.009710`。

因此，真机出现错误重心调整、向后退、无法按 reference 小步前进，与这条
确定性输入错误完全一致。摩擦、电机、延迟和初始姿态仍可能是后续 sim2real
问题，但不需要它们也能解释当前 action 已经在进入物理系统前被显著改变。

## 2. 本文证明什么、不证明什么

### 2.1 已由源码和可重复数值实验确认

1. FaceE 源 motion 和 FK 输出处于 MuJoCo/MJCF 语义顺序。
2. 当前 NPZ 打包没有执行 MuJoCo → IsaacLab 重排。
3. catalog loader 和 ZMQ publisher 原样发送数组。
4. protocol v1 明确要求 IsaacLab 顺序。
5. C++ receiver、stream merger 和 encoder gatherer 都按原索引复制，没有重排。
6. 仅引入这项排列错误，就足以让 v73 ONNX token 和 action 大幅改变。
7. 既有 MuJoCo 视频使用了正确的 MuJoCo → IsaacLab 重排，因而不等价于当前
   真机 ZMQ 输入链路。

### 2.2 本文没有宣称

- 它是所有真机问题的唯一原因；
- 修复排列后真机就必然安全通过；
- PhysX、MuJoCo 和真机接触动力学完全一致；
- 真机初始站姿、reference 首帧速度、控制延迟、PD 参数和电机能力已经验证；
- 本文的离线 ONNX 实验等同于物理 rollout。

这些是修复 joint order 后需要继续隔离验证的下一层问题。当前应先修复这个
位于 policy 输入之前、无需接触物理系统就能确定的错误。

### 2.3 已发现但从本实验中严格隔离的其他 parity 项

本报告实验时的 C++ `GatherEncoderMode(..., 3)` 对 `encode_mode=0` 产生
`[0, 0, 0, 0]`，而 Python 官方 export contract 构造的是 `[0, 1, 0, 0]`。
当前分支已把 C++ 修正为后一种 scalar selector + one-hot 布局；真机进入 motion 前的
默认站姿与 reference 首帧也可能不同。这些都需要单独审计，但本文的两个 ONNX
分支使用完全相同的 C++ prefix、root orientation 和 proprioception fixture，
所以它们不会贡献本文报告的 order-only token/action 差值。

## 3. 两种顺序到底是什么

### 3.1 MuJoCo/MJCF 顺序

当前 G1 MJCF 按一侧腿的运动链向下遍历，再处理另一侧腿和腰/手臂。前 15 个
关节是：

```text
0  left_hip_pitch_joint
1  left_hip_roll_joint
2  left_hip_yaw_joint
3  left_knee_joint
4  left_ankle_pitch_joint
5  left_ankle_roll_joint
6  right_hip_pitch_joint
7  right_hip_roll_joint
8  right_hip_yaw_joint
9  right_knee_joint
10 right_ankle_pitch_joint
11 right_ankle_roll_joint
12 waist_yaw_joint
13 waist_roll_joint
14 waist_pitch_joint
```

### 3.2 IsaacLab 顺序

IsaacLab 的顺序在左右腿和腰之间交错，前 15 个关节是：

```text
0  left_hip_pitch_joint
1  right_hip_pitch_joint
2  waist_yaw_joint
3  left_hip_roll_joint
4  right_hip_roll_joint
5  waist_roll_joint
6  left_hip_yaw_joint
7  right_hip_yaw_joint
8  waist_pitch_joint
9  left_knee_joint
10 right_knee_joint
11 left_shoulder_pitch_joint
12 right_shoulder_pitch_joint
13 left_ankle_pitch_joint
14 right_ankle_pitch_joint
```

这两个向量形状都是 `[29]`，dtype 也相同，所以 shape、finite、checksum 等普通
校验全部会通过。错误只存在于“第 `i` 列代表哪个关节”的语义层。

### 3.3 官方映射

映射定义在
[`gear_sonic/envs/manager_env/robots/g1.py`](../../gear_sonic/envs/manager_env/robots/g1.py#L28-L123)：

```python
G1_MUJOCO_TO_ISAACLAB_DOF = [
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
    16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28,
]

# q_mj: [..., 29], q_il: [..., 29]
q_il = q_mj[..., G1_MUJOCO_TO_ISAACLAB_DOF]
```

通用转换器也明确实现了同一方向：

```python
class G1Converter(...):
    ...
    ("mujoco", "isaaclab"): G1_MUJOCO_TO_ISAACLAB_DOF

converter.to_isaaclab(data)
```

见
[`gear_sonic/trl/utils/order_converter.py`](../../gear_sonic/trl/utils/order_converter.py#L83-L118)。

## 4. 完整错误调用链

```text
GRAIL robot PKL
pose_aa / dof: MuJoCo order
        │
        ▼
Humanoid_Batch(MJCF).fk_batch()
fk.dof_pos / fk.dof_vels: MuJoCo/MJCF semantic order
        │
        ▼
build_facee_chair_catalog.py
直接保存 joint_pos / joint_vel                 ← 缺少 MJ → IL 转换
        │
        ▼
facee_chair_13s/d1pXX.npz
shape 正确，但 29 列仍是 MuJoCo order
        │
        ▼
ChairMotionCatalog.select()
np.load 后原样返回
        │
        ▼
send_reference_motion()
转成 little-endian f32 后原样序列化
        │
        ▼
ZMQ protocol v1
契约要求：IsaacLab order
        │
        ▼
ZMQEndpointInterface
按 wire slot 解码，无重排
        │
        ▼
StreamedMotionMerger::CopyIncomingDataToMotion()
data[frame][joint] → motion[frame][joint]，无重排
        │
        ▼
GatherMotionJointPositions/VelocitiesMultiFrame()
motion 连续内存 → encoder observation，仍无重排
        │
        ▼
v73 encoder/decoder
把 MuJoCo 槽位当成 IsaacLab 槽位解释
        │
        ▼
错误 raw action → scale/default angle → 真机 motor target
```

关键点不是某一步“显式转换错方向”，而是应该发生一次转换的边界上**完全没有
转换**。之后每一层都忠实地原样传递了错误语义。

## 5. 源码证据逐层展开

### 5.1 源 motion 与 FK 使用 MuJoCo/MJCF 约定

项目约定文档明确写明：motion PKL 中 `dof`、`pose_aa` 是 MuJoCo order，训练
进入 IsaacLab 时由 `order_converter.py` 自动转换：

```text
Motion PKL data (`dof`, `pose_aa`) is stored in MuJoCo order.
Isaac Lab simulation uses IsaacLab order.
```

见
[`docs/source/references/conventions.md`](../source/references/conventions.md#joint-ordering)。

FaceE 打包器创建 `Humanoid_Batch` 时使用的是 MJCF：

```python
cfg = OmegaConf.create({
    "asset": {
        "assetRoot": ... / "robot_description/mjcf",
        "assetFileName": "g1_29dof_rev_1_0.xml",
    },
    "extend_config": [],
})
humanoid = Humanoid_Batch(cfg, torch.device("cpu"))
```

见
[`tools/build_facee_chair_catalog.py`](../../tools/build_facee_chair_catalog.py#L102-L116)。

`Humanoid_Batch` 解析 MJCF body tree，按 XML tree 顺序递归生成 `node_names` 和
`body_to_joint`，并据此构造 `actuated_joints_idx`：

```python
self.mjcf_data = self.from_mjcf(self.mjcf_file)
self.body_names = copy.deepcopy(mjcf_data["node_names"])
self.actuated_joints_idx = np.array(
    [self.body_names.index(k) for k, v in mjcf_data["body_to_joint"].items()]
)
```

见
[`torch_humanoid_batch.py`](../../gear_sonic/utils/motion_lib/torch_humanoid_batch.py#L153-L188)
和同文件的
[`from_mjcf()`](../../gear_sonic/utils/motion_lib/torch_humanoid_batch.py#L257-L319)。

`fk_batch(return_full=True)` 最后按该索引生成 `dof_pos`，并由它计算 `dof_vels`：

```python
return_dict.dof_pos = pose.sum(dim=-1)[..., self.actuated_joints_idx]
dof_vel = (dof_pos[:, 1:] - dof_pos[:, :-1]) / dt
return_dict.dof_vels = torch.cat([dof_vel, dof_vel[:, -2:-1]], dim=1)
```

见
[`torch_humanoid_batch.py`](../../gear_sonic/utils/motion_lib/torch_humanoid_batch.py#L428-L451)。

因此，从这条 MJCF FK 路径取得的 29 维 DOF 不能直接当成 IsaacLab observation。

### 5.2 FaceE 打包器缺少转换

当前打包器直接取 FK 输出：

```python
fk = humanoid.fk_batch(...)
joint_pos = fk.dof_pos[0].cpu().numpy().astype(np.float32)
joint_vel = fk.dof_vels[0].cpu().numpy().astype(np.float32)
```

随后原样写入：

```python
np.savez_compressed(
    motion_file,
    joint_pos=joint_pos,
    joint_vel=joint_vel,
    ...,
)
```

见
[`tools/build_facee_chair_catalog.py`](../../tools/build_facee_chair_catalog.py#L166-L193)。

这段代码中没有：

- `G1_MUJOCO_TO_ISAACLAB_DOF`；
- `G1Converter.to_isaaclab()`；
- 任何等价的 `[..., mapping]`；
- 用于声明或校验 `joint_order` 的 manifest 字段。

这就是错误被引入的位置。

### 5.3 Catalog loader 只校验 shape，不校验语义顺序

`ChairMotionCatalog` 校验 checksum、shape 和 finite，然后直接返回 NPZ arrays：

```python
with np.load(path, allow_pickle=False) as archive:
    frames = {name: archive[name] for name in archive.files}
self._validate_frames(frames, record)
return ChairMotion(..., frames=frames)
```

见
[`chair_motion_catalog.py`](../../gear_sonic/vigil_bridge/chair_motion_catalog.py#L64-L85)。

它只知道 `joint_pos.shape == (650, 29)`，不知道第 1 列是
`right_hip_pitch_joint` 还是 `left_hip_roll_joint`。当前 manifest 也没有可供它
拒绝错误资产的 `joint_order` 字段。

### 5.4 真机 adapter 没有转换

真机调用路径最终是：

```python
publisher.send_reference_motion(frames)
```

见
[`real_adapter.py`](../../gear_sonic/vigil_bridge/real_adapter.py#L242-L265)。

publisher 只做 dtype 和形状处理：

```python
joint_pos = np.asarray(frames.get("joint_pos"), dtype="<f4")
joint_vel = np.asarray(frames.get("joint_vel"), dtype="<f4")
...
np.ascontiguousarray(joint_pos).tobytes()
np.ascontiguousarray(joint_vel).tobytes()
```

见
[`mujoco_adapter.py`](../../gear_sonic/vigil_bridge/mujoco_adapter.py#L246-L283)。

`ascontiguousarray()` 只改变内存连续性，`tobytes()` 只序列化现有列；二者都不会
改变关节顺序。

### 5.5 ZMQ protocol v1 明确要求 IsaacLab 顺序

协议文档不是模糊地写“29 joints”，而是明确规定：

| field | shape | required semantic order |
|---|---:|---|
| `joint_pos` | `[N, 29]` | IsaacLab order |
| `joint_vel` | `[N, 29]` | IsaacLab order |

见
[`docs/source/tutorials/zmq.md`](../source/tutorials/zmq.md#protocol-v1--joint-based-encode-mode-0)。

Observation 文档也再次说明，当前 motion sequence 的所有 joint data 都按
IsaacLab ordering 使用：

> All joint data uses IsaacLab joint ordering (29 joints).

见
[`docs/source/references/observation_config.md`](../source/references/observation_config.md#motion-reference-observations)。

因此不能把“C++ 没有重排”解释为 C++ 支持 MuJoCo 输入；协议恰恰把重排责任
放在 sender/asset producer 一侧。

### 5.6 C++ ZMQ decoder 按 wire slot 原样解码

receiver 的核心循环是：

```cpp
for (int frame = 0; frame < num_frames; ++frame) {
    for (int joint = 0; joint < num_joints; ++joint) {
        std::memcpy(&val,
            pos_buf.data() + (frame * num_joints + joint) * sizeof(float),
            sizeof(float));
        decoded_joint_pos[frame][joint] = static_cast<double>(val);
    }
}
```

velocity 同样处理。见
[`zmq_endpoint_interface.hpp`](../../gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/zmq_endpoint_interface.hpp#L1004-L1058)。

这里的 `joint` 是 wire column index，不是 joint name；没有映射表参与。

### 5.7 C++ merger 再次按相同 index 原样复制

```cpp
motion->JointPositions(dst_frame_offset + frame)[joint] =
    data.joint_pos[frame][joint];
motion->JointVelocities(dst_frame_offset + frame)[joint] =
    data.joint_vel[frame][joint];
```

见
[`streamed_motion_merger.hpp`](../../gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/streamed_motion_merger.hpp#L464-L478)。

这一步也没有重排。

### 5.8 Encoder gatherer 直接复制当前 motion

position observation 的核心是：

```cpp
const auto motion_joint_pos = current_motion_->JointPositions(target_frame);
std::copy(
    motion_joint_pos,
    motion_joint_pos + num_joints,
    target_buffer.begin() + frame_offset
);
```

velocity observation 使用相同方式。见
[`g1_deploy_onnx_ref.cpp`](../../gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp#L764-L815)
和
[`g1_deploy_onnx_ref.cpp`](../../gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp#L821-L862)。

到这里，NPZ 第 `j` 列已经未经语义转换地成为 encoder 第 `j` 槽。

### 5.9 Action 最终会成为真机目标

C++ 明确把 policy output 当作 IsaacLab order，再映射到硬件/MuJoCo order，并
乘 `g1_action_scale`、加 `default_angles`：

```cpp
const double action_value =
    floatarr[isaaclab_to_mujoco[i]] * g1_action_scale[i];
motor_command_tmp.q_target.at(i) = default_angles[i] + action_value;
```

见
[`g1_deploy_onnx_ref.cpp`](../../gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp#L3226-L3261)。

所以本文报告的 `raw action` 差值不是直接的弧度差，但它会确定性地改变真机
关节目标。尤其腰、髋、踝的符号和幅度变化会直接影响质心与支撑脚策略。

## 6. 29 个槽位如何错位

下表中的“正确来源 MJ index”表示：若 encoder slot 是 IsaacLab index `i`，应该
从原始 MuJoCo vector 的哪个 index 取值。当前链路没有做这一步，而是直接取
MuJoCo index `i`。

| encoder slot | encoder 期望（IsaacLab） | 正确来源 MJ index | 当前未重排实际含义 | 状态 |
|---:|---|---:|---|---|
| 0 | `left_hip_pitch_joint` | 0 | `left_hip_pitch_joint` | 仅巧合正确 |
| 1 | `right_hip_pitch_joint` | 6 | `left_hip_roll_joint` | **错误** |
| 2 | `waist_yaw_joint` | 12 | `left_hip_yaw_joint` | **错误** |
| 3 | `left_hip_roll_joint` | 1 | `left_knee_joint` | **错误** |
| 4 | `right_hip_roll_joint` | 7 | `left_ankle_pitch_joint` | **错误** |
| 5 | `waist_roll_joint` | 13 | `left_ankle_roll_joint` | **错误** |
| 6 | `left_hip_yaw_joint` | 2 | `right_hip_pitch_joint` | **错误** |
| 7 | `right_hip_yaw_joint` | 8 | `right_hip_roll_joint` | **错误** |
| 8 | `waist_pitch_joint` | 14 | `right_hip_yaw_joint` | **错误** |
| 9 | `left_knee_joint` | 3 | `right_knee_joint` | **错误** |
| 10 | `right_knee_joint` | 9 | `right_ankle_pitch_joint` | **错误** |
| 11 | `left_shoulder_pitch_joint` | 15 | `right_ankle_roll_joint` | **错误** |
| 12 | `right_shoulder_pitch_joint` | 22 | `waist_yaw_joint` | **错误** |
| 13 | `left_ankle_pitch_joint` | 4 | `waist_roll_joint` | **错误** |
| 14 | `right_ankle_pitch_joint` | 10 | `waist_pitch_joint` | **错误** |
| 15 | `left_shoulder_roll_joint` | 16 | `left_shoulder_pitch_joint` | **错误** |
| 16 | `right_shoulder_roll_joint` | 23 | `left_shoulder_roll_joint` | **错误** |
| 17 | `left_ankle_roll_joint` | 5 | `left_shoulder_yaw_joint` | **错误** |
| 18 | `right_ankle_roll_joint` | 11 | `left_elbow_joint` | **错误** |
| 19 | `left_shoulder_yaw_joint` | 17 | `left_wrist_roll_joint` | **错误** |
| 20 | `right_shoulder_yaw_joint` | 24 | `left_wrist_pitch_joint` | **错误** |
| 21 | `left_elbow_joint` | 18 | `left_wrist_yaw_joint` | **错误** |
| 22 | `right_elbow_joint` | 25 | `right_shoulder_pitch_joint` | **错误** |
| 23 | `left_wrist_roll_joint` | 19 | `right_shoulder_roll_joint` | **错误** |
| 24 | `right_wrist_roll_joint` | 26 | `right_shoulder_yaw_joint` | **错误** |
| 25 | `left_wrist_pitch_joint` | 20 | `right_elbow_joint` | **错误** |
| 26 | `right_wrist_pitch_joint` | 27 | `right_wrist_roll_joint` | **错误** |
| 27 | `left_wrist_yaw_joint` | 21 | `right_wrist_pitch_joint` | **错误** |
| 28 | `right_wrist_yaw_joint` | 28 | `right_wrist_yaw_joint` | 仅巧合正确 |

只有 slot 0 和 slot 28 是 fixed point。于是每个 reference frame 有 27/29 个语义
槽位错误；encoder 同时采样 10 帧 position 和 10 帧 velocity，因此是：

```text
27 wrong slots/frame × 10 frames × 2 fields = 540 wrong semantic assignments
```

“540”是语义错位数；某些不同关节在某一帧可能刚好都为 `0`，所以以浮点值是否
相等统计时，本次 `d1p70` 的实际非零差值是 490。这两种统计不矛盾。

## 7. `d1p70` 第一帧的具体例子

审计对象：

```text
motion:
  gear_sonic/vigil_bridge/data/facee_chair_13s/d1p70.npz
sha256:
  9309af1078e3a43a61e689c2dc8f84535a5b0b7e01baf6f16f868c32fd60982e
shape / rate:
  650 frames × 29 joints, 50 Hz, 13 s
```

第一帧的前 10 个 encoder 槽位如下：

| slot | 正确的 IsaacLab 关节/值（rad） | 当前实际发送的 MuJoCo 关节/值（rad） |
|---:|---|---|
| 0 | `left_hip_pitch_joint` `+0.176505998` | `left_hip_pitch_joint` `+0.176505998` |
| 1 | `right_hip_pitch_joint` `+0.198895991` | `left_hip_roll_joint` `+0.014469783` |
| 2 | `waist_yaw_joint` `+0.030484870` | `left_hip_yaw_joint` `-0.002258260` |
| 3 | `left_hip_roll_joint` `+0.014469783` | `left_knee_joint` `+0.231315091` |
| 4 | `right_hip_roll_joint` `-0.134945601` | `left_ankle_pitch_joint` `+0.000405629` |
| 5 | `waist_roll_joint` `-0.022382680` | `left_ankle_roll_joint` `+0.007109104` |
| 6 | `left_hip_yaw_joint` `-0.002258260` | `right_hip_pitch_joint` `+0.198895991` |
| 7 | `right_hip_yaw_joint` `+0.002115651` | `right_hip_roll_joint` `-0.134945601` |
| 8 | `waist_pitch_joint` `+0.316459477` | `right_hip_yaw_joint` `+0.002115651` |
| 9 | `left_knee_joint` `+0.231315091` | `right_knee_joint` `+0.141257688` |

三个最直观的错误是：

1. encoder 以为 slot 1 是右髋 pitch `+0.198896`，实际收到左髋 roll
   `+0.014470`；
2. encoder 以为 slot 2 是腰 yaw `+0.030485`，实际收到左髋 yaw
   `-0.002258`；
3. encoder 以为 slot 8 是腰 pitch `+0.316459`，实际收到右髋 yaw
   `+0.002116`。

第三项尤其重要：reference 初始有明显腰 pitch，但当前输入几乎把它报告为零。
策略据此估计的躯干目标、质心和支撑状态自然会错误。

## 8. 可重复的 ONNX 反事实实验

### 8.1 实验问题

只问一个问题：

> 在 motion、模型、root orientation 和 robot proprioception 都不变时，只把
> reference 从正确 IsaacLab 排列替换成当前未重排 MuJoCo 排列，policy 输出会
> 改变多少？

这比比较两个物理 rollout 更干净，因为没有摩擦、接触、随机初态或求解器差异。

### 8.2 两个分支

正确分支：

```python
q_il  = q_mj[:, G1_MUJOCO_TO_ISAACLAB_DOF]
dq_il = dq_mj[:, G1_MUJOCO_TO_ISAACLAB_DOF]
```

当前错误分支：

```python
q_current  = q_mj
dq_current = dq_mj
```

两个分支都取 frame `0, 5, 10, ..., 45`，构造 10 帧 future reference。

### 8.3 严格固定的量

- reference root quaternion 和相对 6D orientation；
- `encode_mode=0` 的四维 C++ prefix；
- teleop/SMPL 等未选中的 encoder branch；
- decoder 的 10 帧 base angular velocity；
- decoder 的 10 帧 robot joint position；
- decoder 的 10 帧 robot joint velocity；
- decoder 的 10 帧 previous action；
- decoder 的 10 帧 projected gravity。

因此结果不是“真机状态不一样”造成的。唯一自变量是 580 个 reference
position/velocity 槽位的排列。

### 8.4 模型指纹

```text
encoder:
  gear_sonic_deploy/policy/facee_v73_noheight/model_encoder.onnx
  sha256 33c88827b13d0270a1e7f036b882284e6a2429e37f9bdb21b60c706d916ca849
  input [1, 1751] → output [1, 64]

decoder:
  gear_sonic_deploy/policy/facee_v73_noheight/model_decoder.onnx
  sha256 1bea1e29e8f243cc312e15fe0858c751d48e97ccbaa45ef419e6e4db210c14f7
  input [1, 994] → output [1, 29]
```

### 8.5 结果

| 对比层 | 维度 | 改变数量 | max abs | L2 |
|---|---:|---:|---:|---:|
| sampled reference | 580 | 490 个数值；540 个语义槽位 | 1.606396 | 7.488401 |
| full encoder input | 1751 | 490 | 1.606396 | 7.488401 |
| encoded token | 64 | 56 | 0.437500 | 1.629801 |
| raw action | 29 | 29 | 5.164087 | 7.139823 |

raw action 的最大变化包括：

| IsaacLab action index | joint | 正确排列 | 当前未重排 | delta |
|---:|---|---:|---:|---:|
| 22 | `right_elbow_joint` | +0.040760 | -5.123327 | -5.164087 |
| 21 | `left_elbow_joint` | -0.017255 | -3.852257 | -3.835002 |
| 8 | `waist_pitch_joint` | +0.943958 | -1.065753 | -2.009710 |
| 12 | `right_shoulder_pitch_joint` | -0.508310 | -2.032783 | -1.524473 |
| 11 | `left_shoulder_pitch_joint` | -0.609983 | -1.906437 | -1.296454 |
| 14 | `right_ankle_pitch_joint` | -0.198105 | +0.387741 | +0.585846 |
| 0 | `left_hip_pitch_joint` | +0.808239 | +0.363309 | -0.444930 |
| 5 | `waist_roll_joint` | -0.194470 | -0.544332 | -0.349862 |

注意：这些是 decoder 的 normalized/raw action，不可直接当成关节弧度。C++ 会
再按每个硬件关节乘 `g1_action_scale` 并加 `default_angles`。但 action 的符号、
相对幅度和最终 motor target 都已经改变。

早期一次性诊断使用了略有不同的固定 observation fixture，得到的最大 raw action
差是 `5.1666`；本次纳入仓库、可重复执行的审计得到 `5.164087`，两者相差约
`0.00254`（约 `0.05%`）。这不是根因结论变化，而是 decoder 的其余固定输入值
略有不同。后续引用应以本脚本输出和模型 SHA256 为准。

### 8.6 复现命令

脚本：
[`tools/analysis/audit_facee_reference_joint_order.py`](../../tools/analysis/audit_facee_reference_joint_order.py)

当前机器的普通 `python3` 没有 `onnxruntime`，使用已恢复的 Sonic 环境：

```bash
cd /workspace/fangs1@xiaopeng.com/workspace_fs/GR00T-WholeBodyControl_Vigil

/opt/conda/envs/sonic/bin/python \
  tools/analysis/audit_facee_reference_joint_order.py \
  --tag d1p70 \
  --json-output /tmp/facee_joint_order_audit_d1p70.json
```

预期关键输出：

```text
status: BUG_CONFIRMED
mapping: 27/29 slots mismatch per frame; unchanged slots=[0, 28]
sampled semantic assignments changed: 540/580
encoded_token_64: changed=56, max_abs=0.4375, l2=1.6298006
raw_action_29: changed=29, max_abs=5.16408721, l2=7.1398228
```

脚本具有以下可审计特性：

- 从 `g1.py` AST 读取真实映射，不 import Isaac Lab，也不复制一份隐藏映射；
- 校验 NPZ SHA256 与 manifest 一致；
- 记录 ONNX SHA256、Git HEAD、采样帧和完整 29 槽映射；
- `--json-output` 可保存机器可读证据；
- 不启动仿真、不连接 ZMQ、不写机器人、不执行硬件动作。

## 9. 为什么以前的 MuJoCo 视频会成功，而真机会失败

已有 FaceE MuJoCo runner 并没有直接把这个 NPZ 的 MuJoCo-order 29 列送入
encoder。它从源 PKL 读取 `q_mujoco/dq_mujoco` 后，显式执行：

```python
q_isaaclab = q_mujoco[:, sonic.MJ2IL]
dq_isaaclab = dq_mujoco[:, sonic.MJ2IL]
```

代码位于相邻仓库：

```text
agent_sim_r2s_ego_exp_real/
  demo/experiments/g1_sonic/run_sonic_mujoco_sit_chair.py:559-560
```

而当前真机链路是：

```text
FaceE NPZ (MJ) → ZMQ raw copy → C++ raw copy → encoder assumes IL
```

所以两条链路并不等价：

| 环节 | 已验证 MuJoCo runner | 当前真机 bridge |
|---|---|---|
| 原始 reference | MuJoCo order | MuJoCo order |
| 进入 encoder 前重排 | **有**，`q[:, MJ2IL]` | **没有** |
| encoder 实际收到 | IsaacLab order | MuJoCo order 被误解释为 IsaacLab |
| 物理之前的 policy action | 正常链路 | 已显著改变 |

因此“MuJoCo 能站稳”不能反证真机输入正确。它只说明：在正确重排和该 MuJoCo
物理配置下，policy 对部分 motion 有积极结果。

后续 sim2real 验证必须让 MuJoCo 也经过同一个 catalog、ZMQ protocol v1、C++
decode/merge/observation 逻辑，或至少逐字节/逐 observation 与 C++ 链路做 parity。

## 10. 为什么会表现为后退、走不动和初态发弓

这是由网络输入语义推导出的机制解释，不是额外的实验结论：

1. 左右髋 pitch 不再成对出现。slot 1 本应是右髋 pitch，却收到左髋 roll；
   gait phase 的双腿关系被破坏。
2. 腰 yaw、roll、pitch 分别收到左髋 yaw、左踝 roll、右髋 yaw；躯干 orientation
   与质心控制所依据的 reference 不再是真实动作。
3. 膝和踝 reference 被交换到另一条腿或上肢槽位，抬脚和支撑脚的时序含义被破坏。
4. velocity 也发生同样排列错误，policy 不只误判“目标姿态”，还误判“目标正在
   向哪个方向运动”。
5. decoder 给出的腰 pitch、髋 pitch、踝 pitch action 明显变化；经过 action
   scale 和 PD tracking 后，机器人会用额外步伐寻找平衡，外观上就是后退、
   不能按 reference 小步向前或保持发弓姿态。

这条因果链发生在座椅接触之前，因此不能只用“真机地面摩擦不同”解释。

## 11. 正确修复位置

### 11.1 推荐：在资产打包边界转换一次

推荐在 `build_facee_chair_catalog.py` 中，FK 输出后、写 NPZ 前做一次且只做一次：

```python
joint_pos_mujoco = fk.dof_pos[0].cpu().numpy().astype(np.float32)
joint_vel_mujoco = fk.dof_vels[0].cpu().numpy().astype(np.float32)

mapping = np.asarray(G1_MUJOCO_TO_ISAACLAB_DOF, dtype=np.int64)
joint_pos = joint_pos_mujoco[:, mapping]
joint_vel = joint_vel_mujoco[:, mapping]
```

理由：

- protocol v1 已经定义 sender 发送 IsaacLab order；
- catalog 资产生成后即可自描述、校验并被多个 sender 安全复用；
- 不会改变已正确遵守 protocol v1 的其他 publisher；
- Python、C++ 和真机 runtime 不需要为 FaceE 加特殊分支。

### 11.2 Manifest 必须声明语义

建议每条 motion 和 catalog 顶层都增加机器可检查字段：

```json
{
  "source_joint_order": "mujoco",
  "joint_order": "isaaclab",
  "joint_order_mapping": "G1_MUJOCO_TO_ISAACLAB_DOF",
  "protocol_version": 1
}
```

`ChairMotionCatalog` 应拒绝：

- 缺少 `joint_order`；
- `joint_order != "isaaclab"`；
- protocol version 与 asset contract 不一致。

Shape `[650, 29]` 不足以保护语义契约。

### 11.3 必须重新生成资产和 checksum

转换后 18 个 NPZ 的内容都会变化，因此必须：

1. 重新生成所有 `d1p15` 到 `d2p00` NPZ；
2. 重算每个 NPZ SHA256；
3. 更新 manifest；
4. 不得保留旧 checksum 或只修改 manifest 描述；
5. 将旧资产标记为 invalid，不允许静默回退。

position 和 velocity 必须同时重排；只修 position 会留下另一半错误输入。

## 12. 不推荐的修复方式

### 12.1 不要在 C++ receiver 中无条件重排

protocol v1 已要求输入是 IsaacLab order。如果 C++ 对所有 v1 消息再做一次
MuJoCo → IsaacLab，会破坏其他正确 publisher，并让修复后的 FaceE 被二次重排。

只有设计新的、带明确 `joint_order` 元数据的 protocol version 时，receiver 才能
根据元数据选择转换。当前最小正确修复是 asset producer 遵守既有 v1 契约。

### 12.2 不要依靠 action 后处理补偿

这个错误同时污染 10 帧 q、10 帧 dq 和 encoder token，是高维非线性输入错误。
在 decoder 后对 waist 或髋增加固定 bias，无法恢复正确的 gait phase，也会使不同
reference 的行为更不可预测。

### 12.3 不要因某段视频“看起来还能动”就接受

不同关节的值在局部时刻可能恰好接近或为零；slot 0 和 28 也碰巧不变。网络仍
可能输出某种可站立动作，但这不是 reference tracking 的证据。

### 12.4 不要把修复混入模型重训

不应该让 policy 学会适应错误排列。那会创建一套只适用于错误部署链路的新隐式
协议，并破坏与训练、IsaacLab、MuJoCo 和官方工具的兼容性。先修接口，再决定
是否需要 sim2real 训练。

## 13. 修复后的验收门槛

### 13.1 静态资产测试

1. 映射互逆：

   ```python
   np.array_equal(
       G1_ISAACLAB_TO_MUJOCO_DOF[G1_MUJOCO_TO_ISAACLAB_DOF],
       np.arange(29),
   )
   ```

2. 使用非对称 sentinel，例如 `q[t, j] = 1000*t + j`，保证 29 个槽位都能唯一
   识别，不能用全零动作测试顺序。
3. 对 q 和 dq 分别测试。
4. 校验 manifest `joint_order=isaaclab`。
5. 校验 18 条 motion 的 shape、finite、fps、frame count 和新 checksum。

### 13.2 Python asset-to-wire 测试

1. 从 catalog 加载修复后的 NPZ；
2. 捕获 `send_reference_motion()` 的 wire bytes；
3. 解码第一帧并逐 joint name 对照正确 IsaacLab vector；
4. 对 position 和 velocity 都执行；
5. 确保没有第二次转换。

### 13.3 C++ observation parity

至少加入一个 debug/offline test：

1. Python 生成已知 protocol-v1 message；
2. C++ `ZMQEndpointInterface` 解码；
3. `StreamedMotionMerger` 写入 `MotionSequence`；
4. gather 10-frame step-5 observation；
5. 将 C++ 580 值与 Python 的 `q_il/dq_il` bitwise 或严格 tolerance 对比；
6. encoder token 与 Python ONNX Runtime 对比；
7. decoder raw action 对比。

只有比较最终视频不够，因为视频无法定位错位发生在哪一层。

### 13.4 部署等价 MuJoCo 回归

修复后的 MuJoCo 验证必须满足：

- reference 来自同一个 `facee_chair_13s` catalog；
- 使用同一个 protocol-v1 semantic contract；
- 使用同一个 1751 encoder / 994 decoder observation layout；
- 使用同一个 50 Hz reference cursor 和 hold-last-frame 规则；
- 记录第一控制周期和前 1 秒的 encoder input/token/action；
- 和 C++ runtime 的离线 trace 对齐。

不能继续使用“源 PKL → runner 内部偷偷重排”的成功视频作为真机链路 parity 证据。

### 13.5 真机前安全门槛

完成以上静态、wire、C++ 和 MuJoCo parity 后，仍需单独检查：

- 真机进入 motion 前的 q/dq 与 reference 首帧差；
- 起步时是否从 reference q0 初始化/平滑过渡；
- 50 Hz reference 与控制周期是否一致；
- action scale、default angles、joint limits 和硬件 order；
- 失衡、后退、限位和通信超时的急停规则。

这些通过之前，修复 joint order 只代表“policy 收到了正确 reference”，不代表
完整真机坐椅任务已经安全。

## 14. 证据类型与可信度

| 结论 | 证据类型 | 可信度 |
|---|---|---|
| Motion PKL 是 MuJoCo order | 项目约定文档（upstream contract） | 高 |
| FaceE 打包器未转换 | 当前分支源码（local implementation） | 确定 |
| Protocol v1 要求 IsaacLab | 协议文档（upstream contract） | 确定 |
| Python/C++ 中间层均未转换 | 当前分支源码（mixed upstream/local path） | 确定 |
| 27/29 槽位语义错位 | 官方映射直接推导（derived） | 确定 |
| token/action 大幅变化 | 固定其他输入的 ONNX 实验（derived experiment） | 可重复、确定 |
| 错位足以造成异常平衡动作 | action 差值 + 控制链推理（derived） | 高 |
| 修复后真机一定成功 | 尚无证据 | **不能下结论** |

## 15. 最终判断

当前最确定的根因不是“MuJoCo 摩擦比真机理想”，而是两条被拿来比较的链路在
进入 encoder 前已经不同：

```text
验证 MuJoCo：MuJoCo reference → 显式 MJ2IL → encoder
当前真机：  MuJoCo reference → 无重排       → encoder
```

这使得当前真机实验无法用来评价 v73 是否具备小步向前或坐椅能力，因为 policy
根本没有收到与验证视频相同语义的 reference。应先在 FaceE asset packaging
边界完成一次 MuJoCo → IsaacLab 转换，重新生成 18 条资产，然后通过 wire/C++
observation parity 和部署等价 MuJoCo 回归，最后才进入下一轮真机验证。
