# 【历史诊断】Vigil 仓库内 MuJoCo / FaceE exact-v3 实施记录（2026-08-03）

> 本页保留的是 C++ runtime 尚未恢复时的阶段记录，其中“C++ actor 阻塞、没有
> closed-loop rollout”的结论已经过期。当前机器已恢复 C++/TensorRT/ORT 环境，
> d1p50 已跑满 13 秒并生成三栏视频。请以
> [FaceE E0019 模型的 MuJoCo Sim 完整验证流程](facee_e0019_mujoco_sim_validation_workflow.md)
> 为准，不要继续按本页的旧 blocker 判断当前状态。

## 结论

这条链路是纯仿真链路，不依赖真机。代码已经完成以下部分：

1. `backend=mujoco` 会真正加载 FaceE catalog；
2. `sonic.sit_chair` 会向 C++ deploy 发送 `planner=false` 启动 burst 和唯一一个
   protocol-v1 `pose`，不再继承 dry-run 假成功；
3. E0019 exact-v3 的 18 条最终 reference 已打包成 650 帧、50 Hz、13 秒 NPZ，
   FK 后的 29 关节数据只做一次 MuJoCo→IsaacLab 重排；
4. 与 reference 配套的 E0017 iteration-8 actor 已导出并打包成新的显式 opt-in
   encoder/decoder ONNX；旧 v73 与默认 release 均未被覆盖；
5. 新增独立 FaceE MuJoCo scene：0.41 m 椅面、0.50×0.45 m、无靠背，并复用
   E0019 经审计的 258 顶点/512 三角形碰撞 mesh；
6. 场景和 reference 共用同一 manifest 的距离选择与椅子位姿。`1.12→1.15`、
   `1.17→1.20`，不会出现“场景选一条、actor 收另一条”的漂移；
7. 仓库 MuJoCo 已实际发布 `rt/lowstate` 和 `rt/odostate`，HTTP→ZMQ reference
   下发也已实测。

当前仍不能宣称“仓库内 MuJoCo closed-loop rollout 成功”：本机缺少预编译
`g1_deploy_onnx_ref`、TensorRT/ONNX Runtime C++ 开发库，也没有 Docker runtime，
因此中间的 C++ SONIC actor 物理闭环尚未启动。bridge 返回的是明确的
`completion_source=reference_dispatched`，不会把下发成功冒充成坐椅成功。

## 四层边界

```text
E0019 reference catalog + chair layout
            |
            v
HTTP bridge -- ZMQ command/pose --> C++ g1_deploy_onnx_ref / SONIC actor
                                      |
                                      v
                             Unitree lowcmd over DDS
                                      |
                                      v
                         repository MuJoCo + chair scene
                                      |
                         lowstate / odostate / g1_debug
```

| 层级 | 状态 | 证据 |
| --- | --- | --- |
| exact-v3 资产与 joint order | 通过 | 18 条均能校验并加载为 `(650,29)` |
| FaceE MuJoCo 场景 | 通过 | 1.15/1.50/2.00 m 实例化；E0019 mesh 顶点数 258 |
| MuJoCo DDS | 通过 | 独立进程收到 `rt/lowstate`、`rt/odostate` |
| HTTP→ZMQ reference | 通过 | d1p50 收到 11 个 command、1 个 650 帧 pose |
| C++ actor→MuJoCo closed loop | 阻塞 | 当前机器缺 C++ deploy 可执行文件及链接环境 |
| 13 秒 rollout / 视频 / 成败 | 未执行 | 不用 reference dispatch 替代物理证据 |

## 策略与 reference 必须成对

### Actor

完整训练 checkpoint：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/facee_crosssim_dual_agent_v1/rounds/
R0008_mujoco_warp_torch/E0017_mjwarp_shared_ppo/evidence/
s0_iter0008_newbest/full_checkpoint/model_step_mjwarp_s0_iter0008.pt
```

SHA-256：

```text
3727209b0340fc31c499d1e069386c7e10297fce80adb4f87e9db10e840e7650
```

仓库内 opt-in ONNX：

```text
gear_sonic_deploy/policy/facee_e0019_exact_v3/model_encoder.onnx
gear_sonic_deploy/policy/facee_e0019_exact_v3/model_decoder.onnx
gear_sonic_deploy/policy/facee_e0019_exact_v3/observation_config.yaml
gear_sonic_deploy/policy/facee_e0019_exact_v3/manifest.json
```

模型接口与导出 parity：

| 模块 | 接口 | SHA-256 | PyTorch/ONNX 最大误差 |
| --- | --- | --- | --- |
| encoder | `1×1751 → 1×64` | `1cf476...e459` | `0` |
| decoder | `1×994 → 1×29` | `5464bd...7382` | `1.1921e-6` |

这是一个共享 actor，没有距离/reference selector、没有按距离分支，也没有输入
height map、vision、椅子位姿或 task ID。`encoder_mode_4` 是 SONIC 原有的
g1/teleop/smpl 输入模态标记，不是距离 selector。启动 history 契约是
`repeat_reset`。

### Reference catalog

```text
gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json
gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/d1p15.npz
...
gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/d2p00.npz
```

来源 task set：

```text
/workspace/fangs1@xiaopeng.com/workspace_fs/facee_crosssim_dual_agent_v1/rounds/
R0008_mujoco_warp_torch/E0019_crop_transfer_search/taskset_exact_v3_all18.json
```

task set SHA-256：

```text
35eed93b181ad0c4e1a4fc498f2cd05f99d99ce3e28c2a65ccd9e68819ddba1d
```

PKL 边界是骨架数据；`Humanoid_Batch.fk_batch()` 输出 MuJoCo/MJCF order；部署
NPZ 在序列化前用 `G1_MUJOCO_TO_ISAACLAB_DOF` 重排一次，因此 ZMQ 和 actor 收到
的是 IsaacLab order。每个 NPZ 都显式记录：

```text
source_joint_order = mujoco
joint_order = isaaclab
protocol_version = 1
```

构建工具：

```text
tools/build_facee_exact_v3_catalog.py
tools/package_facee_e0019_exact_v3_policy.py
```

## 椅子几何、距离与朝向

场景：

```text
gear_sonic/data/robot_model/model_data/g1/scene_facee_backless.xml
```

碰撞 mesh：

```text
gear_sonic/data/robot_model/model_data/g1/meshes/facee_backless_41cm_proxy.obj
```

mesh SHA-256：

```text
47bf22f5589627e4cb04c43bf7625e9f96a464027344a8b28ab1be11f970b8cd
```

它从 E0019 权威 MuJoCo scene 中逐顶点/三角形提取，不是临时 box 近似。几何范围
为 X `[-0.225,0.225]`、Y `[-0.25,0.25]`、Z `[0,0.41]` m；没有靠背。

manifest 中每条 motion 都保存 `chair_layout`。`center_xy_m` 与 `yaw_rad` 是把
E0019 世界坐标的椅面中心/朝向变换到该条 reference 第 0 帧机器人 heading frame
后的值。仓库 MuJoCo 的机器人从 XY=0、yaw=0 启动，所以可以直接设置固定 chair
body 的 pos/quat。

注意：历史名称中的 `d1p50` 不是“初始 pelvis 到椅面中心欧氏距离恰好 1.50 m”。
这是原数据生成链沿机器人到椅子的审计轴定义的 nominal grid。场景必须使用
manifest 里的完整 XY 和 yaw，不能自行把椅子简化为 `(1.50,0)`。

实测三条布局：

| tag | center XY in initial robot frame (m) | yaw (rad) | center norm (m) |
| --- | --- | --- | --- |
| d1p15 | `(1.021458, 0.119513)` | `-2.984804` | `1.028426` |
| d1p50 | `(1.305758, 0.383481)` | `-2.807409` | `1.360904` |
| d2p00 | `(1.861927, -0.003790)` | `-3.030455` | `1.861931` |

场景实现：

- `configs.py` 只在显式传入 `--robot-scene` 时覆盖默认 scene；
- 只在显式传入 `--facee-chair-distance-m` 时启用 FaceE chair；
- `facee_chair_scene.py` 与 reference catalog 共用 ceiling 选择函数；
- 默认 `scene_43dof.xml`、release policy 和真机 launcher 都保持不变。

## 修复的原始两个缺口

### Catalog 加载

`MujocoPrimitiveExecutor.__post_init__()` 现在按
`MujocoBridgeConfig.chair_motion_catalog` 构造 `ChairMotionCatalog`。未配置时仍然
fail closed。

### 禁止 dry-run 假成功

`MujocoRuntimeClient.play_sonic_reference_motion()` 现在：

1. 校验 frames payload；
2. 重复发送 `command(start=true, stop=false, planner=false)`，覆盖 PUB/SUB slow join；
3. 发送唯一一个 protocol-v1 `pose` chunk；
4. 返回 `reference_dispatched`，不声称动作已经坐稳。

`MujocoPrimitiveExecutor` 已 override 同名方法，成功 telemetry 为：

```text
controller = mujoco_zmq_wbc
dry_run = false
completion_source = reference_dispatched
settled = false
```

异常路径调用 `halt()` 并返回失败。

## 实际运行证据

### MuJoCo scene + DDS

仓库 MuJoCo 进程启动后，独立 subscriber 收到：

```text
rt/lowstate received=True, motor_state=35
rt/odostate received=True, position_z≈0.964522 m
```

早期日志中的重复 DDS Domain 初始化警告来自 `run_sim_loop.py` 和
`BaseSimulator` 各初始化一次。本次移除了前者未使用的重复调用，复跑后警告消失，
DDS 两个 topic 继续正常收到。

### HTTP → ZMQ d1p50

实际启动 MuJoCo 与 HTTP bridge 后请求 d=1.50 m，response 为：

```text
ok=true
controller=mujoco_zmq_wbc
dry_run=false
tag=d1p50
motion_name=faceE_crop_targetd1p50_refd1p90_f060_hold13s_e0019
frame_count=650
completion_source=reference_dispatched
```

独立 ZMQ subscriber 实收：

```text
command messages = 11
pose messages = 1
pose count = 650
pose data bytes = 166409
joint_pos shape = [650,29]
joint_vel shape = [650,29]
body_quat_w shape = [650,4]
```

首次 `reset_episode` 若把 state timeout 人为缩短为 2 秒，可能早于 CycloneDDS discovery
而返回暂时无状态；数秒后 action 的 `robot_state_after.source=rt/odostate`。提供的运行
脚本使用 10 秒 timeout。

## 复现命令

统一入口：

```bash
cd /workspace/fangs1@xiaopeng.com/workspace_fs/GR00T-WholeBodyControl_Vigil
```

四个终端依次运行：

```bash
tools/run_facee_e0019_repo_mujoco.sh scene 1.50
tools/run_facee_e0019_repo_mujoco.sh deploy
tools/run_facee_e0019_repo_mujoco.sh bridge
tools/run_facee_e0019_repo_mujoco.sh request 1.50
```

`scene`、`bridge` 和 `request` 已在当前机器验证。`deploy` 当前会停在本机缺少
TensorRT/ONNX Runtime C++ build/runtime 的环境条件；在标准部署 Docker 或具备同等
依赖的机器上运行。

该 helper 的 deploy 子命令写死 `deploy.sh sim`，不会切到真机。

## 自动化验证

定向回归：

```text
52 passed in 3.81s
```

包含 scene、exact-v3 policy、18 条 catalog、MuJoCo adapter、ZMQ packing 和 joint
order。

完整 bridge + FaceE 相关集合：

```text
100 passed, 1 failed in 16.83s
```

唯一失败是既有 audio WebSocket 测试与当前 `websockets==12.0` async context-manager
写法不兼容，和 FaceE/MuJoCo 改动无关。

## 后续唯一正确的验收

补齐 C++ deploy 环境后，先运行 1.15、1.50、2.00 m：

1. `g1_debug` 必须持续产生 policy action 与关节状态；
2. 每条必须跑满 13 秒；
3. 检查座面接触、末 2 秒平衡、躯干倾角、关节限位和落座前小腿/椅子接触；
4. 生成 reference / E0019 Isaac / repository MuJoCo 三栏视频；
5. 三条通过后扩展 18 条。

只有这些闭环指标和视频才能称为 repository-mode MuJoCo rollout。当前完成的是
实现、资产契约、场景、DDS 和 reference transport，不是物理成败结论。
