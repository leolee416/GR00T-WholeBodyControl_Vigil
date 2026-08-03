# run-repo 进度：Task14 SONIC 多椅距真机 reference

## Phase -1：定位与恢复

- 状态：完成
- 仓库：`GR00T-WholeBodyControl_Vigil`
- 基线：`Vigil_WBC_dev@3ec1d9fc623dfc4bc8b040dc8bb48d46653319da`
- 目标分支：`er2s_ego_exp2`
- 基线工作区：创建分支前 clean
- 外部正式数据源：
  `GRAIL/out/faceE_all_success_hold13_selected_v64/results.json`

## Phase 0：仓库与调用链盘点

- 状态：完成
- 当前范围：Docker 真机启动、SONIC C++ deploy、Vigil bridge、
  reference 加载/播放、真实机器人安全状态。
- 明确排除：训练、论文、benchmark judge、无关音频/LED功能。
- 证据和问题同步记录：
  `/workspace/fangs1@xiaopeng.com/workspace_fs/fs_tasks/task14_process.md`

## Phase 1：环境和离线验证

- 状态：完成
- 原则：沿用仓库 Docker 启动方式；不修改真机宿主环境；不执行硬件动作。
- `upstream`：现有 ZMQManager 已支持 `planner=false` 与 protocol-v1
  streamed motion；未改受保护 C++。
- `derived`：使用 GRAIL 自带 FK/50 Hz 插值生成 18 个 650 帧 NPZ。
- `local_patch`：新增向上 5 cm 距离映射 catalog、`sonic.sit_chair`、launcher 参数与
  fail-closed real executor。

## Phase 2：功能 smoke

- 状态：完成
- 预期验证：18 条 manifest 全部可解析；距离按 ceiling 规则映射；越界距离
  fail closed；fake transport 能收到唯一选定 motion；异常触发 halt。

## Phase 6：复现文档

- 状态：完成
- 预期产物：分支内中文 `run.md`、Docker/真机操作说明、输入资产哈希与测试证据。

## 2026-07-30 验证快照

- 运行时资源：18 条、1.15–2.00 m、0.05 m 网格、650 帧/条、约 1.5 MiB。
- catalog/packing/safety 新增测试：11/11 通过。
- Vigil bridge + catalog + packing 最终回归：59/59 通过。
- dry-run HTTP：`1.80 -> d1p80` 成功；按用户确认更新 ceiling 规则后，
  `1.12 -> 1.15`、`1.17 -> 1.20`、`1.951 -> 2.00` 均通过。
- 未验证边界：当前 worktree 没有真实 ONNX runtime assets，且没有真机动作授权。

## 2026-08-03：MuJoCo bridge / FaceE 坐椅链路复核

- 状态：完成诊断；FaceE 物理坐椅链路尚未接通。
- 真实 headless MuJoCo 已启动，并通过 Unitree DDS 读到 `rt/lowstate` 和
  `rt/odostate`；HTTP MuJoCo adapter 可返回真实仿真 robot state。
- 直接请求 `sonic.sit_chair` 被拒绝：MuJoCo executor 没有加载配置中已经传入的
  chair catalog。
- 仅为诊断手工注入 catalog 后，请求虽然返回 `ok=true`/650 帧/13 秒，但仍调用
  dry-run 实现，ZMQ 实测 `command/pose` 消息数为 0。
- MuJoCo/catalog/packing/joint-order 定向回归：`39 passed in 2.02s`；现有测试未
  覆盖 MuJoCo sit 必须真实发布 pose 的契约。
- 默认 `scene_43dof.xml` 只有 G1 和地面，没有椅子；仓库 v2 catalog 也是旧的 v73
  集合，不是 E0019 exact-v3 最终 reference 集合。
- 完整证据与建议修复顺序：
  `docs/integration/vigil_mujoco_facee_validation_20260803.md`。

## 2026-08-03：FaceE E0019 仓库内 MuJoCo 实施

- 状态：第 1–5 层已完成；C++ actor 物理闭环受当前机器构建环境阻塞。
- MuJoCo executor 已加载 catalog，并 override `play_sonic_reference_motion()`；
  response 为 `controller=mujoco_zmq_wbc`、`dry_run=false`。
- 实测 HTTP d1p50 下发 11 个 `planner=false` command 和唯一一个 650 帧 pose；
  pose payload 为 166409 bytes。
- E0019 exact-v3 18 条 catalog 已打包：
  `gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json`。
- E0017 iteration-8 配套 ONNX 已导出并通过 parity：
  `gear_sonic_deploy/policy/facee_e0019_exact_v3/`。
- 新 scene 使用 E0019 原始 258 顶点/512 三角形无靠背 proxy，椅面 top=0.41 m；
  默认 floor-only scene 未改变。
- 1.15/1.50/2.00 场景实例化通过；DDS `rt/lowstate` 和 `rt/odostate` 实收通过；
  重复 DDS Domain 初始化已经消除。
- 定向测试 `52 passed`；完整 bridge + FaceE 集合 `100 passed, 1 failed`，唯一失败
  是既有 audio WebSocket / websockets 12 兼容问题。
- 当前没有物理 rollout 视频或成功率：本机无 `g1_deploy_onnx_ref`、TensorRT 与
  ONNX Runtime C++ 环境，严格保留为 blocker，不把 reference dispatch 当 rollout。

## 2026-08-03：C++ deploy 阻塞范围与环境诊断

- MuJoCo scene、接触动力学和 DDS 本身不受阻；受阻的是
  `lowstate/reference -> actor inference -> lowcmd` 的策略闭环。
- 当前机器是 Ubuntu 22.04 x86_64、4×A100，NVIDIA driver 580.105.08，CUDA
  Toolkit 12.1 与 `nvcc` 已存在。
- 当前缺少仓库契约要求的 x86_64 TensorRT 10.13（`TensorRT_ROOT` 为空），也没有
  TensorRT C++ headers/libraries。
- Conda 环境只有 Python wheel 携带的 `libonnxruntime.so`，没有
  `onnxruntime_cxx_api.h` 和 CMake package，不能用于当前 C++ build。
- 还缺少 `just`、`clang`、`libmsgpack-dev`、`libzmq3-dev`；Eigen3 与 zlib
  development package 已存在。
- 正确恢复顺序：持久目录安装 TensorRT 10.13 和 ONNX Runtime C++ 1.16.3，补齐
  小型系统构建依赖，设置 `TensorRT_ROOT`、`onnxruntime_ROOT`、
  `CUDAToolkit_ROOT` 与 `LD_LIBRARY_PATH`，编译后先用 `ldd` 做链接门禁，再运行
  `deploy.sh sim`。全程不需要连接真机。

## 2026-08-03：C++ runtime 恢复与 d1p50 仓库闭环

- 状态：实现与代表性验证完成；strict no-kick 仍保留证据边界。
- 持久 C++ runtime 已恢复到
  `/workspace/fangs1@xiaopeng.com/workspace_fs/sonic_cpp_runtime_x86_64_20260803`，
  C++ actor 已完成 TensorRT encoder/decoder 推理和 loopback DDS lowcmd。
- 修正四个数值契约：MuJoCo 3.3.4 pin、静态 chair 在 MjSpec compile 前放置、
  reference reset 不修改 `model.qpos0`、floating-base reset 保持 float64。
- scene/deploy/bridge/request 完整链路对 d1p50 跑满 650 action samples；无倒地，
  最终 pelvis z=0.5509 m，最后 2 秒高度波动小于 0.07 mm。
- 生成 13.00 秒三栏视频：
  `outputs/vigil_rollouts/20260803T082456_135023Z_sonic_sit_chair_auto/`
  `d1p50_reference_isaac_repo_mujoco.mp4`。
- 当前严格边界：waist pitch 单 sample 超限 0.000285 rad；recorder 未保存每 physics
  substep contact force，所以不能把仓库链标为 strict/no-kick PASS。
- 最终定向 Python 回归：52 passed（使用 Sonic Python 与固定的 MuJoCo 3.3.4
  `PYTHONPATH`）。
- 完整复现文档：
  `docs/integration/facee_e0019_mujoco_sim_validation_workflow.md`。
