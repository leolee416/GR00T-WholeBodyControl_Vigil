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
