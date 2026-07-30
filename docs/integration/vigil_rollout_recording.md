# Vigil Bridge 真机 Rollout 录制与导出

Vigil Bridge 可以从 deploy 已有的 `g1_debug` ZMQ 流录制 G1 的实测关节状态。默认只保留：

- `sonic.sit_chair` 开始前最多 3 秒；
- 完整坐下动作期间；
- 动作结束后最多 3 秒。

动作后窗口结束时自动导出：

- `raw_real_rollout.npz`：未经重采样的真机原始记录。
- `3dgs_replay.npz`：3DGS/可视化侧运动学回放输入。
- `gear_sonic_reference.npz`：平滑并重采样后的 Gear-Sonic reference 候选。
- `manifest.json`：文件路径、完整性和可用性说明。

实现完全位于 Bridge 层，不会修改或启动底层控制、policy inference 或真机安全路径。

## 数据语义

Bridge 从 `g1_debug` 读取：

- `body_q/body_q_measured[29]`：LowState 派生的实测关节角，MuJoCo 顺序。
- `body_dq[29]`：LowState 派生的实测关节速度，MuJoCo 顺序。
- `base_quat[4]`、`base_ang_vel[3]`：IMU 姿态和角速度。
- `body_q_target[29]`：当前 reference 的目标关节角。
- `last_action[29]`：已映射到 MuJoCo 顺序、乘 scale 并加 default offset 的 policy 关节位置命令；不是网络原始 tensor。

`g1_debug.base_trans_measured` 是 deploy 中固定的可视化占位值，Recorder 明确忽略它。`base_xyz` 只能由外部定位接口写入，且必须携带来源，例如 VIO、动捕、外部里程计或 3DGS 相机定位。缺失或过期时导出 `NaN` 和 `valid=false`，不会伪造轨迹。

`--enable-motion-recording` 记录 reference/target，不代表真机实际执行结果；本功能记录的是 `g1_debug` 中的实测状态。底层 `--enable-csv-logs` 可以同时开启，作为独立的原始日志备份。

## 启动条件

deploy 需要已经由操作员按真机流程启动，并发布 `g1_debug`（通常为 `--output-type all` 或包含 ZMQ 输出）。Bridge 本身不会自动启动真机运动。

示例：

```bash
python gear_sonic_deploy/scripts/run_vigil_bridge.py \
  --backend real \
  --host 0.0.0.0 \
  --state-host localhost \
  --state-port 5557 \
  --state-topic g1_debug \
  --rollout-output-dir outputs/vigil_rollouts
```

外部定位默认超过 0.5 秒即视为过期。可通过
`--rollout-localization-max-age 0.5` 调整。

## 默认坐下窗口

Recorder 在未录制时只维护一个最多 3 秒的内存环形缓存，不会把更早的走路、
等待或调试 pose 写入导出文件。

当 Bridge 收到 `/execute_action` 的 `sonic.sit_chair` 时：

1. 自动创建 session，无需预先调用 `/rollout/start`。
2. 把最近最多 3 秒缓存作为 `pre`。
3. 记录动作调用期间为 `action`。
4. 动作完成后继续记录最多 3 秒作为 `post`。
5. 自动停止并生成三个 NPZ 和 `manifest.json`。

坐下 reference 是异步下发的，因此 Bridge 不会把“发送完 reference”误当成动作结束：
它使用所选 motion 的 `duration_s` 延长 `action` 窗口，motion 播放完成后才开始
计算 3 秒 post-roll。

`raw_real_rollout.npz` 中的 `capture_phase` 会逐帧标记
`pre/action/post`。`manifest.json` 会记录实际保留的
`recorded_pre_roll_s`、`recorded_post_roll_s` 和各阶段样本数。

`pre_roll_s` 和 `post_roll_s` 都被硬限制在 `[0, 3]` 秒，不能配置得更长。

## 一次录制流程

### 1. 可选：预先设置 session 元数据

```bash
curl -X POST http://127.0.0.1:8765/rollout/start \
  -H 'Content-Type: application/json' \
  -d '{
    "session_name": "walk_to_chair_001",
    "episode_id": "real_001",
    "checkpoint": "/path/to/policy.onnx",
    "capture_mode": "skill_window",
    "capture_skill": "sonic.sit_chair",
    "pre_roll_s": 3,
    "post_roll_s": 3,
    "auto_export": true,
    "chair_world_pose": {
      "position_xyz": [2.1, 0.4, 0.0],
      "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
      "frame_id": "map",
      "source": "3dgs_registration",
      "valid": true,
      "estimated": true
    }
  }'
```

这一步不是启动坐下动作，只是提前创建 session 并写入 checkpoint、椅子位姿等
元数据。省略时，第一次执行 `sonic.sit_chair` 会自动创建 session。

`chair_world_pose.estimated` 应准确表示该位姿是否为感知估计，不得把估计值标为 ground truth。

### 2. 持续写入外部 base 定位

定位进程每次获得新位姿时调用：

```bash
curl -X POST http://127.0.0.1:8765/rollout/localization \
  -H 'Content-Type: application/json' \
  -d '{
    "base_xyz": [0.42, -0.08, 0.79],
    "source": "vio",
    "frame_id": "map",
    "timestamp_s": 1785400000.123,
    "valid": true,
    "estimated": true
  }'
```

录制线程会把收到的最近且未过期的定位与每帧 `g1_debug` 对齐。`source` 不能使用 `g1_debug`、`base_trans_measured` 或 `none`。
同一份有效定位也会填入 real backend 的 `/robot_state.base_pose`；过期后自动恢复为
`x_m/y_m/z_m=null` 和 `translation_valid=false`。

如果动作不是通过 `/execute_action` 发起，可以显式设置上下文，但自动动作窗口
只会由 Bridge 的 `sonic.sit_chair` action lifecycle 触发：

```bash
curl -X POST http://127.0.0.1:8765/rollout/context \
  -H 'Content-Type: application/json' \
  -d '{
    "skill_name": "sonic.sit_chair",
    "motion_name": "sit_060cm",
    "reference_distance_m": 0.60,
    "chair_distance_m": 0.61
  }'
```

通过 Bridge 执行的动作会自动写入 skill、episode、step 和可用的 motion/reference distance 上下文。

### 3. 执行坐下并查看状态

正常调用：

```bash
curl -X POST http://127.0.0.1:8765/execute_action \
  -H 'Content-Type: application/json' \
  -d '{
    "runtime_mode": "real",
    "episode_id": "real_001",
    "step_id": 8,
    "skill_name": "sonic.sit_chair",
    "arguments": {"chair_distance_m": 0.61},
    "safety": {}
  }'
```

动作返回后，Recorder 还会等待最多 3 秒 post-roll，然后自动导出。

```bash
curl http://127.0.0.1:8765/rollout/status
```

重点检查：

- `capture.action_started/action_finished`。
- `capture.post_remaining_s`。
- `dropped_sample_count` 为 0。
- `localization.valid` 为 true，且 `age_s` 小于配置阈值。
- 自动完成后 `active=false`，导出结果在 `last_export`。

### 4. 可选：提前停止

通常不需要调用 `/rollout/stop`。如果不想等待完整 post-roll，可提前停止；
此时只导出已经收到的 post 数据，仍不会超过 3 秒。

```bash
curl -X POST http://127.0.0.1:8765/rollout/stop \
  -H 'Content-Type: application/json' \
  -d '{
    "target_fps": 50,
    "smoothing_window": 5
  }'
```

响应会返回 session 目录和三个 NPZ 的绝对路径。

如需调试整个任意时间段，可在 `/rollout/start` 显式传
`"capture_mode": "continuous"`；该模式保留原来的手动 start/stop 行为，
但不用于默认坐椅数据采集。

## 导出文件

### `raw_real_rollout.npz`

主要字段：

- `timestamp_monotonic_s`、`timestamp_wall_s`、`timestamp_relative_s`
- `source_index`、`ros_timestamp_s`
- `body_q[N,29]`、`body_dq[N,29]`
- `base_quat_wxyz[N,4]`、`base_ang_vel[N,3]`
- `base_xyz_world[N,3]`、`base_xyz_valid[N]`
- `base_xyz_source[N]`、`base_xyz_frame_id[N]`
- `body_q_target[N,29]`、`policy_action[N,29]`
- `motion_name[N]`、`reference_distance_m[N]`
- `capture_phase[N]`：`pre`、`action` 或 `post`
- `joint_order[29]`、`metadata_json`

### `3dgs_replay.npz`

使用 `root_pos_world + root_quat_wxyz + joint_pos` 做运动学回放。

- `kinematic_replay_ready=true` 表示关节和朝向足以做原地/运动学回放。
- 只有所有帧都有有效外部 `base_xyz` 时，
  且定位 `frame_id` 一致时，`spatial_replay_ready=true`。
- 椅子位姿通过 `chair_world_*` 字段保存，并保留 `source/estimated/valid`。
- 只有椅子与 base 使用同一个 `frame_id` 时，
  `chair_relative_replay_ready=true`。

### `gear_sonic_reference.npz`

字段与现有 pose/reference 发送路径对齐：

- `joint_pos[M,29]`
- `joint_vel[M,29]`
- `body_quat_w[M,4]`
- `frame_index[M]`
- `encode_mode=0`、`motion_id=-1`

它只是线性重采样和滑动平均后的候选。文件固定标记：

- `physical_validation_required=true`
- `contact_correction_applied=false`
- `deployment_ready=false`（位于 `metadata_json`）

在 MuJoCo/Isaac 或真机使用前，仍需完成脚/地面接触修正、轨迹限幅、安全检查和物理跟踪验证，不能直接把它当成已验证权重或可部署动作。

## 关节顺序

三个导出文件都保存完整 `joint_order`。顺序是 deploy 发布的 MuJoCo 顺序：

1. 左腿 6 DoF
2. 右腿 6 DoF
3. 腰部 yaw/roll/pitch
4. 左臂 7 DoF
5. 右臂 7 DoF

消费端必须读取文件中的 `joint_order`，不要自行假设 IsaacLab 或硬件电机顺序。
