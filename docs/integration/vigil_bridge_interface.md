# Vigil Bridge Interface

本文档面向需要接入 GR00T bridge 的 agent 或后续维护者。Bridge 只暴露机器人/仿真运行时能力，不承接 Vigil 的 benchmark、prompt、trace、judge、score 或 oracle 语义。

## 设计边界

调用链：

```text
external agent / Vigil client
  -> HTTP JSON endpoints
  -> BridgeRequestRouter
  -> VigilBridgeService
  -> PrimitiveExecutor + SensorProvider
  -> dry_run / MuJoCo / real runtime adapter
```

核心文件：

| 文件 | 职责 |
| --- | --- |
| `gear_sonic/vigil_bridge/protocol.py` | 协议常量、TypedDict 消息形状、当前公开 capability |
| `gear_sonic/vigil_bridge/transport.py` | HTTP endpoint 到 service 方法的路由 |
| `gear_sonic/vigil_bridge/service.py` | bridge facade；组合 executor、sensor provider，补充统一 telemetry |
| `gear_sonic/vigil_bridge/primitive_executor.py` | dry-run 动作映射、参数校验、基础 primitive 语义 |
| `gear_sonic/vigil_bridge/sensors.py` | dry-run observation / robot_state |
| `gear_sonic/vigil_bridge/mujoco_adapter.py` | MuJoCo ZMQ/DDS/camera adapter |
| `gear_sonic/vigil_bridge/real_adapter.py` | real robot ZMQ/camera adapter，默认禁止真实运动 |
| `gear_sonic_deploy/scripts/run_vigil_bridge.py` | HTTP bridge 启动入口和 backend 配置 |
| `tests/vigil_bridge/` | protocol、transport、adapter 的轻量测试 |

## 当前运行模式

| backend | runtime_mode | 用途 | 运行时连接 | 运动安全默认值 |
| --- | --- | --- | --- | --- |
| `dry_run` | `dry_run` 或请求传入值 | 协议测试、假客户端对接 | 无真实 runtime；返回 fake state/image | 不下发任何 WBC/机器人命令 |
| `mujoco` | `mujoco` | 已启动 MuJoCo/deploy 后的仿真控制 | ZMQ command/planner、ZMQ `g1_debug`、可选 DDS `rt/odostate`、可选 camera ZMQ | 不自动启动控制，除非传 `--auto-start-control` |
| `real` | `real` | 已启动 real deploy 后的机器人控制 | ZMQ command/planner、ZMQ `g1_debug`、可选/必需 camera ZMQ | `--enable-real-motion` 未开启时拒绝运动 |

Bridge 不负责启动 MuJoCo、deploy、policy inference 或硬件流程。

## HTTP Protocol

默认启动：

```bash
python gear_sonic_deploy/scripts/run_vigil_bridge.py --host 127.0.0.1 --port 8765 --backend dry_run
```

所有 POST request/response body 都是 JSON object。未知 endpoint 返回 HTTP 404 + `ok=false`；非法 JSON 返回 HTTP 400。

| Method | Path | Request | Response | 说明 |
| --- | --- | --- | --- | --- |
| `GET` | `/health` | none | health object | transport/service 存活检查 |
| `POST` | `/handshake` | `HandshakeRequest` | `HandshakeResponse` | 返回 protocol、runtime_mode、capabilities |
| `POST` | `/reset_episode` | `ResetEpisodeRequest` | `ResetEpisodeResponse` | 启动 executor 并采样初始 robot_state |
| `POST` | `/execute_action` | `ExecuteActionRequest` | `ExecuteActionResponse` | 同步执行一个 primitive-level action |
| `POST` | `/observation` | optional object | `ObservationResponse` | 获取最新 image + robot_state |
| `POST` | `/get_observation` | optional object | `ObservationResponse` | `/observation` alias |
| `POST` | `/robot_state` | optional object | `RobotStateResponse` | 获取 robot_state |
| `POST` | `/get_robot_state` | optional object | `RobotStateResponse` | `/robot_state` alias |
| `POST` | `/halt` | optional object | `RuntimeHealth` | 安全停止/idle 接口 |
| `POST` | `/close` | optional object | close response | 释放 bridge 资源；不要假设会停止 deploy/policy |

协议常量见 `gear_sonic/vigil_bridge/protocol.py`：

| 字段 | 当前值 |
| --- | --- |
| `protocol_version` | `vigil_groot_bridge_v1` |
| `bridge.name` | `gear_sonic_vigil_bridge` |
| `bridge.version` | `dry_run_phase1` |
| `capabilities.actions` | `navigate.backward`, `navigate.forward`, `navigate.turn_left`, `navigate.turn_right` |
| `capabilities.observation` | `rgb`, `depth`, `robot_state` |
| `capabilities.oracle_source` | `none` |

## Message Shapes

### Handshake

当前实现要求 request 中提供 `runtime_mode`。

Request:

```json
{
  "protocol_version": "vigil_groot_bridge_v1",
  "client": {"name": "vigil", "component": "GrootWBCEnv"},
  "episode_id": "episode_001",
  "runtime_mode": "mujoco",
  "required_capabilities": {
    "actions": ["navigate.forward"],
    "observation": ["rgb", "depth", "robot_state"],
    "oracle_source": "none"
  }
}
```

Response stable fields:

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `ok` | bool | handshake 是否成功 |
| `error_message` | string/null | 错误信息 |
| `protocol_version` | string | bridge protocol version |
| `runtime_mode` | string | 当前 runtime mode |
| `capabilities.actions` | list[string] | 公开动作名 |
| `capabilities.observation` | list[string] | 公开观测类型 |
| `capabilities.oracle_source` | string | 当前为 `none` |
| `bridge.name` / `bridge.version` | string | bridge 标识 |

### Execute Action

Request:

```json
{
  "episode_id": "episode_001",
  "step_id": 3,
  "runtime_mode": "mujoco",
  "skill_name": "navigate.forward",
  "arguments": {"magnitude": 1},
  "safety": {"max_speed_mps": 0.5, "timeout_s": 8.0}
}
```

Response stable fields:

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `ok` | bool | runtime action 是否成功 |
| `error_message` | string/null | 失败原因 |
| `action_status` | string | `completed`, `timeout`, `rejected`, `failed`; `interrupted` 预留 |
| `executed_arguments` | object | 实际执行参数；失败或拒绝时可为空 |
| `robot_state_before` | object | 执行动作前采样 |
| `robot_state_after` | object | 动作完成/失败后采样 |
| `telemetry.completion.motion_commanded` | bool | 是否下发过运动命令 |
| `telemetry.completion.capture_timing` | string | `after_completion`, `after_settle`, `after_timeout`, `not_started` 等 |
| `telemetry.completion.settled` | bool | 是否认为动作后已稳定 |

`ok=true` 只表示 bridge/runtime primitive 成功，不表示 benchmark task success。

### Observation

Response:

```json
{
  "observation_id": "mujoco_obs_0001",
  "runtime_mode": "mujoco",
  "images": {
    "ego_view": {"encoding": "jpeg-base64", "data": "..."},
    "ego_view_depth": {"encoding": "jpeg-base64", "data": "..."}
  },
  "camera_timestamps": {"ego_view": 123.0, "ego_view_depth": 123.0},
  "robot_state": {},
  "telemetry": {},
  "perception": {"source": "none", "detections": []}
}
```

Observation 只提供 runtime-facing 数据。若未来加入估计感知，必须在 payload 中标明 `estimated` 或 `source`，不得写成 groundtruth。

Depth 当前作为 camera stream 的 image entry 透出，命名沿用上游 camera key，例如 RealSense driver 输出 `ego_view` 和 `ego_view_depth`。Bridge 不把 depth 改写成单独顶层字段；对接方应从 `images.<camera>_depth` 和 `camera_timestamps.<camera>_depth` 读取。

### Robot State

通用字段：

| 字段 | 说明 |
| --- | --- |
| `state_id` | provider 生成的状态 id |
| `base_pose` | base 位姿；real 模式可能只有 heading |
| `base_velocity` | base 速度；缺失值用 `null` |
| `joint_positions` | 当前可为空 |
| `estimated` | 是否为估计值 |
| `source` | 数据来源，例如 `rt/odostate`, `g1_debug_heading`, `fake` |

## 当前能力表

### Actions

| Public `skill_name` | Arguments | Safety keys | dry_run | MuJoCo | real | 主要实现 |
| --- | --- | --- | --- | --- | --- | --- |
| `navigate.forward` | `distance_m` 或 `magnitude` | `max_speed_mps`/`speed_mps`, `timeout_s` | `move_model(+distance)` | `move_model(+distance)` | `move_open_loop(+distance)` | `primitive_executor.py`, `mujoco_adapter.py`, `real_adapter.py` |
| `navigate.backward` | `distance_m` 或 `magnitude` | `max_speed_mps`/`speed_mps`, `timeout_s` | `move_model(-distance)` | `move_model(-distance)` | `move_open_loop(-distance)` | 同上 |
| `navigate.turn_left` | `degrees`/`angle_deg` 或 `magnitude` | `max_rate_deg_s`/`rate_deg_s`, `timeout_s` | `rotate(+degrees)` | `rotate(+degrees)` | `rotate(+degrees)` | 同上 |
| `navigate.turn_right` | `degrees`/`angle_deg` 或 `magnitude` | `max_rate_deg_s`/`rate_deg_s`, `timeout_s` | `rotate(-degrees)` | `rotate(-degrees)` | `rotate(-degrees)` | 同上 |

备注：`DryRunPrimitiveExecutor.execute_action()` 中存在 `report` 分支，但它没有出现在 `SUPPORTED_ACTIONS`/handshake 中。对外 agent 不应依赖它；如需公开，按新增 action 流程登记并补全 backend telemetry。

### Observations

| Public observation | Payload location | dry_run | MuJoCo | real | 主要实现 |
| --- | --- | --- | --- | --- | --- |
| `rgb` | `ObservationResponse.images` | fake `ego_view` | 可选 camera ZMQ，`jpeg-base64` | camera ZMQ，默认 real camera required | `sensors.py`, `mujoco_adapter.py`, `real_adapter.py` |
| `depth` | `ObservationResponse.images.<camera>_depth` | fake `ego_view_depth` | camera stream 如包含 depth key 则透出 | RealSense/OAK 等 camera stream 如包含 depth key 则透出 | `gear_sonic/camera/drivers/realsense.py`, provider normalize methods |
| `robot_state` | `ObservationResponse.robot_state` / `/robot_state` | fake pose/velocity | DDS odom 优先，否则 `g1_debug` heading | `g1_debug` heading，位置可为 `null` | 同上 |

## 扩展登记表

新增 bridge 能力时，先在表中登记，再改代码和测试。

| 类型 | Public name | Request keys | Response/telemetry keys | 需修改文件 | 测试文件 | 状态 |
| --- | --- | --- | --- | --- | --- | --- |
| action | `navigate.forward` | `arguments.distance_m/magnitude`, `safety.max_speed_mps`, `safety.timeout_s` | `executed_arguments.distance_m`, `telemetry.completion`, optional `motion_result` | `protocol.py`, `primitive_executor.py`, `mujoco_adapter.py`, `real_adapter.py` | `tests/vigil_bridge/test_vigil_bridge_service.py`, adapter tests | current |
| action | `navigate.turn_left` | `arguments.degrees/angle_deg/magnitude`, `safety.max_rate_deg_s`, `safety.timeout_s` | `executed_arguments.degrees`, `telemetry.completion`, optional `motion_result` | 同上 | 同上 | current |
| observation | `rgb` | none | `images.<camera>.encoding`, `images.<camera>.data`, `camera_timestamps` | `protocol.py`, provider normalize methods | adapter/provider tests | current |
| observation | `depth` | none | `images.<camera>_depth.encoding`, `images.<camera>_depth.data`, `camera_timestamps.<camera>_depth` | `protocol.py`, camera payload normalize methods | adapter/provider tests | current |
| observation | `robot_state` | none | `robot_state.base_pose`, `robot_state.base_velocity`, `robot_state.estimated`, `robot_state.source` | `protocol.py`, provider state methods | adapter/provider tests | current |
| action | `<new.skill>` | `<arguments.*>`, `<safety.*>` | `<executed_arguments.*>`, `telemetry.completion`, optional `motion_result` | `protocol.py`, executor/backend files, maybe `run_vigil_bridge.py` | service + backend tests | proposed |
| observation | `<new_observation>` | optional request keys | `ObservationResponse.<new field>` or `telemetry.<source>` | `protocol.py`, sensor providers, maybe backend config | provider tests | proposed |
| endpoint | `/<new_endpoint>` | JSON object | JSON object with `ok/error_message` when applicable | `transport.py`, `service.py`, `protocol.py` | transport tests | proposed |

## 新增 Action 范式

1. 在 `gear_sonic/vigil_bridge/protocol.py` 的 `SUPPORTED_ACTIONS` 增加 public `skill_name`。
2. 在 `DryRunPrimitiveExecutor.execute_action()` 增加参数解析和 fake telemetry；不要调用真实 runtime。
3. 如果 action 对应已有 primitive，复用 `move()`/`rotate()` 这类 executor 方法；否则在 executor 中新增 primitive 方法。
4. 在 `MujocoPrimitiveExecutor` 和 `RealPrimitiveExecutor` 中实现同名 primitive 或覆盖必要逻辑。
5. real mode 必须保持：`motion_enabled` gate、保守速度/超时、异常时 `halt()`、缺 state/camera/command 时 fail closed。
6. 返回统一 `ExecuteActionResponse`：`action_status`、`executed_arguments`、`telemetry.completion` 必须稳定。
7. 在 `tests/vigil_bridge/` 增加 dry-run service 测试、HTTP/transport 测试，以及 MuJoCo/real fake runtime 测试。

## 新增 Observation 范式

1. 在 `SUPPORTED_OBSERVATIONS` 增加 public observation 名称。
2. 在 `FakeSensorProvider`、`MujocoSensorProvider`、`RealSensorProvider` 中补齐 payload 或明确返回空/不可用 telemetry。
3. 如果需要 runtime 连接，在对应 `*BridgeConfig` 中加配置项，并在 `run_vigil_bridge.py` 暴露 CLI 参数。
4. 所有 image/data payload 保持 JSON 兼容；二进制数据使用 base64。
5. 估计值必须标注 `estimated`/`source`；bridge 不输出 benchmark oracle 或 hidden success。
6. 增加 provider normalization 测试和缺连接时的结构化错误测试。

## 新增 Backend 范式

1. 新增 `<Backend>BridgeConfig`、`<Backend>RuntimeClient`、`<Backend>PrimitiveExecutor`、`<Backend>SensorProvider`。
2. 提供 `create_<backend>_bridge_service(config)`，返回 `VigilBridgeService(executor=..., sensor_provider=...)`。
3. 在 `run_vigil_bridge.py` 增加 `--backend` choice 和配置参数。
4. 不导入 Vigil；不修改 low-level control、deploy internals、policy inference 或 model path。
5. 补 fake runtime tests，覆盖 handshake 不启动 runtime、动作映射、halt/close 语义、observation normalization。

## Agent 对接流程

1. `GET /health` 确认服务在线。
2. `POST /handshake` 检查 `protocol_version` 和 `capabilities`。
3. `POST /reset_episode` 初始化 runtime-facing session。
4. 循环调用 `POST /observation` 和 `POST /execute_action`。
5. 出错或结束时先 `POST /halt`，再按需 `POST /close`。

对接方只应使用 handshake 中公开的 capability。unsupported action 应被视为 `rejected`，不要在 agent 侧假设 bridge 会静默忽略。

## 轻量验证

```bash
python -m compileall gear_sonic/vigil_bridge
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider tests/vigil_bridge
```
