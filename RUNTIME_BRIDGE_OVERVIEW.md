# GR00T-Sonic-WBC Runtime Bridge 简介

## 定位

GR00T-Sonic-WBC Runtime Bridge 是面向外部 agent / client 的运行时接入层。它通过 HTTP JSON 接口暴露 GR00T-Sonic-WBC 的仿真/真机执行、相机观测、robot state、runtime health 和 telemetry。

Bridge 只负责 runtime 能力，不负责 agent prompt、任务定义、评分、oracle 或 benchmark 逻辑。外部系统可以把它当作一个 embodied runtime backend 使用。

> 当前代码目录仍命名为 `vigil_bridge`，这是历史命名；工程定位上可以按通用 Runtime Bridge 理解。

## 架构

```text
External Agent / Client
  -> HTTP JSON API
  -> BridgeRequestRouter
  -> VigilBridgeService
  -> PrimitiveExecutor + SensorProvider
  -> dry_run / MuJoCo / real adapter
  -> GR00T-Sonic-WBC runtime transports
```

核心代码：

| 路径 | 职责 |
| --- | --- |
| `gear_sonic/vigil_bridge/protocol.py` | 协议常量、capabilities、request/response 结构 |
| `gear_sonic/vigil_bridge/transport.py` | HTTP server 和 endpoint 路由 |
| `gear_sonic/vigil_bridge/service.py` | Bridge facade，组合 executor 和 sensor provider |
| `gear_sonic/vigil_bridge/primitive_executor.py` | dry-run 动作映射和参数校验 |
| `gear_sonic/vigil_bridge/mujoco_adapter.py` | MuJoCo runtime adapter |
| `gear_sonic/vigil_bridge/real_adapter.py` | real robot runtime adapter |
| `gear_sonic_deploy/scripts/run_vigil_bridge.py` | Bridge 启动入口 |

## 运行模式

| Backend | 用途 |
| --- | --- |
| `dry_run` | 协议联调、client 开发、fake 测试；不连接真实 runtime |
| `mujoco` | 连接已启动的 MuJoCo + deploy runtime |
| `real` | 连接已启动的真机 deploy runtime；真实运动默认关闭 |

Bridge 不负责启动 MuJoCo、policy、deploy 或硬件流程，只连接已存在的 runtime transport。

## HTTP 接口

| Method | Path | 说明 |
| --- | --- | --- |
| `GET` | `/health` | 服务存活检查 |
| `POST` | `/handshake` | 获取协议版本、runtime mode 和 capabilities |
| `POST` | `/reset_episode` | 初始化 runtime-facing session |
| `POST` | `/execute_action` | 执行一个 primitive-level action |
| `POST` | `/observation` | 获取相机观测和 robot state |
| `POST` | `/robot_state` | 获取 robot state |
| `POST` | `/halt` | 安全停止 / idle |
| `POST` | `/pause` | policy/deploy 保活，停止推理并让真机回到 `default_angles` |
| `POST` | `/resume` | 从 paused session 重新接入 policy，不重新部署 |
| `POST` | `/close` | 释放 Bridge 侧资源 |

当前公开能力：

```text
Actions:
- navigate.forward
- navigate.backward
- navigate.turn_left
- navigate.turn_right

Observations:
- rgb
- depth
- robot_state
```

## 接入流程

1. `GET /health` 确认服务在线。
2. `POST /handshake` 检查协议版本和 capabilities。
3. `POST /reset_episode` 初始化 session。
4. 循环调用 `POST /observation` 和 `POST /execute_action`。
5. 需要重置场景但保留 policy/deploy 时调用 `POST /pause`。
6. 重置完成后调用 `POST /resume`，或使用 robot-side `./vigil_bridge start` 复用已有 session。
7. 结束或异常时调用 `POST /halt`。
8. 如需释放资源，再调用 `POST /close`。

## Action 示例

```json
{
  "episode_id": "agent_run_001",
  "step_id": 3,
  "runtime_mode": "mujoco",
  "skill_name": "navigate.forward",
  "arguments": {
    "distance_m": 0.5
  },
  "safety": {
    "max_speed_mps": 0.5,
    "timeout_s": 8.0
  }
}
```

`/execute_action` 返回的是 runtime primitive 执行结果，不代表任务是否成功。接入方主要读取：

- `ok`
- `error_message`
- `action_status`
- `executed_arguments`
- `robot_state_before`
- `robot_state_after`
- `telemetry.completion`

`action_status` 常见值为 `completed`、`timeout`、`rejected`、`failed`。

## 安全边界

real backend 采用 fail-closed 设计：

- 仅设置 `--backend real` 不会移动机器人。
- 必须显式设置 `--enable-real-motion` 才允许真实运动。
- 缺少 command/state/camera readiness 时拒绝执行。
- 异常路径会调用 `halt()`。
- `/halt` 是安全停止接口。
- `/pause` 不终止 policy/deploy；真机侧会停止 policy 推理，回到并保持 `default_angles`。
- `/resume` 重新走 policy 接入前准备，再恢复控制。
- `/close` 只释放 Bridge 资源，不应假设它会停止 policy/deploy。
