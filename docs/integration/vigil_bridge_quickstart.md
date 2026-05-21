# Vigil Bridge 入门指南

这份文档用于在机器人侧快速试运行 GR00T Vigil bridge。

**第一次使用必须由 @lizj18 在场协助运行。不要单独执行真实机器人动作命令。**

## 1. SSH 连接机器人

优先使用机器人当前 Wi-Fi IP：

```bash
ssh unitree@192.168.1.113
```

如果 Wi-Fi 不可用，可尝试默认有线地址：

```bash
ssh unitree@192.168.123.164
```

进入 Vigil 仓库：

```bash
cd ~/GR00T-WholeBodyControl_Vigil
```

## 2. 一键启动 Policy 和 Bridge

启动真实部署 policy、Vigil 专用相机服务和 HTTP bridge：

```bash
./vigil_bridge start --max-speed-mps 2 --camera-required --attach
```

常用管理命令：

```bash
./vigil_bridge status
./vigil_bridge logs
./vigil_bridge attach
./vigil_bridge stop
```

说明：launcher 只会在本次 Vigil 会话中启动 `composed_camera_server_vigil.service`，
不会禁用或替换旧的相机服务。

## 3. Curl 命令集

在客户端机器上设置机器人 bridge 地址：

```bash
ROBOT=http://192.168.1.113:8765
```

健康检查：

```bash
curl -s "$ROBOT/health" | python3 -m json.tool
```

初始化 runtime：

```bash
curl -s "$ROBOT/reset_episode" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | python3 -m json.tool
```

获取观测：

```bash
curl -s "$ROBOT/observation" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | python3 -m json.tool
```

在 Mac 上保存 `ego_view` 图像：

```bash
mkdir -p /Users/lizj/Pictures/debug && \
curl -s "$ROBOT/observation" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | \
python3 -c 'import sys,json,base64,datetime,os
p="/Users/lizj/Pictures/debug"
payload=json.load(sys.stdin)
entry=payload["images"]["ego_view"]
b64=entry["data"] if isinstance(entry,dict) else entry
out=os.path.join(p,"ego_view_"+datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+".jpg")
open(out,"wb").write(base64.b64decode(b64))
print(out)'
```

获取机器人状态：

```bash
curl -s "$ROBOT/robot_state" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | python3 -m json.tool
```

小幅前进，仅限 @lizj18 在场时使用：

```bash
curl -s "$ROBOT/execute_action" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real","skill_name":"navigate.forward","arguments":{"distance_m":0.10},"safety":{"max_speed_mps":0.15,"timeout_s":5}}' \
  | python3 -m json.tool
```

小幅左转，仅限 @lizj18 在场时使用：

```bash
curl -s "$ROBOT/execute_action" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real","skill_name":"navigate.turn_left","arguments":{"degrees":10},"safety":{"rate_deg_s":10,"timeout_s":5}}' \
  | python3 -m json.tool
```

小幅右转，仅限 @lizj18 在场时使用：

```bash
curl -s "$ROBOT/execute_action" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real","skill_name":"navigate.turn_right","arguments":{"degrees":10},"safety":{"rate_deg_s":10,"timeout_s":5}}' \
  | python3 -m json.tool
```

安全停止：

```bash
curl -s "$ROBOT/halt" \
  -X POST -H 'Content-Type: application/json' \
  -d '{"runtime_mode":"real"}' | python3 -m json.tool
```
