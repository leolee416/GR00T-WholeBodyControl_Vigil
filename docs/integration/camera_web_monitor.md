# Host 网页相机监控（可直接复制到飞书）

## 1. 目标

在 Host `192.168.1.113` 上启动一个只读网页服务，订阅已有的 ZMQ 相机流。客户端无需安装 OpenCV，直接用浏览器打开网页即可查看所有实时相机画面。

数据链路：

```text
机器人相机服务
  -> ZMQ 相机流 tcp://127.0.0.1:5555
  -> Host 只读 camera_monitor
  -> HTTP http://192.168.1.113:8780/
  -> 客户端浏览器
```

安全边界：

- 程序只订阅相机数据，不连接命令端口，不会启动机器人动作。
- 程序不会自动启动、停止或重启相机 service。
- 网页没有登录和 HTTPS，只应在可信局域网内开放。

## 2. 一次性检查

从客户端登录 Host：

```bash
ssh unitree@192.168.1.113
```

进入仓库，并确认当前 Python 包含 `pyzmq`、`msgpack`。当前 Host 的系统 `python3` 已验证可用：

```bash
cd /home/unitree/GR00T-WholeBodyControl_Vigil
python3 -c 'import msgpack, zmq; print("camera transport dependencies: OK")'
```

检查相机服务和 5555 端口。如果当前任务不是通过该 systemd service 启动相机，可跳过第一条，只检查实际使用的相机进程和端口：

```bash
systemctl status composed_camera_server_vigil.service --no-pager -l
ss -lntp | grep ':5555'
```

## 3. 启动网页监控

在 Host 执行：

```bash
cd /home/unitree/GR00T-WholeBodyControl_Vigil

python3 -m gear_sonic.vigil_bridge.camera_monitor \
  --host 0.0.0.0 \
  --port 8780 \
  --camera-host 127.0.0.1 \
  --camera-port 5555
```

看到以下输出即表示网页服务已监听：

```text
Camera source: tcp://127.0.0.1:5555
Camera monitor: http://<host-ip>:8780/
Read-only monitor started. Press Ctrl-C to stop.
```

注意：网页服务启动成功不等于已经收到相机帧。画面和 `/health` 才是相机流是否正常的依据。

## 4. 浏览器访问

在与 Host 网络互通的电脑上打开：

```text
http://192.168.1.113:8780/
```

网页会自动发现 `ego_view`、腕部相机等当前 payload 中存在的画面。页面顶部显示：

- `画面正常`：最近 2 秒内收到过新帧。
- `waiting for camera frames`：网页已启动，但尚未收到相机 payload。
- `camera stream is stale`：曾经收到过画面，但相机流已经停止更新。

## 5. 健康检查和单帧接口

在任意能访问 Host 的机器执行：

```bash
curl -s http://192.168.1.113:8780/health | python3 -m json.tool
```

正常时关键字段示例：

```json
{
  "ok": true,
  "source_endpoint": "tcp://127.0.0.1:5555",
  "receiving_frames": true,
  "streams": [
    "ego_view"
  ]
}
```

打开某一路单帧：

```text
http://192.168.1.113:8780/snapshot/ego_view
```

打开某一路原始 MJPEG 流：

```text
http://192.168.1.113:8780/stream/ego_view.mjpg
```

## 6. 后台运行（可选）

推荐使用 tmux，避免 SSH 断开后网页服务退出：

```bash
tmux new-session -s camera_web
```

在 tmux 中执行第 3 节启动命令。按 `Ctrl-b`，再按 `d`，即可退出 tmux 而不停止服务。

重新进入：

```bash
tmux attach -t camera_web
```

停止服务：在 tmux 窗口内按 `Ctrl-C`。

## 7. 常见问题

### 7.1 网页打不开

在 Host 检查监听端口：

```bash
ss -lntp | grep ':8780'
curl -s http://127.0.0.1:8780/health | python3 -m json.tool
```

如果 Host 本机能访问，但客户端打不开，检查客户端到 `192.168.1.113` 的路由以及 Host 防火墙。不要把服务直接暴露到公网。

### 7.2 网页能打开，但没有画面

```bash
curl -s http://127.0.0.1:8780/health | python3 -m json.tool
ss -lntp | grep ':5555'
systemctl status composed_camera_server_vigil.service --no-pager -l
journalctl -u composed_camera_server_vigil.service -n 100 --no-pager
```

重点看 `/health` 的 `error_message`、`last_frame_age_s` 和 `decode_warning`。

### 7.3 Python 提示缺少 pyzmq 或 msgpack

说明当前 Python 环境不包含相机 ZMQ 依赖。切换到包含 `pyzmq`、`msgpack` 的运行环境，或按照仓库 `gear_sonic[camera]` 依赖说明创建独立环境；不要直接修改系统 Python。

### 7.4 8780 端口已被占用

改用其他 HTTP 端口，例如：

```bash
python3 -m gear_sonic.vigil_bridge.camera_monitor \
  --host 0.0.0.0 \
  --port 8781 \
  --camera-host 127.0.0.1 \
  --camera-port 5555
```

此时浏览器地址改为 `http://192.168.1.113:8781/`。
