# CaptureServer

基于 Boost.Beast 的本地 HTTP 服务，用于在线调参并拉取最新相机图像。

## 功能

- 默认监听 `127.0.0.1:8080`（可用 `--bind` / `--port` 修改）
- 持续取流并缓存最新一帧
- `GET /args?exposure=*&gain=*` 动态设置相机参数（参数可单独传）
- `GET /image` 返回最新 JPEG 图像
- `GET /` 返回网页控制界面 `capture_control.html`

> ⚠️ 该服务**没有任何鉴权**，任何人只要能访问端口就能改曝光/增益并拉取图像。
> 因此默认只监听回环地址；确实需要远程访问时用 `--bind 0.0.0.0` 显式开启，并自行限制网络环境。

## 构建

在项目根目录执行：

```bash
cmake -S . -B build
cmake --build build --target capture_server -j
```

## 运行

```bash
./build/Tools/CameraTune/CaptureServer/capture_server            # 监听 127.0.0.1:8080
./build/Tools/CameraTune/CaptureServer/capture_server --bind 0.0.0.0 --port 8080
```

## 调用示例

```bash
curl "http://127.0.0.1:8080/args?exposure=10000&gain=8"
curl "http://127.0.0.1:8080/args?gain=12"
curl "http://127.0.0.1:8080/image" --output latest.jpg
```
