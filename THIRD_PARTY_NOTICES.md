# 第三方依赖与声明

本项目采用 [MIT 许可证](LICENSE)。下面列出构建与运行所依赖的第三方组件及其许可证。
其中**没有任何第三方代码被复制进本仓库**（唯一例外见“裁判系统 CRC 表”一节），
依赖均通过系统包管理器或构建时下载获得。

## 直接依赖

| 组件 | 用途 | 许可证 | 说明 |
| --- | --- | --- | --- |
| [OpenCV](https://opencv.org/) | 图像处理、相机标定、可视化 | Apache-2.0 | 系统包 `libopencv-dev` |
| [Eigen3](https://eigen.tuxfamily.org/) | 线性代数（旋转矩阵、卡尔曼滤波） | MPL-2.0 | 仅使用头文件，未修改源码；系统包 `libeigen3-dev` |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 配置文件解析 | MIT | 系统包 `libyaml-cpp-dev` |
| [Boost](https://www.boost.org/) | Asio 串口/网络、Beast HTTP | BSL-1.0 | 仅使用头文件；系统包 `libboost-all-dev` |
| [Foxglove SDK](https://github.com/foxglove/foxglove-sdk) | 日志、可视化 WebSocket 服务、mcap 录制 | MIT | 构建时由 CMake 从官方 Release 下载预编译包（`FoxGlove/CMakeLists.txt`），不随本仓库分发 |
| [mcap](https://github.com/foxglove/mcap) | mcap 文件读取（`Tools/Export`） | MIT | `FetchContent` 拉取，固定 tag `releases/cpp/v2.1.3` |
| [Protocol Buffers](https://protobuf.dev/) | `Tools/Export` 解码 mcap 中的 RawImage | BSD-3-Clause | 系统包 `libprotobuf-dev protobuf-compiler` |

## 需要自行安装、不可再分发的组件

| 组件 | 用途 | 说明 |
| --- | --- | --- |
| 海康机器人 MVS SDK（`MvCameraControl`） | 驱动海康工业相机取流 | 闭源商业 SDK，**本仓库不包含也不得再分发**。请从海康机器人官网下载安装，并通过 `-DMVS_ROOT=<安装路径>`（默认 `/opt/MVS`）指定 |

### 关于 `Detect/MVSCamera/fake-sdk/`

该目录下的 `MvCameraControl.h` 与 `MvCameraControl_stub.cpp` 是本项目为「未安装 MVS SDK 时的编译验证」
自行编写的**空实现替身**（clean-room stub）：接口名称与官方 SDK 相同，但**不含海康的任何代码、头文件或二进制**，
也不实现任何相机功能（枚举不到设备），仅在 `-DUSE_FAKE_MVS=ON` 时参与构建，且只产出静态库。
真实部署仍须自行从海康机器人官网获取官方 SDK，遵守其许可条款。

## 裁判系统 CRC 表

`Communication/Serial/CRCCheck/` 中的 CRC8 / CRC16 查表实现来自 DJI 官方公开的
RoboMaster 裁判系统串口通信协议文档（`crc.h` / `crc.cpp` 中的表与初值与该文档一致），
用于与裁判系统及云台电控的帧校验。若你重新分发本仓库，请一并保留本说明。

- CRC8：初值 `0xFF`，生成多项式 `x^8 + x^5 + x^4 + 1`
- CRC16：初值 `0xFFFF`，生成多项式 `0x1021`，反射实现（等价 CRC-16/MCRF4XX），低字节在前

## 商标

RoboMaster、DJI 为深圳市大疆创新科技有限公司的商标。本项目为参赛队自研代码，
与 DJI / RoboMaster 官方无隶属或背书关系。
