# 配置参数说明

本文档说明 `config.yaml` 的全部参数含义、单位与默认值。

- 示例文件：仓库根目录的 `config.example.yaml`（带注释，可直接复制为 `config.yaml`）。
- 默认值来源：`Config::generate_default_config()`；运行主程序时若当前目录没有 `config.yaml`，会自动生成一份默认配置并加载。
- ⚠️ **默认配置中的相机内参与标定参数为占位值**（取两位有效数字或 0），实车使用前必须按 `docs/calibration.md` 重新标定并写入运行目录的 `config.yaml`。

## CameraConfig

相机内参、畸变与曝光参数（由 `Tools/CameraTune` 标定）。

| 参数                | 含义                        | 单位               |
| ------------------- | --------------------------- | ------------------ |
| `intrinsic_matrix`  | 相机内参矩阵（3x3，行优先） | 无                 |
| `distortion_coeffs` | 畸变系数（k1,k2,p1,p2,k3）  | 无                 |
| `image_width`       | 图像宽度                    | 像素               |
| `image_height`      | 图像高度                    | 像素               |
| `gain`              | 相机增益                    | dB（相机内部单位） |
| `exposure`          | 曝光时间                    | μs（相机内部单位） |

## TripodHeadConfig

云台标定参数与运动极限。

| 参数                      | 含义                                 | 单位       |
| ------------------------- | ------------------------------------ | ---------- |
| `x_k`                     | 激光光路在相机坐标系的 $x=kz+b$ 斜率 | 无         |
| `x_b`                     | 激光光路在相机坐标系的 $x=kz+b$ 截距 | 无         |
| `y_k`                     | 激光光路在相机坐标系的 $y=kz+b$ 斜率 | 无         |
| `y_b`                     | 激光光路在相机坐标系的 $y=kz+b$ 截距 | 无         |
| `invert_yaw_angle`        | 反转 yaw 目标角度方向                | bool       |
| `invert_pitch_angle`      | 反转 pitch 目标角度方向              | bool       |
| `invert_yaw_encoder`      | 反转 yaw 编码器方向                  | bool       |
| `invert_pitch_encoder`    | 反转 pitch 编码器方向                | bool       |
| `yaw_encoder_per_round`   | yaw 编码器每圈计数                   | counts/rev |
| `pitch_encoder_per_round` | pitch 编码器每圈计数                 | counts/rev |
| `max_acceleration`        | 云台最大加速度                       | deg/s^2    |
| `max_velocity`            | 云台最大速度                         | deg/s      |
| `max_pitch_angle`         | pitch 相对初始角的最大允许角度       | deg        |
| `max_yaw_angle`           | yaw 相对初始角的最大允许角度         | deg        |

> `x_k/x_b/y_k/y_b` 由 `Tools/LazerCalibration` 用不同距离下的激光光斑拟合得到；
> 默认 0 表示“假设激光与相机光轴重合”，未标定时激光点会落在图像主点上。
> `max_pitch_angle` / `max_yaw_angle` 是安全护栏：目标相对角超过限制时程序停止下发指令并转入扫描（见 `GimbalController::work_thread_func`）。

## ServoConfig

云台视觉伺服与扫描补偿参数（误差单位为弧度）。

| 参数                  | 含义                           | 单位    |
| --------------------- | ------------------------------ | ------- |
| `k_p`                 | PID 比例系数                   | 无      |
| `k_i`                 | PID 积分系数                   | 无      |
| `k_d`                 | PID 微分系数                   | 无      |
| `k_ff`                | 前馈系数                       | 无      |
| `integral_limit`      | 积分限幅                       | 无      |
| `integrate_deadzone`  | 积分死区（误差小于该值不积分） | 无      |
| `min_offset`          | 最小控制输出偏置               | 无      |
| `scan_initial_radius` | 同心圆扫描初始半径             | 像素    |
| `scan_radius_delta`   | 同心圆扫描半径步进             | 像素    |
| `scan_max_radius`     | 同心圆扫描最大半径             | 像素    |
| `scan_speed`          | 同心圆扫描速度                 | 像素/秒 |

> 未命中时的动态标定偏置（bias）由 `BiasKalmanFilter` 维护，其过程噪声与观测噪声目前在
> `App/src/Main.cpp` 中设置，不再通过配置文件暴露。

## TrackStrategyConfig

目标丢失后的扫描与雷达引导参数。

| 参数                            | 含义                          | 单位  |
| ------------------------------- | ----------------------------- | ----- |
| `reset_time_threshold`          | 目标丢失多久后进入扫描        | s     |
| `scan_yaw_angle`                | 方形扫描 yaw 角度幅度         | deg   |
| `scan_pitch_angle`              | 方形扫描 pitch 角度幅度       | deg   |
| `scan_period`                   | 方形扫描周期                  | s     |
| `line_scan_yaw_min`             | 逐行扫描 yaw 最小角度         | deg   |
| `line_scan_yaw_max`             | 逐行扫描 yaw 最大角度         | deg   |
| `line_scan_pitch_min`           | 逐行扫描 pitch 最小角度       | deg   |
| `line_scan_pitch_max`           | 逐行扫描 pitch 最大角度       | deg   |
| `line_scan_speed`               | 逐行扫描 yaw 角速度           | deg/s |
| `line_scan_pitch_step`          | 逐行扫描 pitch 行间步进       | deg   |
| `guide_translation`             | 雷达系→云台系的平移 (x, y, z) | m     |
| `guide_rotation_euler_zyx_deg`  | 雷达系→云台系的 ZYX 欧拉角 (yaw, pitch, roll) | deg |

> 方形扫描以网络收到的目标位置为中心，使用 `guide_translation` 与
> `guide_rotation_euler_zyx_deg` 组成的 4x4 变换把雷达坐标系的点转到云台角
> （由 `Tools/LidarGuideCalibration` 标定，默认全 0 表示两坐标系重合）。
> 逐行扫描使用绝对角度，不依赖雷达数据，可作为雷达引导失败时的备选方案（`--scan-mode line`）。

## DetectConfig

目标检测阈值参数。

| 参数                     | 含义               | 单位     |
| ------------------------ | ------------------ | -------- |
| `light_blob_x_threshold` | 亮斑 x 方向阈值    | 像素     |
| `light_blob_y_threshold` | 亮斑 y 方向阈值    | 像素     |
| `min_hit_difference`     | 命中判定的最小差值 | 强度阈值 |
| `adaptive_threshold_c`   | 自适应阈值的常量 C | 强度阈值 |

## SerialConfig

与云台电控的串口通信参数。

| 参数                 | 含义                 | 单位  |
| -------------------- | -------------------- | ----- |
| `buffer_size`        | 接收缓冲区大小       | bytes |
| `prepare_size`       | 预处理缓冲区大小     | bytes |
| `baud_rate`          | 波特率               | bps   |
| `reset_pack_count`   | 复位时发送的包计数   | count |
| `port_name`          | 串口设备路径         | path  |
| `max_send_frequency` | 最大发送频率         | Hz    |

> 协议（帧格式、CRC、控制模式）见 `docs/PROTOCOL.md`。
> `reset_pack_count` 是启动阶段持续发送“回正”指令直到收到这么多个反馈包，用于确认电控已上电并稳定。

## NetworkConfig

与雷达主程序的 UDP 通信参数。

| 参数               | 含义                        | 单位 |
| ------------------ | --------------------------- | ---- |
| `listen_ip`        | 监听地址（`0.0.0.0` 为所有网卡） | IPv4 |
| `listen_port`      | 接收目标位置的端口          | port |
| `send_target_ip`   | 心跳发送目标（雷达主程序）地址 | IPv4 |
| `send_target_port` | 心跳发送目标端口            | port |

> 数据包格式与心跳定义见 `docs/PROTOCOL.md`。

## 备注

- 单位与含义以源码注释为准；若使用不同相机或云台，请同步调整标定与限制参数。
- 修改 `config.yaml` 后需要重启程序生效。
