# 标定

本仓库的标定分为三部分，另有一项在运行中自动进行的动态标定。所有结果最终都写进运行目录的 `config.yaml`（字段含义见 `CONFIG.md`）。

| 步骤 | 标定内容 | 工具 | 写入的配置 |
| --- | --- | --- | --- |
| 1 | 相机内参、畸变 | `Tools/CameraTune/camera_tune`、`camera_calibration` | `CameraConfig.intrinsic_matrix` / `distortion_coeffs` |
| 2 | 激光光路（激光与相机的相对位姿） | `Tools/LazerCalibration/lazer_calibration` | `TripodHeadConfig.x_k/x_b/y_k/y_b` |
| 3 | 雷达坐标系 → 云台坐标系外参 | `Tools/LidarGuideCalibration/*` | `TrackStrategyConfig.guide_translation` / `guide_rotation_euler_zyx_deg` |
| 4 | 激光偏置（动态） | 主程序自动进行 | 不写配置，运行时维护 |

> 标定必须使用真实相机与官方 MVS SDK：`-DUSE_FAKE_MVS=ON` 只能让代码编过，枚举不到设备、也取不到图像。
>
> 这套方案对标定精度的要求并不苛刻：动态标定足以覆盖一定的标定误差。
> 但**激光光路**与**雷达引导外参**必须至少各标一次，否则激光不会落在目标上、雷达引导也无法使用。

## 1. 相机内参

用 `Tools/CameraTune/camera_calibration` 拍摄棋盘格/标定板图像并求解内参（OpenCV 的 `calibrateCamera`），
或直接用 `camera_tune` 观察画面、调整增益与曝光。

- 图像分辨率需与 `CameraConfig.image_width/height` 一致。
- 标定完成后把 3×3 内参与 5 个畸变系数填入 `CameraConfig`。
- 更换镜头、重新对焦或改变分辨率后必须重新标定。

## 2. 激光光路标定（`Tools/LazerCalibration`）

思路：激光在图像中的位置随距离变化，用不同距离下的激光光斑拟合出激光在相机坐标系下的直线方程
`x = x_k·z + x_b`、`y = y_k·z + y_b`。

步骤：

1. 固定云台，保持相机与激光的相对位置不动（这一步之后**不要**再碰相机、激光或镜头对焦环）。
2. 用一块板子接住激光，尽量让光斑落在板子中央；记录板子到云台的距离。
3. 改变板子距离，重复第 2 步若干次（建议覆盖 5 m～20 m，至少 5 个距离点）。
4. 用 `Tools/CameraTune/CaptureServer` 远程截图，把图像命名为 `01.jpg, 02.jpg, 03.jpg ...`
   放入运行目录下的 `CalibrationData/`，并在同目录的 `distance.txt` 中按同样顺序写入对应距离（米，每行一个）。
5. 运行 `lazer_calibration`，得到 `x_k, x_b, y_k, y_b`，填入 `TripodHeadConfig`。

注意事项：

- 尽量在较暗的环境下采集，避免环境光干扰激光检测。
- 距离测量误差控制在 0.5 m 以内即可；拟合误差一般在半个像素内。
- 距离数据可以用卷尺/激光测距仪直接测量，也可以用雷达点云读出板面到云台的距离（若手上有点云查看工具）。
- `Tools/LazerCalibration/lazer_calibration_once` 可以在画面中用滑条手动框选激光点，用于确认检测是否正常。

## 3. 雷达引导外参标定（`Tools/LidarGuideCalibration`）

雷达主程序发来的是**雷达坐标系**下的米制坐标，云台需要用 `guide_translation` +
`guide_rotation_euler_zyx_deg` 把它转到云台角度。标定方法：

1. 让雷达进入正常工作状态，在场地内放置一个可以被雷达稳定检测到的目标（例如一个金矿）。
2. 采集若干组「雷达坐标 ↔ 云台角度」对应数据：
   - 将激光指向雷达检测到的目标
   - 用 `lidar_guide_encoder_monitor` 读取云台编码器/角度；
   - 记录同一时刻雷达给出的目标坐标（可从 Foxglove 的 `/points/lidar_target` 通道读取，或由雷达主程序侧记录）。
3. 用 `lidar_guide_calibration` 求解雷达坐标系→云台坐标系的变换：以雷达点为物点、云台角换算出的单位方向向量为像点，用 `cv::solvePnPRansac` 求解旋转与平移，并打印内点数与叉乘残差（均值/最大值）用于评估标定质量，结果写入 `GuideTransform.yaml` 或 `GuideTransform.txt`。
4. 用 `lidar_guide_point_to_angle`（读 YAML）或 `lidar_guide_point_to_angle_rotation` / `_eular`（读 TXT，支持运行中热重载）
   验证：给出一个雷达坐标，检查算出的云台角是否指向目标；加 `--send` 可直接驱动云台。

`lidar_guide_calibration_gen` 可以生成带噪声的合成数据（`points.txt` / `angles.txt` / 真值变换），
在没有雷达的情况下验证解算流程是否正确。

## 4. 动态标定（运行中自动进行）

主程序在跟踪目标时同时做两件事：

1. **命中判定**：检测目标是否正被激光照射（在线 2-means 区分“被照射/未被照射”的亮度聚类）。
2. **偏置修正**：未命中时在预设激光点周围做同心圆扫描；当扫描**扫过命中区域**（miss→hit→miss 两个边沿）时，
   取弦中点作为命中区域圆心，用卡尔曼滤波修正激光偏置。滤波器按距离分段（每 5 m 一段），跨段时继承上一段的估计。

因此标定不必一次做到完美：只要激光点离目标中心不太远，运行中的动态标定会逐步把它拉正。
调试时可以观察 Foxglove 的两组标注：

- 绿色十字：程序认为的激光落点；
- 红色/黄色圆圈：目标中心，红色表示判定为命中。

## 5. 常见问题

| 现象 | 排查方向 |
| --- | --- |
| 激光点检测不到 | 环境光太亮；`DetectConfig.adaptive_threshold_c` 不合适；光斑太小（距离太远） |
| 激光总是偏一个方向 | `x_k/x_b/y_k/y_b` 未标定或标定后相机/激光被移动过 |
| 目标在画面里但云台不跟 | `invert_yaw_angle` / `invert_pitch_angle` 方向不对；网络未收到有效坐标 |
| 云台频繁进入扫描 | 角度护栏 `max_pitch_angle`/`max_yaw_angle` 触发，或雷达坐标超出 `(0, 40] m` 被丢弃 |
| 雷达引导方向不对 | `guide_translation` / `guide_rotation_euler_zyx_deg` 未标定（默认全 0 表示两坐标系重合） |
