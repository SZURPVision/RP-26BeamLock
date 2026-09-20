#include <BiasKF.h>
#include <Config.h>
#include <FoxGloveServer.h>
#include <GimbalController.h>
#include <MVSCamera.h>
#include <Network.h>
#include <ScanMethod.h>
#include <TargetDetecter.h>
#include <TimeUtils.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <sstream>

constexpr double k_rad_to_deg = 180.0 / 3.14159265358979323846;

CommonTypes::GimbalAngles direction_to_gimbal_angles(const Eigen::Vector3d& direction)
{
    Eigen::Vector3d dir = direction.normalized();

    const double yaw_clockwise = std::atan2(-dir.y(), dir.x());
    Eigen::Vector3d axis_z(0.0, 0.0, 1.0);
    Eigen::AngleAxisd yaw_rotation(-yaw_clockwise, axis_z);

    Eigen::Vector3d base = yaw_rotation * Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d rotated_y = yaw_rotation * Eigen::Vector3d(0.0, 1.0, 0.0);

    const double right_hand_angle = std::atan2(rotated_y.dot(base.cross(dir)), base.dot(dir));
    const double pitch_clockwise = -right_hand_angle;

    return CommonTypes::GimbalAngles{
        .pitch = pitch_clockwise * k_rad_to_deg,
        .yaw = yaw_clockwise * k_rad_to_deg,
    };
}

std::atomic_bool g_running{true};
void handle_sigint(int) { g_running.store(false); }

bool do_log_image = true;

// NOLINTBEGIN(readability-identifier-naming)
struct ServoControllerState
{
    double integral_yaw = 0.0;
    double integral_pitch = 0.0;
    double last_error_yaw = 0.0;
    double last_error_pitch = 0.0;
    double last_target_x = 0.0;
    double last_target_y = 0.0;
    CommonTypes::Second last_time = 0.0;
    bool has_last = false;
};
// NOLINTEND(readability-identifier-naming)

enum Status : uint8_t
{
    Initializing = 0x00,
    Tracking = 0x01,
    Scanning = 0x02,
    Lost = 0x03
};

int main(int argc, char** argv)
{
    FoxGloveServer::get_server().start();
    ScanMode scan_mode = ScanMode::Line;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--no-image-log")
        {
            do_log_image = false;
            FoxGloveServer::get_server().log_message("Image logging disabled.", FoxGloveServer::LogLevel::WARNING);
        }
        else if (arg == "--scan-mode" && i + 1 < argc)
        {
            std::string mode = argv[++i];
            if (mode == "line")
            {
                scan_mode = ScanMode::Line;
            }
            else if (mode == "square")
            {
                scan_mode = ScanMode::Square;
            }
            else
            {
                FoxGloveServer::get_server().log_message("Unknown scan mode: " + mode + ", fallback to line.", FoxGloveServer::LogLevel::WARNING);
                scan_mode = ScanMode::Line;
            }
        }
    }

    if (scan_mode == ScanMode::Line)
    {
        FoxGloveServer::get_server().log_message("Scan mode set to line.", FoxGloveServer::LogLevel::INFO);
    }
    else
    {
        FoxGloveServer::get_server().log_message("Scan mode set to square.", FoxGloveServer::LogLevel::INFO);
    }

    std::signal(SIGINT, handle_sigint);
    try
    {
        Config::get_config().load_from_file();
        FoxGloveServer::get_server().log_message("Config loaded successfully.", FoxGloveServer::LogLevel::INFO);
    }
    catch (const std::exception& e)
    {
        FoxGloveServer::get_server().log_message(std::string("Failed to load config file: ") + e.what(), FoxGloveServer::LogLevel::WARNING);
        FoxGloveServer::get_server().log_message("Generating default config file.", FoxGloveServer::LogLevel::INFO);
        Config::get_config().generate_default_config();
        Config::get_config().load_from_file();
    }

    CameraManager camera_manager;
    auto cam_list = camera_manager.list_cameras();
    if (cam_list.is_empty())
    {
        FoxGloveServer::get_server().log_message("No cameras found!", FoxGloveServer::LogLevel::FATAL);
        FoxGloveServer::get_server().shutdown();
        return -1;
    }
    auto camera = camera_manager.create_camera(cam_list, 0);
    try
    {
        camera->start_grabbing();
        camera->set_gain(Config::get_config().get_camera_config()->m_gain);
        camera->set_exposure_time(Config::get_config().get_camera_config()->m_exposure);
    }
    catch (const std::exception& e)
    {
        FoxGloveServer::get_server().log_message(std::string("Camera init failed: ") + e.what(), FoxGloveServer::LogLevel::FATAL);
        FoxGloveServer::get_server().shutdown();
        return -1;
    }
    auto intrinsic_matrix = Config::get_config().get_camera_config()->m_intrinsic_matrix;
    auto distortion_coeffs = Config::get_config().get_camera_config()->m_distortion_coeffs;

    FoxGloveServer::get_server().log_message("Camera initialized.", FoxGloveServer::LogLevel::INFO);
    // cv::namedWindow("BeamLock", cv::WINDOW_NORMAL);

    GimbalController serial;
    FoxGloveServer::get_server().log_message("Serial port " + Config::get_config().get_serial_config()->m_port_name + " opened.", FoxGloveServer::LogLevel::INFO);

    NetworkHandler network(Config::get_config().get_network_config()->m_listen_ip, Config::get_config().get_network_config()->m_listen_port);
    FoxGloveServer::get_server().log_message("Network handler initialized, listening on " + Config::get_config().get_network_config()->m_listen_ip + ":" + std::to_string(Config::get_config().get_network_config()->m_listen_port),
                                             FoxGloveServer::LogLevel::INFO);

    Status status = Status::Initializing;
    CommonTypes::Second last_track_time = 0;
    CommonTypes::Second last_hit_time = TimeUtils::now_seconds();

    // 按距离分段，0-5,5-10,10-15,15-20,20-25,25-30
    // 其中0,1和5大概率用不到
    BiasKalmanFilter bias_kalman_filter[6];
    // 让bias滤波器能持续跟踪（过程噪声不能近似为0，否则增益塌缩后bias被冻结在初始位置）。
    // 观测噪声按"命中只保证落在目标半径R内"设置（R约4px → 方差16px²），可按实测目标大小/运动速度调节。
    for (auto& filter : bias_kalman_filter)
    {
        filter.set_process_noise_density(0.2);    // px^2/s
        filter.set_measurement_noise(16.0, 16.0); // px^2
    }
    Eigen::Vector2d last_scan_pixel = Eigen::Vector2d::Zero();
    ServoControllerState servo_state;

    // 命中校准：让扫描扫穿命中圆盘，同时利用 miss→hit(进入) 和 hit→miss(退出) 两个边沿，
    // 取弦中点直接估计"命中区域圆心"，比只采上升沿均值更高效，使bias向"激光正中目标中心"收敛。
    bool probing = true;               // 是否处于扫穿校准状态
    bool have_entry = false;           // 是否已记录进入圆盘的扫描偏移
    bool prev_hit = false;             // 上一帧是否命中（用于检测边沿）
    Eigen::Vector2d sweep_entry = Eigen::Vector2d::Zero(); // 进入命中区域时的扫描偏移
    int hold_miss_count = 0;           // 锁定时连续未命中帧数
    constexpr int k_hold_miss_threshold = 10; // 锁定后连续未命中多少帧触发重新扫穿校准
    BiasKalmanFilter* last_kf = nullptr; // 上一帧使用的距离段滤波器（跨段时继承估计）

    const auto servo_config = Config::get_config().get_servo_config();
    const double k_p = servo_config->m_k_p;
    const double k_i = servo_config->m_k_i;
    const double k_d = servo_config->m_k_d;
    const double k_ff = servo_config->m_k_ff;
    const double integral_limit = servo_config->m_integral_limit;
    const double integrate_deadzone = servo_config->m_integrate_deadzone;
    const double min_offset = servo_config->m_min_offset;
    while (!serial.is_running())
    {
        network.send_heartbeat(static_cast<uint8_t>(status));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    status = Status::Scanning;
    while (g_running.load())
    {
        cv::Mat frame;
        network.send_heartbeat(static_cast<uint8_t>(status));
        try
        {
            // 注意：此处的frame是**BayerRG格式**的单通道图像
            // BayerRG的排列方式是：
            // R G R G ...
            // G B G B ...
            // ...
            camera->get_frame(frame, 1000);
        }
        catch (const std::exception& e)
        {
            FoxGloveServer::get_server().log_message(std::string("Failed to get frame: ") + e.what(), FoxGloveServer::LogLevel::FATAL);
            continue;
        }
        if (do_log_image)
        {
            FoxGloveServer::get_server().log_camera_image(frame);
        }
        CommonTypes::Second timestamp = TimeUtils::now_seconds();
        CommonTypes::GimbalAngles current_angles = serial.get_current_angles();

        auto result = TargetDetecter::detect_best_target(frame);
        const auto& feature = result.feature;

        CommonTypes::NetworkPack latest_pack = network.get_latest_pack();
        double distance = latest_pack.x * latest_pack.x + latest_pack.y * latest_pack.y + latest_pack.z * latest_pack.z;
        distance = std::sqrt(distance);
        if (distance <= 0.0 || distance > 40.0)
        {
            FoxGloveServer::get_server().log_message("Received invalid distance: " + std::to_string(distance) + ", use default distance: 15.0", FoxGloveServer::LogLevel::WARNING);
            distance = 15.0;
        }
        const double retry_distance = 15.0;
        // 将雷达测距值投影为图像中的激光点（由距离换算激光在云台系下的坐标并重投影）
        const auto project_lazer = [&](double dist)
        {
            const double lx = Config::get_config().get_tripodhead_config()->m_x_k * dist + Config::get_config().get_tripodhead_config()->m_x_b;
            const double ly = Config::get_config().get_tripodhead_config()->m_y_k * dist + Config::get_config().get_tripodhead_config()->m_y_b;
            std::vector<cv::Point2d> pts;
            cv::projectPoints(std::vector<cv::Point3d>{cv::Point3d(lx, ly, dist)}, cv::Vec3d::zeros(), cv::Vec3d::zeros(), intrinsic_matrix, distortion_coeffs, pts);
            return pts[0];
        };
        const auto is_in_image = [&](const cv::Point2d& p) { return p.x >= 0.0 && p.x < frame.cols && p.y >= 0.0 && p.y < frame.rows; };

        std::vector<cv::Point2d> lazer_point_image{project_lazer(distance)};

        auto* kf = &bias_kalman_filter[std::min(static_cast<size_t>(distance / 5), static_cast<size_t>(5))];

        // 检验激光点是否在图像内，如果不是则将distance设置为15重试，仍然不行则设为图像中心
        if (!is_in_image(lazer_point_image[0]))
        {
            const cv::Point2d retry_projected = project_lazer(retry_distance);
            lazer_point_image[0] = is_in_image(retry_projected) ? retry_projected : cv::Point2d(frame.cols / 2.0, frame.rows / 2.0);

            kf = &bias_kalman_filter[std::min(static_cast<size_t>(retry_distance / 5), static_cast<size_t>(5))];
        }

        // 目标移动导致距离跨段时，用上一段的bias估计初始化新段，避免bias跳回0造成脱靶
        if (last_kf != nullptr && kf != last_kf)
        {
            kf->reset(last_kf->state());
            probing = true;
            have_entry = false;
            prev_hit = false;
            last_scan_pixel = Eigen::Vector2d::Zero();
            last_hit_time = timestamp;
        }
        last_kf = kf;
        auto bias = kf->state();

        double lazer_pixel_x = lazer_point_image[0].x + bias.x();
        double lazer_pixel_y = lazer_point_image[0].y + bias.y();

        FoxGloveServer::get_server().log_lazer_annotation(cv::Point2d(lazer_pixel_x, lazer_pixel_y));

        if (!latest_pack.allow_counter)
        {
            // 将激光点往下（y正）偏移一定像素，保证云台找到目标还能进Track模式但是不反制
            lazer_pixel_y += 100;
        }

#ifdef CV_SHOW
        cv::Mat overlay = frame.clone();
        cv::line(overlay, cv::Point(lazer_pixel_x - 10, lazer_pixel_y), cv::Point(lazer_pixel_x + 10, lazer_pixel_y), cv::Scalar(255), 2);
        cv::line(overlay, cv::Point(lazer_pixel_x, lazer_pixel_y - 10), cv::Point(lazer_pixel_x, lazer_pixel_y + 10), cv::Scalar(255), 2);
#endif

        // 如果云台角度超出范围，转入Scan模式，优先级高于锁定和丢失
        if (serial.consume_angle_exceeded_flag())
        {
            FoxGloveServer::get_server().log_message("Gimbal target angles exceed limits, switch to scanning mode.", FoxGloveServer::LogLevel::WARNING);
            status = Status::Scanning;
        }
        else if (!feature.empty())
        {
            if (status != Status::Tracking)
            {
                FoxGloveServer::get_server().log_message("Target founded. Transfer to tracking mode.", FoxGloveServer::LogLevel::INFO);
                status = Status::Tracking;
            }

            double avg_pixel_x = 0.0, avg_pixel_y = 0.0;
            for (const auto& pt : feature)
            {
                avg_pixel_x += pt.x;
                avg_pixel_y += pt.y;
            }
            avg_pixel_x /= feature.size();
            avg_pixel_y /= feature.size();

            FoxGloveServer::get_server().log_target_annotation(cv::Point2f(static_cast<float>(avg_pixel_x), static_cast<float>(avg_pixel_y)), result.is_hit);
#ifdef CV_SHOW
            cv::circle(overlay, cv::Point(static_cast<int>(avg_pixel_x), static_cast<int>(avg_pixel_y)), 10, cv::Scalar(255), 2);
#endif

            // ---- 命中校准：扫穿命中圆盘，利用 miss→hit 与 hit→miss 两个边沿求弦中点 ----
            Eigen::Vector2d scan_delta = Eigen::Vector2d::Zero();
            bool apply_scan = false;

            if (result.is_hit)
            {
                if (probing)
                {
                    if (!prev_hit)
                    {
                        // 上升沿：扫进命中区域，记录进入偏移
                        sweep_entry = last_scan_pixel;
                        have_entry = true;
                    }
                    // 扫穿模式：命中帧也沿螺旋继续前进，以便从另一侧扫出圆盘
                    apply_scan = true;
                }
                else
                {
                    hold_miss_count = 0; // 锁定时命中，重置丢失计数
                }
            }
            else
            {
                if (probing)
                {
                    if (have_entry && prev_hit)
                    {
                        // 下降沿：扫出命中区域 → 弦中点即圆心估计，校准bias并转入锁定
                        const Eigen::Vector2d sweep_exit = last_scan_pixel;
                        const Eigen::Vector2d disk_center = (sweep_entry + sweep_exit) * 0.5;

                        // clang-format off
                        FoxGloveServer::get_server().log_message(
                            "Bias calibrate: chord center (" 
                            + std::to_string(disk_center.x()) + ","
                            + std::to_string(disk_center.y()) + "), bias -> ("
                            + std::to_string(bias.x() + disk_center.x()) + "," 
                            + std::to_string(bias.y() + disk_center.y()) + ")",
                            FoxGloveServer::LogLevel::DEBUG
                        );
                        // clang-format on

                        // 测量 = 当前bias + 圆心修正 ≈ 理想bias（让激光正中目标中心）
                        kf->step(bias + disk_center, timestamp);

                        have_entry = false;
                        probing = false; // 校准完成，进入锁定（瞄准基点，不加扫描）
                        hold_miss_count = 0;
                    }
                    else
                    {
                        apply_scan = true; // 尚未完成一次完整穿越，继续扫
                    }
                }
                else
                {
                    // 锁定模式：不加扫描，先让伺服把目标拉回基点
                    ++hold_miss_count;
                    if (hold_miss_count >= k_hold_miss_threshold)
                    {
                        // 丢失过久，重新扫穿校准
                        probing = true;
                        have_entry = false;
                        last_hit_time = timestamp; // 从最小半径重新扫
                        apply_scan = true;
                    }
                }
            }

            if (apply_scan)
            {
                // 叠加同心扫描偏移，扫穿命中区域
                // clang-format off
                scan_delta = get_concentric_scan_delta_pixel(
                    timestamp,
                    servo_config->m_scan_initial_radius,
                    servo_config->m_scan_radius_delta,
                    servo_config->m_scan_max_radius,
                    servo_config->m_scan_speed,
                    last_hit_time
                );
                // clang-format on
                lazer_pixel_x += scan_delta.x();
                lazer_pixel_y += scan_delta.y();
            }
            last_scan_pixel = apply_scan ? scan_delta : Eigen::Vector2d::Zero();

            double fx = intrinsic_matrix.at<double>(0, 0);
            double fy = intrinsic_matrix.at<double>(1, 1);
            double pixel_x_offset = avg_pixel_x - lazer_pixel_x;
            double pixel_y_offset = avg_pixel_y - lazer_pixel_y;

            double error_yaw = std::atan(pixel_x_offset / fx) * Config::get_config().get_tripodhead_config()->m_invert_yaw_angle;
            double error_pitch = std::atan(pixel_y_offset / fy) * Config::get_config().get_tripodhead_config()->m_invert_pitch_angle;

            double dt = 0.0;
            if (servo_state.has_last)
            {
                dt = timestamp - servo_state.last_time;
                if (dt < 0)
                    dt = 0.0;
            }

            double error_rate_yaw = 0.0;
            double error_rate_pitch = 0.0;
            double ff_yaw = 0.0;
            double ff_pitch = 0.0;

            if (dt > 0.0)
            {
                error_rate_yaw = (error_yaw - servo_state.last_error_yaw) / dt;
                error_rate_pitch = (error_pitch - servo_state.last_error_pitch) / dt;

                double target_vel_x = (avg_pixel_x - servo_state.last_target_x) / dt;
                double target_vel_y = (avg_pixel_y - servo_state.last_target_y) / dt;
                ff_yaw = std::atan(target_vel_x / fx) * Config::get_config().get_tripodhead_config()->m_invert_yaw_angle;
                ff_pitch = std::atan(target_vel_y / fy) * Config::get_config().get_tripodhead_config()->m_invert_pitch_angle;

                if (std::abs(error_yaw) > integrate_deadzone)
                {
                    servo_state.integral_yaw += k_i * error_yaw * dt;
                    servo_state.integral_yaw = std::clamp(servo_state.integral_yaw, -integral_limit, integral_limit);
                }
                if (std::abs(error_pitch) > integrate_deadzone)
                {
                    servo_state.integral_pitch += k_i * error_pitch * dt;
                    servo_state.integral_pitch = std::clamp(servo_state.integral_pitch, -integral_limit, integral_limit);
                }
            }

            double d_yaw = k_p * error_yaw + servo_state.integral_yaw + k_d * error_rate_yaw + k_ff * ff_yaw;
            double d_pitch = k_p * error_pitch + servo_state.integral_pitch + k_d * error_rate_pitch + k_ff * ff_pitch;

            if (std::isnan(d_yaw) || std::isnan(d_pitch))
            {
                FoxGloveServer::get_server().log_message("Calculated NaN control signal, skipping this frame.", FoxGloveServer::LogLevel::ERROR);
                continue;
            }

            if (!result.is_hit)
            {
                // 如果没有击中目标，强制将移动角度提到最小值以上
                if (abs(d_yaw) < min_offset)
                    d_yaw = (d_yaw > 0 ? min_offset : -min_offset);
                if (abs(d_pitch) < min_offset)
                    d_pitch = (d_pitch > 0 ? min_offset : -min_offset);
            }

            CommonTypes::GimbalAngles target_angles = current_angles;
            target_angles.yaw += d_yaw / M_PI * 180;
            target_angles.pitch += d_pitch / M_PI * 180;

            FoxGloveServer::get_server().log_gimbal_target_rotation(target_angles);

            // serial.set_target_angles(target_angles, timestamp);
            serial.set_target_speeds(
                CommonTypes::GimbalAngles{
                    .pitch = d_pitch,
                    .yaw = d_yaw,
                },
                timestamp,
                5);
            last_track_time = timestamp;

            servo_state.last_error_yaw = error_yaw;
            servo_state.last_error_pitch = error_pitch;
            servo_state.last_target_x = avg_pixel_x;
            servo_state.last_target_y = avg_pixel_y;
            servo_state.last_time = timestamp;
            servo_state.has_last = true;
        }
        else
        {
            servo_state = ServoControllerState{};
            if (status == Status::Tracking)
            {
                FoxGloveServer::get_server().log_message("Lost target, waiting for reacquisition...", FoxGloveServer::LogLevel::INFO);
                status = Status::Lost;
            }
            if (status == Status::Lost && timestamp - last_track_time > 5)
            {
                FoxGloveServer::get_server().log_message("Target lost for more than 5 seconds, transfer to scanning mode.", FoxGloveServer::LogLevel::INFO);
                status = Status::Scanning;
            }
        }
        // 记录本帧命中状态，供下一帧检测 miss→hit 边界
        prev_hit = result.is_hit;
        if (status == Status::Scanning)
        {
            // clang-format off
            const auto track_strategy = Config::get_config().get_track_strategy_config();
            CommonTypes::GimbalAngles scan_delta = (scan_mode == ScanMode::Line)
                ? get_line_scan_delta_angles(
                      timestamp,
                      track_strategy->m_line_scan_yaw_min,
                      track_strategy->m_line_scan_yaw_max,
                      track_strategy->m_line_scan_pitch_min,
                      track_strategy->m_line_scan_pitch_max,
                      track_strategy->m_line_scan_speed,
                      track_strategy->m_line_scan_pitch_step)
                : get_square_scan_delta_angles(
                      timestamp,
                      track_strategy->m_scan_yaw_angle,
                      track_strategy->m_scan_pitch_angle,
                      track_strategy->m_scan_period);
            // clang-format on
            CommonTypes::GimbalAngles target_angles{};
            if (scan_mode == ScanMode::Line)
            {
                target_angles = scan_delta;
            }
            else
            {
                // 使用激光引导变换矩阵将雷达点转换为云台角度
                const auto& guide_transform = track_strategy->m_guide_transform;
                Eigen::Vector4d homogeneous_point(latest_pack.x, latest_pack.y, latest_pack.z, 1.0);
                Eigen::Vector4d gimbal_dir_vec = guide_transform * homogeneous_point;
                Eigen::Vector3d gimbal_dir(gimbal_dir_vec(0), gimbal_dir_vec(1), gimbal_dir_vec(2));

                auto base_angles = direction_to_gimbal_angles(gimbal_dir);

                target_angles = CommonTypes::GimbalAngles{
                    .pitch = base_angles.pitch + scan_delta.pitch,
                    .yaw = base_angles.yaw + scan_delta.yaw,
                };
            }
            serial.set_target_encoder_angles(target_angles, timestamp);
        }
#ifdef CV_SHOW
        cv::imshow("BeamLock", overlay);
        cv::waitKey(1);
#endif
    }
    if (!g_running.load())
    {
        FoxGloveServer::get_server().log_message("SIGINT received, shutting down.", FoxGloveServer::LogLevel::INFO);
    }
    FoxGloveServer::get_server().shutdown();
    return 0;
}