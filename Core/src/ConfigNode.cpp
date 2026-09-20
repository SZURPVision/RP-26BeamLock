#include <Config.h>

// clang-format off

std::shared_ptr<Config::CameraConfig>
Config::CameraConfig::create(
    const cv::Mat& intrinsic_matrix, 
    const cv::Mat& distortion_coeffs, 
    const cv::Size& image_size, 
    double gain, 
    double exposure)
{
    return std::shared_ptr<CameraConfig>(new CameraConfig(
        intrinsic_matrix, 
        distortion_coeffs, 
        image_size, 
        gain, 
        exposure));
}
std::shared_ptr<Config::TripodHeadConfig>
Config::TripodHeadConfig::create(
    double x_k,
    double x_b,
    double y_k,
    double y_b,
    bool invert_yaw_angle,
    bool invert_pitch_angle,
    bool invert_yaw_encoder,
    bool invert_pitch_encoder,
    int yaw_encoder_per_round,
    int pitch_encoder_per_round,
    double max_acceleration,
    double max_velocity,
    double max_pitch_angle,
    double max_yaw_angle)
{
    return std::shared_ptr<TripodHeadConfig>(new TripodHeadConfig(
        x_k,
        x_b,
        y_k,
        y_b,
        invert_yaw_angle ? -1 : 1, 
        invert_pitch_angle ? -1 : 1,
        invert_yaw_encoder ? -1 : 1,
        invert_pitch_encoder ? -1 : 1,
        yaw_encoder_per_round,
        pitch_encoder_per_round,
        max_acceleration,
        max_velocity,
        max_pitch_angle,
        max_yaw_angle));
}

std::shared_ptr<Config::ServoConfig>
Config::ServoConfig::create(
    double k_p,
    double k_i,
    double k_d,
    double k_ff,
    double integral_limit,
    double integrate_deadzone,
    double min_offset,
    double scan_initial_radius,
    double scan_radius_delta,
    double scan_max_radius,
    double scan_speed)
{
    return std::shared_ptr<ServoConfig>(new ServoConfig(
        k_p,
        k_i,
        k_d,
        k_ff,
        integral_limit,
        integrate_deadzone,
        min_offset,
        scan_initial_radius,
        scan_radius_delta,
        scan_max_radius,
        scan_speed));
}
std::shared_ptr<Config::TrackStrategyConfig> 
Config::TrackStrategyConfig::create(
    double reset_time_threshold, 
    double scan_yaw_angle, 
    double scan_pitch_angle, 
    double scan_period,
    double line_scan_yaw_min,
    double line_scan_yaw_max,
    double line_scan_pitch_min,
    double line_scan_pitch_max,
    double line_scan_speed,
    double line_scan_pitch_step,
    double guide_tx,
    double guide_ty,
    double guide_tz,
    double guide_yaw_deg,
    double guide_pitch_deg,
    double guide_roll_deg)
{
    // 将 ZYX 欧拉角 (yaw, pitch, roll) 转换为旋转矩阵
    constexpr double k_deg_to_rad = 3.14159265358979323846 / 180.0;
    const double a = guide_yaw_deg * k_deg_to_rad;
    const double b = guide_pitch_deg * k_deg_to_rad;
    const double c = guide_roll_deg * k_deg_to_rad;

    const double ca = std::cos(a), sa = std::sin(a);
    const double cb = std::cos(b), sb = std::sin(b);
    const double cc = std::cos(c), sc = std::sin(c);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Matrix4d guide_transform = Eigen::Matrix4d::Identity();
    guide_transform(0, 0) = ca * cb;
    guide_transform(0, 1) = ca * sb * sc - sa * cc;
    guide_transform(0, 2) = ca * sb * cc + sa * sc;
    guide_transform(1, 0) = sa * cb;
    guide_transform(1, 1) = sa * sb * sc + ca * cc;
    guide_transform(1, 2) = sa * sb * cc - ca * sc;
    guide_transform(2, 0) = -sb;
    guide_transform(2, 1) = cb * sc;
    guide_transform(2, 2) = cb * cc;

    // 设置平移
    guide_transform(0, 3) = guide_tx;
    guide_transform(1, 3) = guide_ty;
    guide_transform(2, 3) = guide_tz;

    return std::shared_ptr<TrackStrategyConfig>(new TrackStrategyConfig(
        reset_time_threshold, 
        scan_yaw_angle, 
        scan_pitch_angle, 
        scan_period,
        line_scan_yaw_min,
        line_scan_yaw_max,
        line_scan_pitch_min,
        line_scan_pitch_max,
        line_scan_speed,
        line_scan_pitch_step,
        guide_transform));
}
std::shared_ptr<Config::DetectConfig>
Config::DetectConfig::create(
    double light_blob_x_threshold,
    double light_blob_y_threshold,
    double min_hit_difference,
    double adaptive_threshold_c)
{
    return std::shared_ptr<DetectConfig>(new DetectConfig(
        light_blob_x_threshold,
        light_blob_y_threshold,
        min_hit_difference,
        adaptive_threshold_c));
}
std::shared_ptr<Config::SerialConfig> 
Config::SerialConfig::create(
    int buffer_size, 
    int prepare_size, 
    unsigned int baud_rate, 
    int reset_pack_count, 
    const std::string& port_name, 
    double max_send_frequency)
{
    return std::shared_ptr<SerialConfig>(new SerialConfig(
        buffer_size, 
        prepare_size, 
        baud_rate, 
        reset_pack_count, 
        port_name, 
        max_send_frequency));
}
std::shared_ptr<Config::NetworkConfig> 
Config::NetworkConfig::create(
    const std::string& listen_ip, 
    uint16_t listen_port, 
    const std::string& send_target_ip,
    uint16_t send_target_port)
{
    return std::shared_ptr<NetworkConfig>(new NetworkConfig(
        listen_ip, 
        listen_port, 
        send_target_ip,
        send_target_port));
}

// clang-format on
