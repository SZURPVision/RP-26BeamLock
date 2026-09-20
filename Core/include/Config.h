#pragma once
#include <cstring>
#include <Eigen/Eigen>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>

class Config
{
    static constexpr auto default_config_file = "config.yaml";

    // clang-format off

    struct CameraConfig
    {
        const cv::Mat m_intrinsic_matrix;
        const cv::Mat m_distortion_coeffs;
        const cv::Size m_image_size;

        const double m_gain;
        const double m_exposure;

        CameraConfig() = delete;
        static std::shared_ptr<CameraConfig> create(
            const cv::Mat& intrinsic_matrix, 
            const cv::Mat& distortion_coeffs, 
            const cv::Size& image_size, 
            double gain, 
            double exposure);

    private:
        CameraConfig(
            const cv::Mat& intrinsic_matrix, 
            const cv::Mat& distortion_coeffs, 
            const cv::Size& image_size,
            double gain, 
            double exposure)
            : m_intrinsic_matrix(intrinsic_matrix)
            , m_distortion_coeffs(distortion_coeffs)
            , m_image_size(image_size)
            , m_gain(gain)
            , m_exposure(exposure){};
    };

    struct TripodHeadConfig
    {
        // 表示激光光路在相机坐标系中的直线方程参数，x = m_x_k * z + m_x_b, y = m_y_k * z + m_y_b
        const double m_x_k;
        const double m_x_b;
        const double m_y_k;
        const double m_y_b;

        const double m_invert_yaw_angle;
        const double m_invert_pitch_angle;

        const double m_invert_yaw_encoder;
        const double m_invert_pitch_encoder;

        const int m_yaw_encoder_per_round;
        const int m_pitch_encoder_per_round;

        const double m_max_acceleration; // deg/s^2, 云台的最大加速度
        const double m_max_velocity; // deg/s, 云台的最大速度
        const double m_max_pitch_angle; // deg, pitch 相对初始角的最大允许角度
        const double m_max_yaw_angle; // deg, yaw 相对初始角的最大允许角度

        TripodHeadConfig() = delete;
        static std::shared_ptr<TripodHeadConfig> create(
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
            double max_yaw_angle);

    private:
        TripodHeadConfig(
            double x_k,
            double x_b,
            double y_k,
            double y_b,
            double invert_yaw_angle, 
            double invert_pitch_angle,
            double invert_yaw_encoder,
            double invert_pitch_encoder,
            int yaw_encoder_per_round,
            int pitch_encoder_per_round,
            double max_acceleration,
            double max_velocity,
            double max_pitch_angle,
            double max_yaw_angle)
            : m_x_k(x_k)
            , m_x_b(x_b)
            , m_y_k(y_k)
            , m_y_b(y_b)
            , m_invert_yaw_angle(invert_yaw_angle)
            , m_invert_pitch_angle(invert_pitch_angle)
            , m_invert_yaw_encoder(invert_yaw_encoder)
            , m_invert_pitch_encoder(invert_pitch_encoder)
            , m_yaw_encoder_per_round(yaw_encoder_per_round)
            , m_pitch_encoder_per_round(pitch_encoder_per_round)
            , m_max_acceleration(max_acceleration)
            , m_max_velocity(max_velocity)
            , m_max_pitch_angle(max_pitch_angle)
            , m_max_yaw_angle(max_yaw_angle){};
    };

    struct ServoConfig
    {
        const double m_k_p;
        const double m_k_i;
        const double m_k_d;
        const double m_k_ff;
        const double m_integral_limit;
        const double m_integrate_deadzone;
        const double m_min_offset;
        const double m_scan_initial_radius;
        const double m_scan_radius_delta;
        const double m_scan_max_radius;
        const double m_scan_speed;

        ServoConfig() = delete;
        static std::shared_ptr<ServoConfig> create(
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
            double scan_speed);

    private:
        ServoConfig(
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
            : m_k_p(k_p)
            , m_k_i(k_i)
            , m_k_d(k_d)
            , m_k_ff(k_ff)
            , m_integral_limit(integral_limit)
            , m_integrate_deadzone(integrate_deadzone)
            , m_min_offset(min_offset)
            , m_scan_initial_radius(scan_initial_radius)
            , m_scan_radius_delta(scan_radius_delta)
            , m_scan_max_radius(scan_max_radius)
            , m_scan_speed(scan_speed){};
    };

    struct TrackStrategyConfig
    {
        const double m_reset_time_threshold; // s, 目标丢失多长时间后开始扫描
        const double m_scan_yaw_angle; // deg, 扫描时将在目标值+-这个角度范围内扫描
        const double m_scan_pitch_angle; // deg, 扫描时将在目标值+-这个角度范围内扫描
        const double m_scan_period; // s, 扫描一轮的时间
        const double m_line_scan_yaw_min; // deg, 逐行扫描的yaw最小角度
        const double m_line_scan_yaw_max; // deg, 逐行扫描的yaw最大角度
        const double m_line_scan_pitch_min; // deg, 逐行扫描的pitch最小角度
        const double m_line_scan_pitch_max; // deg, 逐行扫描的pitch最大角度
        const double m_line_scan_speed; // deg/s, 逐行扫描的yaw扫描速度
        const double m_line_scan_pitch_step; // deg, 每行之间的pitch步进
        const Eigen::Matrix4d m_guide_transform; // 激光引导变换矩阵 (4x4), 由 guide_translation 和 guide_rotation_euler_zyx_deg 转换得到

        TrackStrategyConfig() = delete;
        static std::shared_ptr<TrackStrategyConfig> create(
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
            double guide_roll_deg);
    
    private:
        TrackStrategyConfig(
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
            const Eigen::Matrix4d& guide_transform)
            : m_reset_time_threshold(reset_time_threshold)
            , m_scan_yaw_angle(scan_yaw_angle)
            , m_scan_pitch_angle(scan_pitch_angle)
            , m_scan_period(scan_period)
            , m_line_scan_yaw_min(line_scan_yaw_min)
            , m_line_scan_yaw_max(line_scan_yaw_max)
            , m_line_scan_pitch_min(line_scan_pitch_min)
            , m_line_scan_pitch_max(line_scan_pitch_max)
            , m_line_scan_speed(line_scan_speed)
            , m_line_scan_pitch_step(line_scan_pitch_step)
            , m_guide_transform(guide_transform){};
    };

    struct DetectConfig
    {
        const double m_light_blob_x_threshold;
        const double m_light_blob_y_threshold;
        const double m_min_hit_difference;
        const double m_adaptive_threshold_c;

        DetectConfig() = delete;
        static std::shared_ptr<DetectConfig> create(
            double light_blob_x_threshold,
            double light_blob_y_threshold,
            double min_hit_difference,
            double adaptive_threshold_c);

    private:
        DetectConfig(
            double light_blob_x_threshold,
            double light_blob_y_threshold,
            double min_hit_difference,
            double adaptive_threshold_c)
            : m_light_blob_x_threshold(light_blob_x_threshold)
            , m_light_blob_y_threshold(light_blob_y_threshold)
            , m_min_hit_difference(min_hit_difference)
            , m_adaptive_threshold_c(adaptive_threshold_c){};
    };

    struct SerialConfig
    {
        const int m_buffer_size;
        const int m_prepare_size;
        const unsigned int m_baud_rate;
        const int m_reset_pack_count;
        const std::string m_port_name;
        const double m_max_send_frequency;

        SerialConfig() = delete;
        static std::shared_ptr<SerialConfig> create(
            int buffer_size, 
            int prepare_size, 
            unsigned int baud_rate, 
            int reset_pack_count, 
            const std::string& port_name, 
            double max_send_frequency);

    private:
        SerialConfig(
            int buffer_size,
            int prepare_size, 
            unsigned int baud_rate, 
            int reset_pack_count, 
            const std::string& port_name, 
            double max_send_frequency)
            : m_buffer_size(buffer_size)
            , m_prepare_size(prepare_size)
            , m_baud_rate(baud_rate)
            , m_reset_pack_count(reset_pack_count)
            , m_port_name(port_name)
            , m_max_send_frequency(max_send_frequency){};
    };

    struct NetworkConfig
    {
        const std::string m_listen_ip;
        const uint16_t m_listen_port;
        const std::string m_send_target_ip;
        const uint16_t m_send_target_port;

        NetworkConfig() = delete;
        static std::shared_ptr<NetworkConfig> create(
            const std::string& listen_ip, 
            uint16_t listen_port,
            const std::string& send_target_ip,
            uint16_t send_target_port);

    private:
        NetworkConfig(
            const std::string& listen_ip, 
            uint16_t listen_port,
            const std::string& send_target_ip,
            uint16_t send_target_port) 
            : m_listen_ip(listen_ip)
            , m_listen_port(listen_port)
            , m_send_target_ip(send_target_ip)
            , m_send_target_port(send_target_port){};
    };

    // clang-format on

private:
    std::shared_ptr<CameraConfig> m_camera_config;
    std::shared_ptr<TripodHeadConfig> m_tripodhead_config;
    std::shared_ptr<ServoConfig> m_servo_config;
    std::shared_ptr<TrackStrategyConfig> m_track_strategy_config;
    std::shared_ptr<DetectConfig> m_detect_config;
    std::shared_ptr<SerialConfig> m_serial_config;
    std::shared_ptr<NetworkConfig> m_network_config;
    bool m_is_loaded = false;

public:
    static Config& get_config()
    {
        static Config config;
        return config;
    }
    std::shared_ptr<CameraConfig> get_camera_config()
    {
        if (!m_is_loaded)
            throw std::runtime_error("Config not loaded!");
        return m_camera_config;
    }
    std::shared_ptr<TripodHeadConfig> get_tripodhead_config()
    {
        if (!m_is_loaded)
            throw std::runtime_error("Config not loaded!");
        return m_tripodhead_config;
    }
    std::shared_ptr<ServoConfig> get_servo_config()
    {
        if (!m_is_loaded)
            throw std::runtime_error("Config not loaded!");
        return m_servo_config;
    }
    std::shared_ptr<TrackStrategyConfig> get_track_strategy_config()
    {
        if (!m_is_loaded)
            throw std::runtime_error("Config not loaded!");
        return m_track_strategy_config;
    }
    std::shared_ptr<DetectConfig> get_detect_config()
    {
        if (!m_is_loaded)
            throw std::runtime_error("Config not loaded!");
        return m_detect_config;
    }
    std::shared_ptr<SerialConfig> get_serial_config()
    {
        if (!m_is_loaded)
            throw std::runtime_error("Config not loaded!");
        return m_serial_config;
    }
    std::shared_ptr<NetworkConfig> get_network_config()
    {
        if (!m_is_loaded)
            throw std::runtime_error("Config not loaded!");
        return m_network_config;
    }
    bool is_loaded() const { return m_is_loaded; }
    void load_from_file(const std::string& filename = default_config_file);
    void generate_default_config(const std::string& filename = default_config_file);
};