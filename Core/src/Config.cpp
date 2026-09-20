#include <Config.h>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>

void Config::load_from_file(const std::string& filename)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(filename);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Failed to load config file: " + filename + ", error: " + std::string(e.what()));
    }

    if (!root.IsMap())
        throw std::runtime_error("Invalid YAML format in config file: " + filename);

    // 解析 CameraConfig
    try
    {
        auto camera_obj = root["CameraConfig"];
        if (!camera_obj || !camera_obj.IsMap())
            throw std::runtime_error("CameraConfig is missing or invalid");

        auto intrinsic_array = camera_obj["intrinsic_matrix"];
        auto distortion_array = camera_obj["distortion_coeffs"];
        if (!intrinsic_array || !intrinsic_array.IsSequence() || intrinsic_array.size() != 9)
            throw std::runtime_error("CameraConfig.intrinsic_matrix must be a 9-element array");
        if (!distortion_array || !distortion_array.IsSequence() || distortion_array.size() != 5)
            throw std::runtime_error("CameraConfig.distortion_coeffs must be a 5-element array");

        cv::Mat intrinsic_matrix(3, 3, CV_64F);
        cv::Mat distortion_coeffs(5, 1, CV_64F);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                intrinsic_matrix.at<double>(i, j) = intrinsic_array[i * 3 + j].as<double>();
        for (int i = 0; i < 5; ++i)
            distortion_coeffs.at<double>(i, 0) = distortion_array[i].as<double>();
        cv::Size image_size(camera_obj["image_width"].as<int>(), camera_obj["image_height"].as<int>());
        double gain = camera_obj["gain"].as<double>();
        double exposure = camera_obj["exposure"].as<double>();
        m_camera_config = CameraConfig::create(intrinsic_matrix, distortion_coeffs, image_size, gain, exposure);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error parsing CameraConfig: " + std::string(e.what()));
    }

    // 解析 TripodHeadConfig
    try
    {
        auto tripod_obj = root["TripodHeadConfig"];
        if (!tripod_obj || !tripod_obj.IsMap())
            throw std::runtime_error("TripodHeadConfig is missing or invalid");

        double x_k = tripod_obj["x_k"].as<double>();
        double x_b = tripod_obj["x_b"].as<double>();
        double y_k = tripod_obj["y_k"].as<double>();
        double y_b = tripod_obj["y_b"].as<double>();
        bool invert_yaw_angle = tripod_obj["invert_yaw_angle"].as<bool>();
        bool invert_pitch_angle = tripod_obj["invert_pitch_angle"].as<bool>();
        bool invert_yaw_encoder = tripod_obj["invert_yaw_encoder"].as<bool>();
        bool invert_pitch_encoder = tripod_obj["invert_pitch_encoder"].as<bool>();
        int yaw_encoder_per_round = tripod_obj["yaw_encoder_per_round"].as<int>();
        int pitch_encoder_per_round = tripod_obj["pitch_encoder_per_round"].as<int>();
        double max_acceleration = tripod_obj["max_acceleration"].as<double>();
        double max_velocity = tripod_obj["max_velocity"].as<double>();
        double max_pitch_angle = tripod_obj["max_pitch_angle"].as<double>();
        double max_yaw_angle = tripod_obj["max_yaw_angle"].as<double>();
        m_tripodhead_config =
            TripodHeadConfig::create(x_k, x_b, y_k, y_b, invert_yaw_angle, invert_pitch_angle, invert_yaw_encoder, invert_pitch_encoder, yaw_encoder_per_round, pitch_encoder_per_round, max_acceleration, max_velocity, max_pitch_angle, max_yaw_angle);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error parsing TripodHeadConfig: " + std::string(e.what()));
    }

    // 解析 ServoConfig
    try
    {
        auto servo_obj = root["ServoConfig"];
        if (!servo_obj || !servo_obj.IsMap())
            throw std::runtime_error("ServoConfig is missing or invalid");

        double k_p = servo_obj["k_p"].as<double>();
        double k_i = servo_obj["k_i"].as<double>();
        double k_d = servo_obj["k_d"].as<double>();
        double k_ff = servo_obj["k_ff"].as<double>();
        double integral_limit = servo_obj["integral_limit"].as<double>();
        double integrate_deadzone = servo_obj["integrate_deadzone"].as<double>();
        double min_offset = servo_obj["min_offset"].as<double>();
        double scan_initial_radius = servo_obj["scan_initial_radius"].as<double>();
        double scan_radius_delta = servo_obj["scan_radius_delta"].as<double>();
        double scan_max_radius = servo_obj["scan_max_radius"].as<double>();
        double scan_speed = servo_obj["scan_speed"].as<double>();
        m_servo_config = ServoConfig::create(k_p, k_i, k_d, k_ff, integral_limit, integrate_deadzone, min_offset, scan_initial_radius, scan_radius_delta, scan_max_radius, scan_speed);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error parsing ServoConfig: " + std::string(e.what()));
    }

    // 解析 TrackStrategyConfig
    try
    {
        auto track_strategy_obj = root["TrackStrategyConfig"];
        if (!track_strategy_obj || !track_strategy_obj.IsMap())
            throw std::runtime_error("TrackStrategyConfig is missing or invalid");

        double reset_time_threshold = track_strategy_obj["reset_time_threshold"].as<double>();
        double scan_yaw_angle = track_strategy_obj["scan_yaw_angle"].as<double>();
        double scan_pitch_angle = track_strategy_obj["scan_pitch_angle"].as<double>();
        double scan_period = track_strategy_obj["scan_period"].as<double>();
        double line_scan_yaw_min = track_strategy_obj["line_scan_yaw_min"].as<double>();
        double line_scan_yaw_max = track_strategy_obj["line_scan_yaw_max"].as<double>();
        double line_scan_pitch_min = track_strategy_obj["line_scan_pitch_min"].as<double>();
        double line_scan_pitch_max = track_strategy_obj["line_scan_pitch_max"].as<double>();
        double line_scan_speed = track_strategy_obj["line_scan_speed"].as<double>();
        double line_scan_pitch_step = track_strategy_obj["line_scan_pitch_step"].as<double>();

        // 解析 GuideTransform: translation (3 元素数组)
        auto guide_translation = track_strategy_obj["guide_translation"];
        if (!guide_translation || !guide_translation.IsSequence() || guide_translation.size() != 3)
            throw std::runtime_error("TrackStrategyConfig.guide_translation must be a 3-element array");
        double guide_tx = guide_translation[0].as<double>();
        double guide_ty = guide_translation[1].as<double>();
        double guide_tz = guide_translation[2].as<double>();

        // 解析 GuideTransform: rotation euler ZYX angles (yaw, pitch, roll in degrees, 3 元素数组)
        auto guide_rotation = track_strategy_obj["guide_rotation_euler_zyx_deg"];
        if (!guide_rotation || !guide_rotation.IsSequence() || guide_rotation.size() != 3)
            throw std::runtime_error("TrackStrategyConfig.guide_rotation_euler_zyx_deg must be a 3-element array");
        double guide_yaw_deg = guide_rotation[0].as<double>();
        double guide_pitch_deg = guide_rotation[1].as<double>();
        double guide_roll_deg = guide_rotation[2].as<double>();

        m_track_strategy_config = TrackStrategyConfig::create(reset_time_threshold,
                                                              scan_yaw_angle,
                                                              scan_pitch_angle,
                                                              scan_period,
                                                              line_scan_yaw_min,
                                                              line_scan_yaw_max,
                                                              line_scan_pitch_min,
                                                              line_scan_pitch_max,
                                                              line_scan_speed,
                                                              line_scan_pitch_step,
                                                              guide_tx,
                                                              guide_ty,
                                                              guide_tz,
                                                              guide_yaw_deg,
                                                              guide_pitch_deg,
                                                              guide_roll_deg);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error parsing TrackStrategyConfig: " + std::string(e.what()));
    }

    // 解析 DetectConfig
    try
    {
        auto detect_obj = root["DetectConfig"];
        if (!detect_obj || !detect_obj.IsMap())
            throw std::runtime_error("DetectConfig is missing or invalid");

        double light_blob_x_threshold = detect_obj["light_blob_x_threshold"].as<double>();
        double light_blob_y_threshold = detect_obj["light_blob_y_threshold"].as<double>();
        double min_hit_difference = detect_obj["min_hit_difference"].as<double>();
        double adaptive_threshold_c = detect_obj["adaptive_threshold_c"].as<double>();
        m_detect_config = DetectConfig::create(light_blob_x_threshold, light_blob_y_threshold, min_hit_difference, adaptive_threshold_c);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error parsing DetectConfig: " + std::string(e.what()));
    }

    // 解析 SerialConfig
    try
    {
        auto serial_obj = root["SerialConfig"];
        if (!serial_obj || !serial_obj.IsMap())
            throw std::runtime_error("SerialConfig is missing or invalid");

        int buffer_size = serial_obj["buffer_size"].as<int>();
        int prepare_size = serial_obj["prepare_size"].as<int>();
        unsigned int baud_rate = serial_obj["baud_rate"].as<unsigned int>();
        int reset_pack_count = serial_obj["reset_pack_count"].as<int>();
        std::string port_name = serial_obj["port_name"].as<std::string>();
        double max_send_frequency = serial_obj["max_send_frequency"].as<double>();
        m_serial_config = SerialConfig::create(buffer_size, prepare_size, baud_rate, reset_pack_count, port_name, max_send_frequency);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error parsing SerialConfig: " + std::string(e.what()));
    }

    // 解析 NetworkConfig
    try
    {
        auto network_obj = root["NetworkConfig"];
        if (!network_obj || !network_obj.IsMap())
            throw std::runtime_error("NetworkConfig is missing or invalid");

        std::string listen_ip = network_obj["listen_ip"].as<std::string>();
        uint16_t listen_port = network_obj["listen_port"].as<uint16_t>();
        std::string send_target_ip = network_obj["send_target_ip"].as<std::string>();
        uint16_t send_target_port = network_obj["send_target_port"].as<uint16_t>();
        m_network_config = NetworkConfig::create(listen_ip, listen_port, send_target_ip, send_target_port);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error parsing NetworkConfig: " + std::string(e.what()));
    }

    m_is_loaded = true;
}

void Config::generate_default_config(const std::string& filename)
{
    // 注意：本函数生成的是“未标定”的默认配置。
    // 相机内参取两位有效数字、激光光路与雷达引导外参为占位值，
    // 实车使用前必须按 README/CONFIG.md 的标定流程重新标定并写入运行目录的 config.yaml。
    YAML::Node root;

    // CameraConfig
    YAML::Node camera_obj;
    camera_obj["intrinsic_matrix"] = std::vector<double>{16000.0, 0.0, 790.0, 0.0, 16000.0, 580.0, 0.0, 0.0, 1.0};
    camera_obj["distortion_coeffs"] = std::vector<double>{0.097, -24.0, 0.0084, -0.0010, -0.18};
    camera_obj["image_width"] = 1440;
    camera_obj["image_height"] = 1080;
    camera_obj["gain"] = 16.0;
    camera_obj["exposure"] = 400.0;
    root["CameraConfig"] = camera_obj;

    // TripodHeadConfig
    YAML::Node tripod_obj;
    // 激光光路在相机坐标系下的直线方程 x = x_k * z + x_b, y = y_k * z + y_b
    // 默认为 0，表示“假设激光与光轴重合”，实际必须用 LazerCalibration 标定后填入
    tripod_obj["x_k"] = 0.0;
    tripod_obj["x_b"] = 0.0;
    tripod_obj["y_k"] = 0.0;
    tripod_obj["y_b"] = 0.0;
    tripod_obj["invert_yaw_angle"] = false;
    tripod_obj["invert_pitch_angle"] = true;
    tripod_obj["invert_yaw_encoder"] = true;
    tripod_obj["invert_pitch_encoder"] = false;
    tripod_obj["yaw_encoder_per_round"] = 8192;
    tripod_obj["pitch_encoder_per_round"] = 8192;
    tripod_obj["max_acceleration"] = 200.0; // deg/s^2, 云台的最大加速度
    tripod_obj["max_velocity"] = 400.0;     // deg/s, 云台的最大速度
    tripod_obj["max_pitch_angle"] = 20.0;   // deg, pitch 相对初始角的最大允许角度
    tripod_obj["max_yaw_angle"] = 80.0;     // deg, yaw 相对初始角的最大允许角度
    root["TripodHeadConfig"] = tripod_obj;

    // ServoConfig
    YAML::Node servo_obj;
    servo_obj["k_p"] = 500;
    servo_obj["k_i"] = 0.0;
    servo_obj["k_d"] = 0.0;
    servo_obj["k_ff"] = 0.0;
    servo_obj["integral_limit"] = 0.4;
    servo_obj["integrate_deadzone"] = 0.005;
    servo_obj["min_offset"] = 0.0005;
    servo_obj["scan_initial_radius"] = 4.0;
    servo_obj["scan_radius_delta"] = 4.0;
    servo_obj["scan_max_radius"] = 16.0;
    servo_obj["scan_speed"] = 20.0;
    root["ServoConfig"] = servo_obj;

    // TrackStrategyConfig
    YAML::Node track_strategy_obj;
    track_strategy_obj["reset_time_threshold"] = 5.0;  // s,
    track_strategy_obj["scan_yaw_angle"] = 3.0;        // deg
    track_strategy_obj["scan_pitch_angle"] = 3.0;      // deg
    track_strategy_obj["scan_period"] = 4.0;           // s
    track_strategy_obj["line_scan_yaw_min"] = 5.0;     // deg
    track_strategy_obj["line_scan_yaw_max"] = 25.0;    // deg
    track_strategy_obj["line_scan_pitch_min"] = -15.0; // deg
    track_strategy_obj["line_scan_pitch_max"] = 0.0;   // deg
    track_strategy_obj["line_scan_speed"] = 20.0;      // deg/s
    track_strategy_obj["line_scan_pitch_step"] = 1.5;  // deg
    // 激光引导变换矩阵参数: translation (x, y, z) 和 ZYX 欧拉角 (yaw, pitch, roll in degrees)
    // 默认全 0，表示“假设雷达坐标系与云台坐标系重合”，实际必须用 LidarGuideCalibration 标定后填入
    track_strategy_obj["guide_translation"] = std::vector<double>{0.0, 0.0, 0.0};
    track_strategy_obj["guide_rotation_euler_zyx_deg"] = std::vector<double>{0.0, 0.0, 0.0};
    root["TrackStrategyConfig"] = track_strategy_obj;

    // DetectConfig
    YAML::Node detect_obj;
    detect_obj["light_blob_x_threshold"] = 20.0;
    detect_obj["light_blob_y_threshold"] = 5.0;
    detect_obj["min_hit_difference"] = 10.0;
    detect_obj["adaptive_threshold_c"] = -10.0;
    root["DetectConfig"] = detect_obj;

    // SerialConfig
    YAML::Node serial_obj;
    serial_obj["buffer_size"] = 1024;
    serial_obj["prepare_size"] = 512;
    serial_obj["baud_rate"] = 115200;
    serial_obj["reset_pack_count"] = 8192;
    serial_obj["port_name"] = "/dev/ttyACM0";
    serial_obj["max_send_frequency"] = 1000.0;
    root["SerialConfig"] = serial_obj;

    // NetworkConfig
    YAML::Node network_obj;
    network_obj["listen_ip"] = "0.0.0.0"; // 监听所有网卡，实际使用时按需改为雷达主程序所在网段的地址
    network_obj["listen_port"] = 9000;
    network_obj["send_target_ip"] = "127.0.0.1"; // 心跳发送目标（雷达主程序）IP，按实际部署修改
    network_obj["send_target_port"] = 9001;      // 发送目标Port
    root["NetworkConfig"] = network_obj;

    std::ofstream file(filename);
    if (!file.is_open())
        throw std::runtime_error("Failed to open config file for writing: " + filename);
    file << root;
    file.close();
}