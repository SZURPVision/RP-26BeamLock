#include <FoxGloveServer.h>
#include <TimeUtils.h>
#include <filesystem>
#include <iostream>

FoxGloveServer::FoxGloveServer() { foxglove::setLogLevel(foxglove::LogLevel::Debug); }

void FoxGloveServer::log_message(const std::string& msg, LogLevel level)
{
    if (!m_log_channel)
        return;

    if (level == LogLevel::INFO)
        std::cout << "\33[32m[INFO ]\33[0m " << msg << std::endl;
    else if (level == LogLevel::WARNING)
        std::cerr << "\33[33m[WARN ]\33[0m " << msg << std::endl;
    else if (level == LogLevel::FATAL)
        std::cerr << "\33[31m[FATAL]\33[0m " << msg << std::endl;
    foxglove::schemas::Log log_msg;
    CommonTypes::Nanosecond now_ns = TimeUtils::now_nanoseconds();
    log_msg.level = level;
    log_msg.message = msg;
    log_msg.timestamp = foxglove::schemas::Timestamp{
        .sec = static_cast<uint32_t>(now_ns / 1'000'000'000),
        .nsec = static_cast<uint32_t>(now_ns % 1'000'000'000),
    };
    m_log_channel->log(log_msg);
}

void FoxGloveServer::log_camera_image(std::pair<cv::Mat, CommonTypes::Nanosecond> image_pair)
{
    if (!m_camera_channel)
        return;
    foxglove::schemas::RawImage img_msg;
    img_msg.width = image_pair.first.cols;
    img_msg.height = image_pair.first.rows;
    img_msg.step = static_cast<uint32_t>(image_pair.first.cols * image_pair.first.elemSize());
    img_msg.encoding = "bayer_rggb8";
    img_msg.frame_id = "camera_frame";
    img_msg.timestamp = foxglove::schemas::Timestamp{
        .sec = static_cast<uint32_t>(image_pair.second / 1'000'000'000),
        .nsec = static_cast<uint32_t>(image_pair.second % 1'000'000'000),
    };
    size_t data_size = image_pair.first.total() * image_pair.first.elemSize();
    std::vector<std::byte> data_vector(data_size);
    std::memcpy(data_vector.data(), image_pair.first.data, data_size);
    img_msg.data = std::move(data_vector);
    m_camera_channel->log(img_msg);
}

void FoxGloveServer::image_saving_thread_func()
{
    while (m_image_saving_thread_running)
    {
        std::pair<cv::Mat, CommonTypes::Nanosecond> image_pair;
        bool flag = false;
        {
            std::lock_guard<std::mutex> lock(m_image_queue_mutex);
            if (!m_image_queue.empty())
            {
                image_pair = m_image_queue.front();
                m_image_queue.pop();
                flag = true;
            }
        }
        if (!flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        log_camera_image(image_pair);
    }
}

void FoxGloveServer::log_camera_image(const cv::Mat& image)
{
    CommonTypes::Nanosecond now_ns = TimeUtils::now_nanoseconds();
    {
        std::lock_guard<std::mutex> lock(m_image_queue_mutex);
        if (m_image_queue.size() >= 100)
        {
            log_message("Image queue is full, dropping oldest image", LogLevel::WARNING);
            m_image_queue.pop();
        }
        m_image_queue.emplace(std::make_pair(image, now_ns));
    }
}

void FoxGloveServer::log_lazer_annotation(const cv::Point2f& lazer)
{
    if (!m_lazer_annotation_channel)
        return;
    foxglove::schemas::ImageAnnotations send_msg;
    foxglove::schemas::PointsAnnotation row_line, column_line;
    row_line.type = foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_STRIP;
    column_line.type = foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_STRIP;
    foxglove::schemas::Color line_color{.r = 0, .g = 1, .b = 0, .a = 1};
    constexpr double line_length = 20;
    constexpr double half_line_length = line_length / 2;
    row_line.outline_color = line_color;
    column_line.outline_color = line_color;
    row_line.thickness = 2;
    column_line.thickness = 2;
    row_line.points.push_back(foxglove::schemas::Point2{.x = lazer.x - half_line_length, .y = lazer.y});
    row_line.points.push_back(foxglove::schemas::Point2{.x = lazer.x + half_line_length, .y = lazer.y});
    column_line.points.push_back(foxglove::schemas::Point2{.x = lazer.x, .y = lazer.y - half_line_length});
    column_line.points.push_back(foxglove::schemas::Point2{.x = lazer.x, .y = lazer.y + half_line_length});

    send_msg.points.push_back(row_line);
    send_msg.points.push_back(column_line);
    m_lazer_annotation_channel->log(send_msg, TimeUtils::now_nanoseconds());
}

void FoxGloveServer::log_target_annotation(const cv::Point2f& target, bool is_hit)
{
    if (!m_target_annotation_channel)
        return;
    foxglove::schemas::ImageAnnotations send_msg;
    foxglove::schemas::CircleAnnotation annotation_msg;
    if (is_hit)
        annotation_msg.outline_color = foxglove::schemas::Color{.r = 1, .g = 0, .b = 0, .a = 1};
    else
        annotation_msg.outline_color = foxglove::schemas::Color{.r = 1, .g = 1, .b = 0, .a = 1};
    annotation_msg.fill_color = foxglove::schemas::Color{.r = 0, .g = 0, .b = 0, .a = 0};
    annotation_msg.position = foxglove::schemas::Point2{.x = target.x, .y = target.y};
    annotation_msg.thickness = 3;
    annotation_msg.diameter = 20;
    send_msg.circles.push_back(annotation_msg);
    m_target_annotation_channel->log(send_msg, TimeUtils::now_nanoseconds());
}

void FoxGloveServer::log_angle_message(const CommonTypes::GimbalAngles& angles, std::unique_ptr<foxglove::RawChannel>& channel)
{
    if (!channel)
        return;
    std::string json_msg = "{\"pitch\": " + std::to_string(angles.pitch) + ", \"yaw\": " + std::to_string(angles.yaw) + "}";
    channel->log(reinterpret_cast<const std::byte*>(json_msg.data()), json_msg.size(), TimeUtils::now_nanoseconds());
}

void FoxGloveServer::log_point_message(const CommonTypes::Point& point, std::unique_ptr<foxglove::schemas::Point3Channel>& channel)
{
    if (!channel)
        return;
    foxglove::schemas::Point3 point_msg;
    point_msg.x = point.x();
    point_msg.y = point.y();
    point_msg.z = point.z();
    channel->log(point_msg, TimeUtils::now_nanoseconds());
}

void FoxGloveServer::log_network_pack(const CommonTypes::NetworkPack& pack)
{
    if (!m_lidar_target_channel)
        return;
    std::string json_msg =
        "{\"frame_id\": " + std::to_string(pack.frame_id) + ", \"x\": " + std::to_string(pack.x) + ", \"y\": " + std::to_string(pack.y) + ", \"z\": " + std::to_string(pack.z) + ", \"allow_counter\": " + (pack.allow_counter ? "true" : "false") + "}";
    m_lidar_target_channel->log(reinterpret_cast<const std::byte*>(json_msg.data()), json_msg.size(), TimeUtils::now_nanoseconds());
}

std::string get_mcap_path()
{
    try
    {
        std::filesystem::create_directories("mcaps");
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to create mcaps directory: " << e.what() << std::endl;
    }

    // 获取当前时间
    time_t now = time(0);
    tm ltm = {};
    localtime_r(&now, &ltm);

    // 根据日期时间生成文件名 (格式: YYYY-MM-DD_HH-MM-SS.mcap)
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S.mcap", &ltm);
    return "mcaps/" + std::string(buffer);
}

template <typename T> std::unique_ptr<T> unwrap_or_log(foxglove::FoxgloveResult<T>&& result, const char* what)
{
    if (!result.has_value())
    {
        std::cerr << "Foxglove " << what << " create failed" << std::endl;
        return nullptr;
    }
    return std::make_unique<T>(std::move(result.value()));
}

void FoxGloveServer::start()
{
#ifdef SAVE_MCAP
    auto mcap_result = foxglove::McapWriter::create(foxglove::McapWriterOptions{
        .path = get_mcap_path(),
        .compression = foxglove::McapCompression::None,
    });
    m_mcap_writer = unwrap_or_log(std::move(mcap_result), "McapWriter");
    if (!m_mcap_writer)
        return;
#endif

    auto ws_result = foxglove::WebSocketServer::create(foxglove::WebSocketServerOptions{
        .host = "0.0.0.0",
        .port = 8765,
    });
    m_ws_server = unwrap_or_log(std::move(ws_result), "WebSocketServer");
    if (!m_ws_server)
        return;

    m_log_channel = unwrap_or_log(foxglove::schemas::LogChannel::create("/logs"), "LogChannel");
    m_camera_channel = unwrap_or_log(foxglove::schemas::RawImageChannel::create("/camera/image"), "RawImageChannel");
    m_lazer_annotation_channel = unwrap_or_log(foxglove::schemas::ImageAnnotationsChannel::create("/camera/lazer_annotation"), "ImageAnnotationsChannel(lazer)");
    m_target_annotation_channel = unwrap_or_log(foxglove::schemas::ImageAnnotationsChannel::create("/camera/target_annotation"), "ImageAnnotationsChannel(target)");
    m_gimbal_real_channel = unwrap_or_log(foxglove::RawChannel::create("/gimbal/real_rotation", "json"), "RawChannel(real_rotation)");
    m_gimbal_current_channel = unwrap_or_log(foxglove::RawChannel::create("/gimbal/current_rotation", "json"), "RawChannel(current_rotation)");
    m_lidar_target_channel = unwrap_or_log(foxglove::RawChannel::create("/points/lidar_target", "json"), "RawChannel(lidar_target)");
    m_gimbal_target_channel = unwrap_or_log(foxglove::RawChannel::create("/gimbal/target_rotation", "json"), "RawChannel(target_rotation)");

    m_image_saving_thread_running = true;
    m_image_saving_thread = std::thread(&FoxGloveServer::image_saving_thread_func, this);
}

void FoxGloveServer::shutdown()
{
    m_image_saving_thread_running = false;
    if (m_image_saving_thread.joinable())
        m_image_saving_thread.join();

    {
        std::lock_guard<std::mutex> lock(m_image_queue_mutex);
        while (!m_image_queue.empty())
        {
            log_camera_image(m_image_queue.front());
            m_image_queue.pop();
        }
    }

    if (m_ws_server)
    {
        auto error = m_ws_server->stop();
        if (error != foxglove::FoxgloveError::Ok)
        {
            std::cerr << "Foxglove WebSocketServer stop failed: " << static_cast<int>(error) << std::endl;
        }
        m_ws_server.reset();
    }

    m_log_channel.reset();
    m_camera_channel.reset();
    m_lazer_annotation_channel.reset();
    m_target_annotation_channel.reset();
    m_gimbal_real_channel.reset();
    m_gimbal_current_channel.reset();
    m_lidar_target_channel.reset();
    m_gimbal_target_channel.reset();
    m_mcap_writer.reset();
}