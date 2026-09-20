#pragma once
#include <CommonTypes.h>

#if !defined(USE_FAKE_FOXGLOVE)
#include <atomic>
#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/schemas.hpp>
#include <foxglove/server.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>
#else
#include <iostream>
#include <string>

namespace cv
{
class Mat;
}
#endif

#if !defined(USE_FAKE_FOXGLOVE)
class FoxGloveServer
{
private:
    std::unique_ptr<foxglove::McapWriter> m_mcap_writer;
    std::unique_ptr<foxglove::WebSocketServer> m_ws_server;

    std::unique_ptr<foxglove::schemas::LogChannel> m_log_channel;
    std::unique_ptr<foxglove::schemas::RawImageChannel> m_camera_channel;
    std::unique_ptr<foxglove::schemas::ImageAnnotationsChannel> m_lazer_annotation_channel;
    std::unique_ptr<foxglove::schemas::ImageAnnotationsChannel> m_target_annotation_channel;

    std::unique_ptr<foxglove::RawChannel> m_gimbal_real_channel;
    std::unique_ptr<foxglove::RawChannel> m_gimbal_current_channel;
    std::unique_ptr<foxglove::RawChannel> m_lidar_target_channel;

    std::unique_ptr<foxglove::RawChannel> m_gimbal_target_channel;

    void log_angle_message(const CommonTypes::GimbalAngles& angles, std::unique_ptr<foxglove::RawChannel>& channel);
    void log_point_message(const CommonTypes::Point& point, std::unique_ptr<foxglove::schemas::Point3Channel>& channel);

    std::atomic<bool> m_image_saving_thread_running{false};
    std::mutex m_image_queue_mutex;
    std::queue<std::pair<cv::Mat, CommonTypes::Nanosecond>> m_image_queue;
    std::thread m_image_saving_thread;
    void log_camera_image(std::pair<cv::Mat, CommonTypes::Nanosecond> image_pair);
    void image_saving_thread_func();

    FoxGloveServer();

public:
    static FoxGloveServer& get_server()
    {
        static FoxGloveServer instance;
        return instance;
    }
    ~FoxGloveServer() = default;
    FoxGloveServer(const FoxGloveServer&) = delete;
    FoxGloveServer& operator=(const FoxGloveServer&) = delete;
    FoxGloveServer(FoxGloveServer&&) = delete;
    FoxGloveServer& operator=(FoxGloveServer&&) = delete;

    using LogLevel = foxglove::schemas::Log::LogLevel;
    void log_message(const std::string& msg, LogLevel level = LogLevel::INFO);
    void log_camera_image(const cv::Mat& image);
    void log_lazer_annotation(const cv::Point2f& lazer);
    void log_target_annotation(const cv::Point2f& target, bool is_hit);

    void log_gimbal_real_rotation(const CommonTypes::GimbalAngles& angles) { log_angle_message(angles, m_gimbal_real_channel); }
    void log_gimbal_current_rotation(const CommonTypes::GimbalAngles& angles) { log_angle_message(angles, m_gimbal_current_channel); }

    void log_network_pack(const CommonTypes::NetworkPack& pack);

    void log_gimbal_target_rotation(const CommonTypes::GimbalAngles& angles) { log_angle_message(angles, m_gimbal_target_channel); }
    void start();
    void shutdown();
};
#else
class FoxGloveServer
{
public:
    enum class LogLevel
    {
        DEBUG = 0,
        INFO = 1,
        WARNING = 2,
        ERROR = 3,
        FATAL = 4,
    };

    static FoxGloveServer& get_server()
    {
        static FoxGloveServer instance;
        return instance;
    }

    void log_message(const std::string& msg, LogLevel level = LogLevel::INFO);
    void log_camera_image(const cv::Mat& image);
    void log_lazer_annotation(const cv::Point2f& lazer);
    void log_target_annotation(const cv::Point2f& target, bool is_hit);
    void log_gimbal_current_rotation(const CommonTypes::GimbalAngles& angles);
    void log_gimbal_real_rotation(const CommonTypes::GimbalAngles& angles);
    void log_lidar_target_points(const CommonTypes::Point& point);
    void log_detect_point(const CommonTypes::Point& point);
    void log_track_point(const CommonTypes::Point& point);
    void log_gimbal_target_rotation(const CommonTypes::GimbalAngles& angles);
    void start();
    void shutdown();

private:
    FoxGloveServer() = default;
};
#endif