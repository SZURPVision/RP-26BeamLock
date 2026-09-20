#include <FoxGloveServer.h>

namespace
{
const char* to_level(FoxGloveServer::LogLevel level)
{
    switch (level)
    {
    case FoxGloveServer::LogLevel::DEBUG:
        return "DEBUG";
    case FoxGloveServer::LogLevel::INFO:
        return "INFO";
    case FoxGloveServer::LogLevel::WARNING:
        return "WARNING";
    case FoxGloveServer::LogLevel::ERROR:
        return "ERROR";
    case FoxGloveServer::LogLevel::FATAL:
        return "FATAL";
    default:
        return "UNKNOWN";
    }
}
} // namespace

void FoxGloveServer::log_message(const std::string& msg, LogLevel level) { std::cout << "[FoxGloveFake][" << to_level(level) << "] " << msg << std::endl; }

void FoxGloveServer::log_camera_image(const cv::Mat&) { std::cout << "[FoxGloveFake][IMAGE] camera image received" << std::endl; }

void FoxGloveServer::log_gimbal_real_rotation(const CommonTypes::GimbalAngles& angles) { std::cout << "[FoxGloveFake][GIMBAL_REAL] pitch=" << angles.pitch << " yaw=" << angles.yaw << std::endl; }

void FoxGloveServer::log_gimbal_current_rotation(const CommonTypes::GimbalAngles& angles) { std::cout << "[FoxGloveFake][GIMBAL_CURRENT] pitch=" << angles.pitch << " yaw=" << angles.yaw << std::endl; }

void FoxGloveServer::log_lidar_target_points(const CommonTypes::Point& point) { std::cout << "[FoxGloveFake][LIDAR_TARGET] x=" << point.x() << " y=" << point.y() << " z=" << point.z() << std::endl; }

void FoxGloveServer::log_detect_point(const CommonTypes::Point& point) { std::cout << "[FoxGloveFake][DETECT_POINT] x=" << point.x() << " y=" << point.y() << " z=" << point.z() << std::endl; }

void FoxGloveServer::log_track_point(const CommonTypes::Point& point) { std::cout << "[FoxGloveFake][TRACK_POINT] x=" << point.x() << " y=" << point.y() << " z=" << point.z() << std::endl; }

void FoxGloveServer::log_gimbal_target_rotation(const CommonTypes::GimbalAngles& angles) { std::cout << "[FoxGloveFake][GIMBAL_TARGET] pitch=" << angles.pitch << " yaw=" << angles.yaw << std::endl; }

void FoxGloveServer::start() { std::cout << "[FoxGloveFake] start" << std::endl; }

void FoxGloveServer::shutdown() { std::cout << "[FoxGloveFake] shutdown" << std::endl; }
