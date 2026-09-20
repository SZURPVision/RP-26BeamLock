#include <Config.h>
#include <GimbalController.h>
#include <TimeUtils.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>

namespace
{
constexpr double k_rad_to_deg = 180.0 / 3.14159265358979323846;

double degrees(double rad) { return rad * k_rad_to_deg; }

void to_yaw_pitch(const cv::Vec3d& direction, double& pitch_deg, double& yaw_deg)
{
    cv::Vec3d dir = direction;
    double norm = cv::norm(dir);
    if (norm <= 1e-12)
    {
        pitch_deg = 0.0;
        yaw_deg = 0.0;
        return;
    }
    dir /= norm;

    const double yaw_clockwise = std::atan2(-dir[1], dir[0]);
    const double cos_yaw = std::cos(-yaw_clockwise);
    const double sin_yaw = std::sin(-yaw_clockwise);
    cv::Vec3d base(cos_yaw, sin_yaw, 0.0);
    cv::Vec3d rotated_y(-sin_yaw, cos_yaw, 0.0);

    const double right_hand_angle = std::atan2(rotated_y.dot(base.cross(dir)), base.dot(dir));
    const double pitch_clockwise = -right_hand_angle;

    yaw_deg = degrees(yaw_clockwise);
    pitch_deg = degrees(pitch_clockwise);
}

bool read_guide_transform(const std::string& path, cv::Mat& tvec, cv::Mat& rotation)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "File not found: " << path << std::endl;
        return false;
    }

    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "Failed to open: " << path << std::endl;
        return false;
    }

    tvec = cv::Mat::zeros(3, 1, CV_64F);
    rotation = cv::Mat::eye(3, 3, CV_64F);

    std::string line;
    // 1. Read Translation
    if (std::getline(file, line))
    {
        std::istringstream iss(line);
        if (!(iss >> tvec.at<double>(0) >> tvec.at<double>(1) >> tvec.at<double>(2)))
        {
            std::cerr << "Failed to read translation from first line." << std::endl;
            return false;
        }
    }

    // 2. Read Empty line (skip)
    std::getline(file, line);

    // 3. Read Rotation Matrix
    for (int r = 0; r < 3; ++r)
    {
        if (std::getline(file, line))
        {
            std::istringstream iss(line);
            if (!(iss >> rotation.at<double>(r, 0) >> rotation.at<double>(r, 1) >> rotation.at<double>(r, 2)))
            {
                std::cerr << "Failed to read rotation matrix at row " << r << std::endl;
                return false;
            }
        }
        else
        {
            std::cerr << "Unexpected end of file while reading rotation matrix." << std::endl;
            return false;
        }
    }
    file.close();

    std::cout << "tvec: " << tvec.t() << std::endl;
    std::cout << "rotation:\n" << rotation << std::endl;

    return true;
}

// NOLINTBEGIN(readability-identifier-naming)
struct SharedState
{
    std::mutex mutex;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    bool point_valid = false;   // whether a valid point has been received at least once
    bool has_new_point = false; // flag: consumed by main thread to trigger recalculation
    std::atomic<bool> running{true};
};
// NOLINTEND(readability-identifier-naming)

void input_thread_func(SharedState& state)
{
    std::string line;
    while (state.running.load() && std::getline(std::cin, line))
    {
        if (line == "q" || line == "quit" || line == "exit")
        {
            state.running.store(false);
            break;
        }

        std::istringstream iss(line);
        double x = 0.0, y = 0.0, z = 0.0;
        if (!(iss >> x >> y >> z))
        {
            std::cout << "Invalid input. Please enter: x y z" << std::endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(state.mutex);
            state.x = x;
            state.y = y;
            state.z = z;
            state.point_valid = true;
            state.has_new_point = true;
        }
    }
    state.running.store(false);
}

} // namespace

int main(int argc, char** argv)
{
    std::string input_path = "GuideTransform.txt";
    bool enable_send = false;

    if (argc == 2)
    {
        std::string arg = argv[1];
        if (arg == "--send")
            enable_send = true;
        else
            input_path = arg;
    }
    else if (argc == 3)
    {
        input_path = argv[1];
        std::string arg = argv[2];
        if (arg == "--send")
            enable_send = true;
        else
        {
            std::cout << "Usage: " << argv[0] << " [GuideTransform.txt] [--send]" << std::endl;
            return 1;
        }
    }
    else if (argc > 3)
    {
        std::cout << "Usage: " << argv[0] << " [GuideTransform.txt] [--send]" << std::endl;
        return 1;
    }

    // Initial transform read
    cv::Mat tvec, rotation;
    if (!read_guide_transform(input_path, tvec, rotation))
        return 1;

    std::filesystem::file_time_type last_write_time;
    bool have_file_time = false;
    try
    {
        last_write_time = std::filesystem::last_write_time(input_path);
        have_file_time = true;
    }
    catch (const std::filesystem::filesystem_error&)
    {
    }

    // Initialize controller if --send
    std::unique_ptr<GimbalController> controller;
    if (enable_send)
    {
        try
        {
            Config::get_config().load_from_file();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to load config file: " << e.what() << std::endl;
            std::cerr << "Generating default config file." << std::endl;
            Config::get_config().generate_default_config();
            Config::get_config().load_from_file();
        }
        controller = std::make_unique<GimbalController>();
    }

    // Start input thread
    SharedState state;
    std::cout << "Input points as: x y z (or 'q' to quit)" << std::endl;
    std::thread input_thread(input_thread_func, std::ref(state));

    while (state.running.load())
    {
        bool transform_changed = false;
        bool need_recalc = false;
        double cur_x = 0.0, cur_y = 0.0, cur_z = 0.0;
        bool has_point = false;

        // 1. Check transform file timestamp and re-read if changed
        if (have_file_time && std::filesystem::exists(input_path))
        {
            try
            {
                auto current_time = std::filesystem::last_write_time(input_path);
                if (current_time != last_write_time)
                {
                    last_write_time = current_time;
                    if (read_guide_transform(input_path, tvec, rotation))
                    {
                        transform_changed = true;
                        need_recalc = true;
                    }
                }
            }
            catch (const std::filesystem::filesystem_error&)
            {
            }
        }

        // 2. Consume shared state from input thread under lock
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            if (state.has_new_point)
            {
                state.has_new_point = false;
                need_recalc = true;
            }
            if (state.point_valid)
            {
                cur_x = state.x;
                cur_y = state.y;
                cur_z = state.z;
                has_point = true;
            }
        }

        // 3. Recalculate if triggered by new point or updated transform
        if (need_recalc && has_point)
        {
            cv::Mat point = (cv::Mat_<double>(3, 1) << cur_x, cur_y, cur_z);
            cv::Mat gimbal_dir_mat = rotation * point + tvec;
            cv::Vec3d gimbal_dir(gimbal_dir_mat.at<double>(0), gimbal_dir_mat.at<double>(1), gimbal_dir_mat.at<double>(2));

            double norm = cv::norm(gimbal_dir);
            gimbal_dir /= norm;
            double pitch_deg = 0.0, yaw_deg = 0.0;
            to_yaw_pitch(gimbal_dir, pitch_deg, yaw_deg);

            std::cout << "pitch=" << pitch_deg << " yaw=" << yaw_deg;
            if (transform_changed)
                std::cout << " (transform updated)";
            std::cout << std::endl;

            if (controller)
            {
                CommonTypes::GimbalAngles target_angles{.pitch = pitch_deg, .yaw = yaw_deg};
                controller->set_target_encoder_angles(target_angles, TimeUtils::now_seconds());
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    input_thread.join();
    return 0;
}
