#include <Config.h>
#include <GimbalController.h>
#include <TimeUtils.h>
#include <filesystem>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

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

cv::Vec3d from_image_direction(const cv::Vec3d& image_dir)
{
    cv::Vec3d dir(-image_dir[1], -image_dir[2], image_dir[0]);
    double norm = cv::norm(dir);
    if (norm <= 1e-12)
        return cv::Vec3d(1.0, 0.0, 0.0);
    return dir / norm;
}
} // namespace

int main(int argc, char** argv)
{
    std::string input_path = "GuideTransform.yaml";
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
            std::cout << "Usage: " << argv[0] << " [GuideTransform.yaml] [--send]" << std::endl;
            return 1;
        }
    }
    else if (argc > 3)
    {
        std::cout << "Usage: " << argv[0] << " [GuideTransform.yaml] [--send]" << std::endl;
        return 1;
    }

    if (!std::filesystem::exists(input_path))
    {
        std::cerr << "File not found: " << input_path << std::endl;
        return 1;
    }

    cv::FileStorage fs(input_path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open: " << input_path << std::endl;
        return 1;
    }

    cv::Mat rvec = fs["rvec"].mat();
    cv::Mat tvec = fs["tvec"].mat();
    cv::Mat rotation = fs["rotation"].mat();
    cv::Mat transform = fs["transform"].mat();
    fs.release();

    if (rotation.empty() && transform.empty())
    {
        std::cerr << "No transform data found in: " << input_path << std::endl;
        return 1;
    }

    if (!transform.empty())
    {
        rotation = transform(cv::Rect(0, 0, 3, 3)).clone();
        tvec = transform(cv::Rect(3, 0, 1, 3)).clone();
    }
    if (rotation.empty() || tvec.empty())
    {
        std::cerr << "Missing rotation or translation in: " << input_path << std::endl;
        return 1;
    }

    if (!rvec.empty())
        std::cout << "rvec: " << rvec.t() << std::endl;
    if (!tvec.empty())
        std::cout << "tvec: " << tvec.t() << std::endl;
    if (!rotation.empty())
        std::cout << "rotation:\n" << rotation << std::endl;
    if (!transform.empty())
        std::cout << "transform:\n" << transform << std::endl;

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

    std::cout << "Input points as: x y z (or 'q' to quit)" << std::endl;
    std::string line;
    while (std::getline(std::cin, line))
    {
        if (line == "q" || line == "quit" || line == "exit")
            break;

        std::istringstream iss(line);
        double x = 0.0, y = 0.0, z = 0.0;
        if (!(iss >> x >> y >> z))
        {
            std::cout << "Invalid input. Please enter: x y z" << std::endl;
            continue;
        }

        cv::Mat point = (cv::Mat_<double>(3, 1) << x, y, z);
        cv::Mat point_camera = rotation * point + tvec;
        cv::Vec3d dir_image(point_camera.at<double>(0), point_camera.at<double>(1), point_camera.at<double>(2));

        double norm = cv::norm(dir_image);
        if (norm <= 1e-12)
        {
            std::cout << "Degenerate direction, skip." << std::endl;
            continue;
        }
        dir_image /= norm;

        cv::Vec3d gimbal_dir = from_image_direction(dir_image);
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
        to_yaw_pitch(gimbal_dir, pitch_deg, yaw_deg);

        std::cout << "pitch=" << pitch_deg << " yaw=" << yaw_deg << std::endl;

        if (controller)
        {
            CommonTypes::GimbalAngles target_angles{.pitch = pitch_deg, .yaw = yaw_deg};
            controller->set_target_encoder_angles(target_angles, TimeUtils::now_seconds());
        }
    }

    return 0;
}
