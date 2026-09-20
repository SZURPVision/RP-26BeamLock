#include <Config.h>
#include <MVSCamera.h>
#include <TargetDetecter.h>
#include <atomic>
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>

std::atomic_bool g_running{true};
void handle_sigint(int) { g_running.store(false); }

int main()
{
    std::signal(SIGINT, handle_sigint);
    try
    {
        Config::get_config().load_from_file();
    }
    catch (const std::exception&)
    {
        Config::get_config().generate_default_config();
        Config::get_config().load_from_file();
    }

    CameraManager camera_manager;
    auto cam_list = camera_manager.list_cameras();
    if (cam_list.is_empty())
    {
        std::cerr << "No cameras found!" << std::endl;
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
        std::cerr << "Camera init failed: " << e.what() << std::endl;
        return -1;
    }

    auto intrinsic_matrix = Config::get_config().get_camera_config()->m_intrinsic_matrix;
    auto distortion_coeffs = Config::get_config().get_camera_config()->m_distortion_coeffs;

    cv::namedWindow("TargetDetecterTest", cv::WINDOW_NORMAL);

    while (g_running.load())
    {
        cv::Mat frame;
        try
        {
            camera->get_frame(frame, 1000);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to get frame: " << e.what() << std::endl;
            continue;
        }

        auto result = TargetDetecter::detect_best_target(frame);
        auto& feature = result.feature;
        const bool is_hit = result.is_hit;

        cv::Mat overlay;
        cv::cvtColor(frame, overlay, cv::COLOR_BayerBG2BGR);

        if (!feature.empty())
        {
            double avg_pixel_x = 0.0, avg_pixel_y = 0.0;
            for (const auto& pt : feature)
            {
                avg_pixel_x += pt.x;
                avg_pixel_y += pt.y;
            }
            avg_pixel_x /= feature.size();
            avg_pixel_y /= feature.size();

            cv::circle(overlay, cv::Point(static_cast<int>(avg_pixel_x), static_cast<int>(avg_pixel_y)), 10, is_hit ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("TargetDetecterTest", overlay);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
            break;
    }

    return 0;
}
