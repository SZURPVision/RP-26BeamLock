#include <Config.h>
#include <MVSCamera.h>
#include <atomic>
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>

std::atomic_bool g_running{true};
void handle_sigint(int) { g_running.store(false); }

struct CameraControlState
{
    std::shared_ptr<Camera> camera;
    int gain_x10 = 0;
    int exposure_x10 = 0;
};

void on_gain_change(int value, void* userdata)
{
    auto* state = static_cast<CameraControlState*>(userdata);
    if (!state || !state->camera)
        return;
    state->gain_x10 = value;
    state->camera->set_gain(static_cast<float>(value) / 10.0f);
}

void on_exposure_change(int value, void* userdata)
{
    auto* state = static_cast<CameraControlState*>(userdata);
    if (!state || !state->camera)
        return;
    state->exposure_x10 = value;
    state->camera->set_exposure_time(static_cast<float>(value * 10));
}

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

    CameraControlState state;
    state.camera = camera;

    const int gain_max = 100;
    const int exposure_max = 3000;

    state.gain_x10 = static_cast<int>(Config::get_config().get_camera_config()->m_gain * 10.0f);
    state.exposure_x10 = static_cast<int>(Config::get_config().get_camera_config()->m_exposure / 10.0f);

    state.gain_x10 = std::max(0, std::min(gain_max, state.gain_x10));
    state.exposure_x10 = std::max(0, std::min(exposure_max, state.exposure_x10));

    cv::namedWindow("CameraTune", cv::WINDOW_NORMAL);
    cv::createTrackbar("Gain x0.1", "CameraTune", &state.gain_x10, gain_max, on_gain_change, &state);
    cv::createTrackbar("Exposure x10", "CameraTune", &state.exposure_x10, exposure_max, on_exposure_change, &state);

    on_gain_change(state.gain_x10, &state);
    on_exposure_change(state.exposure_x10, &state);

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

        cv::Mat overlay = frame.clone();
        const int cx = overlay.cols / 2;
        const int cy = overlay.rows / 2;
        cv::line(overlay, cv::Point(0, cy), cv::Point(overlay.cols - 1, cy), cv::Scalar(255), 1);
        cv::line(overlay, cv::Point(cx, 0), cv::Point(cx, overlay.rows - 1), cv::Scalar(255), 1);

        cv::imshow("CameraTune", overlay);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
            break;
    }

    return 0;
}
