#include <Config.h>
#include <MVSCamera.h>
#include <Serial.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>

std::mutex mutex;
std::atomic_bool g_running{true};

struct State
{
    double target_pitch = 0.0;
    double target_yaw = 0.0;
    SerialHandler::ControlMode control_mode = SerialHandler::ControlMode::Encoder;
    bool control = false;
} current_state;

struct CameraControlState
{
    std::shared_ptr<Camera> camera;
    int gain_x10 = 0;
    int exposure_x10 = 0;
};

std::thread send_thread;
std::thread display_thread;
void send_loop(SerialHandler& serial)
{
    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            serial.send_target_angles(CommonTypes::GimbalAngles{.pitch = current_state.target_pitch, .yaw = current_state.target_yaw}, current_state.control_mode, current_state.control);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void display_loop(const std::shared_ptr<Camera>& camera)
{
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
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        cv::imshow("CameraTune", frame);
        cv::waitKey(1);
    }
}

void wait_for_stabilization(SerialHandler& serial, int stable_count_threshold = 5, int angle_threshold = 1)
{
    int stable_count = 0;
    int last_pitch_encoder = serial.get_pitch_encoder_angle();
    int last_yaw_encoder = serial.get_yaw_encoder_angle();
    while (stable_count < stable_count_threshold)
    {
        int now_pitch_encoder = serial.get_pitch_encoder_angle();
        int now_yaw_encoder = serial.get_yaw_encoder_angle();
        if (std::abs(now_pitch_encoder - last_pitch_encoder) < angle_threshold && std::abs(now_yaw_encoder - last_yaw_encoder) < angle_threshold)
        {
            stable_count++;
        }
        else
        {
            stable_count = 0;
            last_pitch_encoder = now_pitch_encoder;
            last_yaw_encoder = now_yaw_encoder;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

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

int main(int argc, char** argv)
{
    try
    {
        Config::get_config().load_from_file();
    }
    catch (const std::exception& e)
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

    const int gain_max = 170;
    const int exposure_max = 20000;

    state.gain_x10 = static_cast<int>(Config::get_config().get_camera_config()->m_gain * 10.0f);
    state.exposure_x10 = static_cast<int>(Config::get_config().get_camera_config()->m_exposure / 10.0f);

    state.gain_x10 = std::max(0, std::min(gain_max, state.gain_x10));
    state.exposure_x10 = std::max(0, std::min(exposure_max, state.exposure_x10));

    cv::namedWindow("CameraTune", cv::WINDOW_NORMAL);
    cv::createTrackbar("Gain x0.1", "CameraTune", &state.gain_x10, gain_max, on_gain_change, &state);
    cv::createTrackbar("Exposure x10", "CameraTune", &state.exposure_x10, exposure_max, on_exposure_change, &state);

    on_gain_change(state.gain_x10, &state);
    on_exposure_change(state.exposure_x10, &state);

    display_thread = std::thread(display_loop, state.camera);

    SerialHandler serial(Config::get_config().get_serial_config()->m_port_name, Config::get_config().get_serial_config()->m_baud_rate);
    std::cout << "Serial test started on " << Config::get_config().get_serial_config()->m_port_name << std::endl;
    send_thread = std::thread(send_loop, std::ref(serial));
    send_thread.detach();

    double yaw1 = std::numeric_limits<double>::quiet_NaN();
    double yaw2 = std::numeric_limits<double>::quiet_NaN();
    double pitch1 = std::numeric_limits<double>::quiet_NaN();
    double pitch2 = std::numeric_limits<double>::quiet_NaN();
    double step = 0.5;

    std::string command;
    while (std::cin >> command)
    {
        if (command == "q")
            break;
        if (command == "send")
        {
            double pitch = 0.0;
            double yaw = 0.0;
            if (!(std::cin >> pitch >> yaw))
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Invalid input format." << std::endl;
                continue;
            }

            current_state.target_pitch = pitch;
            current_state.target_yaw = yaw;
            current_state.control_mode = SerialHandler::ControlMode::Encoder;
            current_state.control = true;
            std::cout << "\33[32mSet target pitch=" << pitch << " yaw=" << yaw << "\33[0m" << std::endl;
            continue;
        }
        if (command == "set")
        {
            std::string arg;
            if (!(std::cin >> arg))
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Invalid input format." << std::endl;
                continue;
            }
            if (arg == "yaw1")
            {
                yaw1 = current_state.target_yaw;
                std::cout << "\33[32mSet yaw1=" << yaw1 << "\33[0m" << std::endl;
            }
            else if (arg == "yaw2")
            {
                yaw2 = current_state.target_yaw;
                std::cout << "\33[32mSet yaw2=" << yaw2 << "\33[0m" << std::endl;
            }
            else if (arg == "pitch1")
            {
                pitch1 = current_state.target_pitch;
                std::cout << "\33[32mSet pitch1=" << pitch1 << "\33[0m" << std::endl;
            }
            else if (arg == "pitch2")
            {
                pitch2 = current_state.target_pitch;
                std::cout << "\33[32mSet pitch2=" << pitch2 << "\33[0m" << std::endl;
            }
            else if (arg == "step")
            {
                std::cin >> step;
                std::cout << "\33[32mSet step=" << step << "\33[0m" << std::endl;
            }
            continue;
        }
        std::cout << "Unknown command." << std::endl;
    }

    g_running.store(false);
    if (display_thread.joinable())
        display_thread.join();

    cv::destroyAllWindows();

    if (std::isnan(yaw1) || std::isnan(yaw2) || std::isnan(pitch1) || std::isnan(pitch2))
    {
        std::cout << "Please set yaw1, yaw2, pitch1, pitch2 first." << std::endl;
        return -1;
    }

    double max_yaw = std::max(yaw1, yaw2);
    double min_yaw = std::min(yaw1, yaw2);
    double max_pitch = std::max(pitch1, pitch2);
    double min_pitch = std::min(pitch1, pitch2);
    camera->set_gain(state.gain_x10 / 10.0f);
    camera->set_exposure_time(state.exposure_x10 * 10.0f);

    int frame_count = 0;
    std::filesystem::create_directory("CapturedFrames");
    cv::namedWindow("CapturedFrame", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    for (double yaw = min_yaw; yaw <= max_yaw; yaw += step)
    {
        for (double pitch = min_pitch; pitch <= max_pitch; pitch += step)
        {
            {
                std::lock_guard<std::mutex> lock(mutex);
                current_state.target_pitch = pitch;
                current_state.target_yaw = yaw;
                current_state.control_mode = SerialHandler::ControlMode::Encoder;
                current_state.control = true;
            }
            std::cout << "\33[32mSet target pitch=" << pitch << " yaw=" << yaw << std::endl;
            wait_for_stabilization(serial);
            cv::Mat frame;
            try
            {
                camera->get_frame(frame, 2000);
                camera->get_frame(frame, 2000);
                camera->get_frame(frame, 2000);
            }
            catch (const std::exception& e)
            {
                std::cerr << "Failed to get frame: " << e.what() << std::endl;
                continue;
            }
            frame_count++;
            cv::imwrite("CapturedFrames/frame_" + std::to_string(frame_count) + ".png", frame);
        }
    }
    cv::destroyAllWindows();
    std::cout << "Calibration capture completed. Total frames captured: " << frame_count << std::endl;

    return 0;
}
