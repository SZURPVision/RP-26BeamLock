#include <Config.h>
#include <FoxGloveServer.h>
#include <GimbalController.h>
#include <TimeUtils.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>

std::atomic_bool g_running{true};
void handle_sigint(int) { g_running.store(false); }

bool parse_angle(const std::string& text, double& value)
{
    try
    {
        size_t end_pos = 0;
        value = std::stod(text, &end_pos);
        return end_pos > 0 && end_pos <= text.size();
    }
    catch (...)
    {
        return false;
    }
}

CommonTypes::GimbalAngles get_square_scan_delta_angles(CommonTypes::Second time_seconds, double max_yaw_delta, double max_pitch_delta, CommonTypes::Second scan_period)
{
    double total_delta = 4 * (max_yaw_delta + max_pitch_delta);
    double delta = std::fmod(time_seconds, scan_period) / scan_period * total_delta;
    if (delta < 2 * max_yaw_delta)
        return CommonTypes::GimbalAngles{
            .pitch = -max_pitch_delta,
            .yaw = delta - max_yaw_delta,
        };
    else if (delta < 2 * max_yaw_delta + 2 * max_pitch_delta)
        return CommonTypes::GimbalAngles{
            .pitch = (delta - 2 * max_yaw_delta) - max_pitch_delta,
            .yaw = max_yaw_delta,
        };
    else if (delta < 4 * max_yaw_delta + 2 * max_pitch_delta)
        return CommonTypes::GimbalAngles{
            .pitch = max_pitch_delta,
            .yaw = max_yaw_delta - (delta - 2 * max_yaw_delta - 2 * max_pitch_delta),
        };
    else
        return CommonTypes::GimbalAngles{
            .pitch = max_pitch_delta - (delta - 4 * max_yaw_delta - 2 * max_pitch_delta),
            .yaw = -max_yaw_delta,
        };
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    FoxGloveServer::get_server().start();
    std::signal(SIGINT, handle_sigint);

    try
    {
        Config::get_config().load_from_file();
        FoxGloveServer::get_server().log_message("Config loaded successfully.", FoxGloveServer::LogLevel::INFO);
    }
    catch (const std::exception& e)
    {
        FoxGloveServer::get_server().log_message(std::string("Failed to load config file: ") + e.what(), FoxGloveServer::LogLevel::WARNING);
        FoxGloveServer::get_server().log_message("Generating default config file.", FoxGloveServer::LogLevel::INFO);
        Config::get_config().generate_default_config();
        Config::get_config().load_from_file();
    }

    GimbalController controller;
    std::mutex input_state_mutex;
    bool scan_mode = false;
    CommonTypes::GimbalAngles scan_center_angles{.pitch = 0.0, .yaw = 0.0};

    std::cout << "请输入目标角度: <pitch_deg> <yaw_deg>，或输入 q 退出" << std::endl;
    std::cout << "输入 t 切换 angle/encoder 模式，输入 s 切换方形扫描（encoder 模式）" << std::endl;
    std::cout << "输入 s <pitch_deg> <yaw_deg> 可指定扫描中心角" << std::endl;
    std::cout << "输入 v <pitch_speed> <yaw_speed> [max_time] 设置目标速度" << std::endl;

    std::thread input_thread(
        [&]()
        {
            bool encoder_mode = false;
            std::string line;
            while (g_running.load() && std::getline(std::cin, line))
            {
                if (line == "q" || line == "quit" || line == "exit")
                {
                    g_running.store(false);
                    break;
                }
                if (line == "t")
                {
                    encoder_mode = !encoder_mode;
                    std::cout << "Switched to " << (encoder_mode ? "encoder" : "angle") << " control mode." << std::endl;
                    continue;
                }
                if (line == "s")
                {
                    std::lock_guard<std::mutex> lock(input_state_mutex);
                    scan_mode = !scan_mode;
                    if (scan_mode)
                    {
                        scan_center_angles = controller.get_current_angles();
                        std::cout << "Square scan enabled. center pitch=" << scan_center_angles.pitch << " yaw=" << scan_center_angles.yaw << std::endl;
                    }
                    else
                    {
                        std::cout << "Square scan disabled." << std::endl;
                    }
                    continue;
                }
                std::istringstream iss(line);
                std::string pitch_text;
                std::string yaw_text;
                if (!(iss >> pitch_text >> yaw_text))
                {
                    std::cout << "输入格式错误，请输入: <pitch_deg> <yaw_deg>，s，s <pitch_deg> <yaw_deg>，v <pitch_speed> <yaw_speed> [max_time] 或 q 退出" << std::endl;
                    continue;
                }
                if (pitch_text == "v")
                {
                    double target_pitch_speed = 0.0;
                    double target_yaw_speed = 0.0;
                    if (!parse_angle(yaw_text, target_pitch_speed))
                    {
                        std::cout << "速度 pitch 解析失败，请重试。" << std::endl;
                        continue;
                    }
                    std::string yaw_speed_text;
                    if (!(iss >> yaw_speed_text) || !parse_angle(yaw_speed_text, target_yaw_speed))
                    {
                        std::cout << "速度 yaw 解析失败，请输入: v <pitch_speed> <yaw_speed> [max_time]" << std::endl;
                        continue;
                    }
                    double max_constant_time = std::numeric_limits<double>::infinity();
                    std::string max_time_text;
                    if (iss >> max_time_text)
                    {
                        if (!parse_angle(max_time_text, max_constant_time))
                        {
                            std::cout << "max_time 解析失败，请重试。" << std::endl;
                            continue;
                        }
                    }
                    {
                        std::lock_guard<std::mutex> lock(input_state_mutex);
                        scan_mode = false;
                    }
                    CommonTypes::GimbalAngles target_speeds{.pitch = target_pitch_speed, .yaw = target_yaw_speed};
                    controller.set_target_speeds(target_speeds, TimeUtils::now_seconds(), max_constant_time);
                    std::cout << "Target speeds set: pitch=" << target_pitch_speed << " yaw=" << target_yaw_speed;
                    if (std::isfinite(max_constant_time))
                    {
                        std::cout << " max_time=" << max_constant_time;
                    }
                    std::cout << std::endl;
                    continue;
                }
                if (pitch_text == "s")
                {
                    double center_pitch = 0.0;
                    double center_yaw = 0.0;
                    if (!parse_angle(yaw_text, center_pitch))
                    {
                        std::cout << "扫描中心 pitch 解析失败，请重试。" << std::endl;
                        continue;
                    }
                    std::string center_yaw_text;
                    if (!(iss >> center_yaw_text) || !parse_angle(center_yaw_text, center_yaw))
                    {
                        std::cout << "扫描中心 yaw 解析失败，请输入: s <pitch_deg> <yaw_deg>" << std::endl;
                        continue;
                    }
                    {
                        std::lock_guard<std::mutex> lock(input_state_mutex);
                        scan_center_angles = CommonTypes::GimbalAngles{.pitch = center_pitch, .yaw = center_yaw};
                        scan_mode = true;
                    }
                    std::cout << "Square scan enabled with center pitch=" << center_pitch << " yaw=" << center_yaw << std::endl;
                    continue;
                }
                double target_pitch = 0.0;
                double target_yaw = 0.0;
                if (parse_angle(pitch_text, target_pitch) && parse_angle(yaw_text, target_yaw))
                {
                    {
                        std::lock_guard<std::mutex> lock(input_state_mutex);
                        scan_mode = false;
                    }
                    if (!encoder_mode)
                    {
                        CommonTypes::GimbalAngles target_angles{.pitch = target_pitch, .yaw = target_yaw};
                        controller.set_target_angles(target_angles, TimeUtils::now_seconds());
                        std::cout << "Target set: pitch=" << target_pitch << " yaw=" << target_yaw << std::endl;
                    }
                    else
                    {
                        CommonTypes::GimbalAngles target_angles{.pitch = target_pitch, .yaw = target_yaw};
                        controller.set_target_encoder_angles(target_angles, TimeUtils::now_seconds());
                        std::cout << "Target encoder angles set: pitch=" << target_pitch << " yaw=" << target_yaw << std::endl;
                    }
                }
                else
                {
                    std::cout << "角度解析失败，请重试。" << std::endl;
                }
            }
            g_running.store(false);
        });

    CommonTypes::Second last_print_time = 0.0;
    while (g_running.load())
    {
        bool local_scan_mode = false;
        CommonTypes::GimbalAngles local_scan_center{.pitch = 0.0, .yaw = 0.0};
        {
            std::lock_guard<std::mutex> lock(input_state_mutex);
            local_scan_mode = scan_mode;
            local_scan_center = scan_center_angles;
        }
        CommonTypes::Second now = TimeUtils::now_seconds();
        if (local_scan_mode)
        {
            auto strategy_config = Config::get_config().get_track_strategy_config();
            CommonTypes::GimbalAngles scan_delta = get_square_scan_delta_angles(now, strategy_config->m_scan_yaw_angle, strategy_config->m_scan_pitch_angle, strategy_config->m_scan_period);
            CommonTypes::GimbalAngles target_angles{
                .pitch = local_scan_center.pitch + scan_delta.pitch,
                .yaw = local_scan_center.yaw + scan_delta.yaw,
            };
            controller.set_target_encoder_angles(target_angles, now);
        }

        auto angles = controller.get_current_angles();
        if (now - last_print_time >= 1.0)
        {
            std::cout << "Current angles: pitch=" << angles.pitch << " yaw=" << angles.yaw << (local_scan_mode ? " [square scan]" : "") << std::endl;
            last_print_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (input_thread.joinable())
        input_thread.join();

    FoxGloveServer::get_server().shutdown();
    return 0;
}
