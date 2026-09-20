// 串口调试工具：向云台电控发送目标角度、切换控制模式，并打印回传的角度/编码器值。
//
// 用途：
//   1) 没有真实云台时，可用同目录的 serial_simulate 模拟电控回包（配合 socat 虚拟串口）；
//   2) 检查接线、波特率、方向约定（invert_* 配置）是否正确。
//
// 使用前请确认 config.yaml 的 SerialConfig.port_name / baud_rate 指向正确设备。
#include <Config.h>
#include <Serial.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

std::atomic_bool g_running{true};
std::mutex mutex;
// NOLINTBEGIN(readability-identifier-naming)
struct State
{
    double target_pitch = 0.0;
    double target_yaw = 0.0;
    SerialHandler::ControlMode control_mode = SerialHandler::ControlMode::Gyroscope;
    bool control = false;
    bool auto_status = true;
} current_state;
// NOLINTEND(readability-identifier-naming)
std::thread send_thread;
std::thread status_thread;

bool is_initialized(const SerialHandler& serial) { return serial.get_package_count() > 0; }

void send_loop(SerialHandler& serial)
{
    while (g_running.load())
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            serial.send_target_angles(CommonTypes::GimbalAngles{.pitch = current_state.target_pitch, .yaw = current_state.target_yaw}, current_state.control_mode, current_state.control);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void status_loop(SerialHandler& serial)
{
    while (g_running.load())
    {
        bool should_print = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            should_print = current_state.auto_status;
        }
        if (should_print)
        {
            const auto angles = serial.get_current_angles();
            std::cout << "Current pitch=" << angles.pitch << " yaw=" << angles.yaw << " encoder_pitch=" << serial.get_pitch_encoder_angle() << " encoder_yaw=" << serial.get_yaw_encoder_angle()
                      << " packets=" << serial.get_package_count() << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void print_help()
{
    std::cout << "Commands:\n"
              << "  send <pitch> <yaw> [gyro|encoder|reset]\n"
              << "  control <on|off>                      set control state\n"
              << "  reset                                 send a reset (回正) command\n"
              << "  status                                print current angles (auto every 1s)\n"
              << "  toggle                                toggle auto status output\n"
              << "  wait                                  wait until the first feedback packet arrives\n"
              << "  q                                     quit\n";
}

SerialHandler::ControlMode parse_mode(const std::string& mode)
{
    if (mode == "gyro")
        return SerialHandler::ControlMode::Gyroscope;
    if (mode == "encoder")
        return SerialHandler::ControlMode::Encoder;
    if (mode == "reset")
        return SerialHandler::ControlMode::Reset;
    return SerialHandler::ControlMode::Gyroscope;
}

int main()
{
    try
    {
        Config::get_config().load_from_file();
    }
    catch (const std::exception&)
    {
        Config::get_config().generate_default_config();
        Config::get_config().load_from_file();
    }

    SerialHandler serial(Config::get_config().get_serial_config()->m_port_name, Config::get_config().get_serial_config()->m_baud_rate);
    std::cout << "Serial test started on " << Config::get_config().get_serial_config()->m_port_name << " @ " << Config::get_config().get_serial_config()->m_baud_rate << " baud." << std::endl;
    print_help();
    send_thread = std::thread(send_loop, std::ref(serial));
    status_thread = std::thread(status_loop, std::ref(serial));

    std::string command;
    while (std::cin >> command)
    {
        if (command == "q")
            break;
        if (command == "control")
        {
            std::string control;
            if (!(std::cin >> control))
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Invalid input format." << std::endl;
                continue;
            }

            if (control != "on" && control != "off")
            {
                std::cout << "Invalid control value. Use on/off." << std::endl;
                continue;
            }

            current_state.control = (control == "on");
            std::cout << "\33[32mSet control " << control << ".\33[0m" << std::endl;
            continue;
        }
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

            std::string mode = "gyro";
            if (std::cin.peek() != '\n')
                std::cin >> mode;

            current_state.target_pitch = pitch;
            current_state.target_yaw = yaw;
            current_state.control_mode = parse_mode(mode);
            std::cout << "\33[32mSet target pitch=" << pitch << " yaw=" << yaw << " mode=" << mode << ".\33[0m" << std::endl;
            continue;
        }
        if (command == "reset")
        {
            serial.send_target_angles(CommonTypes::GimbalAngles{.pitch = 0.0, .yaw = 0.0}, SerialHandler::ControlMode::Reset, true);
            std::cout << "\33[32mReset command sent.\33[0m" << std::endl;
            continue;
        }
        if (command == "status")
        {
            const auto angles = serial.get_current_angles();
            std::cout << "\33[32mCurrent pitch=" << angles.pitch << " yaw=" << angles.yaw << " encoder_pitch=" << serial.get_pitch_encoder_angle() << " encoder_yaw=" << serial.get_yaw_encoder_angle()
                      << " packets=" << serial.get_package_count() << ".\33[0m" << std::endl;
            continue;
        }
        if (command == "toggle")
        {
            {
                std::lock_guard<std::mutex> lock(mutex);
                current_state.auto_status = !current_state.auto_status;
            }
            std::cout << "\33[32mAuto status " << (current_state.auto_status ? "on" : "off") << ".\33[0m" << std::endl;
            continue;
        }
        if (command == "wait")
        {
            while (!is_initialized(serial))
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::cout << "\33[32mSerial feedback received.\33[0m" << std::endl;
            continue;
        }

        std::cout << "Unknown command." << std::endl;
        print_help();
    }

    g_running.store(false);
    if (send_thread.joinable())
        send_thread.join();
    if (status_thread.joinable())
        status_thread.join();
    return 0;
}
