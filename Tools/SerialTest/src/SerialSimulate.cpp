// 云台电控模拟器：在串口上按 ECSSerialPack 周期回发当前角度/编码器值，并解析上位机发来的
// VisionSerialPack，从而在没有真实电控的情况下联调串口协议。
//
// 通常与 socat 配合使用（见 Tools/SerialTest/README.md）：
//   socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
//   ./serial_simulate --port /tmp/ttyV1        # 本模拟器
//   # 再把 config.yaml 的 SerialConfig.port_name 指向 /tmp/ttyV0 运行主程序
// NOLINTBEGIN(readability-identifier-naming)
#include "crc.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using boost::asio::serial_port;

namespace
{
#pragma pack(push, 1)
struct VisionSerialPack
{
    uint8_t SOF;
    float pitch_target_angle;
    float yaw_target_angle;
    bool control;
    uint8_t control_mode;
    uint16_t CRC16;
};

struct ECSSerialPack
{
    uint8_t SOF;
    float pitch_gyro_angle;
    float yaw_gyro_angle;
    uint16_t pitch_encoder_angle;
    uint16_t yaw_encoder_angle;
    uint16_t CRC16;
};
#pragma pack(pop)

static constexpr uint8_t kSof = 0xA5;
static constexpr double kEncoderCountsPerRev = 8192.0;
static constexpr double kDegreesPerRev = 360.0;

uint16_t encoder_from_degrees(double degrees)
{
    const double counts = degrees / kDegreesPerRev * kEncoderCountsPerRev;
    double wrapped = std::fmod(counts, kEncoderCountsPerRev);
    if (wrapped < 0.0)
        wrapped += kEncoderCountsPerRev;
    return static_cast<uint16_t>(std::lround(wrapped));
}

const char* control_mode_to_string(uint8_t control_mode)
{
    switch (control_mode)
    {
    case 0:
        return "Reset";
    case 1:
        return "Gyroscope";
    case 2:
        return "Encoder";
    default:
        return "Unknown";
    }
}

struct SimulatorState
{
    double current_pitch = 0.0;
    double current_yaw = 0.0;
    double target_pitch = 0.0;
    double target_yaw = 0.0;
    double follow_rate_deg_per_sec = 90.0;
    bool follow_target = true;
    bool last_control = true;
    uint8_t last_control_mode = 1;
};

double clamp_step(double current, double target, double max_step)
{
    const double delta = target - current;
    if (delta > max_step)
        return current + max_step;
    if (delta < -max_step)
        return current - max_step;
    return target;
}

class SerialLowerSimulator
{
public:
    SerialLowerSimulator(const std::string& port, unsigned int baud_rate, double send_hz) : serial_(io_), send_interval_(static_cast<int>(1000.0 / send_hz))
    {
        serial_.open(port);
        serial_.set_option(serial_port::baud_rate(baud_rate));
        serial_.set_option(serial_port::character_size(8));
        serial_.set_option(serial_port::parity(serial_port::parity::none));
        serial_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        serial_.set_option(serial_port::flow_control(serial_port::flow_control::none));
    }

    ~SerialLowerSimulator() { stop(); }

    void start()
    {
        running_.store(true);
        rx_thread_ = std::thread(&SerialLowerSimulator::rx_loop, this);
        tx_thread_ = std::thread(&SerialLowerSimulator::tx_loop, this);
    }

    void stop()
    {
        running_.store(false);
        if (serial_.is_open())
        {
            boost::system::error_code ec;
            serial_.cancel(ec);
            serial_.close(ec);
        }
        if (rx_thread_.joinable())
            rx_thread_.join();
        if (tx_thread_.joinable())
            tx_thread_.join();
    }

    void set_current(double pitch, double yaw)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.current_pitch = pitch;
        state_.current_yaw = yaw;
    }

    void set_target(double pitch, double yaw)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.target_pitch = pitch;
        state_.target_yaw = yaw;
    }

    void set_follow(bool follow)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.follow_target = follow;
    }

    void set_rate(double rate)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.follow_rate_deg_per_sec = rate;
    }

    SimulatorState snapshot()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

private:
    void rx_loop()
    {
        std::vector<uint8_t> buffer;
        buffer.reserve(256);
        std::array<uint8_t, 128> temp{};
        std::size_t packet_count = 0;

        while (running_.load())
        {
            boost::system::error_code ec;
            const std::size_t bytes = serial_.read_some(boost::asio::buffer(temp), ec);
            if (ec)
            {
                if (running_.load())
                    std::cerr << "Serial read error: " << ec.message() << std::endl;
                continue;
            }

            buffer.insert(buffer.end(), temp.begin(), temp.begin() + static_cast<long>(bytes));

            while (buffer.size() >= sizeof(VisionSerialPack))
            {
                auto sof_it = std::find(buffer.begin(), buffer.end(), kSof);
                if (sof_it == buffer.end())
                {
                    buffer.clear();
                    break;
                }

                if (static_cast<size_t>(buffer.end() - sof_it) < sizeof(VisionSerialPack))
                {
                    buffer.erase(buffer.begin(), sof_it);
                    break;
                }

                VisionSerialPack pack{};
                std::memcpy(&pack, &(*sof_it), sizeof(VisionSerialPack));
                if (!Verify_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&pack), sizeof(VisionSerialPack)))
                {
                    buffer.erase(buffer.begin(), sof_it + 1);
                    continue;
                }

                buffer.erase(buffer.begin(), sof_it + sizeof(VisionSerialPack));

                ++packet_count;
                const double pitch = static_cast<double>(pack.pitch_target_angle);
                const double yaw = static_cast<double>(pack.yaw_target_angle);

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    state_.target_pitch = pitch;
                    state_.target_yaw = yaw;
                    state_.last_control = pack.control;
                    state_.last_control_mode = pack.control_mode;
                    if (pack.control_mode == 0)
                    {
                        state_.target_pitch = 0.0;
                        state_.target_yaw = 0.0;
                    }
                }

                if (packet_count % 10 == 0)
                {
                    std::cout << "Recv target P=" << pitch << " Y=" << yaw << " control=" << pack.control << " mode=" << control_mode_to_string(pack.control_mode) << std::endl;
                }
            }
        }
    }

    void tx_loop()
    {
        auto last_time = std::chrono::steady_clock::now();
        std::size_t frame = 0;

        while (running_.load())
        {
            const auto now = std::chrono::steady_clock::now();
            const std::chrono::duration<double> dt = now - last_time;
            last_time = now;

            SimulatorState snapshot;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                snapshot = state_;
                if (snapshot.follow_target && snapshot.last_control)
                {
                    const double max_step = snapshot.follow_rate_deg_per_sec * dt.count();
                    state_.current_pitch = clamp_step(state_.current_pitch, snapshot.target_pitch, max_step);
                    state_.current_yaw = clamp_step(state_.current_yaw, snapshot.target_yaw, max_step);
                }
            }

            ECSSerialPack pack{};
            pack.SOF = kSof;
            pack.pitch_gyro_angle = static_cast<float>(snapshot.current_pitch);
            pack.yaw_gyro_angle = static_cast<float>(snapshot.current_yaw);
            pack.pitch_encoder_angle = encoder_from_degrees(snapshot.current_pitch);
            pack.yaw_encoder_angle = encoder_from_degrees(snapshot.current_yaw);
            pack.CRC16 = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&pack), sizeof(ECSSerialPack) - sizeof(uint16_t), CRC_INIT);

            boost::system::error_code ec;
            boost::asio::write(serial_, boost::asio::buffer(&pack, sizeof(pack)), ec);
            if (ec)
            {
                if (running_.load())
                    std::cerr << "Serial write error: " << ec.message() << std::endl;
            }
            else if (++frame % 50 == 0)
            {
                std::cout << "Send current P=" << snapshot.current_pitch << " Y=" << snapshot.current_yaw << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(send_interval_));
        }
    }

    boost::asio::io_context io_;
    serial_port serial_;
    std::atomic<bool> running_{false};
    std::thread rx_thread_;
    std::thread tx_thread_;
    std::mutex mutex_;
    SimulatorState state_{};
    int send_interval_;
};

void print_help()
{
    std::cout << "Commands:\n"
              << "  p <value>          set current pitch\n"
              << "  y <value>          set current yaw\n"
              << "  t <pitch> <yaw>    set target pitch/yaw\n"
              << "  rate <value>       set follow rate (deg/s)\n"
              << "  follow on|off      enable/disable follow\n"
              << "  status             print current state\n"
              << "  q                  quit\n";
}

} // namespace

int main(int argc, char** argv)
{
    std::string port = "/dev/ttyUSB0";
    unsigned int baud = 115200;
    double send_hz = 50.0;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc)
        {
            port = argv[++i];
        }
        else if (arg == "--baud" && i + 1 < argc)
        {
            baud = static_cast<unsigned int>(std::strtoul(argv[++i], nullptr, 10));
        }
        else if (arg == "--hz" && i + 1 < argc)
        {
            send_hz = std::strtod(argv[++i], nullptr);
        }
        else if (arg == "--help")
        {
            std::cout << "Usage: serial_simulate [--port <device>] [--baud <baud>] [--hz <send_hz>]\n";
            return 0;
        }
    }

    try
    {
        SerialLowerSimulator simulator(port, baud, send_hz);
        simulator.start();

        std::cout << "Serial lower simulator started on " << port << " @ " << baud << " baud, " << send_hz << "Hz." << std::endl;
        print_help();

        std::string command;
        while (std::cin >> command)
        {
            if (command == "q")
            {
                break;
            }
            if (command == "p" || command == "y")
            {
                double value = 0.0;
                if (!(std::cin >> value))
                {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Invalid input format." << std::endl;
                    continue;
                }
                auto snapshot = simulator.snapshot();
                if (command == "p")
                    simulator.set_current(value, snapshot.current_yaw);
                else
                    simulator.set_current(snapshot.current_pitch, value);
                continue;
            }
            if (command == "t")
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
                simulator.set_target(pitch, yaw);
                continue;
            }
            if (command == "rate")
            {
                double rate = 0.0;
                if (!(std::cin >> rate))
                {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Invalid input format." << std::endl;
                    continue;
                }
                simulator.set_rate(rate);
                continue;
            }
            if (command == "follow")
            {
                std::string value;
                if (!(std::cin >> value))
                {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Invalid input format." << std::endl;
                    continue;
                }
                simulator.set_follow(value == "on");
                continue;
            }
            if (command == "status")
            {
                const auto snapshot = simulator.snapshot();
                std::cout << "Current P=" << snapshot.current_pitch << " Y=" << snapshot.current_yaw << " Target P=" << snapshot.target_pitch << " Y=" << snapshot.target_yaw << " Follow=" << (snapshot.follow_target ? "on" : "off")
                          << " Rate=" << snapshot.follow_rate_deg_per_sec << " Control=" << (snapshot.last_control ? "on" : "off") << " Mode=" << control_mode_to_string(snapshot.last_control_mode) << std::endl;
                continue;
            }

            std::cout << "Unknown command." << std::endl;
            print_help();
        }

        simulator.stop();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
// NOLINTEND(readability-identifier-naming)
