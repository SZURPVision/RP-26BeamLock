// 雷达主程序模拟器：向本机 9000 端口周期发送 CommonTypes::NetworkPack（10 Hz），
// 支持交互式修改目标的 x/y/z 与 allow_counter，用于在没有雷达主程序时调试云台跟踪与反制逻辑。
// 协议见 docs/PROTOCOL.md。
// NOLINTBEGIN(readability-identifier-naming)
#include "Network.h"

#include <CommonTypes.h>
#include <TimeUtils.h>
#include <atomic>
#include <boost/asio.hpp>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

using boost::asio::ip::udp;

namespace
{
struct RampState
{
    bool active = false;
    double duration_sec = 0.0;
    double total_delta = 0.0;
    double applied_delta = 0.0;
    CommonTypes::TimePoint start_time;
};

class SharedState
{
public:
    SharedState()
    {
        pack_.x = 0.0;
        pack_.y = 0.0;
        pack_.z = 0.0;
        pack_.allow_counter = true;
    }

    CommonTypes::NetworkPack snapshot_with_ramp(CommonTypes::TimePoint now)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        apply_ramp(pack_.x, ramp_x_, now);
        apply_ramp(pack_.y, ramp_y_, now);
        apply_ramp(pack_.z, ramp_z_, now);
        return pack_;
    }

    void set_axis(char axis, double value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (axis == 'x')
            pack_.x = value;
        else if (axis == 'y')
            pack_.y = value;
        else if (axis == 'z')
            pack_.z = value;
    }

    void start_ramp(char axis, double duration, double delta, CommonTypes::TimePoint now)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        RampState* ramp = get_ramp(axis);
        if (!ramp)
            return;

        double* target = get_axis_ptr(axis);
        if (!target)
            return;

        apply_ramp(*target, *ramp, now);
        ramp->active = true;
        ramp->duration_sec = duration;
        ramp->total_delta = delta;
        ramp->applied_delta = 0.0;
        ramp->start_time = now;
    }

    void set_allow_counter(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pack_.allow_counter = value;
    }

    bool allow_counter() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return pack_.allow_counter;
    }

    CommonTypes::NetworkPack current_pack() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return pack_;
    }

private:
    static void apply_ramp(double& value, RampState& ramp, CommonTypes::TimePoint now)
    {
        if (!ramp.active)
        {
            return;
        }

        const double elapsed = TimeUtils::elapsed_seconds(ramp.start_time, now);
        if (elapsed >= ramp.duration_sec)
        {
            value += ramp.total_delta - ramp.applied_delta;
            ramp.active = false;
            ramp.applied_delta = 0.0;
            return;
        }

        const double target_applied = ramp.total_delta * (elapsed / ramp.duration_sec);
        const double delta = target_applied - ramp.applied_delta;
        value += delta;
        ramp.applied_delta += delta;
    }

    RampState* get_ramp(char axis)
    {
        if (axis == 'x')
            return &ramp_x_;
        if (axis == 'y')
            return &ramp_y_;
        if (axis == 'z')
            return &ramp_z_;
        return nullptr;
    }

    double* get_axis_ptr(char axis)
    {
        if (axis == 'x')
            return &pack_.x;
        if (axis == 'y')
            return &pack_.y;
        if (axis == 'z')
            return &pack_.z;
        return nullptr;
    }

    mutable std::mutex mutex_;
    CommonTypes::NetworkPack pack_{};
    RampState ramp_x_{};
    RampState ramp_y_{};
    RampState ramp_z_{};
};

void send_thread_func(udp::socket* socket, const udp::endpoint& endpoint, SharedState* state, const std::atomic<bool>* running)
{
    uint32_t frame_count = 0;
    while (running->load())
    {
        const auto now = TimeUtils::now();
        CommonTypes::NetworkPack current_pack = state->snapshot_with_ramp(now);
        current_pack.frame_id = frame_count++;

        try
        {
            socket->send_to(boost::asio::buffer(&current_pack, sizeof(CommonTypes::NetworkPack)), endpoint);
            if (frame_count % 10 == 0)
            {
                std::cout << "Sent Frame: " << current_pack.frame_id << " X: " << current_pack.x << " Y: " << current_pack.y << " Z: " << current_pack.z << " allow_counter: " << current_pack.allow_counter << std::endl;
            }
        }
        catch (std::exception& e)
        {
            std::cerr << "Send error: " << e.what() << std::endl;
        }

        TimeUtils::sleep_for_milliseconds(100); // 10Hz
    }
}
} // namespace

int main()
{
    try
    {
        boost::asio::io_context io_context;

        // 1. 创建 UDP socket
        udp::socket socket(io_context);
        socket.open(udp::v4());

        // 2. 定义目标地址和端口 (例如: 本地回环 127.0.0.1, 端口 8080)
        std::string target_ip = "127.0.0.1";
        unsigned short target_port = 9000;
        udp::endpoint receiver_endpoint(boost::asio::ip::make_address(target_ip), target_port);

        SharedState state;
        std::atomic<bool> running(true);

        // 启动发送线程
        std::thread sender(send_thread_func, &socket, receiver_endpoint, &state, &running);

        std::cout << "Server started. Sending at 10Hz." << std::endl;
        std::cout << "Commands: 'x <value>', 'y <value>', 'z <value>', 'dx <seconds> <delta>', 'dy <seconds> <delta>', 'dz <seconds> <delta>', 'allow <0|1>', 'q' (quit)" << std::endl;

        std::string command;
        while (running.load() && std::cin >> command)
        {
            if (command == "q")
            {
                running = false;
                break;
            }

            if (command == "allow")
            {
                int value = 0;
                if (!(std::cin >> value))
                {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Invalid input format. Usage: allow <0|1>" << std::endl;
                    continue;
                }

                state.set_allow_counter(value != 0);
                std::cout << "allow_counter set to: " << state.allow_counter() << std::endl;
                continue;
            }

            if (command == "x" || command == "y" || command == "z")
            {
                double value = 0.0;
                if (!(std::cin >> value))
                {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Invalid input format." << std::endl;
                    continue;
                }

                char axis = command[0]; // 'x', 'y', or 'z'
                state.set_axis(axis, value);
                const CommonTypes::NetworkPack current_pack = state.current_pack();
                std::cout << "Updated: X=" << current_pack.x << " Y=" << current_pack.y << " Z=" << current_pack.z << std::endl;
                continue;
            }

            if (command == "dx" || command == "dy" || command == "dz")
            {
                double duration = 0.0;
                double delta = 0.0;
                if (!(std::cin >> duration >> delta))
                {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Invalid input format." << std::endl;
                    continue;
                }

                if (duration <= 0.0)
                {
                    std::cout << "Duration must be positive." << std::endl;
                    continue;
                }

                char axis = command[1]; // 'x', 'y', or 'z'

                const auto now = TimeUtils::now();
                state.start_ramp(axis, duration, delta, now);

                std::cout << "Ramping " << command << " over " << duration << "s by " << delta << std::endl;
                const CommonTypes::NetworkPack current_pack = state.current_pack();
                std::cout << "Current: X=" << current_pack.x << " Y=" << current_pack.y << " Z=" << current_pack.z << std::endl;
                continue;
            }

            std::cout << "Unknown command" << std::endl;
        }

        running = false;
        if (sender.joinable())
            sender.join();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}
// NOLINTEND(readability-identifier-naming)