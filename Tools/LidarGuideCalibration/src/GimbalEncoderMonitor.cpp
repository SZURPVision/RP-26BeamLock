#include <Config.h>
#include <Serial.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{
std::atomic_bool g_running{true};

void handle_sigint(int) { g_running.store(false); }

double angle_normalize(double angle)
{
    while (angle > 180.0)
        angle -= 360.0;
    while (angle < -180.0)
        angle += 360.0;
    return angle;
}

double encoder_to_relative_angle(uint16_t current_encoder, uint16_t zero_encoder, int invert, int per_round)
{
    int diff = static_cast<int>(current_encoder) - static_cast<int>(zero_encoder);
    double angle = static_cast<double>(invert * diff) / static_cast<double>(per_round) * 360.0;
    return angle_normalize(angle);
}

void write_records(const std::string& file_path, const std::vector<std::pair<double, double>>& records)
{
    std::ofstream file(file_path, std::ios::trunc);
    if (!file.is_open())
    {
        std::cerr << "Failed to open output file: " << file_path << std::endl;
        return;
    }
    for (const auto& record : records)
    {
        file << record.first << " " << record.second << "\n";
    }
}
} // namespace

int main(int argc, char** argv)
{
    std::string output_path = "encoder_angles.txt";
    if (argc == 2)
    {
        output_path = argv[1];
    }
    else if (argc > 2)
    {
        std::cout << "Usage: " << argv[0] << " [output_file]" << std::endl;
        return 1;
    }
    std::signal(SIGINT, handle_sigint);

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

    auto serial_config = Config::get_config().get_serial_config();
    auto tripod_config = Config::get_config().get_tripodhead_config();

    SerialHandler serial(serial_config->m_port_name, serial_config->m_baud_rate);

    while (serial.get_package_count() < serial_config->m_reset_pack_count && g_running.load())
    {
        serial.send_target_angles(CommonTypes::GimbalAngles{.pitch = 0.0, .yaw = 0.0}, SerialHandler::ControlMode::Reset, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!g_running.load())
        return 0;

    uint16_t pitch_zero = serial.get_pitch_encoder_angle();
    uint16_t yaw_zero = serial.get_yaw_encoder_angle();

    serial.send_target_angles(CommonTypes::GimbalAngles{.pitch = 0.0, .yaw = 0.0}, SerialHandler::ControlMode::Gyroscope, false);

    std::cout << "Motor torque disabled (control=false)." << std::endl;
    std::cout << "Encoder zero: pitch=" << pitch_zero << " yaw=" << yaw_zero << std::endl;
    std::cout << "Commands: s=save current angles, d=delete last record, q=quit" << std::endl;
    std::cout << "Output file: " << output_path << std::endl;

    std::atomic<double> latest_pitch{0.0};
    std::atomic<double> latest_yaw{0.0};
    std::mutex records_mutex;
    std::vector<std::pair<double, double>> records;
    write_records(output_path, records);

    std::thread input_thread(
        [&]()
        {
            std::string line;
            while (g_running.load() && std::getline(std::cin, line))
            {
                if (line == "q" || line == "quit" || line == "exit")
                {
                    g_running.store(false);
                    break;
                }
                if (line == "s")
                {
                    double pitch = latest_pitch.load();
                    double yaw = latest_yaw.load();
                    size_t total_records = 0;
                    {
                        std::lock_guard<std::mutex> lock(records_mutex);
                        records.emplace_back(pitch, yaw);
                        write_records(output_path, records);
                        total_records = records.size();
                    }
                    std::cout << "Saved: pitch=" << pitch << " yaw=" << yaw << " | total=" << total_records << std::endl;
                    continue;
                }
                if (line == "d")
                {
                    size_t total_records = 0;
                    std::lock_guard<std::mutex> lock(records_mutex);
                    if (!records.empty())
                    {
                        records.pop_back();
                        write_records(output_path, records);
                        total_records = records.size();
                        std::cout << "Deleted last record. | total=" << total_records << std::endl;
                    }
                    else
                    {
                        std::cout << "No records to delete." << std::endl;
                    }
                    continue;
                }
            }
        });

    while (g_running.load())
    {
        uint16_t pitch_encoder = serial.get_pitch_encoder_angle();
        uint16_t yaw_encoder = serial.get_yaw_encoder_angle();

        double pitch_angle = encoder_to_relative_angle(pitch_encoder, pitch_zero, tripod_config->m_invert_pitch_encoder, tripod_config->m_pitch_encoder_per_round);
        double yaw_angle = encoder_to_relative_angle(yaw_encoder, yaw_zero, tripod_config->m_invert_yaw_encoder, tripod_config->m_yaw_encoder_per_round);

        latest_pitch.store(pitch_angle);
        latest_yaw.store(yaw_angle);

        std::cout << "pitch_encoder=" << pitch_encoder << " yaw_encoder=" << yaw_encoder << " | pitch_rel=" << pitch_angle << " yaw_rel=" << yaw_angle << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (input_thread.joinable())
        input_thread.join();

    return 0;
}
