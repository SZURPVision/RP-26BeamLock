#pragma once
#include <CommonTypes.h>
#include <atomic>
#include <boost/asio.hpp>
#include <cstdint>
#include <iostream>
#include <thread>

class SerialHandler
{
public:
    enum ControlMode : uint8_t
    {
        Reset = 0,     // 回正
        Gyroscope = 1, // 陀螺仪角度
        Encoder = 2,   // 编码器角度
    };

private:
    // NOLINTBEGIN(readability-identifier-naming)
#pragma pack(push, 1)
    struct VisionSerialPack
    {
        uint8_t SOF;

        float pitch_target_angle;
        float yaw_target_angle;

        bool control; // 电机是否发力 false为不发力
        ControlMode control_mode;

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
    // NOLINTEND(readability-identifier-naming)

    static constexpr int serial_pack_size = sizeof(ECSSerialPack);
    static constexpr uint8_t serial_sof = 0xA5;

private:
    std::atomic<double> m_current_pitch_angle = 0;
    std::atomic<double> m_current_yaw_angle = 0;
    std::atomic<uint16_t> m_current_pitch_encoder = 0;
    std::atomic<uint16_t> m_current_yaw_encoder = 0;

    std::atomic<uint64_t> m_package_count = 0;

    boost::asio::io_context m_io;
    boost::asio::serial_port m_serial;
    boost::asio::streambuf m_read_buffer;
    std::thread m_io_thread;

    std::atomic<CommonTypes::Nanosecond> m_last_receive_time = 0;

    void serial_receive_callback(const boost::system::error_code& error, std::size_t bytes_transferred);

public:
    explicit SerialHandler(const std::string& port_name, unsigned int baud_rate);
    ~SerialHandler();

    SerialHandler(const SerialHandler&) = delete;
    SerialHandler& operator=(const SerialHandler&) = delete;
    SerialHandler(SerialHandler&&) = delete;
    SerialHandler& operator=(SerialHandler&&) = delete;

    CommonTypes::GimbalAngles get_current_angles() const { return CommonTypes::GimbalAngles{.pitch = m_current_pitch_angle.load(), .yaw = m_current_yaw_angle.load()}; }

    uint16_t get_yaw_encoder_angle() const { return m_current_yaw_encoder.load(); }
    uint16_t get_pitch_encoder_angle() const { return m_current_pitch_encoder.load(); }

    bool send_target_angles(CommonTypes::GimbalAngles target_angles, ControlMode control_mode = ControlMode::Gyroscope, bool control = true);
    uint64_t get_package_count() const { return m_package_count.load(); }
};