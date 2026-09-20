#include "crc.h"

#include <Config.h>
#include <FoxGloveServer.h>
#include <Serial.h>
#include <TimeUtils.h>

void SerialHandler::serial_receive_callback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    // FoxGloveServer::get_server().log_message("Received " + std::to_string(bytes_transferred) + " bytes from serial.", FoxGloveServer::LogLevel::DEBUG);
    if (error)
    {
        FoxGloveServer::get_server().log_message("Serial receive error: " + error.message(), FoxGloveServer::LogLevel::ERROR);
        if (error == boost::asio::error::operation_aborted) // serial 关闭时会触发
            return;
        if (error == boost::asio::error::eof) // 连接断开
            return;
        m_read_buffer.consume(m_read_buffer.size());
    }
    else
    {
        m_read_buffer.commit(bytes_transferred);
        // find SOF and process complete packets
        while (m_read_buffer.size() >= serial_pack_size)
        {
            auto it = boost::asio::buffers_begin(m_read_buffer.data());
            if (static_cast<uint8_t>(*it) != serial_sof)
            {
                m_read_buffer.consume(1);
                continue;
            }
            if (m_read_buffer.size() < serial_pack_size)
                break;
            ECSSerialPack pack = {};

            // 使用 buffer_copy 预览数据而不消耗缓冲区
            boost::asio::buffer_copy(boost::asio::buffer(&pack, sizeof(pack)), m_read_buffer.data());

            auto result = Verify_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&pack), sizeof(pack));
            if (!result)
            {
                FoxGloveServer::get_server().log_message("Invalid CRC16 in received serial pack.", FoxGloveServer::LogLevel::WARNING);
                m_read_buffer.consume(1);
                // CRC 校验失败，只消耗一个字节（当前的伪帧头），继续寻找下一个帧头
                continue;
            }

            // CRC 校验通过，消耗整个数据包
            m_read_buffer.consume(serial_pack_size);

            FoxGloveServer::get_server().log_gimbal_real_rotation(CommonTypes::GimbalAngles{
                .pitch = static_cast<double>(pack.pitch_gyro_angle),
                .yaw = static_cast<double>(pack.yaw_gyro_angle),
            });
            m_current_pitch_encoder = pack.pitch_encoder_angle;
            m_current_yaw_encoder = pack.yaw_encoder_angle;
            m_current_pitch_angle = static_cast<double>(pack.pitch_gyro_angle);
            m_current_yaw_angle = static_cast<double>(pack.yaw_gyro_angle);
            m_last_receive_time = TimeUtils::now_nanoseconds();
            m_package_count.fetch_add(1);
        }
    }

    m_serial.async_read_some(m_read_buffer.prepare(Config::get_config().get_serial_config()->m_prepare_size), std::bind(&SerialHandler::serial_receive_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

SerialHandler::SerialHandler(const std::string& port_name, unsigned int baud_rate) : m_serial(m_io), m_read_buffer(Config::get_config().get_serial_config()->m_buffer_size)
{
    m_serial.open(port_name);
    m_serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

    m_serial.async_read_some(m_read_buffer.prepare(Config::get_config().get_serial_config()->m_prepare_size), std::bind(&SerialHandler::serial_receive_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

    m_io_thread = std::thread(
        [this]()
        {
            auto work = boost::asio::make_work_guard(m_io);
            FoxGloveServer::get_server().log_message("Serial IO thread started.", FoxGloveServer::LogLevel::INFO);
            m_io.run();
        });
}

SerialHandler::~SerialHandler()
{
    if (m_serial.is_open())
    {
        // 终止所有异步操作
        m_serial.cancel();

        // 发送control=false包，停止电机
        send_target_angles(CommonTypes::GimbalAngles{.pitch = 0.0, .yaw = 0.0}, ControlMode::Gyroscope, false);

        // 关闭串口
        m_serial.close();
    }

    // 停止IO上下文和线程
    m_io.stop();
    if (m_io_thread.joinable())
        m_io_thread.join();
}

bool SerialHandler::send_target_angles(CommonTypes::GimbalAngles target_angles, ControlMode control_mode, bool control)
{

    if (std::isnan(target_angles.pitch) || std::isnan(target_angles.yaw))
    {
        FoxGloveServer::get_server().log_message("Attempted to send NaN target angles. Ignoring.", FoxGloveServer::LogLevel::ERROR);
        return false;
    }

    FoxGloveServer::get_server().log_gimbal_current_rotation(CommonTypes::GimbalAngles{
        .pitch = static_cast<double>(target_angles.pitch),
        .yaw = static_cast<double>(target_angles.yaw),
    });

    VisionSerialPack pack = {};
    pack.SOF = serial_sof;
    pack.pitch_target_angle = static_cast<float>(target_angles.pitch);
    pack.yaw_target_angle = static_cast<float>(target_angles.yaw);

    pack.control = control;
    pack.control_mode = control_mode;
    pack.CRC16 = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&pack), sizeof(VisionSerialPack) - sizeof(uint16_t), CRC_INIT);

    boost::system::error_code ec;
    boost::asio::write(m_serial, boost::asio::buffer(&pack, sizeof(pack)), ec);
    if (ec)
    {
        FoxGloveServer::get_server().log_message("Serial send error: " + ec.message(), FoxGloveServer::LogLevel::ERROR);
        return false;
    }
    return true;
}