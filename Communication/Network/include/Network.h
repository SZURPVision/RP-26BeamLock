#pragma once
#include <CommonTypes.h>
#include <atomic>
#include <boost/asio.hpp>
#include <thread>

class NetworkHandler
{

// NOLINTBEGIN(readability-identifier-naming)
#pragma pack(push, 1)
    struct HeartbeatPack
    {
        uint8_t SOF;          // reserve
        uint16_t data_length; // reserve
        uint8_t seq;          // reserve
        uint8_t difficulty;   // reserve
        uint16_t cmd_id;      // 0xFFFF
        uint8_t device_id;    // 0x01
        uint8_t status;
    };
#pragma pack(pop)
    // NOLINTEND(readability-identifier-naming)

    boost::asio::io_context m_io;
    boost::asio::ip::udp::endpoint m_remote_endpoint;
    boost::asio::ip::udp::socket m_socket;
    boost::asio::ip::udp::socket m_send_socket;
    boost::asio::ip::udp::endpoint m_send_endpoint;
    std::thread m_io_thread;
    CommonTypes::NetworkPack m_receive_buffer;

    std::mutex m_mutex;
    CommonTypes::NetworkPack m_last_received_pack;

    std::atomic<CommonTypes::Second> m_last_heartbeat_time = 0.0;

    void network_receive_callback(const boost::system::error_code& error, std::size_t bytes_transferred);

public:
    NetworkHandler(const std::string& listen_ip, uint16_t listen_port);
    ~NetworkHandler();

    NetworkHandler(const NetworkHandler&) = delete;
    NetworkHandler& operator=(const NetworkHandler&) = delete;
    NetworkHandler(NetworkHandler&&) = delete;
    NetworkHandler& operator=(NetworkHandler&&) = delete;

    CommonTypes::NetworkPack get_latest_pack()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_last_received_pack;
    }

    void send_heartbeat(uint8_t status);
};