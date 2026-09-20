#include <Config.h>
#include <FoxGloveServer.h>
#include <Network.h>
#include <TimeUtils.h>

void NetworkHandler::send_heartbeat(uint8_t status)
{
    CommonTypes::Second current_time = TimeUtils::now_seconds();
    if (current_time - m_last_heartbeat_time.load() < 1.0)
        return;
    m_last_heartbeat_time.store(current_time);

    if (!m_send_socket.is_open())
    {
        FoxGloveServer::get_server().log_message("Send socket is not open. Cannot send heartbeat.", FoxGloveServer::LogLevel::WARNING);
        return;
    }

    HeartbeatPack heartbeat;
    heartbeat.SOF = 0;          // reserve
    heartbeat.data_length = 0;  // reserve
    heartbeat.seq = 0;          // reserve
    heartbeat.difficulty = 0;   // reserve
    heartbeat.cmd_id = 0xFFFF;  // heartbeat command id
    heartbeat.device_id = 0x01; // device id
    heartbeat.status = status;

    try
    {
        m_send_socket.send_to(boost::asio::buffer(&heartbeat, sizeof(HeartbeatPack)), m_send_endpoint);
        FoxGloveServer::get_server().log_message("Sent heartbeat with status: " + std::to_string(status), FoxGloveServer::LogLevel::DEBUG);
        m_last_heartbeat_time.store(current_time);
    }
    catch (const std::exception& e)
    {
        FoxGloveServer::get_server().log_message("Failed to send heartbeat: " + std::string(e.what()), FoxGloveServer::LogLevel::ERROR);
    }
}
void NetworkHandler::network_receive_callback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    FoxGloveServer::get_server().log_message("Received network data: " + std::to_string(bytes_transferred) + " bytes", FoxGloveServer::LogLevel::DEBUG);
    if (error)
    {
        if (error == boost::asio::error::operation_aborted) // socket 关闭时会触发
            return;
        FoxGloveServer::get_server().log_message("Network error: " + error.message(), FoxGloveServer::LogLevel::ERROR);
    }
    else if (bytes_transferred != sizeof(CommonTypes::NetworkPack))
    {
        FoxGloveServer::get_server().log_message("Received network pack with unexpected size. Ignoring.", FoxGloveServer::LogLevel::WARNING);
    }
    else
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            // if (m_receive_buffer.frame_id <= m_last_received_pack.frame_id)
            //     FoxGloveServer::get_server().log_message("Received out-of-order or duplicate network pack (seq=" + std::to_string(m_receive_buffer.frame_id) + "). Ignoring.", FoxGloveServer::LogLevel::WARNING);
            // else
            // {
            m_last_received_pack = m_receive_buffer;
            // }
        }
        FoxGloveServer::get_server().log_network_pack(get_latest_pack());
    }
    // 继续接收下一个数据包
    m_socket.async_receive_from(
        boost::asio::buffer(&m_receive_buffer, sizeof(CommonTypes::NetworkPack)), m_remote_endpoint, std::bind(&NetworkHandler::network_receive_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

NetworkHandler::NetworkHandler(const std::string& listen_ip, uint16_t listen_port) : m_socket(m_io), m_send_socket(m_io), m_receive_buffer(CommonTypes::NetworkPack{.frame_id = 0, .x = 0, .y = 0, .z = 0})
{
    m_remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(listen_ip), listen_port);
    m_socket = boost::asio::ip::udp::socket(m_io, m_remote_endpoint);

    auto network_config = Config::get_config().get_network_config();
    m_send_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(network_config->m_send_target_ip), network_config->m_send_target_port);
    m_send_socket.open(boost::asio::ip::udp::v4());

    FoxGloveServer::get_server().log_message("NetworkHandler listening on " + listen_ip + ":" + std::to_string(listen_port), FoxGloveServer::LogLevel::INFO);
    FoxGloveServer::get_server().log_message("NetworkHandler sending to " + network_config->m_send_target_ip + ":" + std::to_string(network_config->m_send_target_port), FoxGloveServer::LogLevel::INFO);

    m_io_thread = std::thread(
        [this]()
        {
            auto work = boost::asio::make_work_guard(m_io);
            m_io.run();
        });

    m_socket.async_receive_from(
        boost::asio::buffer(&m_receive_buffer, sizeof(CommonTypes::NetworkPack)), m_remote_endpoint, std::bind(&NetworkHandler::network_receive_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

    m_last_received_pack = CommonTypes::NetworkPack{.frame_id = 0, .x = 0, .y = 0, .z = 0, .allow_counter = true};
}

NetworkHandler::~NetworkHandler()
{
    if (m_socket.is_open())
    {
        m_socket.cancel();
        m_socket.close();
    }
    if (m_send_socket.is_open())
    {
        m_send_socket.cancel();
        m_send_socket.close();
    }
    m_io.stop();
    if (m_io_thread.joinable())
        m_io_thread.join();
}