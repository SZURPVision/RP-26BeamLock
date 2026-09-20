#include <Config.h>
#include <MVSCamera.h>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;

class CaptureServer
{
public:
    // 默认只监听回环地址：该服务没有任何鉴权，暴露到网络上等于把相机控制权交出去。
    // 确实需要远程访问时用 --bind 0.0.0.0 显式开启，并自行限制网络环境。
    CaptureServer(std::shared_ptr<Camera> camera, asio::io_context& io_context, const std::string& bind_address = "127.0.0.1", uint16_t port = 8080)
        : m_camera(std::move(camera))
        , m_bind_address(bind_address)
        , m_port(port)
        , m_acceptor(io_context, tcp::endpoint(asio::ip::make_address(bind_address), port))
    {
    }

    void run()
    {
        std::cout << "CaptureServer listening on " << m_bind_address << ":" << m_port << std::endl;
        while (true)
        {
            tcp::socket socket(m_acceptor.get_executor());
            m_acceptor.accept(socket);
            handle_session(std::move(socket));
        }
    }

private:
    std::shared_ptr<Camera> m_camera;
    std::string m_bind_address;
    uint16_t m_port;
    std::mutex m_frame_mutex;
    cv::Mat m_latest_frame;
    std::atomic_bool m_has_frame{false};
    enum class OutputMode
    {
        Bayer,
        Rgb
    };
    std::atomic<OutputMode> m_output_mode{OutputMode::Bayer};
    tcp::acceptor m_acceptor;

    static std::optional<std::string> load_control_page_html()
    {
        std::vector<std::filesystem::path> candidate_paths;
        candidate_paths.emplace_back("capture_control.html");

        std::error_code ec;
        const std::filesystem::path exe_path = std::filesystem::read_symlink("/proc/self/exe", ec);
        if (!ec)
        {
            candidate_paths.emplace_back(exe_path.parent_path() / "capture_control.html");
        }

        for (const auto& path : candidate_paths)
        {
            std::ifstream file(path, std::ios::in | std::ios::binary);
            if (!file)
            {
                continue;
            }

            std::ostringstream stream;
            stream << file.rdbuf();
            return stream.str();
        }

        return std::nullopt;
    }

    static std::optional<double> get_query_value(std::string_view query, std::string_view key)
    {
        std::size_t start = 0;
        while (start < query.size())
        {
            std::size_t amp_pos = query.find('&', start);
            if (amp_pos == std::string_view::npos)
            {
                amp_pos = query.size();
            }

            std::string_view token = query.substr(start, amp_pos - start);
            std::size_t eq_pos = token.find('=');
            if (eq_pos != std::string_view::npos)
            {
                std::string_view token_key = token.substr(0, eq_pos);
                std::string_view token_value = token.substr(eq_pos + 1);
                if (token_key == key)
                {
                    try
                    {
                        return std::stod(std::string(token_value));
                    }
                    catch (const std::exception&)
                    {
                        return std::nullopt;
                    }
                }
            }

            start = amp_pos + 1;
        }
        return std::nullopt;
    }

    static std::optional<std::string> get_query_string(std::string_view query, std::string_view key)
    {
        std::size_t start = 0;
        while (start < query.size())
        {
            std::size_t amp_pos = query.find('&', start);
            if (amp_pos == std::string_view::npos)
            {
                amp_pos = query.size();
            }

            std::string_view token = query.substr(start, amp_pos - start);
            std::size_t eq_pos = token.find('=');
            if (eq_pos != std::string_view::npos)
            {
                std::string_view token_key = token.substr(0, eq_pos);
                std::string_view token_value = token.substr(eq_pos + 1);
                if (token_key == key)
                {
                    return std::string(token_value);
                }
            }

            start = amp_pos + 1;
        }
        return std::nullopt;
    }

    static bool is_disconnect_error(const beast::error_code& ec) { return ec == asio::error::broken_pipe || ec == asio::error::connection_reset || ec == asio::error::eof || ec == asio::error::connection_aborted || ec == asio::error::not_connected; }

    template <typename ResponseBody> static void safe_write(tcp::socket& socket, http::response<ResponseBody>& response)
    {
        beast::error_code ec;
        http::write(socket, response, ec);
        if (ec && !is_disconnect_error(ec))
        {
            std::cerr << "HTTP write error: " << ec.message() << std::endl;
        }
    }

    void grab_loop()
    {
        while (true)
        {
            cv::Mat frame;
            try
            {
                m_camera->get_frame(frame, 1000);
                {
                    std::lock_guard<std::mutex> lock(m_frame_mutex);
                    m_latest_frame = frame.clone();
                }
                m_has_frame.store(true);
            }
            catch (const std::exception& e)
            {
                std::cerr << "Failed to get frame: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    void handle_args_request(const http::request<http::string_body>& request, http::response<http::string_body>& response)
    {
        std::string_view target = request.target();
        std::size_t query_pos = target.find('?');
        std::string_view query = (query_pos == std::string_view::npos) ? std::string_view{} : target.substr(query_pos + 1);

        auto exposure = get_query_value(query, "exposure");
        auto gain = get_query_value(query, "gain");
        auto mode = get_query_string(query, "mode");

        if (!exposure.has_value() && !gain.has_value() && !mode.has_value())
        {
            response.result(http::status::bad_request);
            response.body() = "missing query args, expected exposure, gain, or mode";
            return;
        }

        if (mode.has_value())
        {
            if (mode.value() == "rgb")
            {
                m_output_mode.store(OutputMode::Rgb);
            }
            else if (mode.value() == "bayer")
            {
                m_output_mode.store(OutputMode::Bayer);
            }
            else
            {
                response.result(http::status::bad_request);
                response.body() = "invalid mode, expected rgb or bayer";
                return;
            }
        }

        try
        {
            if (gain.has_value())
            {
                m_camera->set_gain(static_cast<float>(gain.value()));
            }
            if (exposure.has_value())
            {
                m_camera->set_exposure_time(static_cast<float>(exposure.value()));
            }
        }
        catch (const std::exception& e)
        {
            response.result(http::status::internal_server_error);
            response.body() = std::string("failed to apply args: ") + e.what();
            return;
        }

        response.result(http::status::ok);
        response.body() = "ok";
    }

    void handle_image_request(http::response<http::vector_body<unsigned char>>& response)
    {
        if (!m_has_frame.load())
        {
            response.result(http::status::service_unavailable);
            response.set(http::field::content_type, "text/plain");
            std::string msg = "frame not ready";
            response.body().assign(msg.begin(), msg.end());
            return;
        }

        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(m_frame_mutex);
            frame = m_latest_frame.clone();
        }

        if (m_output_mode.load() == OutputMode::Rgb)
        {
            cv::Mat converted;
            cv::cvtColor(frame, converted, cv::COLOR_BayerBG2BGR);
            frame = std::move(converted);
        }

        std::vector<unsigned char> buffer;
        if (!cv::imencode(".jpg", frame, buffer))
        {
            response.result(http::status::internal_server_error);
            response.set(http::field::content_type, "text/plain");
            std::string msg = "image encode failed";
            response.body().assign(msg.begin(), msg.end());
            return;
        }

        response.result(http::status::ok);
        response.set(http::field::content_type, "image/jpeg");
        response.body() = std::move(buffer);
    }

    void handle_session(tcp::socket socket)
    {
        beast::flat_buffer buffer;
        http::request<http::string_body> request;
        beast::error_code ec;
        http::read(socket, buffer, request, ec);
        if (ec)
        {
            if (!is_disconnect_error(ec))
            {
                std::cerr << "HTTP read error: " << ec.message() << std::endl;
            }
            return;
        }

        std::string_view target = request.target();
        std::size_t query_pos = target.find('?');
        std::string_view path = target.substr(0, query_pos);

        if (request.method() != http::verb::get)
        {
            http::response<http::string_body> response{http::status::method_not_allowed, request.version()};
            response.set(http::field::content_type, "text/plain");
            response.set(http::field::server, BOOST_BEAST_VERSION_STRING);
            response.body() = "only GET is supported";
            response.prepare_payload();
            safe_write(socket, response);
            beast::error_code shutdown_ec;
            socket.shutdown(tcp::socket::shutdown_send, shutdown_ec);
            return;
        }

        if (path == "/args")
        {
            http::response<http::string_body> response{http::status::ok, request.version()};
            response.set(http::field::content_type, "text/plain");
            response.set(http::field::server, BOOST_BEAST_VERSION_STRING);
            handle_args_request(request, response);
            response.prepare_payload();
            safe_write(socket, response);
        }
        else if (path == "/")
        {
            http::response<http::string_body> response{http::status::ok, request.version()};
            response.set(http::field::content_type, "text/html; charset=utf-8");
            response.set(http::field::server, BOOST_BEAST_VERSION_STRING);

            auto html = load_control_page_html();
            if (!html.has_value())
            {
                response.result(http::status::internal_server_error);
                response.set(http::field::content_type, "text/plain");
                response.body() = "capture_control.html not found";
            }
            else
            {
                response.body() = std::move(html.value());
            }

            response.prepare_payload();
            safe_write(socket, response);
        }
        else if (path == "/image")
        {
            http::response<http::vector_body<unsigned char>> response{http::status::ok, request.version()};
            response.set(http::field::server, BOOST_BEAST_VERSION_STRING);
            handle_image_request(response);
            response.content_length(response.body().size());
            safe_write(socket, response);
        }
        else
        {
            http::response<http::string_body> response{http::status::not_found, request.version()};
            response.set(http::field::content_type, "text/plain");
            response.set(http::field::server, BOOST_BEAST_VERSION_STRING);
            response.body() = "not found";
            response.prepare_payload();
            safe_write(socket, response);
        }

        beast::error_code shutdown_ec;
        socket.shutdown(tcp::socket::shutdown_send, shutdown_ec);
    }

public:
    std::thread start_grabbing_thread() { return std::thread(&CaptureServer::grab_loop, this); }
};

int main(int argc, char** argv)
{
    std::string bind_address = "127.0.0.1";
    uint16_t port = 8080;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--bind" && i + 1 < argc)
        {
            bind_address = argv[++i];
        }
        else if (arg == "--port" && i + 1 < argc)
        {
            port = static_cast<uint16_t>(std::stoi(argv[++i]));
        }
        else if (arg == "--help" || arg == "-h")
        {
            std::cout << "Usage: " << argv[0] << " [--bind <address>] [--port <port>]\n"
                      << "  --bind  监听地址，默认 127.0.0.1（该服务无鉴权，请勿随意暴露到网络）\n"
                      << "  --port  监听端口，默认 8080\n";
            return 0;
        }
    }

    try
    {
        Config::get_config().load_from_file();
    }
    catch (const std::exception&)
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

    asio::io_context io_context;
    CaptureServer server(camera, io_context, bind_address, port);
    auto grabbing_thread = server.start_grabbing_thread();

    server.run();

    grabbing_thread.join();
    return 0;
}
