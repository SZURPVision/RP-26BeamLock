#pragma once
#include <MvCameraControl.h>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>

class SDKContext
{
private:
    SDKContext() { initialize_sdk(); }

    static void initialize_sdk()
    {
        bool expected = false;
        if (s_initialized.compare_exchange_strong(expected, true))
        {
            MV_CC_Initialize();
            std::cout << "Camera SDK Initialized" << std::endl;
        }
    }

    static void finalize_sdk()
    {
        bool expected = true;
        if (s_initialized.compare_exchange_strong(expected, false))
        {
            std::cout << "Camera SDK Finalized" << std::endl;
            MV_CC_Finalize();
        }
    }

public:
    static std::shared_ptr<SDKContext> get_context()
    {
        std::call_once(s_once, [] { s_instance.reset(new SDKContext()); });
        return s_instance;
    }
    ~SDKContext() { finalize_sdk(); }
    SDKContext(const SDKContext&) = delete;
    SDKContext& operator=(const SDKContext&) = delete;
    SDKContext(SDKContext&&) = delete;
    SDKContext& operator=(SDKContext&&) = delete;

private:
    inline static std::once_flag s_once;
    inline static std::atomic_bool s_initialized{false};
    inline static std::shared_ptr<SDKContext> s_instance;
};

class Camera
{
public:
    static const int image_node_num = 4;

    class HandleManager
    {
    private:
        void* m_handle = nullptr;
        std::mutex m_camera_lock;
        std::mutex m_buffer_lock;

    public:
        struct Handle
        {
            void* m_handle;
            std::unique_lock<std::mutex> m_lock;

            Handle(void* p, std::mutex& m);
        };

        explicit HandleManager(void* h);
        ~HandleManager();

        HandleManager(const HandleManager&) = delete;
        HandleManager& operator=(const HandleManager&) = delete;
        HandleManager(HandleManager&&) = delete;
        HandleManager& operator=(HandleManager&&) = delete;

        Handle get_camera();
        Handle get_buffer();
    };

private:
    std::unique_ptr<HandleManager> m_handle_manager;
    std::shared_ptr<SDKContext> m_sdk_context;
    bool m_is_open = false;
    bool m_is_streaming = false;

public:
    explicit Camera(void* handle);
    ~Camera();
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    Camera(Camera&&) = delete;
    Camera& operator=(Camera&&) = delete;

    friend class CameraManager;
    void start_grabbing();
    void stop_grabbing();

    void get_frame(cv::Mat& frame, int timeout_ms = 1000);

    void get_int_value(const char* name, int& value);
    void set_int_value(const char* name, const int& value);
    void get_enum_value(const char* name, unsigned int& value);
    void set_enum_value(const char* name, const unsigned int& value);
    void get_bool_value(const char* name, bool& value);
    void set_bool_value(const char* name, const bool& value);
    void get_float_value(const char* name, float& value);
    void set_float_value(const char* name, const float& value);
    void get_string_value(const char* name, char* value, int size);
    void set_string_value(const char* name, const char* value);
    void set_command_value(const char* name);

    void set_gain(const float& gain);
    void set_exposure_time(const float& exposure_time);
};

class CameraManager
{
private:
    std::shared_ptr<SDKContext> m_sdk_context;

public:
    class CameraInfoList
    {
        friend class CameraManager;

    private:
        std::shared_ptr<MV_CC_DEVICE_INFO_LIST> m_device_info_list;
        explicit CameraInfoList(const std::shared_ptr<MV_CC_DEVICE_INFO_LIST>& device_info_list);

    public:
        size_t size() const;
        bool is_empty() const;
        std::string get_model_name(size_t index) const;
        std::string get_serial_number(size_t index) const;
    };

    CameraManager() : m_sdk_context(SDKContext::get_context()){};

    CameraInfoList list_cameras();

    std::shared_ptr<Camera> create_camera(const CameraInfoList& info, int id);
};

class CameraException : public std::runtime_error
{
    static bool check_error(int status_code, std::string function_name = "", std::string step_name = "");
    friend class CameraManager;
    friend class Camera;

public:
    explicit CameraException(const std::string& message);
    CameraException(const std::string& message, const std::string& function_name);
};