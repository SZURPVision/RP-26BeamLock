#include <MVSCamera.h>


Camera::Camera(void* handler) : m_is_streaming(false), m_sdk_context(SDKContext::get_context())
{
    m_handle_manager = std::make_unique<HandleManager>(handler);
    HandleManager::Handle handle = m_handle_manager->get_camera();
    CameraException::check_error(MV_CC_OpenDevice(handle.m_handle), "create_camera", "MV_CC_OpenDevice");
    CameraException::check_error(MV_CC_SetImageNodeNum(handle.m_handle, image_node_num), "create_camera", "MV_CC_SetImageNodeNum");
    CameraException::check_error(MV_CC_SetEnumValue(handle.m_handle, "TriggerMode", MV_TRIGGER_MODE_OFF), "create_camera", "MV_CC_SetEnumValue");
    CameraException::check_error(MV_CC_SetEnumValue(handle.m_handle, "PixelFormat", PixelType_Gvsp_BayerRG8), "create_camera", "MV_CC_SetEnumValue");
}

Camera::~Camera()
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    CameraException::check_error(MV_CC_CloseDevice(handle.m_handle), "destroy_camera", "MV_CC_CloseDevice");
}

void Camera::start_grabbing()
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    CameraException::check_error(MV_CC_StartGrabbing(handle.m_handle), "start_grabbing", "MV_CC_StartGrabbing");
    m_is_streaming = true;
}
void Camera::stop_grabbing()
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    m_is_streaming = false;
    CameraException::check_error(MV_CC_StopGrabbing(handle.m_handle), "stop_grabbing", "MV_CC_StopGrabbing");
}

void Camera::get_frame(cv::Mat& frame, int timeout_ms)
{
    HandleManager::Handle handle = m_handle_manager->get_buffer();
    MV_FRAME_OUT image_info;
    CameraException::check_error(MV_CC_GetImageBuffer(handle.m_handle, &image_info, timeout_ms), "get_frame", "MV_CC_GetImageBuffer");

    cv::Mat img(image_info.stFrameInfo.nHeight, image_info.stFrameInfo.nWidth, CV_8UC1, image_info.pBufAddr);
    frame = img.clone();

    CameraException::check_error(MV_CC_FreeImageBuffer(handle.m_handle, &image_info), "get_frame", "MV_CC_FreeImageBuffer");
}

void Camera::get_int_value(const char* name, int& value)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    MVCC_INTVALUE_EX v;
    CameraException::check_error(MV_CC_GetIntValueEx(handle.m_handle, name, &v), "get_int_value", "MV_CC_GetIntValueEx");
    value = v.nCurValue;
}
void Camera::set_int_value(const char* name, const int& value)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    CameraException::check_error(MV_CC_SetIntValueEx(handle.m_handle, name, value), "set_int_value", "MV_CC_SetIntValueEx");
}
void Camera::get_enum_value(const char* name, unsigned int& value)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    MVCC_ENUMVALUE_EX v;
    CameraException::check_error(MV_CC_GetEnumValueEx(handle.m_handle, name, &v), "get_enum_value", "MV_CC_GetEnumValueEx");
    value = v.nCurValue;
}
void Camera::set_enum_value(const char* name, const unsigned int& value)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    CameraException::check_error(MV_CC_SetEnumValue(handle.m_handle, name, value), "set_enum_value", "MV_CC_SetEnumValue");
}
void Camera::get_bool_value(const char* name, bool& value)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    bool v = false;
    CameraException::check_error(MV_CC_GetBoolValue(handle.m_handle, name, &v), "get_bool_value", "MV_CC_GetBoolValue");
    value = v;
}
void Camera::set_bool_value(const char* name, const bool& value)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    CameraException::check_error(MV_CC_SetBoolValue(handle.m_handle, name, value), "set_bool_value", "MV_CC_SetBoolValue");
}
void Camera::get_float_value(const char* name, float& value)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    MVCC_FLOATVALUE v;
    CameraException::check_error(MV_CC_GetFloatValue(handle.m_handle, name, &v), "get_float_value", "MV_CC_GetFloatValue");
    value = v.fCurValue;
}
void Camera::set_float_value(const char* name, const float& value)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    CameraException::check_error(MV_CC_SetFloatValue(handle.m_handle, name, value), "set_float_value", "MV_CC_SetFloatValue");
}
void Camera::get_string_value(const char* name, char* value, int size)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    MVCC_STRINGVALUE v;
    CameraException::check_error(MV_CC_GetStringValue(handle.m_handle, name, &v), "get_string_value", "MV_CC_GetStringValue");
    strncpy(value, v.chCurValue, size);
}
void Camera::set_string_value(const char* name, const char* value)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    CameraException::check_error(MV_CC_SetStringValue(handle.m_handle, name, value), "set_string_value", "MV_CC_SetStringValue");
}
void Camera::set_command_value(const char* name)
{
    HandleManager::Handle handle = m_handle_manager->get_camera();
    CameraException::check_error(MV_CC_SetCommandValue(handle.m_handle, name), "set_command_value", "MV_CC_SetCommandValue");
}

void Camera::set_gain(const float& gain) { set_float_value("Gain", gain); }
void Camera::set_exposure_time(const float& exposure_time) { set_float_value("ExposureTime", exposure_time); }