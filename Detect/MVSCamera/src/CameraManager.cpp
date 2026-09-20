#include <MVSCamera.h>

CameraManager::CameraInfoList CameraManager::list_cameras()
{
    std::shared_ptr<MV_CC_DEVICE_INFO_LIST> device_list = std::make_shared<MV_CC_DEVICE_INFO_LIST>();
    memset(device_list.get(), 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    CameraException::check_error(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, device_list.get()), "list_cameras", "MV_CC_EnumDevices");
    return CameraInfoList(device_list);
}

std::shared_ptr<Camera> CameraManager::create_camera(const CameraInfoList& info, int id)
{
    if (id < 0 || static_cast<size_t>(id) >= info.size())
        throw std::out_of_range("Camera ID out of range");

    void* handle = nullptr;
    CameraException::check_error(MV_CC_CreateHandle(&handle, info.m_device_info_list->pDeviceInfo[id]), "create_camera", "MV_CC_CreateHandle");
    return std::make_shared<Camera>(handle);
}