#include <MVSCamera.h>

CameraManager::CameraInfoList::CameraInfoList(const std::shared_ptr<MV_CC_DEVICE_INFO_LIST>& device_info_list) : m_device_info_list(device_info_list) {}

size_t CameraManager::CameraInfoList::size() const { return m_device_info_list->nDeviceNum; }

bool CameraManager::CameraInfoList::is_empty() const { return !m_device_info_list || m_device_info_list->nDeviceNum == 0; }

std::string CameraManager::CameraInfoList::get_model_name(size_t index) const
{
    if (!m_device_info_list || index >= m_device_info_list->nDeviceNum)
        throw std::out_of_range("CameraInfoList index out of range");
    if (m_device_info_list->pDeviceInfo[index]->nTLayerType == MV_USB_DEVICE)
        return std::string(reinterpret_cast<char*>(m_device_info_list->pDeviceInfo[index]->SpecialInfo.stUsb3VInfo.chModelName));
    else if (m_device_info_list->pDeviceInfo[index]->nTLayerType == MV_GIGE_DEVICE)
        return std::string(reinterpret_cast<char*>(m_device_info_list->pDeviceInfo[index]->SpecialInfo.stGigEInfo.chModelName));
    else
        return std::string("Unknown");
}

std::string CameraManager::CameraInfoList::get_serial_number(size_t index) const
{
    if (!m_device_info_list || index >= m_device_info_list->nDeviceNum)
        throw std::out_of_range("CameraInfoList index out of range");
    if (m_device_info_list->pDeviceInfo[index]->nTLayerType == MV_USB_DEVICE)
        return std::string(reinterpret_cast<char*>(m_device_info_list->pDeviceInfo[index]->SpecialInfo.stUsb3VInfo.chSerialNumber));
    else if (m_device_info_list->pDeviceInfo[index]->nTLayerType == MV_GIGE_DEVICE)
        return std::string(reinterpret_cast<char*>(m_device_info_list->pDeviceInfo[index]->SpecialInfo.stGigEInfo.chSerialNumber));
    else
        return std::string("Unknown");
}