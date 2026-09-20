#include <MVSCamera.h>

Camera::HandleManager::Handle Camera::HandleManager::get_camera() { return Handle(m_handle, m_camera_lock); }
Camera::HandleManager::Handle Camera::HandleManager::get_buffer() { return Handle(m_handle, m_buffer_lock); }
Camera::HandleManager::Handle::Handle(void* p, std::mutex& m) : m_handle(p), m_lock(m) {}

Camera::HandleManager::HandleManager(void* h) : m_handle(h) {}
Camera::HandleManager::~HandleManager()
{
    if (m_handle)
    {
        CameraException::check_error(MV_CC_DestroyHandle(m_handle), "~HandleManager", "MV_CC_DestroyHandle");
        m_handle = nullptr;
    }
}
