#include <MVSCamera.h>

CameraException::CameraException(const std::string& message) : std::runtime_error(message) {}

CameraException::CameraException(const std::string& message, const std::string& function_name) : std::runtime_error("Error in function '" + function_name + "': " + message) {}

bool CameraException::check_error(int status_code, std::string function_name, std::string step_name)
{
    if (status_code != MV_OK)
    {
        std::ostringstream oss;
        oss << "Camera error in function '" << function_name << "' at step '" << step_name << "' with status code: 0x" << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << static_cast<unsigned int>(status_code);
        throw CameraException(oss.str());
        return false;
    }
    return true;
}