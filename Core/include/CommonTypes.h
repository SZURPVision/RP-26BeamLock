#pragma once
#include <chrono>
#include <Eigen/Eigen>

namespace CommonTypes
{

using Clock = std::chrono::system_clock;
using TimePoint = Clock::time_point;
using Second = double;
using Millisecond = int64_t;
using Nanosecond = uint64_t;

using Point = Eigen::Vector3d;
struct GimbalAngles
{
    double pitch; // NOLINT(readability-identifier-naming)
    double yaw;   // NOLINT(readability-identifier-naming)
};

// NOLINTBEGIN(readability-identifier-naming)
#pragma pack(push, 1)
struct NetworkPack
{
    uint32_t frame_id;
    double x;
    double y;
    double z;
    bool allow_counter;
};
#pragma pack(pop)
// NOLINTEND(readability-identifier-naming)

} // namespace CommonTypes