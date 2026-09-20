#pragma once

#include <CommonTypes.h>
#include <chrono>
#include <cstdint>

namespace TimeUtils
{

CommonTypes::TimePoint now();
CommonTypes::Second now_seconds();
CommonTypes::Nanosecond now_nanoseconds();
CommonTypes::Second elapsed_seconds(CommonTypes::TimePoint start, CommonTypes::TimePoint end);
void sleep_for_seconds(CommonTypes::Second seconds);
void sleep_for_milliseconds(CommonTypes::Millisecond milliseconds);

} // namespace TimeUtils
