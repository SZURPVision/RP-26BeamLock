#include <TimeUtils.h>
#include <thread>

namespace TimeUtils
{
CommonTypes::TimePoint now() { return CommonTypes::Clock::now(); }

CommonTypes::Second now_seconds() { return std::chrono::duration<double>(CommonTypes::Clock::now().time_since_epoch()).count(); }

CommonTypes::Nanosecond now_nanoseconds() { return std::chrono::duration_cast<std::chrono::nanoseconds>(CommonTypes::Clock::now().time_since_epoch()).count(); }

CommonTypes::Second elapsed_seconds(CommonTypes::TimePoint start, CommonTypes::TimePoint end) { return std::chrono::duration<double>(end - start).count(); }

void sleep_for_seconds(CommonTypes::Second seconds) { std::this_thread::sleep_for(std::chrono::duration<double>(seconds)); }

void sleep_for_milliseconds(CommonTypes::Millisecond milliseconds) { std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds)); }
} // namespace TimeUtils
