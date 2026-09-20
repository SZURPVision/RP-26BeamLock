#pragma once
#include <CommonTypes.h>
#include <Config.h>
#include <FoxGloveServer.h>

enum class ScanMode
{
    Square,
    Line
};

// 方型扫描
inline CommonTypes::GimbalAngles get_square_scan_delta_angles(CommonTypes::Second time_seconds, double max_yaw_delta, double max_pitch_delta, CommonTypes::Second scan_period)
{
    double total_delta = 4 * (max_yaw_delta + max_pitch_delta);
    double delta = std::fmod(time_seconds, scan_period) / scan_period * total_delta;
    if (delta < 2 * max_yaw_delta)
        return CommonTypes::GimbalAngles{
            .pitch = -max_pitch_delta,
            .yaw = delta - max_yaw_delta,
        };
    else if (delta < 2 * max_yaw_delta + 2 * max_pitch_delta)
        return CommonTypes::GimbalAngles{
            .pitch = (delta - 2 * max_yaw_delta) - max_pitch_delta,
            .yaw = max_yaw_delta,
        };
    else if (delta < 4 * max_yaw_delta + 2 * max_pitch_delta)
        return CommonTypes::GimbalAngles{
            .pitch = max_pitch_delta,
            .yaw = max_yaw_delta - (delta - 2 * max_yaw_delta - 2 * max_pitch_delta),
        };
    else
        return CommonTypes::GimbalAngles{
            .pitch = max_pitch_delta - (delta - 4 * max_yaw_delta - 2 * max_pitch_delta),
            .yaw = -max_yaw_delta,
        };
}

// 逐行扫描：yaw方向来回扫，pitch按行步进
inline CommonTypes::GimbalAngles get_line_scan_delta_angles(CommonTypes::Second time_seconds, double yaw_min, double yaw_max, double pitch_min, double pitch_max, double scan_speed, double pitch_step)
{
    if (scan_speed <= 0.0)
        return CommonTypes::GimbalAngles{.pitch = pitch_min, .yaw = yaw_min};

    if (yaw_min > yaw_max)
        std::swap(yaw_min, yaw_max);
    if (pitch_min > pitch_max)
        std::swap(pitch_min, pitch_max);

    double yaw_span = yaw_max - yaw_min;
    double pitch_span = pitch_max - pitch_min;
    if (yaw_span <= 0.0)
        return CommonTypes::GimbalAngles{.pitch = pitch_min, .yaw = yaw_min};

    double sweep_time = yaw_span / scan_speed;
    if (sweep_time <= 0.0)
        return CommonTypes::GimbalAngles{.pitch = pitch_min, .yaw = yaw_min};

    double pitch_step_clamped = std::max(std::abs(pitch_step), 1e-6);
    int line_count = (pitch_span <= 0.0) ? 1 : (static_cast<int>(std::floor(pitch_span / pitch_step_clamped)) + 1);
    line_count = std::max(1, line_count);

    int line_index = static_cast<int>(std::floor(time_seconds / sweep_time));
    int pitch_cycle = std::max(1, 2 * line_count - 2);
    int pitch_line_index = line_index % pitch_cycle;
    if (pitch_line_index >= line_count)
        pitch_line_index = pitch_cycle - pitch_line_index;

    double pitch = pitch_min + pitch_line_index * pitch_step_clamped;
    pitch = std::clamp(pitch, pitch_min, pitch_max);

    double t_in_line = std::fmod(time_seconds, sweep_time);
    bool is_forward = (line_index % 2 == 0);
    double progress = t_in_line / sweep_time;
    double yaw = is_forward ? (yaw_min + progress * yaw_span) : (yaw_max - progress * yaw_span);

    return CommonTypes::GimbalAngles{.pitch = pitch, .yaw = yaw};
}

// 同心圆扫描，需要传入初始时间和固定速度
// 半径先是不断增加，然后不断减少，再增加......
inline Eigen::Vector2d get_concentric_scan_delta_pixel(CommonTypes::Second time_seconds, double initial_radius, double radius_delta, double max_radius, double scan_speed, CommonTypes::Second start_time_seconds)
{
    CommonTypes::Second elapsed_time = time_seconds - start_time_seconds;
    // 计算一个周期要多久
    // 如果initial=1,delta=1,max=3,则radius的变化是1,2,3,2,1,2,3,2,1,...，一个周期是4个半径
    // 如果max-initial不是delta的整数倍，将max调整为超过max的最小的initial+N*delta的值，这样周期内的半径数量就是一个整数
    int radius_count = 0;
    radius_count = static_cast<int>(std::ceil((max_radius - initial_radius) / radius_delta));
    // 特判radius_count为0的情况，此时半径不变，直接绕着一个圆扫描
    if (radius_count == 0)
    {
        double scan_period = (2 * M_PI * initial_radius) / scan_speed;
        double angle = std::fmod(elapsed_time, scan_period) / scan_period * 2 * M_PI;
        return Eigen::Vector2d{
            initial_radius * std::sin(angle),
            initial_radius * std::cos(angle),
        };
    }
    // 扫描周期是一个等差数列，半径的变化是initial, initial+delta, initial+2*delta, ...,
    // 所以扫描时间是(2*pi*radius)/scan_speed的等差数列，公差是(2*pi*delta)/scan_speed
    // 等差数列求和公式是n/2*(2*a1+(n-1)*d)
    double scan_period_1 = (radius_count / 2.0) * (2 * (2 * M_PI * initial_radius / scan_speed) + (radius_count - 1) * (2 * M_PI * radius_delta / scan_speed));
    // 这个时间是对应 1 2 的，还需要补上后面的 3 2 的时间
    double scan_period_2 = scan_period_1 * 2 + radius_count * ((2 * M_PI * radius_delta / scan_speed));

    CommonTypes::Second time_in_cycle = std::fmod(elapsed_time, scan_period_2);
    if (time_in_cycle <= scan_period_1)
    {
        // 对应半径 1 2 的情况
        double radius = initial_radius;
        CommonTypes::Second time_accumulated = 0;
        for (int i = 1; i <= radius_count; ++i)
        {
            CommonTypes::Second current_scan_time = (2 * M_PI * radius / scan_speed);
            if (time_in_cycle <= time_accumulated + current_scan_time)
            {
                // 还在扫描这个半径
                double angle = (time_in_cycle - time_accumulated) / current_scan_time * 2 * M_PI;
                return Eigen::Vector2d(radius * std::sin(angle), radius * std::cos(angle));
            }
            time_accumulated += current_scan_time;
            radius += radius_delta;
        }
    }
    else
    {
        // 对应半径 3 2 的情况
        double radius = initial_radius + radius_count * radius_delta;
        CommonTypes::Second time_accumulated = scan_period_1;
        for (int i = 1; i <= radius_count; ++i)
        {
            CommonTypes::Second current_scan_time = (2 * M_PI * radius / scan_speed);
            if (time_in_cycle <= time_accumulated + current_scan_time)
            {
                // 还在扫描这个半径
                double angle = (time_in_cycle - time_accumulated) / current_scan_time * 2 * M_PI;
                return Eigen::Vector2d(radius * std::sin(angle), radius * std::cos(angle));
            }
            time_accumulated += current_scan_time;
            radius -= radius_delta;
        }
    }
    // 理论上不会走到这里，因为上面的循环应该能覆盖所有情况
    FoxGloveServer::get_server().log_message("Fail to calculate scan pixel.", FoxGloveServer::LogLevel::WARNING);
    return Eigen::Vector2d::Zero();
}