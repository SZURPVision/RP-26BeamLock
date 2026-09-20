#include <Config.h>
#include <GimbalController.h>

GimbalController::AngleTrajectoryPlanner::AngleTrajectoryPlanner(double initial_angle)
    : m_current_plan{}, m_max_acceleration(Config::get_config().get_tripodhead_config()->m_max_acceleration), m_max_velocity(Config::get_config().get_tripodhead_config()->m_max_velocity)
{
    m_current_plan.m_start_state.m_angle = initial_angle;
    m_current_plan.m_target_angle = initial_angle;
}

double angle_normalize(double angle)
{
    while (angle > 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
    return angle;
}
double angle_difference(double target, double current)
{
    double diff = angle_normalize(target - current);
    return diff;
}
// 编码器读数 → 相对零点（初始位置）的物理角度。编码器无漂移，可用作可信的护栏基准。
double encoder_to_relative_angle(uint16_t encoder, uint16_t encoder_zero, int invert, int per_round)
{
    double angle = static_cast<double>(invert * (static_cast<int>(encoder) - static_cast<int>(encoder_zero))) / static_cast<double>(per_round) * 360.0;
    return angle_normalize(angle);
}

GimbalController::AngleTrajectoryPlanner::State GimbalController::AngleTrajectoryPlanner::get_current_state(CommonTypes::Second now)
{
    Plan plan_copy;
    {
        std::lock_guard<std::mutex> lock(m_plan_mutex);
        plan_copy = m_current_plan;
    }
    auto s = plan_copy.m_start_state;
    if (now < plan_copy.m_start_time)
        return s;

    CommonTypes::Second t = now - plan_copy.m_start_time;
    double acceleration = plan_copy.m_acceleration_sign * m_max_acceleration;
    if (t < plan_copy.m_acceleration_time)
    {
        // 加速阶段
        double angle = s.m_angle + s.m_velocity * t + 0.5 * acceleration * t * t;
        double velocity = s.m_velocity + acceleration * t;
        angle = angle_normalize(angle);
        return State{.m_angle = angle, .m_velocity = velocity, .m_timestamp = now};
    }
    else if (t < plan_copy.m_acceleration_time + plan_copy.m_constant_time)
    {
        // 匀速阶段
        double velocity = plan_copy.m_velocity_peak * plan_copy.m_velocity_sign;
        double angle = plan_copy.m_angle_after_acceleration + velocity * (t - plan_copy.m_acceleration_time);
        angle = angle_normalize(angle);
        return State{.m_angle = angle, .m_velocity = velocity, .m_timestamp = now};
    }
    else if (t < plan_copy.m_acceleration_time + plan_copy.m_constant_time + plan_copy.m_deceleration_time)
    {
        // 减速阶段
        acceleration = -plan_copy.m_acceleration_sign * m_max_acceleration;
        double velocity = plan_copy.m_velocity_peak * plan_copy.m_velocity_sign;

        t = t - plan_copy.m_acceleration_time - plan_copy.m_constant_time;
        double angle = plan_copy.m_angle_after_constant + velocity * t + 0.5 * acceleration * t * t;
        velocity = velocity + acceleration * t;
        angle = angle_normalize(angle);
        return State{.m_angle = angle, .m_velocity = velocity, .m_timestamp = now};
    }
    else
    {
        // 轨迹结束
        return State{.m_angle = plan_copy.m_target_angle, .m_velocity = 0, .m_timestamp = now};
    }
}

void GimbalController::AngleTrajectoryPlanner::update_target_angle(double target_angle, CommonTypes::Second now)
{
    State state = get_current_state(now);
    Plan new_plan;
    new_plan.m_start_state = state;
    new_plan.m_target_angle = target_angle;
    new_plan.m_start_time = now;
    double angle_diff = angle_difference(target_angle, state.m_angle);

    // delta_x=v0*t0+0.5*a*t0^2+vp*t1+0.5*(-a)*t1^2
    // t0=(vp-v0)/a t1=(vp)/a
    // vp=sqrt(a*delta_x+0.5*v0^2)

    // 计算刹车距离判断加速度方向
    double braking_time = std::abs(state.m_velocity) / m_max_acceleration;
    double braking_distance = state.m_velocity * braking_time + 0.5 * (state.m_velocity > 0 ? -1 : 1) * m_max_acceleration * braking_time * braking_time;

    if (state.m_velocity == 0)
    {
        // 静止时直接朝目标加速
        new_plan.m_acceleration_sign = (angle_diff > 0) ? 1 : -1;
        new_plan.m_velocity_sign = new_plan.m_acceleration_sign;
    }
    else if (state.m_velocity * angle_diff >= 0 && std::abs(braking_distance) <= std::abs(angle_diff))
    {
        // 此时加速段的加速度可以与速度方向相同
        new_plan.m_acceleration_sign = (angle_diff > 0) ? 1 : -1;
        // 匀速段速度方向必定与当前相同
        new_plan.m_velocity_sign = (state.m_velocity >= 0) ? 1 : -1;
    }
    else
    {
        // 此时加速段加速度必定与速度方向相反
        new_plan.m_acceleration_sign = (state.m_velocity > 0) ? -1 : 1;
        // 匀速段速度方向必定与当前速度相反，或不存在匀速段
        new_plan.m_velocity_sign = (state.m_velocity >= 0) ? -1 : 1;
    }

    double acceleration = new_plan.m_acceleration_sign * m_max_acceleration;
    // 计算没有速度限制时的峰值速度
    assert(acceleration * angle_diff + 0.5 * state.m_velocity * state.m_velocity >= 0);
    double v_peak = std::sqrt(acceleration * angle_diff + 0.5 * state.m_velocity * state.m_velocity);
    if (v_peak > m_max_velocity)
    {
        // 存在匀速段
        new_plan.m_velocity_peak = m_max_velocity;
        v_peak = m_max_velocity * new_plan.m_velocity_sign;
        new_plan.m_acceleration_time = (v_peak - state.m_velocity) / acceleration;
        assert(new_plan.m_acceleration_time >= 0);
        new_plan.m_angle_after_acceleration = state.m_angle + state.m_velocity * new_plan.m_acceleration_time + 0.5 * acceleration * new_plan.m_acceleration_time * new_plan.m_acceleration_time;

        new_plan.m_deceleration_time = v_peak / acceleration;
        assert(new_plan.m_deceleration_time >= 0);

        double remaining_angle = angle_diff - (new_plan.m_angle_after_acceleration - state.m_angle) - (0.5 * acceleration * new_plan.m_deceleration_time * new_plan.m_deceleration_time);
        new_plan.m_constant_time = remaining_angle / v_peak;
        assert(new_plan.m_constant_time >= 0);
        new_plan.m_angle_after_constant = new_plan.m_angle_after_acceleration + v_peak * new_plan.m_constant_time;
    }
    else
    {
        // 无匀速段
        new_plan.m_velocity_peak = v_peak;
        v_peak = v_peak * new_plan.m_velocity_sign;
        new_plan.m_acceleration_time = (v_peak - state.m_velocity) / acceleration;
        assert(new_plan.m_acceleration_time >= 0);
        new_plan.m_angle_after_acceleration = state.m_angle + state.m_velocity * new_plan.m_acceleration_time + 0.5 * acceleration * new_plan.m_acceleration_time * new_plan.m_acceleration_time;

        new_plan.m_constant_time = 0;
        new_plan.m_angle_after_constant = new_plan.m_angle_after_acceleration;

        acceleration = -acceleration;
        new_plan.m_deceleration_time = -v_peak / acceleration;
        assert(new_plan.m_deceleration_time >= 0);
    }

    new_plan.m_angle_after_acceleration = angle_normalize(new_plan.m_angle_after_acceleration);
    new_plan.m_angle_after_constant = angle_normalize(new_plan.m_angle_after_constant);
    new_plan.m_target_angle = angle_normalize(new_plan.m_target_angle);

    // new_plan.print();
    {
        std::lock_guard<std::mutex> lock(m_plan_mutex);
        m_current_plan = new_plan;
    }
}

void GimbalController::AngleTrajectoryPlanner::update_target_speed(double target_speed, CommonTypes::Second now, double max_constant_time)
{
    State state = get_current_state(now);
    Plan new_plan;
    new_plan.m_start_state = state;
    new_plan.m_start_time = now;
    // 规划加速段，无限时间的匀速段
    new_plan.m_acceleration_sign = (target_speed > state.m_velocity) ? 1 : -1;
    new_plan.m_acceleration_time = std::abs(target_speed - state.m_velocity) / m_max_acceleration;
    new_plan.m_angle_after_acceleration = state.m_angle + state.m_velocity * new_plan.m_acceleration_time + 0.5 * new_plan.m_acceleration_sign * m_max_acceleration * new_plan.m_acceleration_time * new_plan.m_acceleration_time;

    new_plan.m_velocity_sign = (target_speed > 0) ? 1 : -1;
    new_plan.m_velocity_peak = std::abs(target_speed);
    new_plan.m_constant_time = max_constant_time;
    new_plan.m_angle_after_constant = new_plan.m_angle_after_acceleration + target_speed * new_plan.m_constant_time;

    new_plan.m_target_angle = new_plan.m_angle_after_constant;
    new_plan.m_deceleration_time = 0;

    {
        std::lock_guard<std::mutex> lock(m_plan_mutex);
        m_current_plan = new_plan;
    }
}


GimbalController::GimbalController() : m_serial(Config::get_config().get_serial_config()->m_port_name, Config::get_config().get_serial_config()->m_baud_rate) { m_control_thread = std::thread(&GimbalController::work_thread_func, this); }

void GimbalController::work_thread_func()
{
    while (m_serial.get_package_count() < Config::get_config().get_serial_config()->m_reset_pack_count)
    {
        m_serial.send_target_angles(CommonTypes::GimbalAngles{.pitch = 0, .yaw = 0}, SerialHandler::ControlMode::Reset, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto initial_angle = m_serial.get_current_angles();
    m_pitch_planner = std::make_unique<AngleTrajectoryPlanner>(initial_angle.pitch);
    m_yaw_planner = std::make_unique<AngleTrajectoryPlanner>(initial_angle.yaw);
    m_pitch_encoder_zero = m_serial.get_pitch_encoder_angle();
    m_yaw_encoder_zero = m_serial.get_yaw_encoder_angle();

    m_running.store(true);
    auto tripod_config = Config::get_config().get_tripodhead_config();
    while (m_running)
    {
        auto now = TimeUtils::now_seconds();
        auto pitch_state = m_pitch_planner->get_current_state(now);
        auto yaw_state = m_yaw_planner->get_current_state(now);
        CommonTypes::GimbalAngles target_angles{.pitch = pitch_state.m_angle, .yaw = yaw_state.m_angle};
        // 基于编码器的护栏：yaw 陀螺仪存在严重零漂，直接用陀螺仪角度判断不可信。
        // 编码器无漂移，用当前编码器读数实时换算出目标对应的物理相对角作为护栏。
        auto current_angles = m_serial.get_current_angles();
        uint16_t current_pitch_encoder = m_serial.get_pitch_encoder_angle();
        uint16_t current_yaw_encoder = m_serial.get_yaw_encoder_angle();

        // 编码器零点对应的陀螺仪角度（实时校准，抵消陀螺仪零漂）
        double pitch_zero = angle_difference(current_angles.pitch, encoder_to_relative_angle(current_pitch_encoder, m_pitch_encoder_zero, tripod_config->m_invert_pitch_encoder, tripod_config->m_pitch_encoder_per_round));
        double yaw_zero = angle_difference(current_angles.yaw, encoder_to_relative_angle(current_yaw_encoder, m_yaw_encoder_zero, tripod_config->m_invert_yaw_encoder, tripod_config->m_yaw_encoder_per_round));

        // 目标角对应的物理相对角（相对初始位置，即编码器零点）
        CommonTypes::GimbalAngles relative_angles{
            .pitch = angle_difference(target_angles.pitch, initial_angle.pitch), // pitch有重力矫正，零漂小很多，直接用陀螺仪角度计算相对角即可
            .yaw = angle_difference(target_angles.yaw, yaw_zero),
        };
        if (abs(relative_angles.pitch) > tripod_config->m_max_pitch_angle || abs(relative_angles.yaw) > tripod_config->m_max_yaw_angle)
        {
            FoxGloveServer::get_server().log_message("Warning: Gimbal target angles exceed limits. Pitch: " + std::to_string(relative_angles.pitch) + ", Yaw: " + std::to_string(relative_angles.yaw), FoxGloveServer::LogLevel::WARNING);
            // 如果角度超出范围，直接跳过发送指令，防止损坏云台
            // 设置flag
            m_angle_exceeded.store(true);
        }
        else
        {
            m_serial.send_target_angles(target_angles);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

GimbalController::~GimbalController()
{
    m_running.store(false);
    if (m_control_thread.joinable())
        m_control_thread.join();
}

void GimbalController::set_target_angles(CommonTypes::GimbalAngles target_angles, CommonTypes::Second now)
{
    if (m_running.load())
    {
        FoxGloveServer::get_server().log_message("Setting target angles: pitch = " + std::to_string(target_angles.pitch) + ", yaw = " + std::to_string(target_angles.yaw), FoxGloveServer::LogLevel::DEBUG);
        m_pitch_planner->update_target_angle(target_angles.pitch, now);
        m_yaw_planner->update_target_angle(target_angles.yaw, now);
    }
}

void GimbalController::set_target_speeds(CommonTypes::GimbalAngles target_speeds, CommonTypes::Second now, double max_constant_time)
{
    if (m_running.load())
    {
        FoxGloveServer::get_server().log_message("Setting target speeds: pitch = " + std::to_string(target_speeds.pitch) + ", yaw = " + std::to_string(target_speeds.yaw), FoxGloveServer::LogLevel::DEBUG);
        m_pitch_planner->update_target_speed(target_speeds.pitch, now, max_constant_time);
        m_yaw_planner->update_target_speed(target_speeds.yaw, now, max_constant_time);
    }
}

void GimbalController::set_target_encoder_angles(CommonTypes::GimbalAngles target_angles, CommonTypes::Second now)
{
    if (m_running.load())
    {
        FoxGloveServer::get_server().log_message("Setting target encoder angles: pitch = " + std::to_string(target_angles.pitch) + ", yaw = " + std::to_string(target_angles.yaw), FoxGloveServer::LogLevel::DEBUG);
        auto current_angles = m_serial.get_current_angles();
        uint16_t current_pitch_encoder = m_serial.get_pitch_encoder_angle();
        uint16_t current_yaw_encoder = m_serial.get_yaw_encoder_angle();

        // 计算出编码器零点对应的陀螺仪角度值
        auto tripod_config = Config::get_config().get_tripodhead_config();
        double pitch_zero = angle_difference(current_angles.pitch, encoder_to_relative_angle(current_pitch_encoder, m_pitch_encoder_zero, tripod_config->m_invert_pitch_encoder, tripod_config->m_pitch_encoder_per_round));
        double yaw_zero = angle_difference(current_angles.yaw, encoder_to_relative_angle(current_yaw_encoder, m_yaw_encoder_zero, tripod_config->m_invert_yaw_encoder, tripod_config->m_yaw_encoder_per_round));

        FoxGloveServer::get_server().log_message("Calculated encoder zeros in angles: pitch_zero = " + std::to_string(pitch_zero) + ", yaw_zero = " + std::to_string(yaw_zero), FoxGloveServer::LogLevel::DEBUG);

        // m_pitch_planner->update_target_angle(angle_normalize(pitch_zero + target_angles.pitch), now);
        m_pitch_planner->update_target_angle(target_angles.pitch, now); // 直接使用pitch角度，pitch有重力矫正，零漂小很多
        m_yaw_planner->update_target_angle(angle_normalize(yaw_zero + target_angles.yaw), now);
    }
}