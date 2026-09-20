#pragma once
#include <CommonTypes.h>
#include <Config.h>
#include <FoxGloveServer.h>
#include <Serial.h>
#include <TimeUtils.h>
#include <atomic>
#include <memory>
#include <thread>

class GimbalController
{
private:
    class AngleTrajectoryPlanner
    {
    public:
        struct State
        {
            double m_angle = 0;
            double m_velocity = 0;
            CommonTypes::Second m_timestamp = 0;
        };
        struct Plan
        {
            State m_start_state;
            double m_target_angle = 0;
            CommonTypes::Second m_start_time = 0;

            double m_acceleration_sign = 0;
            CommonTypes::Second m_acceleration_time = 0;
            double m_angle_after_acceleration = 0;
            double m_velocity_peak = 0;
            double m_velocity_sign = 0;
            CommonTypes::Second m_constant_time = 0;
            double m_angle_after_constant = 0;
            CommonTypes::Second m_deceleration_time = 0;

            void print() const
            {
                std::cout << "Plan: \n target_angle=" << m_target_angle << "\n start_time=" << m_start_time << "\n acceleration_sign=" << m_acceleration_sign << "\n acceleration_time=" << m_acceleration_time
                          << "\n angle_after_acceleration=" << m_angle_after_acceleration << "\n velocity_peak=" << m_velocity_peak << "\n velocity_sign=" << m_velocity_sign << "\n constant_time=" << m_constant_time
                          << "\n angle_after_constant=" << m_angle_after_constant << "\n deceleration_time=" << m_deceleration_time << std::endl;
            }
        };

    private:
        const double m_max_acceleration; // deg/s^2
        const double m_max_velocity;     // deg/s
        std::mutex m_plan_mutex;
        Plan m_current_plan;

    public:
        explicit AngleTrajectoryPlanner(double initial_angle = 0);

        State get_current_state(CommonTypes::Second now);
        void update_target_angle(double target_angle, CommonTypes::Second now);
        void update_target_speed(double target_speed, CommonTypes::Second now, double max_constant_time = std::numeric_limits<double>::infinity());
    };

    std::unique_ptr<AngleTrajectoryPlanner> m_pitch_planner, m_yaw_planner;

    uint16_t m_pitch_encoder_zero = 0;
    uint16_t m_yaw_encoder_zero = 0;

    SerialHandler m_serial;
    std::thread m_control_thread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_angle_exceeded{false};

    void work_thread_func();

public:
    GimbalController();
    ~GimbalController();
    GimbalController(const GimbalController&) = delete;
    GimbalController& operator=(const GimbalController&) = delete;
    GimbalController(GimbalController&&) = delete;
    GimbalController& operator=(GimbalController&&) = delete;

    bool is_running() const { return m_running.load(); }
    bool consume_angle_exceeded_flag() { return m_angle_exceeded.exchange(false); }
    void set_target_angles(CommonTypes::GimbalAngles target_angles, CommonTypes::Second now);
    void set_target_speeds(CommonTypes::GimbalAngles target_speeds, CommonTypes::Second now, double max_constant_time = std::numeric_limits<double>::infinity());
    void set_target_encoder_angles(CommonTypes::GimbalAngles target_angles, CommonTypes::Second now);

    CommonTypes::GimbalAngles get_current_angles() const { return m_serial.get_current_angles(); }
};