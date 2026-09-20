#include <BiasKF.h>
#include <algorithm>

BiasKalmanFilter::BiasKalmanFilter() : m_x(Vec2::Zero()), m_p(Mat2::Identity()), m_r(Mat2::Identity()), m_q_density(1e-6), m_last_update_time(0.0), m_has_last_update_time(false)
{
    m_r(0, 0) = 2;
    m_r(1, 1) = 2;
}

void BiasKalmanFilter::set_process_noise_density(double q_px2_per_s) { m_q_density = std::max(0.0, q_px2_per_s); }

void BiasKalmanFilter::set_measurement_noise(double rx_px2, double ry_px2)
{
    m_r.setZero();
    m_r(0, 0) = std::max(1e-9, rx_px2);
    m_r(1, 1) = std::max(1e-9, ry_px2);
}

void BiasKalmanFilter::reset(const Vec2& initial_bias, const Mat2& initial_cov)
{
    m_x = initial_bias;
    m_p = initial_cov;
    m_p(0, 0) = std::max(1e-9, m_p(0, 0));
    m_p(1, 1) = std::max(1e-9, m_p(1, 1));
    m_last_update_time = 0.0;
    m_has_last_update_time = false;
}

void BiasKalmanFilter::predict(CommonTypes::Second timestamp_s)
{
    if (!m_has_last_update_time)
    {
        m_last_update_time = timestamp_s;
        m_has_last_update_time = true;
        return;
    }

    if (timestamp_s <= m_last_update_time)
    {
        return;
    }

    const double dt = timestamp_s - m_last_update_time;
    const Mat2 q = Mat2::Identity() * (m_q_density * dt);
    m_p += q;
    m_last_update_time = timestamp_s;
}

void BiasKalmanFilter::update(const Vec2& measurement_bias)
{
    const Mat2 s = m_p + m_r;
    const Mat2 k = m_p * s.inverse();

    const Vec2 innovation = measurement_bias - m_x;
    m_x += k * innovation;

    const Mat2 i = Mat2::Identity();
    m_p = (i - k) * m_p;
}

void BiasKalmanFilter::step(const Vec2& measurement_bias, CommonTypes::Second timestamp_s)
{
    predict(timestamp_s);
    update(measurement_bias);
}

const BiasKalmanFilter::Vec2& BiasKalmanFilter::state() const { return m_x; }

const BiasKalmanFilter::Mat2& BiasKalmanFilter::covariance() const { return m_p; }
