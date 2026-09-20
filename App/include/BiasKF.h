#pragma once
#include <CommonTypes.h>
#include <Eigen/Eigen>

class BiasKalmanFilter
{
public:
    // 状态向量 x = [delta_x_px, delta_y_px]^T
    // 状态模型: x_k = x_{k-1} + w_k
    // 观测模型: z_k = x_k + v_k
    using Vec2 = Eigen::Vector2d;
    using Mat2 = Eigen::Matrix2d;

    BiasKalmanFilter();

    // 设置过程噪声强度(单位: px^2/s)
    void set_process_noise_density(double q_px2_per_s);

    // 设置观测噪声方差(单位: px^2)
    void set_measurement_noise(double rx_px2, double ry_px2);

    // 重置滤波器并设置初始状态，同时清空“上次更新时间”
    void reset(const Vec2& initial_bias = Vec2::Zero(), const Mat2& initial_cov = Mat2::Identity());

    // 仅预测一步，传入“当前时间戳(秒)”
    // 内部自动使用与上次更新时间的差值 dt 进行离散化
    void predict(CommonTypes::Second timestamp_s);

    // 用观测值更新
    void update(const Vec2& measurement_bias);

    // 预测 + 更新组合，传入“当前时间戳(秒)”
    void step(const Vec2& measurement_bias, CommonTypes::Second timestamp_s);

    const Vec2& state() const;
    const Mat2& covariance() const;

private:
    Vec2 m_x;
    Mat2 m_p;
    Mat2 m_r;

    // 随机游走过程噪声密度 q，离散化后 Q = q * dt * I
    double m_q_density;

    CommonTypes::Second m_last_update_time;
    bool m_has_last_update_time;
};
