#pragma once
#include <LightRow.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

namespace TargetDetecter
{
class Target
{
public:
    using Ptr = std::shared_ptr<Target>;
    using Feature = std::vector<cv::Point2f>;

private:
    LightRow::Ptr m_blobs_up;
    LightRow::Ptr m_blobs_down;
    double m_score;
    bool m_valid;

    void check_valid();
    void calculate_score();

public:
    Target(const LightRow::Ptr& blobs_up, const LightRow::Ptr& blobs_down);
    double score() const;
    bool is_valid() const;
    // 返回6个特征点：上面3个灯条的中心点和下面3个灯条的中心点
    Feature feature() const;
};
} // namespace TargetDetecter