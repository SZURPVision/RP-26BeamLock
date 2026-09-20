#pragma once
#include <LightBlob.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
namespace TargetDetecter
{
class LightRow
{
public:
    using Ptr = std::shared_ptr<LightRow>;

private:
    std::vector<LightBlob::Ptr> m_blobs;
    bool m_valid;
    double m_score;

    cv::Point2f m_center;
    void check_valid();
    void calculate_score();

public:
    explicit LightRow(const std::vector<LightBlob::Ptr>& blobs);

    bool is_valid() const;
    double score() const;
    const cv::Point2f& center() const;
    const std::vector<LightBlob::Ptr>& blobs() const;
};
} // namespace TargetDetecter