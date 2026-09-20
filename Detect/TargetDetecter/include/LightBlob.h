#pragma once
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
namespace TargetDetecter
{
class LightBlob
{
public:
    using Ptr = std::shared_ptr<LightBlob>;

private:
    // 轮廓信息几乎不可信，这里只做保存
    std::vector<cv::Point> m_contours;
    cv::RotatedRect m_bounding_box;
    cv::Point2f m_centroid;
    double m_area;
    bool m_valid;

    void check_valid();

public:
    // 需要std::move
    explicit LightBlob(std::vector<cv::Point>&& contours);

    double area() const { return m_area; }
    bool is_valid() const { return m_valid; }
    cv::Point2f center() const { return m_bounding_box.center; }
    cv::Point2f centroid() const { return m_centroid; }
};
} // namespace TargetDetecter