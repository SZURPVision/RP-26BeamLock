#pragma once
#include <CommonTypes.h>
#include <LightBlob.h>
#include <Target.h>
#include <opencv2/opencv.hpp>

namespace TargetDetecter
{
// NOLINTBEGIN(readability-identifier-naming)
struct DetectResult
{
    Target::Feature feature;
    bool is_hit; // 是否击中
};
// NOLINTEND(readability-identifier-naming)
DetectResult detect_best_target(const cv::Mat& image);

} // namespace TargetDetecter