#include <LightBlob.h>
#include <cassert>
#include <cmath>

namespace TargetDetecter
{

void LightBlob::check_valid()
{

    // TODO: add rejection rules
    m_valid = true;
}

LightBlob::LightBlob(std::vector<cv::Point>&& contours) : m_contours(contours), m_bounding_box(cv::minAreaRect(contours)), m_area(0), m_valid(false)
{
    cv::Moments moments = cv::moments(contours);
    if (moments.m00 == 0)
    {
        m_valid = false;
        return;
    }
    m_centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);
    m_area = moments.m00;

    check_valid();
}
} // namespace TargetDetecter
