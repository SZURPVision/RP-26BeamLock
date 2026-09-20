#include <LightRow.h>
#include <cmath>

namespace TargetDetecter
{
void LightRow::check_valid()
{
    // TODO: add rejection rules
    m_valid = true;
}

void LightRow::calculate_score()
{
    m_score = 0.0;
    for (const auto& blob : m_blobs)
    {
        m_score += blob->area();
    }
}

LightRow::LightRow(const std::vector<LightBlob::Ptr>& blobs) : m_blobs(blobs), m_valid(false), m_score(-INFINITY)
{
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& blob : m_blobs)
    {
        sum_x += blob->center().x;
        sum_y += blob->center().y;
    }
    m_center = cv::Point2f(sum_x / m_blobs.size(), sum_y / m_blobs.size());

    check_valid();
    if (m_valid)
        calculate_score();
}

bool LightRow::is_valid() const { return m_valid; }

double LightRow::score() const { return m_score; }

const cv::Point2f& LightRow::center() const { return m_center; }

const std::vector<LightBlob::Ptr>& LightRow::blobs() const { return m_blobs; }
} // namespace TargetDetecter
