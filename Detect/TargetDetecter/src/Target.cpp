#include <Target.h>
#include <cmath>

namespace TargetDetecter
{

void Target::check_valid()
{
    if (m_blobs_down->center().y - m_blobs_up->center().y < 10)
    {
        m_valid = false;
        return;
    }
    for (int i = 0; i < 3; i++)
    {
        if (std::abs(m_blobs_up->blobs()[i]->center().x - m_blobs_down->blobs()[i]->center().x) > 10)
        {
            m_valid = false;
            return;
        }
    }
    if (std::abs((m_blobs_up->blobs()[2]->center().x - m_blobs_up->blobs()[0]->center().x) - (m_blobs_down->blobs()[2]->center().x - m_blobs_down->blobs()[0]->center().x)) > 5)
    {
        m_valid = false;
        return;
    }
    double ratio = std::abs((m_blobs_up->blobs()[2]->center().x - m_blobs_up->blobs()[0]->center().x) / (m_blobs_up->blobs()[1]->center().y - m_blobs_down->blobs()[1]->center().y));
    if (ratio < 0.5 || (1 / ratio) < 0.5)
    {
        m_valid = false;
        return;
    }
    m_valid = true;
}

void Target::calculate_score()
{
    m_score = 0.0;
    m_score += m_blobs_up->score();
    m_score += m_blobs_down->score();
}

Target::Target(const LightRow::Ptr& blobs_up, const LightRow::Ptr& blobs_down) : m_blobs_up(blobs_up), m_blobs_down(blobs_down), m_score(-INFINITY), m_valid(false)
{
    check_valid();
    if (m_valid)
        calculate_score();
}

double Target::score() const { return m_score; }

bool Target::is_valid() const { return m_valid; }

Target::Feature Target::feature() const
{
    Feature feature;
    for (const auto& blob : m_blobs_up->blobs())
        feature.push_back(blob->center());
    for (const auto& blob : m_blobs_down->blobs())
        feature.push_back(blob->center());
    return feature;
}

} // namespace TargetDetecter
