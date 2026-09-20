#include <Config.h>
#include <FoxGloveServer.h>
#include <TargetDetecter.h>

class CompareBlobsByX
{
public:
    bool operator()(const TargetDetecter::LightBlob::Ptr& a, const TargetDetecter::LightBlob::Ptr& b) const
    {
        if (a->center().x != b->center().x)
            return a->center().x < b->center().x;
        if (a->center().y != b->center().y)
            return a->center().y < b->center().y;
        return a.get() < b.get();
    }
};
class CompareBlobsByY
{
public:
    bool operator()(const TargetDetecter::LightBlob::Ptr& a, const TargetDetecter::LightBlob::Ptr& b) const
    {
        if (a->center().y != b->center().y)
            return a->center().y < b->center().y;
        if (a->center().x != b->center().x)
            return a->center().x < b->center().x;
        return a.get() < b.get();
    }
};
class CompareTargetsByScore
{
public:
    bool operator()(const TargetDetecter::Target::Ptr& a, const TargetDetecter::Target::Ptr& b) const
    {
        if (a->score() != b->score())
            return a->score() > b->score();
        return a.get() < b.get();
    }
};

cv::Mat conv2d(const cv::Mat& input, const Eigen::MatrixXf& kernel, int stride)
{
    const int input_rows = input.rows;
    const int input_cols = input.cols;
    const int kernel_rows = kernel.rows();
    const int kernel_cols = kernel.cols();
    assert(kernel_rows == 2 && kernel_cols == 2);

    const int output_rows = (input_rows - kernel_rows) / stride + 1;
    const int output_cols = (input_cols - kernel_cols) / stride + 1;

    cv::Mat output(output_rows, output_cols, CV_8UC1);
    output.setTo(0);
    for (int i = 0; i < output_rows; ++i)
    {
        for (int j = 0; j < output_cols; ++j)
        {
            // clang-format off
            float sum = 0.0;
            sum = input.at<uint8_t>(i * stride, j * stride) * kernel(0, 0)
                + input.at<uint8_t>(i * stride, j * stride + 1) * kernel(0, 1)
                + input.at<uint8_t>(i * stride + 1, j * stride) * kernel(1, 0)
                + input.at<uint8_t>(i * stride + 1, j * stride + 1) * kernel(1, 1);
            // clang-format on
            output.at<uint8_t>(i, j) = static_cast<uint8_t>(std::clamp(sum, 0.0f, 255.0f));
        }
    }
    return output;
}

TargetDetecter::DetectResult TargetDetecter::detect_best_target(const cv::Mat& image)
{
    static Target::Feature empty_feature;
    const auto detect_config = Config::get_config().get_detect_config();
    const double light_blob_x_threshold = detect_config->m_light_blob_x_threshold;
    const double light_blob_y_threshold = detect_config->m_light_blob_y_threshold;
    const double min_hit_difference = detect_config->m_min_hit_difference;
    const double adaptive_threshold_c = detect_config->m_adaptive_threshold_c;
    // 注意：此处的frame是**BayerRG格式**的单通道图像
    // BayerRG的排列方式是：
    // R G R G ...
    // G B G B ...

    // 我们可以用一个2x2卷积核来提取颜色特征
    Eigen::MatrixXf kernel(2, 2);
    kernel << 1, 0, 0, 1;

    cv::Mat color_feature = conv2d(image, kernel, 2);
    cv::Mat binary_feature;
    cv::adaptiveThreshold(color_feature, binary_feature, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, adaptive_threshold_c);

    // 形态学膨胀，以矩形为核
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(binary_feature, binary_feature, kernel_dilate);
    // 形态学开，以矩形为核
    cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary_feature, binary_feature, cv::MORPH_OPEN, kernel_open);

    cv::medianBlur(binary_feature, binary_feature, 3);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_feature, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBlob::Ptr> light_blobs;
    for (auto& contour : contours)
    {
        LightBlob::Ptr blob = std::make_shared<LightBlob>(std::move(contour));
        if (blob->is_valid())
            light_blobs.push_back(blob);
    }

    std::vector<LightRow::Ptr> light_rows;
    if (light_blobs.size() >= 3)
    {
        std::sort(light_blobs.begin(), light_blobs.end(), CompareBlobsByY());

        std::set<LightBlob::Ptr, CompareBlobsByX> sorted_buffer;
        std::queue<std::set<LightBlob::Ptr, CompareBlobsByX>::iterator> buffer;

        for (const auto& blob : light_blobs)
        {
            auto res = sorted_buffer.insert(blob);
            if (!res.second)
                continue;
            buffer.push(res.first);
            while (!buffer.empty() && blob->center().y - (*buffer.front())->center().y > light_blob_y_threshold)
            {
                sorted_buffer.erase(buffer.front());
                buffer.pop();
            }
            if (sorted_buffer.size() < 3)
                continue;
            std::vector<LightBlob::Ptr> row_blobs;
            row_blobs.reserve(sorted_buffer.size());
            for (const auto& candidate : sorted_buffer)
            {
                if (!row_blobs.empty() && std::abs(candidate->center().x - row_blobs.back()->center().x) > light_blob_x_threshold)
                    row_blobs.clear();

                row_blobs.push_back(candidate);
                if (row_blobs.size() == 3)
                {
                    LightRow::Ptr row = std::make_shared<LightRow>(row_blobs);
                    if (row->is_valid())
                        light_rows.push_back(row);
                    row_blobs.erase(row_blobs.begin());
                }
            }
        }
    }

    std::vector<Target::Ptr> targets;
    if (light_rows.size() >= 2)
    {
        for (size_t i = 0; i < light_rows.size(); i++)
        {
            for (size_t j = i + 1; j < light_rows.size(); j++)
            {
                Target::Ptr target = std::make_shared<Target>(light_rows[i], light_rows[j]);
                if (target->is_valid())
                    targets.push_back(target);
            }
        }
    }
    if (targets.empty())
        return {empty_feature, false};
    std::sort(targets.begin(), targets.end(), CompareTargetsByScore());

    Target::Feature feature = targets.front()->feature();

    double mean_feature_value = 0.0;

    for (size_t i = 0; i < feature.size(); ++i)
    {
        mean_feature_value += color_feature.at<uint8_t>(static_cast<int>(feature[i].y), static_cast<int>(feature[i].x));
        feature[i].x *= 2;
        feature[i].y *= 2;
    }
    mean_feature_value /= feature.size();

    // 用2-means聚类算法自动确定一个阈值，来区分被照射和未被照射
    static double unhit_value = -1.0;
    static double hit_value = -1.0;
    static uint64_t unhit_count = 0;
    static uint64_t hit_count = 0;

    // 在线 2-means
    if (unhit_value < 0.0 || hit_value < 0.0)
    {
        unhit_value = mean_feature_value;
        hit_value = std::clamp(mean_feature_value + min_hit_difference, 0.0, 255.0);
        unhit_count = 1;
        hit_count = 1;
    }

    const double dist_to_unhit = std::abs(mean_feature_value - unhit_value);
    const double dist_to_hit = std::abs(mean_feature_value - hit_value);
    const bool assign_to_hit = dist_to_hit < dist_to_unhit;

    if (assign_to_hit)
    {
        ++hit_count;
        hit_value += (mean_feature_value - hit_value) / static_cast<double>(hit_count);
    }
    else
    {
        ++unhit_count;
        unhit_value += (mean_feature_value - unhit_value) / static_cast<double>(unhit_count);
    }

    // 约定：unhit_value 始终小于等于 hit_value
    if (unhit_value > hit_value)
    {
        std::swap(unhit_value, hit_value);
        std::swap(unhit_count, hit_count);
    }

    static bool is_cluster_separate_log_printed = false;
    const bool cluster_separated = (hit_value - unhit_value) >= min_hit_difference;
    if (cluster_separated && !is_cluster_separate_log_printed)
    {
        std::cout << "Clusters are now separated! unhit_value: " << unhit_value << ", hit_value: " << hit_value << std::endl;
        is_cluster_separate_log_printed = true;
    }

    const bool is_hit = cluster_separated && (std::abs(mean_feature_value - hit_value) < std::abs(mean_feature_value - unhit_value));

    return {feature, is_hit};
}