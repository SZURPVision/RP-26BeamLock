#include <Config.h>
#include <Eigen/Dense>
#include <MVSCamera.h>
#include <TargetDetecter.h>
#include <atomic>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <queue>
#include <set>
#include <string>

namespace
{
constexpr double light_blob_x_threshold = 20;
constexpr double light_blob_y_threshold = 5;

std::atomic_bool g_running{true};
void handle_sigint(int) { g_running.store(false); }

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
} // namespace

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

void detect(cv::Mat& image)
{
    cv::imshow("Original", image);

    // 我们可以用一个2x2卷积核来提取颜色特征
    Eigen::MatrixXf kernel(2, 2);
    kernel << 1, 0, 0, 1;
    // 我们简单的将红蓝加在一起，相信后面的模式匹配

    cv::Mat color_feature = conv2d(image, kernel, 2);
    cv::imshow("color_feature", color_feature);

    cv::Mat binary_feature;
    // 周围11x11的平均值-(-5)作为自适应阈值
    cv::adaptiveThreshold(color_feature, binary_feature, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, -10);
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(binary_feature, binary_feature, kernel_dilate);
    // 形态学开，以矩形为核
    cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary_feature, binary_feature, cv::MORPH_OPEN, kernel_open);

    cv::medianBlur(binary_feature, binary_feature, 3);

    // 找联通快和找边缘都是O(n)，其中n为像素数
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_feature, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::cvtColor(color_feature, color_feature, cv::COLOR_GRAY2BGR);
    cv::drawContours(color_feature, contours, -1, cv::Scalar(0, 255, 0), 1);

    std::vector<TargetDetecter::LightBlob::Ptr> light_blobs;
    for (auto& contour : contours)
    {
        TargetDetecter::LightBlob::Ptr blob = std::make_shared<TargetDetecter::LightBlob>(std::move(contour));
        if (blob->is_valid())
        {
            light_blobs.push_back(blob);
        }
    }

    cv::imshow("binary_feature", color_feature);

    std::vector<TargetDetecter::LightRow::Ptr> light_rows;
    if (light_blobs.size() >= 3)
    {
        std::sort(light_blobs.begin(), light_blobs.end(), CompareBlobsByY());

        std::set<TargetDetecter::LightBlob::Ptr, CompareBlobsByX> sorted_buffer;
        std::queue<std::set<TargetDetecter::LightBlob::Ptr, CompareBlobsByX>::iterator> buffer;

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
            std::vector<TargetDetecter::LightBlob::Ptr> row_blobs;
            row_blobs.reserve(sorted_buffer.size());
            for (const auto& candidate : sorted_buffer)
            {
                if (!row_blobs.empty() && std::abs(candidate->center().x - row_blobs.back()->center().x) > light_blob_x_threshold)
                    row_blobs.clear();

                row_blobs.push_back(candidate);
                if (row_blobs.size() == 3)
                {
                    TargetDetecter::LightRow::Ptr row = std::make_shared<TargetDetecter::LightRow>(row_blobs);
                    if (row->is_valid())
                        light_rows.push_back(row);
                    row_blobs.erase(row_blobs.begin());
                }
            }
        }
    }

    std::vector<TargetDetecter::Target::Ptr> targets;
    if (light_rows.size() >= 2)
    {
        for (size_t i = 0; i < light_rows.size(); i++)
        {
            for (size_t j = i + 1; j < light_rows.size(); j++)
            {
                TargetDetecter::Target::Ptr target = std::make_shared<TargetDetecter::Target>(light_rows[i], light_rows[j]);
                if (target->is_valid())
                    targets.push_back(target);
            }
        }
    }
    cv::Mat overlay = image.clone();
    cv::cvtColor(overlay, overlay, cv::COLOR_BayerBG2BGR);
    if (!targets.empty())
    {
        std::sort(targets.begin(), targets.end(), CompareTargetsByScore());

        TargetDetecter::Target::Feature feature = targets.front()->feature();

        double sum_x = 0, sum_y = 0;

        for (size_t i = 0; i < feature.size(); ++i)
        {
            feature[i].x *= 2;
            feature[i].y *= 2;
            sum_x += feature[i].x;
            sum_y += feature[i].y;
        }
        sum_x /= feature.size();
        sum_y /= feature.size();

        for (const auto& pt : feature)
        {
            cv::circle(overlay, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)), 10, cv::Scalar(0, 255, 0), 2);
        }
        cv::circle(overlay, cv::Point(static_cast<int>(sum_x), static_cast<int>(sum_y)), 10, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("TargetDetecterTest", overlay);
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, handle_sigint);
    const bool use_video = argc > 1;
    const std::string video_path = use_video ? argv[1] : std::string{};
    try
    {
        Config::get_config().load_from_file();
    }
    catch (const std::exception&)
    {
        Config::get_config().generate_default_config();
        Config::get_config().load_from_file();
    }

    cv::VideoCapture video_capture;
    int video_total_frames = 0;
    int video_pos_frame = 0;
    std::shared_ptr<Camera> camera;
    if (use_video)
    {
        video_capture.open(video_path);
        if (!video_capture.isOpened())
        {
            std::cerr << "Failed to open video: " << video_path << std::endl;
            return -1;
        }
        video_total_frames = static_cast<int>(video_capture.get(cv::CAP_PROP_FRAME_COUNT));
    }
    else
    {
        CameraManager camera_manager;
        auto cam_list = camera_manager.list_cameras();
        if (cam_list.is_empty())
        {
            std::cerr << "No cameras found!" << std::endl;
            return -1;
        }

        camera = camera_manager.create_camera(cam_list, 0);
        try
        {
            camera->start_grabbing();
            camera->set_gain(Config::get_config().get_camera_config()->m_gain);
            camera->set_exposure_time(Config::get_config().get_camera_config()->m_exposure);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Camera init failed: " << e.what() << std::endl;
            return -1;
        }
    }

    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::namedWindow("TargetDetecterTest", cv::WINDOW_NORMAL);
    cv::namedWindow("color_feature", cv::WINDOW_NORMAL);
    cv::namedWindow("binary_feature", cv::WINDOW_NORMAL);
    if (use_video && video_total_frames > 0)
    {
        cv::createTrackbar("Progress", "Original", &video_pos_frame, video_total_frames - 1);
    }

    std::cout << "Press Ctrl+C to exit." << std::endl;

    bool freeze_frame = false;
    int frozen_frame_index = -1;
    cv::Mat frozen_frame;

    constexpr double target_fps = 60.0;
    const double target_frame_ms = 1000.0 / target_fps;

    while (g_running.load())
    {
        const int64 frame_start_tick = cv::getTickCount();
        cv::Mat image;
        if (use_video)
        {
            int requested_frame = video_pos_frame;
            const bool need_refresh = !freeze_frame || frozen_frame.empty() || requested_frame != frozen_frame_index;
            if (need_refresh)
            {
                if (video_total_frames > 0)
                {
                    int actual_frame = static_cast<int>(video_capture.get(cv::CAP_PROP_POS_FRAMES));
                    if (requested_frame != actual_frame)
                    {
                        video_capture.set(cv::CAP_PROP_POS_FRAMES, requested_frame);
                    }
                }
                if (!video_capture.read(image))
                {
                    video_capture.set(cv::CAP_PROP_POS_FRAMES, 0);
                    video_pos_frame = 0;
                    if (!video_capture.read(image))
                    {
                        std::cout << "Video ended or failed to read frame." << std::endl;
                        break;
                    }
                }
                if (image.channels() == 3)
                {
                    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
                }
                else if (image.channels() == 4)
                {
                    cv::cvtColor(image, image, cv::COLOR_BGRA2GRAY);
                }
                if (video_total_frames > 0)
                {
                    video_pos_frame = static_cast<int>(video_capture.get(cv::CAP_PROP_POS_FRAMES));
                    cv::setTrackbarPos("Progress", "Original", video_pos_frame);
                }
                if (freeze_frame)
                {
                    frozen_frame = image.clone();
                    frozen_frame_index = video_pos_frame;
                }
            }
            else
            {
                image = frozen_frame.clone();
            }
        }
        else
        {
            if (freeze_frame && !frozen_frame.empty())
            {
                image = frozen_frame.clone();
            }
            else
            {
                try
                {
                    camera->get_frame(image, 1000);
                }
                catch (const std::exception& e)
                {
                    std::cerr << "Failed to get frame: " << e.what() << std::endl;
                    continue;
                }
                if (freeze_frame)
                {
                    frozen_frame = image.clone();
                    frozen_frame_index = -1;
                }
            }
        }
        detect(image);
        const int64 frame_end_tick = cv::getTickCount();
        const double elapsed_ms = (frame_end_tick - frame_start_tick) * 1000.0 / cv::getTickFrequency();
        const int wait_ms = std::max(1, static_cast<int>(std::round(target_frame_ms - elapsed_ms)));
        int key = cv::waitKey(wait_ms);
        if (key == ' ')
        {
            freeze_frame = !freeze_frame;
            if (freeze_frame)
            {
                frozen_frame = image.clone();
                frozen_frame_index = use_video ? video_pos_frame : -1;
            }
            else
            {
                frozen_frame.release();
                frozen_frame_index = -1;
            }
        }
        if (key == 27 || key == 'q' || key == 'Q')
            break;
    }

    return 0;
}
