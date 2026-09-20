#define MCAP_IMPLEMENTATION
#include <RawImage.pb.h>
#include <iostream>
#include <mcap/reader.hpp>
#include <opencv2/opencv.hpp>

namespace
{
struct DecodedFrame
{
    cv::Mat image;
    bool is_color = false;
};

DecodedFrame decode_raw_image(const foxglove::RawImage& raw_image)
{
    if (raw_image.width() == 0 || raw_image.height() == 0)
        return {};

    const auto expected_min_size = static_cast<size_t>(raw_image.step()) * raw_image.height();
    if (raw_image.data().size() < expected_min_size)
        return {};

    cv::Mat raw;
    if (raw_image.encoding() == "bayer_rggb8")
    {
        raw = cv::Mat(static_cast<int>(raw_image.height()), static_cast<int>(raw_image.width()), CV_8UC1, const_cast<char*>(raw_image.data().data()), static_cast<size_t>(raw_image.step())).clone();
        return {raw, false};
    }
    if (raw_image.encoding() == "mono8")
    {
        raw = cv::Mat(static_cast<int>(raw_image.height()), static_cast<int>(raw_image.width()), CV_8UC1, const_cast<char*>(raw_image.data().data()), static_cast<size_t>(raw_image.step())).clone();
        return {raw, false};
    }
    if (raw_image.encoding() == "rgb8")
    {
        raw = cv::Mat(static_cast<int>(raw_image.height()), static_cast<int>(raw_image.width()), CV_8UC3, const_cast<char*>(raw_image.data().data()), static_cast<size_t>(raw_image.step())).clone();
        cv::Mat bgr;
        cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
        return {bgr, true};
    }
    if (raw_image.encoding() == "bgr8")
    {
        raw = cv::Mat(static_cast<int>(raw_image.height()), static_cast<int>(raw_image.width()), CV_8UC3, const_cast<char*>(raw_image.data().data()), static_cast<size_t>(raw_image.step())).clone();
        return {raw, true};
    }

    return {};
}
} // namespace

int main(int argc, char* argv[])
{
    if (argc < 3 || argc > 4)
    {
        std::cerr << "Usage: " << argv[0] << " <mcap_file> <output_video> [fps]" << std::endl;
        return 1;
    }

    mcap::McapReader reader;
    auto status = reader.open(argv[1]);
    if (!status.ok())
    {
        std::cerr << "Failed to open MCAP file: " << status.message << std::endl;
        return 1;
    }

    double fps = 150.0;
    if (argc == 4)
    {
        try
        {
            fps = std::stod(argv[3]);
        }
        catch (const std::exception&)
        {
            std::cerr << "Invalid fps value: " << argv[3] << std::endl;
            return 1;
        }
    }

    cv::VideoWriter writer;
    bool writer_ready = false;
    size_t frame_count = 0;

    auto messages = reader.readMessages([](const mcap::Status& status) { std::cerr << "Error reading messages: " << status.message << std::endl; });

    for (const auto& message : messages)
    {
        if (message.channel->topic != "/camera/image")
            continue;
        if ((message.schema->encoding != "protobuf") || message.schema->name != "foxglove.RawImage")
        {
            std::cerr << "Skipping message with unexpected schema: " << message.schema->name << " and encoding: " << message.schema->encoding << std::endl;
            continue;
        }
        if (message.channel->messageEncoding != "protobuf")
        {
            std::cerr << "Expected message encoding 'protobuf', got " << message.channel->messageEncoding << std::endl;
            reader.close();
            return 1;
        }

        foxglove::RawImage raw_image;
        if (!raw_image.ParseFromArray(message.message.data, static_cast<int>(message.message.dataSize)))
        {
            std::cerr << "Could not parse RawImage message" << std::endl;
            return 1;
        }

        DecodedFrame frame = decode_raw_image(raw_image);
        if (frame.image.empty())
        {
            std::cerr << "Unsupported or invalid RawImage encoding: " << raw_image.encoding() << std::endl;
            continue;
        }

        if (!writer_ready)
        {
            const auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            if (frame.is_color)
            {
                std::cout << "Detected color images with encoding: " << raw_image.encoding() << std::endl;
            }
            else
            {
                std::cout << "Detected grayscale images with encoding: " << raw_image.encoding() << std::endl;
            }
            if (!writer.open(argv[2], fourcc, fps, frame.image.size(), frame.is_color))
            {
                std::cerr << "Failed to open video writer: " << argv[2] << std::endl;
                return 1;
            }
            writer_ready = true;
        }

        writer.write(frame.image);
        ++frame_count;
        if (frame_count % 100 == 0)
        {
            std::cout << "Processed " << frame_count << " frames..." << std::endl;
        }
    }

    if (!writer_ready)
    {
        std::cerr << "No frames written. Check topic and schema." << std::endl;
        return 1;
    }

    std::cout << "Wrote " << frame_count << " frames to " << argv[2] << std::endl;
    return 0;
}