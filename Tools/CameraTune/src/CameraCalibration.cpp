#include <algorithm>
#include <filesystem>
#include <opencv2/opencv.hpp>

const int chessboard_width = 11; // 12-1
const int chessboard_height = 8; // 9-1
const int chessboard_size = 25;  // millimeters

int main(int argc, char** argv)
{
    std::string capture_path = "CapturedFrames";
    if (!std::filesystem::exists(capture_path) || !std::filesystem::is_directory(capture_path))
    {
        std::cerr << "Capture path does not exist or is not a directory: " << capture_path << std::endl;
        return -1;
    }
    std::vector<std::string> images;
    for (const auto& entry : std::filesystem::directory_iterator(capture_path))
    {
        if (entry.path().extension() == ".png")
            images.push_back(entry.path().string());
    }
    std::sort(images.begin(), images.end());

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size;
    int valid_count = 0;
    cv::namedWindow("Corners", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);

    for (const auto& img_path : images)
    {
        cv::Mat frame = cv::imread(img_path);
        if (frame.empty())
            continue;

        if (image_size.width == 0 && image_size.height == 0)
            image_size = frame.size();

        std::vector<cv::Point2f> corners;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        bool pattern_founded = cv::findChessboardCorners(gray, cv::Size(chessboard_width, chessboard_height), corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if (pattern_founded)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));

            cv::drawChessboardCorners(frame, cv::Size(chessboard_width, chessboard_height), corners, pattern_founded);
            cv::imshow("Corners", frame);
            cv::waitKey(0);

            std::vector<cv::Point3f> obj;
            for (int i = 0; i < chessboard_height; i++)
                for (int j = 0; j < chessboard_width; j++)
                    obj.emplace_back(j * chessboard_size, i * chessboard_size, 0.0f);

            object_points.push_back(obj);
            image_points.push_back(corners);
            valid_count++;
            std::cout << "Processed " << img_path << ": Pattern Found" << std::endl;
        }
        else
        {
            std::cout << "Processed " << img_path << ": Pattern Not Found" << std::endl;
        }
    }
    cv::destroyAllWindows();

    if (valid_count >= 10)
    {
        std::cout << "Calibrating camera with " << valid_count << " captures..." << std::endl;
        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
        std::vector<cv::Mat> rvecs, tvecs;
        double rms = cv::calibrateCamera(object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs);
        std::cout << "Calibration done with RMS error = " << rms << std::endl;
        std::cout << "Camera Matrix: " << std::endl << camera_matrix << std::endl;
        std::cout << "Distortion Coefficients: " << std::endl << dist_coeffs << std::endl;

        cv::FileStorage fs("CalibrationResult.yaml", cv::FileStorage::WRITE);
        fs << "Camera_Matrix" << camera_matrix;
        fs << "Distortion_Coefficients" << dist_coeffs;
        fs.release();
        std::cout << "Calibration data saved to CalibrationResult.yaml" << std::endl;
        // 计算重投影误差
        double total_error = 0.0;
        int total_points = 0;
        for (size_t i = 0; i < object_points.size(); i++)
        {
            std::vector<cv::Point2f> projected_points;
            cv::projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, projected_points);
            double error = cv::norm(image_points[i], projected_points, cv::NORM_L2);
            total_error += error * error;
            total_points += object_points[i].size();
            std::cout << "Capture " << i + 1 << ": Reprojection Error = " << std::sqrt(error * error / object_points[i].size()) << std::endl;
        }
        double avg_error = total_error / total_points;
        std::cout << "Average reprojection error: " << avg_error << std::endl;
    }
    else
    {
        std::cout << "Not enough valid captures for calibration (found " << valid_count << ")." << std::endl;
    }

    return 0;
}