/*
整体流程：
1. 打开激光和激光雷达，积累点云
2. 在积累的点云中点击激光照射的目标点，同时记录当前云台绝对角度
3. 将目标点写入points.txt，格式为x y z，角度写入angles.txt，格式为pitch yaw
*/

#include <Eigen/Dense>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct GimbalAngle
{
    double pitch = 0.0;
    double yaw = 0.0;
};

static bool should_skip_line(const std::string& line)
{
    for (char ch : line)
    {
        if (std::isspace(static_cast<unsigned char>(ch)))
            continue;
        return ch == '#';
    }
    return true;
}

static std::vector<Eigen::Vector3d> read_points(const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
        throw std::runtime_error("Failed to open points file: " + file_path);

    std::vector<Eigen::Vector3d> points;
    std::string line;
    size_t line_no = 0;
    while (std::getline(file, line))
    {
        ++line_no;
        if (line.empty() || should_skip_line(line))
            continue;

        std::istringstream iss(line);
        Eigen::Vector3d point;
        if (!(iss >> point.x() >> point.y() >> point.z()))
            throw std::runtime_error("Invalid point data at line " + std::to_string(line_no));

        std::string extra;
        if (iss >> extra)
            throw std::runtime_error("Unexpected token in points file at line " + std::to_string(line_no));

        points.push_back(point);
    }

    return points;
}

static std::vector<GimbalAngle> read_angles(const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
        throw std::runtime_error("Failed to open angles file: " + file_path);

    std::vector<GimbalAngle> angles;
    std::string line;
    size_t line_no = 0;
    while (std::getline(file, line))
    {
        ++line_no;
        if (line.empty() || should_skip_line(line))
            continue;

        std::istringstream iss(line);
        GimbalAngle angle;
        if (!(iss >> angle.pitch >> angle.yaw))
            throw std::runtime_error("Invalid angle data at line " + std::to_string(line_no));

        std::string extra;
        if (iss >> extra)
            throw std::runtime_error("Unexpected token in angles file at line " + std::to_string(line_no));

        angles.push_back(angle);
    }

    return angles;
}

static void print_usage(const std::string& exe_name)
{
    std::cout << "Usage: " << exe_name << " [calibration_dir]" << std::endl;
    std::cout << "   or: " << exe_name << " [points.txt] [angles.txt]" << std::endl;
}

static Eigen::Vector3d to_unit_vector(const GimbalAngle& angle)
{
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double yaw_clockwise = angle.yaw * kDegToRad;
    const double pitch_clockwise = angle.pitch * kDegToRad;

    Eigen::Vector3d axis_x(1.0, 0.0, 0.0);
    Eigen::Vector3d axis_z(0.0, 0.0, 1.0);

    Eigen::AngleAxisd yaw_rotation(-yaw_clockwise, axis_z);
    Eigen::Vector3d rotated_y = yaw_rotation * Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::AngleAxisd pitch_rotation(-pitch_clockwise, rotated_y);

    Eigen::Vector3d direction = pitch_rotation * (yaw_rotation * axis_x);
    if (direction.norm() <= 0.0)
        return axis_x;

    return direction.normalized();
}

int main(int argc, char** argv)
{
    std::string points_path = "CalibrationData/points.txt";
    std::string angles_path = "CalibrationData/angles.txt";

    if (argc == 2)
    {
        std::filesystem::path base(argv[1]);
        points_path = (base / "points.txt").string();
        angles_path = (base / "angles.txt").string();
    }
    else if (argc == 3)
    {
        points_path = argv[1];
        angles_path = argv[2];
    }
    else if (argc > 3)
    {
        print_usage(argv[0]);
        return 1;
    }

    try
    {
        auto points = read_points(points_path);
        auto angles = read_angles(angles_path);

        if (points.size() != angles.size())
        {
            std::cerr << "Points count (" << points.size() << ") does not match angles count (" << angles.size() << ")." << std::endl;
            return 1;
        }

        std::cout << "Loaded " << points.size() << " points and " << angles.size() << " angle records." << std::endl;

        std::vector<Eigen::Vector3d> directions;
        directions.reserve(angles.size());
        for (const auto& angle : angles)
            directions.push_back(to_unit_vector(angle));

        std::vector<cv::Point3d> object_points;
        std::vector<cv::Point2d> image_points;
        std::vector<Eigen::Vector3d> filtered_directions;
        object_points.reserve(points.size());
        image_points.reserve(points.size());
        filtered_directions.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i)
        {
            const auto& point = points[i];
            const auto& dir = directions[i];
            Eigen::Vector3d dir_image(dir.z(), -dir.x(), -dir.y());
            if (std::abs(dir_image.z()) < 1e-8)
                continue;

            object_points.emplace_back(point.x(), point.y(), point.z());
            image_points.emplace_back(dir_image.x() / dir_image.z(), dir_image.y() / dir_image.z());
            filtered_directions.push_back(dir_image.normalized());
        }

        if (object_points.size() < 4)
        {
            std::cerr << "Not enough valid correspondences for solvePnP (need >= 4)." << std::endl;
            return 1;
        }

        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

        cv::Mat rvec, tvec;
        std::vector<int> inliers;
        const int ransac_iterations = 10000;
        const double ransac_reproj_error = 0.03;
        const double ransac_confidence = 0.999;

        bool ok = cv::solvePnPRansac(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false, ransac_iterations, ransac_reproj_error, ransac_confidence, inliers, cv::SOLVEPNP_ITERATIVE);
        if (!ok)
        {
            std::cerr << "solvePnPRansac failed to find a solution." << std::endl;
            return 1;
        }

        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                rotation(r, c) = rmat.at<double>(r, c);

        Eigen::Vector3d translation(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        double sum_residual = 0.0;
        double max_residual = 0.0;
        size_t residual_count = 0;
        for (size_t i = 0; i < object_points.size(); ++i)
        {
            Eigen::Vector3d point(object_points[i].x, object_points[i].y, object_points[i].z);
            Eigen::Vector3d dir = filtered_directions[i];

            Eigen::Vector3d transformed = rotation * point + translation;
            double residual = transformed.cross(dir).norm();
            sum_residual += residual;
            max_residual = std::max(max_residual, residual);
            ++residual_count;
        }

        const double mean_residual = residual_count ? (sum_residual / static_cast<double>(residual_count)) : 0.0;
        std::cout << "solvePnPRansac result (rvec): " << rvec.t() << std::endl;
        std::cout << "solvePnPRansac result (tvec): " << tvec.t() << std::endl;
        std::cout << "Inliers: " << inliers.size() << " / " << object_points.size() << std::endl;
        std::cout << "Residual mean: " << mean_residual << ", max: " << max_residual << std::endl;

        cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
        rmat.copyTo(transform(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(transform(cv::Rect(3, 0, 1, 3)));

        cv::FileStorage fs("GuideTransform.yaml", cv::FileStorage::WRITE);
        if (!fs.isOpened())
        {
            std::cerr << "Failed to open GuideTransform.yaml for writing." << std::endl;
            return 1;
        }
        fs << "rvec" << rvec;
        fs << "tvec" << tvec;
        fs << "rotation" << rmat;
        fs << "transform" << transform;
        fs.release();
        std::cout << "Saved transform to GuideTransform.yaml" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Data loading failed: " << e.what() << std::endl;
        return 1;
    }


    return 0;
}
