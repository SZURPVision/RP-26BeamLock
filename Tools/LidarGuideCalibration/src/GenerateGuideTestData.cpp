#include <Eigen/Dense>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

double degrees(double rad) { return rad * kRadToDeg; }

double radians(double deg) { return deg * kDegToRad; }

Eigen::Vector3d to_unit_vector(double pitch_deg, double yaw_deg)
{
    const double yaw_clockwise = radians(yaw_deg);
    const double pitch_clockwise = radians(pitch_deg);

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

void to_yaw_pitch(const Eigen::Vector3d& direction, double& pitch_deg, double& yaw_deg)
{
    Eigen::Vector3d dir = direction.normalized();

    const double yaw_clockwise = std::atan2(-dir.y(), dir.x());
    Eigen::Vector3d axis_z(0.0, 0.0, 1.0);
    Eigen::AngleAxisd yaw_rotation(-yaw_clockwise, axis_z);

    Eigen::Vector3d base = yaw_rotation * Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d rotated_y = yaw_rotation * Eigen::Vector3d(0.0, 1.0, 0.0);

    const double right_hand_angle = std::atan2(rotated_y.dot(base.cross(dir)), base.dot(dir));
    const double pitch_clockwise = -right_hand_angle;

    yaw_deg = degrees(yaw_clockwise);
    pitch_deg = degrees(pitch_clockwise);
}

Eigen::Vector3d to_image_direction(const Eigen::Vector3d& gimbal_dir) { return Eigen::Vector3d(gimbal_dir.z(), -gimbal_dir.x(), -gimbal_dir.y()).normalized(); }

Eigen::Vector3d from_image_direction(const Eigen::Vector3d& image_dir)
{
    Eigen::Vector3d dir(-image_dir.y(), -image_dir.z(), image_dir.x());
    if (dir.norm() <= 0.0)
        return Eigen::Vector3d(1.0, 0.0, 0.0);
    return dir.normalized();
}

Eigen::Matrix3d random_rotation(std::mt19937& rng)
{
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    Eigen::Vector3d axis(dist(rng), dist(rng), dist(rng));
    if (axis.norm() <= 1e-6)
        axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    axis.normalize();

    std::uniform_real_distribution<double> angle_dist(-0.6, 0.6);
    double angle = angle_dist(rng);

    Eigen::AngleAxisd aa(angle, axis);
    return aa.toRotationMatrix();
}

Eigen::Vector3d random_translation(std::mt19937& rng)
{
    std::uniform_real_distribution<double> dist(-0.2, 0.2);
    return Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
}

void save_transform_yaml(const std::string& path, const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
{
    cv::Mat rmat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            rmat.at<double>(r, c) = rotation(r, c);

    cv::Mat tvec = (cv::Mat_<double>(3, 1) << translation.x(), translation.y(), translation.z());

    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    rmat.copyTo(transform(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(transform(cv::Rect(3, 0, 1, 3)));

    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened())
        throw std::runtime_error("Failed to open " + path + " for writing.");

    fs << "rotation" << rmat;
    fs << "translation" << tvec;
    fs << "transform" << transform;
    fs.release();
}
} // namespace

int main(int argc, char** argv)
{
    std::string output_dir = "CalibrationData";
    int sample_count = 30;
    unsigned int seed = 42;
    double angle_noise_std_deg = 0.0;
    double point_noise_std = 0.0;

    if (argc >= 2)
        output_dir = argv[1];
    if (argc >= 3)
        sample_count = std::stoi(argv[2]);
    if (argc >= 4)
        seed = static_cast<unsigned int>(std::stoul(argv[3]));
    if (argc >= 5)
        angle_noise_std_deg = std::stod(argv[4]);
    if (argc >= 6)
        point_noise_std = std::stod(argv[5]);
    if (argc > 6)
    {
        std::cout << "Usage: " << argv[0] << " [output_dir] [count] [seed] [angle_noise_std_deg] [point_noise_std]" << std::endl;
        return 1;
    }

    if (sample_count < 4)
    {
        std::cerr << "Sample count must be >= 4." << std::endl;
        return 1;
    }

    std::filesystem::create_directories(output_dir);
    const std::string points_path = (std::filesystem::path(output_dir) / "points.txt").string();
    const std::string angles_path = (std::filesystem::path(output_dir) / "angles.txt").string();
    const std::string gt_path = (std::filesystem::path(output_dir) / "GuideTransform_gt.yaml").string();

    std::ofstream points_file(points_path);
    std::ofstream angles_file(angles_path);
    if (!points_file.is_open() || !angles_file.is_open())
    {
        std::cerr << "Failed to open output files in " << output_dir << std::endl;
        return 1;
    }

    std::mt19937 rng(seed);
    Eigen::Matrix3d rotation = random_rotation(rng);
    Eigen::Vector3d translation = random_translation(rng);
    save_transform_yaml(gt_path, rotation, translation);

    std::uniform_real_distribution<double> depth_dist(15.0, 20.0);
    std::uniform_real_distribution<double> yaw_dist(-40.0, 40.0);
    std::uniform_real_distribution<double> pitch_dist(-20.0, 20.0);
    std::normal_distribution<double> angle_noise(0.0, angle_noise_std_deg);
    std::normal_distribution<double> point_noise(0.0, point_noise_std);

    for (int i = 0; i < sample_count; ++i)
    {
        double yaw_deg = yaw_dist(rng);
        double pitch_deg = pitch_dist(rng);

        Eigen::Vector3d gimbal_dir = to_unit_vector(pitch_deg, yaw_deg);
        Eigen::Vector3d image_dir = to_image_direction(gimbal_dir);

        double depth = depth_dist(rng);
        Eigen::Vector3d point_camera = image_dir * depth;
        Eigen::Vector3d point_lidar = rotation.transpose() * (point_camera - translation);

        if (angle_noise_std_deg > 0.0)
        {
            pitch_deg += angle_noise(rng);
            yaw_deg += angle_noise(rng);
        }

        if (point_noise_std > 0.0)
        {
            point_lidar.x() += point_noise(rng);
            point_lidar.y() += point_noise(rng);
            point_lidar.z() += point_noise(rng);
        }

        points_file << point_lidar.x() << " " << point_lidar.y() << " " << point_lidar.z() << "\n";
        angles_file << pitch_deg << " " << yaw_deg << "\n";
    }

    points_file.close();
    angles_file.close();

    std::cout << "Generated " << sample_count << " samples in " << output_dir << std::endl;
    std::cout << "Ground truth saved to " << gt_path << std::endl;
    return 0;
}
