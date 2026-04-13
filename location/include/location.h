#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <vector>

class Location {
public:
    Location(const std::string& calibrate_path = "/home/zqz/ros2_ws/json/calibrate.json", 
             const std::string& map_config_path = "/home/zqz/ros2_ws/yaml/RM2025_Points.yaml");

    void updateCalibration(const std::string& new_calibrate_path);
    void updateCameraMatrix(const cv::Mat& new_camera_matrix);
    double getHeight(const cv::Point2d& img_point) const;
    cv::Point3d parse(const cv::Point2d& img_point) const;
    void drawRegions(cv::Mat& frame, int thickness = 2) const;


    void update2DProjection();

    cv::Mat camera_matrix, dist_coeffs, rvec, tvec;
    struct Region {
        std::vector<cv::Point3d> points_3d;
        double height;
        std::vector<cv::Point2d> points_2d;
    };
    std::map<std::string, Region> regions;
    std::map<std::string, double> region_height;
};