#include "config.h" 
#include "location.h"
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <iostream>

// 假设Config中有CAMERA_MATRIX和DIST_COEFFS静态成员
using json = nlohmann::json;

Config cfg;
cv::Mat camera_matrix;
cv::Mat dist_coeffs;

Location::Location(const std::string& calibrate_path, const std::string& map_config_path) {
    // region_height 初始化（新地图）
    std::map<std::string, double> region_height = {
        {"Self_Tower", 0.25f}, {"Enemy_Tower", 0.25f}, {"Middle_High", 0.4f},
        {"Self_Left_High", 0.473f}, {"Enemy_Left_High", 0.473f},
        {"Self_Ring_High", 0.2f}, {"Enemy_Ring_High", 0.2f}, {"Enemy_Buff", 0.8f}
    };

    // 加载标定参数
    std::ifstream fin(calibrate_path);
    json jcalib;
    fin >> jcalib;
    cv::eigen2cv(cfg.camera_matrix, camera_matrix);
    cv::eigen2cv(cfg.dist_coeffs, dist_coeffs);
    camera_matrix.convertTo(camera_matrix, CV_64F);
    dist_coeffs.convertTo(dist_coeffs, CV_64F);
    std::vector<double> rvec_vec = jcalib["rvec"];
    std::vector<double> tvec_vec = jcalib["tvec"];
    rvec = cv::Mat(rvec_vec).clone().reshape(1, 3);  
    tvec = cv::Mat(tvec_vec).clone().reshape(1, 3);  

    // 加载地图点
    YAML::Node map = YAML::LoadFile(map_config_path);
    for (const auto& it : map) {
        std::string region_name = it.first.as<std::string>();
        std::vector<cv::Point3d> pts3d;
        for (const auto& p : it.second) {
            pts3d.emplace_back(p["x"].as<double>(), p["y"].as<double>(), p["z"].as<double>());
        }
        regions[region_name] = {pts3d, region_height.count(region_name) ? region_height[region_name] : 0.0f, {}};
    }
    update2DProjection();
}

void Location::update2DProjection() {
    for (auto& kv : regions) {
        auto& region = kv.second;
        if (region.points_3d.empty()) continue;
        std::vector<cv::Point3d> pts3d = region.points_3d;
        std::vector<cv::Point2d> pts2d;
        cv::projectPoints(pts3d, rvec, tvec, camera_matrix, dist_coeffs, pts2d);
        region.points_2d.clear();
        for (const auto& p : pts2d)
            region.points_2d.emplace_back(cv::Point2d(p.x, p.y));
    }
}

void Location::updateCalibration(const std::string& new_calibrate_path) {
    std::ifstream fin(new_calibrate_path);
    json jcalib;
    fin >> jcalib;
    std::vector<double> rvec_vec = jcalib["rvec"];
    std::vector<double> tvec_vec = jcalib["tvec"]; 
    rvec = cv::Mat(3, 1, CV_64FC1, rvec_vec.data());
    tvec = cv::Mat(3, 1, CV_64FC1, tvec_vec.data());
    update2DProjection();
}

void Location::updateCameraMatrix(const cv::Mat& new_camera_matrix) {
    camera_matrix = new_camera_matrix.clone();
    update2DProjection();
}

double Location::getHeight(const cv::Point2d& img_point) const {
    for (const auto& kv : regions) {
        const auto& region = kv.second;
        if (region.points_2d.size() < 3) continue;
        std::vector<cv::Point2f> pts2f;
        pts2f.reserve(region.points_2d.size());
         for (const auto& p : region.points_2d) {
            pts2f.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
        }
        cv::Point2f img_pt_f(static_cast<float>(img_point.x), static_cast<float>(img_point.y));

        double result = cv::pointPolygonTest(pts2f, img_pt_f, false);
        if (result >= 0)
            return region.height;
    }
    return 0.0;
}

cv::Point3d Location::parse(const cv::Point2d& img_point) const 
{

    // 1. undistort
    std::vector<cv::Point2d> pt_dist = {img_point};
    std::vector<cv::Point2d> pt_undist;
    cv::undistortPoints(pt_dist, pt_undist, camera_matrix, dist_coeffs, cv::noArray(), camera_matrix);
    double u = pt_undist[0].x;
    double v = pt_undist[0].y;
    //std::cout << "u: " << u << " v: " << v << std::endl;

    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);

    double x_n = (u - cx) / fx;double y_n = (v - cy) / fy;
    //std::cout << "x_n: " << x_n << " y_n: " << y_n << std::endl;

    double height = getHeight(img_point);
    if (height > 0.79)
        return cv::Point3d(19.322, -1.915, height);

    cv::Mat R;
    //cv::Mat rvec_flat = rvec;
    //rvec_flat = rvec_flat.t();
    cv::Rodrigues(rvec, R);//矩阵->向量
    cv::Mat tvec_flat = tvec.reshape(1, 3);
    //std::cout << "R:\n" << R << std::endl;
    //std::cout << "rvec:\n" << rvec << std::endl;
    //std::cout << "tvec_flat:\n" << tvec_flat << std::endl;

    cv::Mat ray_dir_mat = (cv::Mat_<double>(3, 1) << x_n, y_n, 1.0f);

    if (R.type() != ray_dir_mat.type()) {
        std::cerr << "警告: 射线方向与旋转矩阵不匹配\n";
        std::cout << R.type() << std::endl;
        std::cout << ray_dir_mat.type() << std::endl;
        return cv::Point3d(0, 0, 0);
    }

    cv::Mat ray_dir_world = R.t() * ray_dir_mat;
    ray_dir_world /= cv::norm(ray_dir_world);

    cv::Mat C = -R.t() * tvec_flat;

    double ray_z = ray_dir_world.at<double>(2, 0);
    double C_z = C.at<double>(2, 0);

    if (std::abs(ray_z) < 1e-6f) {
        std::cerr << "警告: 射线几乎平行于平面\n";  
        return cv::Point3d(128, 0, height);
    }

    double t_param = (height - C_z) / ray_z;
    cv::Mat P_w = C + t_param * ray_dir_world;

    std::vector<cv::Point3d> obj_pt = {cv::Point3d(P_w.at<double>(0, 0), P_w.at<double>(1, 0), height)};
    std::vector<cv::Point2d> img_pt_proj;
    cv::projectPoints(obj_pt, rvec, tvec, camera_matrix, dist_coeffs, img_pt_proj);
    //cv::Vec2f pt1(img_pt_proj[0].x, img_pt_proj[0].y);
    //cv::Vec2f pt2 = pt_undist.at<cv::Vec2f>(0, 0);
    double error = cv::norm(img_pt_proj[0] - pt_undist[0]);
    if (error > 100.0){
        std::cerr << "警告: 3D 点与 2D 点的距离过大\n";
        return cv::Point3d(0, 0, 0);
    }
    return cv::Point3d(P_w.at<double>(0, 0), P_w.at<double>(1, 0), height);
}

void Location::drawRegions(cv::Mat& frame, int thickness) const {
    std::map<std::string, cv::Scalar> colors = {
        {"Middle_Line", cv::Scalar(0, 255, 255)},
        {"Self_Ring_High", cv::Scalar(255, 0, 0)},
        {"Enemy_Ring_High", cv::Scalar(255, 0, 0)},
        {"Enemy_Buff", cv::Scalar(0, 0, 255)},
        {"Self_Left_High", cv::Scalar(255, 255, 255)},
        {"Enemy_Left_High", cv::Scalar(255, 255, 255)}
    };
    for (const auto& kv : regions) {
        if (kv.second.points_2d.size() < 3) continue;

        // 转换 double 点到 int 点
        std::vector<cv::Point> int_points;
        for (const auto& p : kv.second.points_2d) {
            int_points.emplace_back(cv::Point(cvRound(p.x), cvRound(p.y)));
        }

        cv::Scalar color = colors.count(kv.first) ? colors.at(kv.first) : cv::Scalar(0, 255, 0);
        cv::polylines(frame, int_points, true, color, thickness);
    }
}
