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
        {"Self_Tower", 0.25}, {"Enemy_Tower", 0.25}, {"Middle_High", 0.4},
        {"Self_Left_High", 0.473}, {"Enemy_Left_High", 0.473},
        {"Self_Ring_High", 0.2}, {"Enemy_Ring_High", 0.2}, {"Enemy_Buff", 0.8}
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

cv::Point3d Location::parse(const cv::Point2d& img_point) const {
    // 提前处理特殊高度情况，减少后续不必要的计算
    double height = getHeight(img_point);
    if (height > 0.79f) {
        return cv::Point3d(19.322f, -1.915f, height);
    }

    // 1. 图像点去畸变 - 使用单点优化
    cv::Point2d pt_undist;
    {
        std::vector<cv::Point2d> pt_dist{img_point};
        std::vector<cv::Point2d> pt_undist_vec;
        cv::undistortPoints(pt_dist, pt_undist_vec, camera_matrix, dist_coeffs, cv::noArray(), camera_matrix);
        pt_undist = pt_undist_vec[0];
    }

    // 2. 计算归一化坐标 - 使用直接访问camera_matrix元素
    const double cx = camera_matrix.at<double>(0, 2);
    const double cy = camera_matrix.at<double>(1, 2);
    const double fx = camera_matrix.at<double>(0, 0);
    const double fy = camera_matrix.at<double>(1, 1);
    
    const double x_n = (pt_undist.x - cx) / fx;
    const double y_n = (pt_undist.y - cy) / fy;

    // 3. 计算旋转矩阵（如果rvec是常量，这部分可以在构造函数中预计算）
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // 4. 计算光线方向 - 使用Vec3f代替Mat以提高效率
    cv::Vec3f ray_dir(x_n, y_n, 1.0f);
    cv::Vec3f ray_dir_world;
    
    // 将旋转矩阵转换为Vec3f进行更高效的计算
    ray_dir_world[0] = R.at<double>(0,0) * ray_dir[0] + R.at<double>(1,0) * ray_dir[1] + R.at<double>(2,0) * ray_dir[2];
    ray_dir_world[1] = R.at<double>(0,1) * ray_dir[0] + R.at<double>(1,1) * ray_dir[1] + R.at<double>(2,1) * ray_dir[2];
    ray_dir_world[2] = R.at<double>(0,2) * ray_dir[0] + R.at<double>(1,2) * ray_dir[1] + R.at<double>(2,2) * ray_dir[2];

    // 归一化方向向量
    const double norm = std::sqrt(ray_dir_world.dot(ray_dir_world));
    ray_dir_world /= norm;

    // 5. 计算相机中心在世界坐标系中的位置
    cv::Vec3f C;
    C[0] = -(R.at<double>(0,0)*tvec.at<double>(0) + R.at<double>(1,0)*tvec.at<double>(1) + R.at<double>(2,0)*tvec.at<double>(2));
    C[1] = -(R.at<double>(0,1)*tvec.at<double>(0) + R.at<double>(1,1)*tvec.at<double>(1) + R.at<double>(2,1)*tvec.at<double>(2));
    C[2] = -(R.at<double>(0,2)*tvec.at<double>(0) + R.at<double>(1,2)*tvec.at<double>(1) + R.at<double>(2,2)*tvec.at<double>(2));

    // 6. 检查射线是否平行于平面
    if (std::abs(ray_dir_world[2]) < 1e-6f) {
        return cv::Point3d(128.0f, 0.0f, height);
    }

    // 7. 计算交点
    const double t_param = (height - C[2]) / ray_dir_world[2];
    cv::Point3d P_w(
        C[0] + t_param * ray_dir_world[0],
        C[1] + t_param * ray_dir_world[1],
        height
    );

    // 8. 验证投影误差
    {
        std::vector<cv::Point3d> obj_pt{P_w};
        std::vector<cv::Point2d> img_pt_proj;
        cv::projectPoints(obj_pt, rvec, tvec, camera_matrix, dist_coeffs, img_pt_proj);
        
        const double error = cv::norm(img_pt_proj[0] - pt_undist);
        if (error > 100.0) {
            return cv::Point3d(0.0, 0.0, 0.0);
        }
    }

    return P_w;
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
