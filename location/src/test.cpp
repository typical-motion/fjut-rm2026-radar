#include "config.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "location.h"
#include <opencv2/core/eigen.hpp>

using json = nlohmann::json;

int main()
{   
    Location locator;
    int img_width = 3072, img_height = 2048;
    cv::Mat frame(img_height, img_width, CV_8UC3, cv::Scalar::all(0));
    locator.drawRegions(frame, 2);

    cv::Point test_point(1749.00, 672.00);
    cv::circle(frame, test_point, 5, cv::Scalar(0, 0, 255), -1);
    cv::putText(frame, "Test Point: " + std::to_string(test_point.x) + "," + std::to_string(test_point.y),
        test_point + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);

    cv::Point3f world_coord = locator.parse(test_point);

    std::cout << "解算结果(原): X=" << world_coord.x << ", Y=" << world_coord.y << ", Z=" << world_coord.z << std::endl;
    std::cout << "解算结果: X=" << 28 - world_coord.x << ", Y=" << -world_coord.y << std::endl;

    std::string result_text = "解算结果: X=" + std::to_string(world_coord.x) + ", Y=" + std::to_string(world_coord.y);
    cv::putText(frame, result_text, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

    cv::Mat frame1;
    cv::resize(frame, frame1, cv::Size(1500, 1000));
    cv::imshow("Regions and Test Point", frame1);
    cv::waitKey(0);
    Config cfg;
    //Location loc;
    cv::Mat camera_matrix, dist_coeffs, rvec, tvec;
    std::ifstream fin("/home/zqz/ros2_ws/json/calibrate.json");
    json scalib;
    fin >> scalib;
    cv::eigen2cv(cfg.camera_matrix,camera_matrix);
    cv::eigen2cv(cfg.dist_coeffs, dist_coeffs);
    std::vector<float> rvec_vec = scalib["rvec"];
    std::vector<float> tvec_vec = scalib["tvec"];
    rvec = cv::Mat(3, 1, CV_64FC1, rvec_vec.data());
    tvec = cv::Mat(3, 1, CV_64FC1, tvec_vec.data());
    
    std::cout << camera_matrix << std::endl;
    return 0;
}