#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>

class Config 
{
public:
    std::string color = "red"; // blue
    std::string camera_model = "test"; // test、hik
    std::string port = "COM3"; // /dev/ttyUSB0

    // 相机参数使用 float
    Eigen::Matrix<double, 3, 3> camera_matrix;
    Eigen::Matrix<double, 5, 1> dist_coeffs;

    std::string armor_model_path = "";
    std::string car_model_path = "";
    std::string armor_yaml_path = "";
    std::string car_yaml_path = "";

    std::vector<float> map_size = {28.0f, 15.0f};

    struct HikConfig
    {
        std::string sn = "DA6214861";
        float exposure = 15000.0f;
        float gain = 15.0f;
        float frame_rate = 210.0f;
        bool rotate_180 = false;
        std::string log_level = "info";
    };
    HikConfig hik_cfg;

    std::string video_path = "/home/zqz/ros2_ws/image/raw.mp4";
    std::string image_path = "/home/zqz/ros2_ws/image/raw.jpg";

    Config()
    {
        camera_matrix << 
            3359.900898, 0.,          1492.081297,
            0.,          3344.312184, 1009.967128,
            0.,          0.,          1.;

        dist_coeffs << 
            -0.113551, 
             0.168385, 
             0.000048, 
            -0.001705, 
             0.000000;
    }
};

#endif // CONFIG_H
