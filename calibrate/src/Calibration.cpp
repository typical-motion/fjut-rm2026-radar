#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <chrono>
//#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/msg/image.hpp>
#include "config.h"
#include "Calibrate.h"

using namespace std;
using namespace cv;
using namespace chrono;

void Calibrate::click_callback(int event, int x, int y, int flags, void* param)
        {
            cv::Mat resize_display_frame;
            Calibrate* calibrate = static_cast<Calibrate*>(param);
            if (!calibrate)
            {
                std::cerr << "空指针" << std::endl;
                return;
            }
            if(event == cv::EVENT_LBUTTONDOWN && calibrate->current_point < 5)
            {
                cv::Point2f real_point(x,y);
                cv::circle(calibrate->display_frame, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
                cv::putText(calibrate->display_frame, calibrate->map_point_name[calibrate->current_point], 
                    cv::Point(x+10, y-10), cv::FONT_HERSHEY_SIMPLEX, 
                    0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
                 
                calibrate->real_map_point.push_back(real_point);
                calibrate->current_point++;
                cv::resize(calibrate->display_frame, resize_display_frame, cv::Size(1280, 720));
                cv::imshow("calibration", resize_display_frame);
                cv::waitKey(1);
                
            }
        }
//class Calibrate_node : public rclcpp::Node
//{
    //public:
    //calibrate_publisher(): Node("calibrate_publisher")
    //{
    //  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("c", 10);
      
  //  }
//

int main()
{
    Config cfg;
    Calibrate cal;
    std::string model_cout;
    std::cout << "请输入模式(hik/test):" << std::endl;
    std::cin >> model_cout;
    
    
    //std::string mode = cfg.camera_model;
   // std::cout << __LINE__ << ": 进入函数" << std::endl;  // 定位崩溃点
    
    std::cout << "开始标定..." << std::endl;
    std::cout << "不要用图片标定!" << std::endl;

    if (model_cout == "test")
    {
        std::cout << "img_path:" << cfg.image_path << std::endl;
        //cal.calibrate_with_image(cfg.image_path);
        cal.calibrate_with_video(cfg.video_path);
        //std::cout << "rvec:" << cal.rvec_msg << std::endl;
        //std::cout << "tvec:" << cal.tvec_msg << std::endl;

    }else if (model_cout == "hik")
    {
        cal.calibrate_with_camera(cfg.hik_cfg);
        //std::cout << "rvec:" << cal.rvec_msg << std::endl;
        //std::cout << "tvec:" << cal.tvec_msg << std::endl;
    }
    return 0;
}
