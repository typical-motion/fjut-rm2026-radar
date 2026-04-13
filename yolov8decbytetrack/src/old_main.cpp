// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

#include <iostream>
#include <opencv4/opencv2/core/types.hpp>
#include <vector>
#include <getopt.h>
#include <tuple>
#include <functional>
#include <thread>
#include <algorithm>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>


#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include "inference.h"
#include "BYTETracker.h"
#include "HikCamera.h"
#include "config.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "tutorial_interfaces/msg/detection.hpp"
#include "tutorial_interfaces/msg/target.hpp"


using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace dnn;
using namespace byte_track;
using json = nlohmann::json;

STrack createStrack(const Detection& detection, int frame_id)
{
    byte_track::Rect<float> rect 
    (
     detection.box.x, 
     detection.box.y, 
     detection.box.width, 
     detection.box.height
    );
    return STrack(rect, detection.confidence);
};

struct detection_info
{
    std::string class_name;
    cv::Point2f point;
    float confidence;
    std::string toString() const
    {
        return class_name + "(" + std::to_string(point.x) + "," + std::to_string(point.y) + ")";
    };
};

int frame_rate = 30;
int track_buffer = 30;
float track_thresh = 0.5;
float high_thresh = 0.6;
float match_thresh = 0.8;



auto inferencethrow(Inference_trt& inf_car, cv::Mat& frame)
{
    //std::vector<Detection> output = inf_car.runInference(frame);
    std::vector<Detection> output = inf_car.runInference_TensorRT(frame);
    //for (auto& detection : detections)
//    {
 //       cv::rectangle(frame, cv::Rect(detection.box.x, detection.box.y, detection.box.width, detection.box.height), cv::Scalar(0, 255, 0), 2);
  //      cv::putText(frame, detection.className + " " + std::to_string(detection.confidence), cv::Point(detection.box.x, detection.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
   // }
    return output;
}




void remove_same_obj(std::vector<Detection>& output_armor)
{
    // 先按 className 分组，再按 confidence 降序排列
    std::sort(output_armor.begin(), output_armor.end(),
              [](const Detection& a, const Detection& b)
              {
                  if (a.className == b.className)
                      return a.confidence > b.confidence; // confidence 从大到小
                  return a.className < b.className; // className 排序
              });

    // unique + erase 去掉同名的，只保留第一个（confidence 最大的）
    output_armor.erase(
        std::unique(output_armor.begin(), output_armor.end(),
                    [](const Detection& a, const Detection& b)
                    {
                        return a.className == b.className;
                    }),
        output_armor.end());
}

//void same_cof_clear(std::vector<Detection>& output_armor)
//{
//    std::sort(output_armor.begin(),output_armor.end(),compare2);
//}

//std::ofstream file1("/home/zqz/ros2_ws/json/output_json.json");
//std::ofstream file2("/home/zqz/ros2_ws/json/output_arrmor_json.json");
std::tuple<std::vector<Detection>, std::vector<cv::Point2f>> trackerthrow(std::vector<Detection>& output, BYTETracker& tracker, Inference& inf_armor, cv::Mat& frame)
{   
    std::vector<std::string> classes_armor{};
    std::vector<Detection> detections_armor{};
    std::vector<cv::Point2f> points;
    cv::Mat roi;
    std::vector<byte_track::Object> detections;
    //json output_json;
    //json output_json_arrmor;
    for (auto& detection : output)
    {
        byte_track::Rect<float> rect
            (
                detection.box.x, 
                detection.box.y, 
                detection.box.width, 
                detection.box.height
            );
            byte_track::Object obj(rect, detection.class_id, detection.confidence);
            detections.push_back(obj);
            //output_json = {{"class_name:", detection.className}, {"confidence:",detection.confidence}};

    }
    //std::ofstream file("/home/zqz/ros2_ws/json/output_json.json");
    //file1 << output_json.dump(4);
    
    std::vector<std::shared_ptr<STrack>> output_track_ptrs = tracker.update(detections);
     for (const auto& track_ptr : output_track_ptrs)
        {
            if (!track_ptr) continue;

            const STrack& track = *track_ptr;
            auto rect = track.getRect();
            cv::Rect box(rect.x(), rect.y(), rect.width(), rect.height());
            // 绘制车辆检测框
            cv::Scalar Box_color(0, 255, 0);
            cv::Scalar Text_color(255, 255, 255);
            
            cv::rectangle(frame, box, Box_color, 2); 
            
            // 添加跟踪ID标签
            std::string trackInfo = "ID: " + std::to_string(track_ptr->getTrackId());
            cv::Size textSize = cv::getTextSize(trackInfo, cv::FONT_HERSHEY_DUPLEX, 0.7, 1, 0);
            cv::Rect textBox(box.x, box.y - 25, textSize.width + 10, textSize.height + 10);
            
            std::string cof_info = "cof:" + std::to_string(track_ptr->getScore());
            cv::Size textSize_cof = cv::getTextSize(cof_info, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, 0);
            cv::Rect textBox_cof(box.x, box.y - 50, textSize_cof.width + 10, textSize_cof.height + 10);
            //std::string cof = detection.confidence;
            //cv::Size textSize_cof = cv::getTextSize(cof, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, 0);
            //cv::Rect textBox_cof(box.x, box.y - 50, textSize_cof.width + 10, textSize_cof.height + 10);

            cv::rectangle(frame, textBox, (0,0,0), cv::FILLED);
            cv::putText(frame, trackInfo, cv::Point(box.x + 5, box.y - 5), 
                       cv::FONT_HERSHEY_DUPLEX, 0.7, Text_color, 1, 0);
            
            cv::rectangle(frame, textBox_cof, (0,0,0), cv::FILLED);
            cv::putText(frame, cof_info, cv::Point(box.x + 5, box.y - 40), 
                       cv::FONT_HERSHEY_DUPLEX, 0.5, Text_color, 1, 0);
            
            // 装甲板检测（在车辆ROI内）
             if (box.x >=0 && box.y >=0 && box.x + box.width <= frame.cols && box.y + box.height <= frame.rows && box.width > 0 && box.height > 0)
            {

            try
            {
              //std::cout << "box.x:" << box.x << " y:" << box.y << " w:" << box.width << " h:" << box.height << std::endl;
              cv::Mat roi = frame(box);
              std::vector<Detection> output_armor = inf_armor.runInference(roi);
              remove_same_obj(output_armor);
                for (const auto& detection_armor : output_armor)
                {
                    detections_armor.push_back(detection_armor);
                    cv::Rect box_armor = detection_armor.box;
                    cv::Scalar armor_color(255, 0, 0); // 蓝色表示装甲板

                     if (box_armor.x >= 0 && box_armor.y >= 0 &&
                         box_armor.x + box_armor.width <= roi.cols &&
                         box_armor.y + box_armor.height <= roi.rows )
                    {
                    
                    cv::rectangle(roi, box_armor, armor_color, 2);

                    std::string classString_armor = detection_armor.className + ' ' + 
                                                std::to_string(detection_armor.confidence).substr(0, 4);
                    
                    cv::Size textSize_armor = cv::getTextSize(classString_armor, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, 0);
                    cv::Rect textBox_armor(box_armor.x, box_armor.y - 20, 
                                        textSize_armor.width + 5, textSize_armor.height + 5);

                    cv::rectangle(roi, textBox_armor, (0,0,0), cv::FILLED);
                    cv::putText(roi, classString_armor, cv::Point(box_armor.x + 3, box_armor.y - 5), 
                            cv::FONT_HERSHEY_DUPLEX, 0.5, Text_color, 1, 0);
                    //output_json_arrmor = {{"class_name:", detection_armor.className}, {"confidence:",detection_armor.confidence}};
                    //file2 << output_json_arrmor.dump(4);
                    
                    // 绘制中心点
                    cv::circle(roi, cv::Point(box_armor.x + box_armor.width/2, 
                                            box_armor.y + box_armor.height), 
                            3, cv::Scalar(0, 0, 255), -1, 8, 0);
                    points.push_back(cv::Point2f(box.x + box_armor.x + box_armor.width/2, box.y + box_armor.y + box_armor.height));
                    classes_armor.push_back(detection_armor.className);
                    //std::cout << "armor.x:" << box_armor.x << " y:" << box_armor.y << " w:" << box_armor.width << " h:" << box_armor.height << std::endl;
                    }
                } 
            }
            catch (const cv::Exception& e)
            {
                std::cerr << "Error: " << e.what() << std::endl;
            }
        }
        else
        {
            std::cout << "invalid roi:x" << box.x << "y" << box.y << "w" << box.width << "h" << box.height << std::endl;
        }

        }
        return std::make_tuple(detections_armor, points);
    }


class inference_node : public rclcpp::Node
{
    public:
    
        inference_node():Node("inference_node"),runOnGPU_(true),tracker_(30.0,30)
        {
            Config cfg;
            //Config::HikConfig hik_config;
            hik_config.sn = cfg.hik_cfg.sn;
            hik_config.exposure = cfg.hik_cfg.exposure;
            hik_config.gain = cfg.hik_cfg.gain;
            hik_config.frame_rate = cfg.hik_cfg.frame_rate;
            hik_config.rotate_180 = cfg.hik_cfg.rotate_180;
            hik_config.log_level = cfg.hik_cfg.log_level;
            int frame_rate = 30;
            int track_buffer = 90;
            float track_thresh = 0.3;
            float high_thresh = 0.6;    
            float match_thresh = 0.7;
            BYTETracker tracker_(frame_rate, track_buffer, track_thresh, high_thresh, match_thresh);
            //tracker_ = std::make_unique<BYTETracker>(frame_rate, track_buffer, track_thresh, high_thresh, match_thresh);
            std::vector<std::string> classes_all{"car","armor","ignore","watcher","base"};
            std::vector<std::string> classes_armor{"B1", "B2", "B3", "B4", "B5", "B7","R1", "R2", "R3", "R4", "R5", "R7"};
            std::vector<std::string> classes_red{"R1", "R2", "R3", "R4", "R5", "R7"};
            std::vector<std::string> classes_blue{"B1", "B2", "B3", "B4", "B5", "B7"};
            //Inference inf_car("/home/zqz/ros2_ws/model2/car.onnx", cv::Size(640, 640), classes_all, runOnGPU_);
            Inference_trt inf_car("/home/zqz/ros2_ws/model2/best8.engine", cv::Size(640,640), classes_all, runOnGPU_);
            //Inference inf_car("/home/zqz/ros2_ws/model2/best4.onnx", cv::Size(640, 640), classes_all, runOnGPU_);
            inf_car_ = std::make_unique<Inference_trt>(inf_car);
            Inference inf_armor("/home/zqz/ros2_ws/model2/armor.onnx", cv::Size(640, 640), classes_armor, runOnGPU_);
            //Inference_trt inf_armor("/home/zqz/ros2_ws/model2/15.engine", cv::Size(640,640), classes_all, runOnGPU_);
            //inf_armor_ = std::make_unique<Inference_trt>(inf_armor);
            inf_armor_ = std::make_unique<Inference>(inf_armor);
            //publisher_ = this->create_publisher<std_msgs::msg::String>("armor_result", 10);
            //subscription_ = this->create_subscription<std_msgs::msg::String>("image_raw", 10, std::bind(&inference_node::timerCallback, this, std::placeholders::_1));
            publisher_detection = this->create_publisher<tutorial_interfaces::msg::Detection>("detection_topic", 10);
            timer_ = this->create_wall_timer(20ms,std::bind(&inference_node::timerCallback, this));
            cv::VideoCapture cap_;
            //cap_.open("/home/zqz/ros2_ws/image/raw.mp4");
            //cap_.open("/home/zqz/ros2_ws/image/raw2.jpg");
            cv::namedWindow("Detection", cv::WINDOW_NORMAL);

        
        }


    private:
        void timerCallback()
        {   
                std::cout << "模式选择(test/hik):" << std::endl;
                std::string mode;
                std::cin >> mode;
                
                if (mode == "hik")
                {
                    std::vector<cv::Mat> video_frames;
                    HikCamera hik_camera(hik_config);//PixelType_Gvsp_RGB8_Packed格式
                    std::cout << "开始采集" << std::endl;
                    bool j = true;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    //std::cout << "---" << std::endl;
                    while(j)
                    {
                    cv::Mat frame_rgb = hik_camera.getLatestFrame();
                    
                    cv::cvtColor(frame_rgb, frame, cv::COLOR_RGB2BGR);

                    //std::cout << "---" << std::endl;
                    if (frame.empty()) 
                    {
                        //file.close();
                        RCLCPP_ERROR(this->get_logger(), "Failed to get frame");
                        return;
                    }
                    //int height = frame.rows;
                    //int width = frame.cols;
                    //std::cout << "wight: " << width <<std::endl;
                    //std::cout << "height: " << height << std::endl;
                    // --- 推理 1: 车辆 ---
                    auto detections = inferencethrow(*inf_car_, frame);
                
                    // --- 跟踪 + 装甲板 ---
                    std::tuple<std::vector<Detection>, std::vector<cv::Point2f>> result = trackerthrow(detections, tracker_, *inf_armor_, frame);
                    std::vector<Detection> result_detections_armor = std::get<0>(result);
                    std::vector<cv::Point2f> result_points = std::get<1>(result);
                    //std::vector<std::string> result_classes_armor = inf_armor_->getClasses();
                    std::vector<std::string> class_name;
                    auto msg = tutorial_interfaces::msg::Detection();
                    for (int i = 0; i < result_detections_armor.size() && i < result_points.size(); i++)
                    {
                        tutorial_interfaces::msg::Target target_msg;
                        //msg.class_number = result_detections_armor.size();
                        target_msg.confidence = result_detections_armor[i].confidence;
                        target_msg.class_name = result_detections_armor[i].className;
                        target_msg.x = result_points[i].x;
                        target_msg.y = result_points[i].y;
                        RCLCPP_INFO(this->get_logger(), "armor_result: %s, %f, %f , %f,", target_msg.class_name.c_str(), target_msg.x, target_msg.y, target_msg.confidence);
                        msg.targets.push_back(target_msg);

                        
                        //RCLCPP_INFO(this->get_logger(), "一共有: %i 个目标", msg.class_number);
                        
                    }
                    publisher_detection->publish(msg);
                    
                
                    //publisher_detection->publish(msg);
                    //RCLCPP_INFO(this->get_logger(), "armor_result: %s, %f, %f , %f,", detection_msg.class_name.c_str(), detection_msg.x, detection_msg.y, detection_msg.confidence);
                    //RCLCPP_INFO(this->get_logger(), "一共有: %i 个目标", msg.class_number);

                    // --- 显示 ---
                    //cv::namedWindow("1", 1);
                    //cv::resizeWindow("1", 1024,720);
                    cv::imshow("Detection", frame);
                    if (cv::waitKey(1) == 27)   
                    {
                        cv::destroyAllWindows();
                        //hik_camera.stop();
                        j = false;
                        //break;
                        //hik_camera.release();
                        //hik_camera.closeDevice();

                        //rclcpp::shutdown();
                    }
                }
                
                }
                else if (mode == "test")
                {
                    //cv::Mat frame;
                    bool j = true;
                    cap_.open("/home/zqz/ros2_ws/image/raw.mp4");
                    if (!cap_.isOpened()) 
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to open video");
                    }
                    RCLCPP_INFO(this->get_logger(), "Video opened");
                    while(j)
                    {
                    cap_ >> frame;
                    if (frame.empty()) 
                    {
                        //file1.close();
                        //file2.close();
                        RCLCPP_INFO(this->get_logger(), "Video finished");
                        //rclcpp::shutdown();
                        return;
                    }

                    //int height = frame.rows;
                    //int width = frame.cols;
                    //std::cout << "wight: " << width <<std::endl;
                    //std::cout << "height: " << height << std::endl;
                    // --- 推理 1: 车辆 ---
                    auto detections = inferencethrow(*inf_car_, frame);
                
                    // --- 跟踪 + 装甲板 ---
                    std::tuple<std::vector<Detection>, std::vector<cv::Point2f>> result = trackerthrow(detections, tracker_, *inf_armor_, frame);
                    std::vector<Detection> result_detections_armor = std::get<0>(result);
                    std::vector<cv::Point2f> result_points = std::get<1>(result);
                    //std::vector<std::string> result_classes_armor = inf_armor_->getClasses();
                    std::vector<std::string> class_name;
                    auto msg = tutorial_interfaces::msg::Detection();
                    for (int i = 0; i < result_detections_armor.size() && i < result_points.size(); i++)
                    {
                        tutorial_interfaces::msg::Target target_msg;
                        //msg.class_number = result_detections_armor.size();
                        target_msg.confidence = result_detections_armor[i].confidence;
                        target_msg.class_name = result_detections_armor[i].className;
                        target_msg.x = result_points[i].x;
                        target_msg.y = result_points[i].y;
                        RCLCPP_INFO(this->get_logger(), "armor_result: %s, %f, %f , %f,", target_msg.class_name.c_str(), target_msg.x, target_msg.y, target_msg.confidence);
                        msg.targets.push_back(target_msg);

                        
                        //RCLCPP_INFO(this->get_logger(), "一共有: %i 个目标", msg.class_number);
                        
                    }
                    publisher_detection->publish(msg);
                    
                
                    //publisher_detection->publish(msg);
                    //RCLCPP_INFO(this->get_logger(), "armor_result: %s, %f, %f , %f,", detection_msg.class_name.c_str(), detection_msg.x, detection_msg.y, detection_msg.confidence);
                    //RCLCPP_INFO(this->get_logger(), "一共有: %i 个目标", msg.class_number);

                    // --- 显示 ---
                    //cv::namedWindow("1", 1);
                    //cv::resizeWindow("1", 1024,720);
                    cv::imshow("Detection", frame);
                    if (cv::waitKey(1) == 27)   
                    {
                        cv::destroyAllWindows();
                        //hik_camera.stop();
                        j = false;
                        //break;
                        //hik_camera.release();
                        //hik_camera.closeDevice();

                        //rclcpp::shutdown();
                    }
                }
                }
                
                
            
            
        }
        
       

        std::vector<detection_info> detection_infos;
        tutorial_interfaces::msg::Detection msg;
        bool runOnGPU_;
        std::string colcor_ = "blue";
        std::vector<std::string> classes_car_;
        std::vector<std::string> classes_armor_;
        std::unique_ptr<Inference_trt> inf_car_;
        //std::unique_ptr<Inference_trt> inf_armor_;
        std::unique_ptr<Inference> inf_armor_;
        BYTETracker tracker_;
        //std::unique_ptr<BYTETracker> tracker_;
        cv::VideoCapture cap_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr publisher_detection;
        rclcpp::TimerBase::SharedPtr timer_;
        Config::HikConfig hik_config;
        cv::Mat frame;
        cv::Mat frame_rgb;
        //json output_json;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<inference_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    //return 0;

    return 0;

    
}






