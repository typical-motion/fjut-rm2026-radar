// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license
// Simplified inference_node: only uses TensorRT inference for "car" detection and tracking.
// Armor-related code removed.

#include <iostream>
#include <opencv4/opencv2/core/types.hpp>
#include <vector>
#include <getopt.h>
#include <tuple>
#include <functional>
#include <thread>
#include <unordered_map>
#include <fstream>
#include <chrono>
#include <memory>
#include <future>


#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

#include "laser_detect.hpp"    // TensorRT-based inference wrapper (your class)
#include "HikCamera.h"
#include "config.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "tutorial_interfaces/msg/detection.hpp"
#include "tutorial_interfaces/msg/target.hpp"

using namespace std::chrono;

struct detection_info
{
    std::string class_name;
    cv::Point2f point;
    float confidence;
    std::string toString() const
    {
        return class_name + "(" + std::to_string(point.x) + "," + std::to_string(point.y) + ")";
    };
};// 推理结果


// Run TensorRT inference for a frame using provided Inference_trt instance
static std::vector<Detection> inferencethrow_trt(Inference_trt& inf_light_trt, cv::Mat& frame)
{
    return inf_light_trt.runInference_TensorRT(frame);
}//trt 加速车辆推理

// Convert detections -> byte_track::Object vector

/// @brief 
class laser_inference_node : public rclcpp::Node
{
public:
    laser_inference_node():Node("laser_inference_node"),runOnGPU_(true)
    {
        Config cfg;
        hik_config.sn = cfg.hik_cfg.sn;
        hik_config.exposure = cfg.hik_cfg.exposure;
        hik_config.gain = cfg.hik_cfg.gain;
        hik_config.frame_rate = cfg.hik_cfg.frame_rate;
        hik_config.rotate_180 = cfg.hik_cfg.rotate_180;
        hik_config.log_level = cfg.hik_cfg.log_level;
        //相机参数
        // classes for car model
        std::vector<std::string> classes_all{"light"};
        //检测目标
        // TensorRT engine paths (adjust to your environment)
        std::string light_engine = "/home/zqz/ros2_ws/model2/light.engine";
        //std::string light_onnx = "/home/zqz/ros2_ws/model2/light.onnx";
        inf_light_trt = std::make_unique<Inference_trt>(light_engine, cv::Size(640,640), classes_all, runOnGPU_);
        
        
        //Inference inf_light(car_onnx, cv::Size(640,640), classes_all, runOnGPU_);
        //inf_light_onnx = std::make_unique<Inference>(inf_light);
        inf_light_trt->setModelConfidenceThreshold(0.25f);
        inf_light_trt->setLetterBoxForSquare(true);


        publisher_detection = this->create_publisher<tutorial_interfaces::msg::Detection>("light_detection_topic", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&laser_inference_node::timerCallback, this));
        cv::namedWindow("Detection", cv::WINDOW_NORMAL);

        RCLCPP_INFO(this->get_logger(), "laser_inference_node initialized");
    }

private:

    void calculateMotorAngles(float target_u, float target_v, float& pitch, float& yaw) 
    {
        // 计算目标点相对于图像中心的偏移(像素)
        float delta_u = target_u - image_center_x;
        float delta_v = target_v - image_center_y;
        
        // 计算角度(弧度)
        float yaw_rad = atan2(delta_u, camera_focal_length_x);
        float pitch_rad = atan2(delta_v, camera_focal_length_y);
        
        // 转换为角度
        yaw = yaw_rad * (180.0f / M_PI) + camera_yaw_offset;
        pitch = pitch_rad * (180.0f / M_PI) + camera_pitch_offset;
        
        // 限制角度范围(根据实际电机范围调整)
        yaw = std::max(-90.0f, std::min(90.0f, yaw));
        pitch = std::max(-45.0f, std::min(45.0f, pitch));
    }


    void timerCallback()
    {
        std::cout << "模式选择(test/hik):" << std::endl;
        std::string mode;
        std::cin >> mode;
        
        if (mode == "hik")
        {
            HikCamera hik_camera(hik_config); // PixelType_Gvsp_RGB8_Packed format
            hik_camera.info();
            std::cout << "开始采集 (HikCamera)" << std::endl;
            bool running = true;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            cv::VideoWriter writer;
            std::cout << "是否保存处理后结果 (y/n)" << std::endl;
            std::string save_option;
            std::cin >> save_option;
            int frame_count = 0;

            auto start_time = std::chrono::high_resolution_clock::now();
            std::chrono::steady_clock::time_point fps_start_time = std::chrono::steady_clock::now();
            int fps_frame_count = 0;
            float fps_value = 0.0f;
            while (running)
            {
                cv::Mat frame_rgb = hik_camera.getLatestFrame();//海康录取到的通道和实际通道存在问题，直接使用opencv修改，其实可以直接改驱动（目前直接堆一个vector存frame
                stl_video.push_back(frame_rgb);
                cv::cvtColor(frame_rgb, frame, cv::COLOR_RGB2BGR);
                if (frame.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get frame");
                    return;
                }//帧检查


                //读取保存路径并设置格式

                // --- 推理: 激光检测装置 (TensorRT) ---
                std::vector<Detection> detections;
                try 
                {
                    detections = inferencethrow_trt(*inf_light_trt, frame);
                    //detections = inferencethrow_onnx(*inf_light_onnx, frame);
                } 
                catch (const std::exception& e) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
                    continue;
                }
                if (!detections.empty())
                {
                    auto msg = tutorial_interfaces::msg::Detection();

                    for (const auto& detection : detections)
                    {
                        cv::rectangle(frame, detection.box, cv::Scalar(0, 255, 0), 2);
                        std::string label = detection.className + ": " + std::to_string(detection.confidence);
                        int baseline = 0;
                        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &baseline);
                        cv::Point text_pos(detection.box.x, std::max(0, detection.box.y - text_size.height - 10));
                        cv::putText(frame, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);

                        float target_u = detection.box.x + detection.box.width / 2;
                        float target_v = detection.box.y + detection.box.height / 2;
                        // 计算目标点相对于图像中心的偏移(像素)
                        float delta_u = target_u - image_center_x;
                        float delta_v = target_v - image_center_y;
                        
                        // 计算角度(弧度)
                        float yaw_rad = atan2(delta_u, camera_focal_length_x);
                        float pitch_rad = atan2(delta_v, camera_focal_length_y);
                        
                        // 转换为角度
                        yaw = yaw_rad * (180.0f / M_PI) + camera_yaw_offset;
                        pitch = pitch_rad * (180.0f / M_PI) + camera_pitch_offset;
                        
                        // 限制角度范围(根据实际电机范围调整)
                        yaw = std::max(-90.0f, std::min(90.0f, yaw));
                        pitch = std::max(-45.0f, std::min(45.0f, pitch));

                        tutorial_interfaces::msg::Target target_msg;
                        // target_msg.confidence = detection.confidence;
                        // target_msg.class_name = detection.className;
                        // target_msg.x = detection.box.x + detection.box.width / 2;
                        // target_msg.y = detection.box.y + detection.box.height / 2;
                        // RCLCPP_INFO(this->get_logger(), "armor_result: %s, %f, %f , %f,", target_msg.class_name.c_str(), target_msg.x, target_msg.y, target_msg.confidence);
                        target_msg.yaw = yaw;
                        target_msg.pitch = pitch;
                        msg.targets.push_back(target_msg);
                        RCLCPP_INFO(this->get_logger(), "yaw: %f, pitch: %f", yaw, pitch);
                            //RCLCPP_INFO(this->get_logger(), "一共有: %i 个目标", msg.class_number);
                    }

                    publisher_detection->publish(msg);
                }


                frame_count++;

                fps_frame_count++;
                auto fps_now = std::chrono::steady_clock::now();
                float fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        fps_now - fps_start_time).count();

                if (fps_elapsed >= 200.0f)   // 每 1 秒更新一次
                {
                    fps_value = fps_frame_count * 1000.0f / fps_elapsed;
                    fps_frame_count = 0;
                    fps_start_time = fps_now;
                }

                cv::rectangle(frame, cv::Point(5,5), cv::Point(220,45), cv::Scalar(0,0,0), -1); // 黑底
                cv::putText(
                    frame,
                    cv::format("FPS: %.2f", fps_value),
                    cv::Point(10, 35),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.9,
                    cv::Scalar(0, 255, 0),
                    2
                );

                cv::namedWindow("hik",cv::WINDOW_NORMAL);
                cv::resizeWindow("hik",800,600);
                cv::imshow("hik", frame);

                if (frame_count % 30 == 0) 
                {
                    auto current_time = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> elapsed = current_time - start_time;
                    double fps_calculated = frame_count / elapsed.count();  // 计算FPS
                    std::cout << "Processed Frames: " << fps_value << " | FPS: " << fps_calculated << std::endl;
                }

                if (cv::waitKey(1) == 27)
                {
                    running = false;
                    std::cout << "\n正在停止录制并保存文件..." << std::endl;
                }
            }

            cv::destroyAllWindows();
            
            // if (recording_started && hik_camera.isRecording())
            // {
            //     hik_camera.stopRecording();
            // }
            std::cout << "已储存" << stl_video.size() << "帧" << std::endl;
            std::cout << "正在处理视频..." << std::endl;
            if (save_option == "y")
            {
                int width = stl_video[1].cols;
                int height = stl_video[1].rows;
                fourcc_code = 0;
                if (fourcc_code == 0)
                {
                    fourcc_code = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
                }

                writer.open(save_path, fourcc_code, 30, cv::Size(width, height), true);

                if (!writer.isOpened())
                {
                    std::cerr << "[VIDEO ERROR] Failed to open video writer!" << std::endl;
                    return;
                }


                for (const auto frame : stl_video)
                {
                    writer.write(frame);
                }
            }
            std::cout << "程序结束，视频已保存至: " << save_path << std::endl;
        }
        else if (mode == "test")
        {
            cap_.open("/home/zqz/ros2_ws/image/raw.mp4");
            //cap_.open("/home/zqz/ros2_ws/image/test2.mp4");
            if (!cap_.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Video opened");
            
            bool running = true;
            std::chrono::steady_clock::time_point fps_start_time = std::chrono::steady_clock::now();
            auto start_time = std::chrono::steady_clock::now();
            int fps_frame_count = 0;
            float fps_value = 0.0f;
            int frame_count = 0;

            while (running)
            {
                cap_ >> frame;
                if (frame.empty())
                {
                    running = false;
                    break;
                }

                // --- 推理: 激光检测装置 (TensorRT) ---
                std::vector<Detection> detections;
                try 
                {
                    detections = inferencethrow_trt(*inf_light_trt, frame);
                    //detections = inferencethrow_onnx(*inf_light_onnx, frame);
                } 
                catch (const std::exception& e) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
                    continue;
                }

                // 处理检测结果
                if (!detections.empty())
                {
                    auto msg = tutorial_interfaces::msg::Detection();

                    for (const auto& detection : detections)
                    {
                        // 绘制检测框
                        cv::rectangle(frame, detection.box, cv::Scalar(0, 255, 0), 2);
                        
                        // 绘制标签
                        std::string label = detection.className + ": " + std::to_string(detection.confidence);
                        int baseline = 0;
                        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &baseline);
                        cv::Point text_pos(detection.box.x, std::max(0, detection.box.y - text_size.height - 10));
                        cv::putText(frame, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);

                        // 填充消息
                        tutorial_interfaces::msg::Target target_msg;
                        target_msg.confidence = detection.confidence;
                        target_msg.class_name = detection.className;
                        target_msg.x = detection.box.x + detection.box.width / 2;
                        target_msg.y = detection.box.y + detection.box.height / 2;
                        
                        RCLCPP_INFO(this->get_logger(), "armor_result: %s, %f, %f , %f,", 
                                target_msg.class_name.c_str(), target_msg.x, target_msg.y, target_msg.confidence);
                        msg.targets.push_back(target_msg);
                    }

                    publisher_detection->publish(msg);
                }

                // FPS计算
                frame_count++;
                fps_frame_count++;
                auto fps_now = std::chrono::steady_clock::now();
                float fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        fps_now - fps_start_time).count();

                if (fps_elapsed >= 200.0f)   // 每 200ms 更新一次
                {
                    fps_value = fps_frame_count * 1000.0f / fps_elapsed;
                    fps_frame_count = 0;
                    fps_start_time = fps_now;
                }

                // 绘制FPS
                cv::rectangle(frame, cv::Point(5,5), cv::Point(220,45), cv::Scalar(0,0,0), -1);
                cv::putText(
                    frame,
                    cv::format("FPS: %.2f", fps_value),
                    cv::Point(10, 35),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.9,
                    cv::Scalar(0, 255, 0),
                    2
                );

                // 显示窗口
                cv::namedWindow("hik", cv::WINDOW_NORMAL);
                cv::resizeWindow("hik", 800, 600);
                cv::imshow("hik", frame);

                // 每30帧输出一次统计信息
                if (frame_count % 30 == 0) 
                {
                    auto current_time = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed = current_time - start_time;
                    double fps_calculated = frame_count / elapsed.count();  // 计算FPS
                    std::cout << "Processed Frames: " << fps_value << " | FPS: " << fps_calculated << std::endl;
                }

                // 检测ESC键
                if (cv::waitKey(1) == 27)
                {
                    running = false;
                    std::cout << "\n正在停止处理..." << std::endl;
                }
            }

            cv::destroyAllWindows();
            
            // 输出最终统计信息
            auto avg_end_time = std::chrono::steady_clock::now();
            double total_time_sec = std::chrono::duration_cast<std::chrono::duration<double>>(avg_end_time - start_time).count();
            double avg_fps = frame_count / std::max(total_time_sec, 1e-6);
            RCLCPP_INFO(
                this->get_logger(),
                "Test video finished. Total frames: %d, Time: %.2f s, Average FPS: %.2f", 
                frame_count, total_time_sec, avg_fps);
        }

    }

    // 计算pitch和yaw角度的函数
    




    // members
    bool runOnGPU_;
    std::string colcor_ = "blue";
    std::unique_ptr<Inference_trt> inf_light_trt;
    //std::unique_ptr<Inference> inf_light_onnx;
    cv::VideoCapture cap_;
    rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr publisher_detection;
    rclcpp::TimerBase::SharedPtr timer_;
    Config::HikConfig hik_config;
    cv::Mat frame;
    std::string save_path = "/home/zqz/ros2_ws/output.avi";
    std::vector<cv::Mat> stl_video;
    int fourcc_code;

    const float camera_focal_length_x = 500.0f;  // 相机x方向焦距(像素)
    const float camera_focal_length_y = 500.0f;  // 相机y方向焦距(像素)
    const float image_center_x = 640.0f;         // 图像中心x坐标
    const float image_center_y = 360.0f;         // 图像中心y坐标
    const float camera_pitch_offset = 0.0f;     // 相机pitch安装角度偏移(度)
    const float camera_yaw_offset = 0.0f;        // 相机yaw安装角度偏移(度)
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<laser_inference_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// ros2 run yolov8decbytetrack inference_node