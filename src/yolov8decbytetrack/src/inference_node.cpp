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
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <memory>
#include <future>


#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

#include "inference.h"    // TensorRT-based inference wrapper (your class)
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
using namespace byte_track;
using json = nlohmann::json;

// create STrack from Detection (for BYTETracker)
STrack createStrackFromDet(const Detection& detection)
{
    byte_track::Rect<float> rect
    (
        detection.box.x,
        detection.box.y,
        detection.box.width,
        detection.box.height
    );
    return STrack(rect, detection.confidence); // 追踪器
}

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
}//在多个装甲板中找到置信度最高的

// Run TensorRT inference for a frame using provided Inference_trt instance
static std::vector<Detection> inferencethrow_trt(Inference_trt& inf_car, cv::Mat& frame)
{
    return inf_car.runInference_TensorRT(frame);
}//trt 加速车辆推理

static std::vector<Detection> inferencethrow_onnx(Inference& inf_car, cv::Mat& frame)
{
    return inf_car.runInference(frame);
} // opencv dnn 推理 trt用不了被选

// Convert detections -> byte_track::Object vector
static std::vector<byte_track::Object> detectionsToObjects(const std::vector<Detection>& dets)
{
    std::vector<byte_track::Object> objs;
    objs.reserve(dets.size());
    for (const auto& d : dets)
    {
        byte_track::Rect<float> rect(d.box.x, d.box.y, d.box.width, d.box.height);
        // class_id not used much here; set to 0 (car) by default
        byte_track::Object obj(rect, 0, d.confidence);
        objs.push_back(obj);
    }
    return objs;
}// 将推理结果转为追踪器对象

/// @brief 
class inference_node : public rclcpp::Node
{
public:
    inference_node():Node("inference_node"),runOnGPU_(true),
                      tracker_(30 /*frame_rate*/, 90 /*track_buffer*/, 0.3f /*track_thresh*/, 0.6f /*high_thresh*/, 0.7f /*match_thresh*/)
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
        std::vector<std::string> classes_all{"car","armor","ignore","watcher","base"};
        std::vector<std::string> classes_armor{"B1", "B2", "B3", "B4", "B5", "B7","R1", "R2", "R3", "R4", "R5", "R7"};
        std::vector<std::string> classes_red{"R1", "R2", "R3", "R4", "R5", "R7"};
        std::vector<std::string> classes_blue{"B1", "B2", "B3", "B4", "B5", "B7"};
        //检测目标
        // TensorRT engine paths (adjust to your environment)
        std::string car_engine = "/home/zqz/ros2_ws/model2/best8.engine";
        //std::string car_onnx = "/home/zqz/ros2_ws/model2/car.onnx";
        inf_car_trt = std::make_unique<Inference_trt>(car_engine, cv::Size(640,640), classes_all, runOnGPU_);
        
        Inference inf_armor("/home/zqz/ros2_ws/model2/armor.onnx", cv::Size(640, 640), classes_armor, runOnGPU_);
        inf_armor_ = std::make_unique<Inference>(inf_armor);
        // create TensorRT inference instance for car detection
        
        //Inference inf_car(car_onnx, cv::Size(640,640), classes_all, runOnGPU_);
        //inf_car_onnx = std::make_unique<Inference>(inf_car);
        inf_car_trt->setModelConfidenceThreshold(0.25f);
        inf_car_trt->setLetterBoxForSquare(true);


        publisher_detection = this->create_publisher<tutorial_interfaces::msg::Detection>("detection_topic", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&inference_node::timerCallback, this));
        cv::namedWindow("Detection", cv::WINDOW_NORMAL);

        RCLCPP_INFO(this->get_logger(), "inference_node initialized");
    }

private:
    // Process tracked results: draw boxes + labels on frame and fill msg
    std::tuple<std::vector<Detection>,std::vector<cv::Point2f>> processAndPublishTracks(const std::vector<std::shared_ptr<STrack>>& tracks, cv::Mat& frame, Inference& inf_armor)
{
    // 创建ROS消息对象
    auto msg = tutorial_interfaces::msg::Detection();
    // 存储装甲板中心点坐标
    std::vector<cv::Point2f> points;
    //std::vector<std::string> classes_armor_{};  // [调试] 存储装甲板类别名称
    // 存储装甲板检测结果
    std::vector<Detection> detections_armor;
    
    // 遍历所有跟踪目标
    for (const auto& track_ptr : tracks)
    {
        // 检查跟踪指针是否有效
        if (!track_ptr) continue;
        // 获取跟踪对象
        const STrack& track = *track_ptr;
        // 获取跟踪框的矩形信息
        auto rect = track.getRect();
        // 转换为OpenCV的Rect格式
        cv::Rect box(static_cast<int>(rect.x()), static_cast<int>(rect.y()),
                     static_cast<int>(rect.width()), static_cast<int>(rect.height()));
        //std::cout << rect.x() << "/" << rect.y() << "/" << rect.width() << "/" << rect.height() << "/" << std::endl;  // [调试] 打印跟踪框坐标

        // 绘制车辆跟踪框
        cv::Scalar Box_color(0, 255, 0);  // 绿色
        cv::rectangle(frame, box, Box_color, 2);

        // 准备绘制文本信息：ID和置信度
        std::string trackInfo = "ID: " + std::to_string(track_ptr->getTrackId());
        std::string cof_info = "cof:" + std::to_string(track_ptr->getScore());

        // 在跟踪框上方绘制ID信息
        cv::putText(frame, trackInfo, cv::Point(box.x + 5, std::max(box.y - 5, 10)),
                    cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(255,255,255), 1, 0);
        // 在ID上方绘制置信度信息
        cv::putText(frame, cof_info, cv::Point(box.x + 5, std::max(box.y - 25, 10)),
                    cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,255,255), 1, 0);

        // 装甲板检测（在车辆ROI区域内）
        // 检查ROI区域是否在图像范围内且尺寸合法
        if (box.x >=0 && box.y >=0 && box.x + box.width <= frame.cols && box.y + box.height <= frame.rows && box.width > 0 && box.height > 0)
        {
            try
            {
                //std::cout << "box.x:" << box.x << " y:" << box.y << " w:" << box.width << " h:" << box.height << std::endl;  // [调试] 打印ROI区域信息
                // 提取车辆ROI区域
                cv::Mat roi(frame, box);
                // 在ROI区域运行装甲板检测推理
                std::vector<Detection> output_armor = inf_armor.runInference(roi);
                // 移除重复的检测结果
                remove_same_obj(output_armor);
                
                // 遍历装甲板检测结果
                for (const auto& detection_armor : output_armor)
                {
                    // 保存检测结果
                    detections_armor.push_back(detection_armor);
                    // 获取装甲板检测框
                    cv::Rect box_armor = detection_armor.box;
                    cv::Scalar armor_color(255, 0, 0); // 蓝色表示装甲板

                    // 检查装甲板框是否在ROI范围内
                    if (box_armor.x >= 0 && box_armor.y >= 0 &&
                        box_armor.x + box_armor.width <= roi.cols &&
                        box_armor.y + box_armor.height <= roi.rows )
                    {
                        // 绘制装甲板检测框
                        cv::rectangle(roi, box_armor, armor_color, 2);

                        // 准备装甲板类别和置信度文本
                        std::string classString_armor = detection_armor.className + ' ' + 
                                                    std::to_string(detection_armor.confidence).substr(0, 4);
                        
                        // 计算文本大小
                        cv::Size textSize_armor = cv::getTextSize(classString_armor, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, 0);
                        // 创建文本背景框
                        cv::Rect textBox_armor(box_armor.x, box_armor.y - 20, 
                                            textSize_armor.width + 5, textSize_armor.height + 5);

                        // 绘制黑色文本背景
                        cv::rectangle(roi, textBox_armor, cv::Scalar(0,0,0), cv::FILLED);
                        // 绘制类别和置信度文本
                        cv::putText(roi, classString_armor, cv::Point(box_armor.x + 3, box_armor.y - 5), 
                                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,255,255), 1, 0);
                        //output_json_arrmor = {{"class_name:", detection_armor.className}, {"confidence:",detection_armor.confidence}};  // [调试] 准备JSON输出
                        //file2 << output_json_arrmor.dump(4);  // [调试] 写入JSON文件
                        
                        // 绘制装甲板中心点（红色）
                        cv::circle(roi, cv::Point(box_armor.x + box_armor.width/2, 
                                                box_armor.y + box_armor.height), 
                                3, cv::Scalar(0, 0, 255), -1, 8, 0);
                        // 保存装甲板中心点坐标（转换到原图坐标系）
                        points.push_back(cv::Point2f(box.x + box_armor.x + box_armor.width/2, box.y + box_armor.y + box_armor.height));
                        //classes_armor_.push_back(detection_armor.className);  // [调试] 保存类别名称
                        //std::cout << "armor.x:" << box_armor.x << " y:" << box_armor.y << " w:" << box_armor.width << " h:" << box_armor.height << std::endl;  // [调试] 打印装甲板框信息
                    }
                } 
            }
            // 捕获OpenCV异常
            catch (const cv::Exception& e)
            {
                std::cerr << "Error: " << e.what() << std::endl;
            }
        }   
        else
        {
            // 打印无效ROI信息
            std::cout << "invalid roi:x" << box.x << "y" << box.y << "w" << box.width << "h" << box.height << std::endl;
        }
    }

    // 返回装甲板检测结果和中心点坐标
    return std::make_tuple(detections_armor, points);
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

                // --- 推理: 车辆 (TensorRT) ---
                std::vector<Detection> detections;
                try 
                {
                    detections = inferencethrow_trt(*inf_car_trt, frame);
                    //detections = inferencethrow_onnx(*inf_car_onnx, frame);
                } 
                catch (const std::exception& e) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
                    return;
                }

                // Convert detections to byte_track::Object and update tracker
                std::vector<byte_track::Object> objects = detectionsToObjects(detections);
                std::vector<std::shared_ptr<STrack>> tracked = tracker_.update(objects);


                // draw and publish
                std::tuple<std::vector<Detection>, std::vector<cv::Point2f>> result = processAndPublishTracks(tracked, frame, *inf_armor_);
                std::vector<Detection> result_detections_armor = std::get<0>(result);
                std::vector<cv::Point2f> result_points = std::get<1>(result);
                std::vector<std::string> classes_name;
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
            auto avg_start_time = fps_start_time;
            int fps_frame_count = 0;
            float fps_value = 0.0f;
            size_t total_frame_count = 0;
            while (running)
            {
                cap_ >> frame;
                //cv::Mat frame = cv::imread("/home/zqz/fjut_radar/image/test_image.jpg");
                if (frame.empty())
                {
                    running = false;
                    break;
                }

                

                // --- 推理: 车辆 (TensorRT) ---
                std::vector<Detection> detections;
                try {
                    detections = inferencethrow_trt(*inf_car_trt, frame);
                    //detections = inferencethrow_onnx(*inf_car_onnx, frame);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
                    return;
                }

                // tracking
                std::vector<byte_track::Object> objects = detectionsToObjects(detections);
                std::vector<std::shared_ptr<STrack>> tracked = tracker_.update(objects);


                // draw and publish
                std::tuple<std::vector<Detection>, std::vector<cv::Point2f>> result = processAndPublishTracks(tracked, frame, *inf_armor_);
                std::vector<Detection> result_detections_armor = std::get<0>(result);
                std::vector<cv::Point2f> result_points = std::get<1>(result);
                std::vector<std::string> classes_name;
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
                fps_frame_count++;
                auto fps_now = std::chrono::steady_clock::now();
                float fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        fps_now - fps_start_time).count();

                if (fps_elapsed >= 200.0f)
                {
                    fps_value = fps_frame_count * 1000.0f / fps_elapsed;
                    fps_frame_count = 0;
                    fps_start_time = fps_now;
                }

                total_frame_count++;

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
                cv::namedWindow("hik",cv::WINDOW_NORMAL);
                cv::resizeWindow("hik",800,600);
                cv::imshow("hik", frame);
                if (cv::waitKey(1) == 27)
                {
                    cv::destroyAllWindows();
                    running = false;
                }
            }
            auto avg_end_time = std::chrono::steady_clock::now();
            double total_time_sec = std::chrono::duration_cast<std::chrono::duration<double>>(avg_end_time - avg_start_time).count();
            double avg_fps = total_frame_count / std::max(total_time_sec, 1e-6);
            RCLCPP_INFO(
            this->get_logger(),
            "Test video finished. Total frames: %zu, Time: %.2f s, Average FPS: %.2f", total_frame_count, total_time_sec, avg_fps);
        }
    }


    // members
    bool runOnGPU_;
    std::string colcor_ = "blue";
    std::vector<std::string> classes_car_;
    std::vector<std::string> classes_armor_;
    std::unique_ptr<Inference_trt> inf_car_trt;
    //std::unique_ptr<Inference> inf_car_onnx;
    std::unique_ptr<Inference> inf_armor_;
    BYTETracker tracker_;
    cv::VideoCapture cap_;
    rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr publisher_detection;
    rclcpp::TimerBase::SharedPtr timer_;
    Config::HikConfig hik_config;
    cv::Mat frame;
    cv::Size frame_size;
    std::string save_path = "/home/zqz/ros2_ws/output.avi";
    std::vector<cv::Mat> stl_video;
    int fourcc_code;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<inference_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// ros2 run yolov8decbytetrack inference_node