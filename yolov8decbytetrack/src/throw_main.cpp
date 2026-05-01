// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

#include <iostream>
#include <vector>
#include <getopt.h>
#include <tuple>
#include <functional>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <chrono>
#include "inference.h"
#include "BYTETracker.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace dnn;
using namespace byte_track;

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
}



auto inferencethrow(Inference& inf_car, cv::Mat& frame)
{
    std::vector<Detection> output = inf_car.runInference(frame);
    //for (auto& detection : detections)
//    {
 //       cv::rectangle(frame, cv::Rect(detection.box.x, detection.box.y, detection.box.width, detection.box.height), cv::Scalar(0, 255, 0), 2);
  //      cv::putText(frame, detection.className + " " + std::to_string(detection.confidence), cv::Point(detection.box.x, detection.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
   // }
    return output;
}

std::tuple<std::vector<std::string>, std::vector<cv::Point2f>> trackerthrow(std::vector<Detection>& output, BYTETracker& tracker, Inference& inf_armor, cv::Mat& frame)
{   
    std::vector<std::string> classes_armor{};
    std::vector<cv::Point2f> points;
    cv::Mat roi;
    std::vector<byte_track::Object> detections;
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
    }
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

            cv::rectangle(frame, textBox, (0,0,0), cv::FILLED);
            cv::putText(frame, trackInfo, cv::Point(box.x + 5, box.y - 5), 
                       cv::FONT_HERSHEY_DUPLEX, 0.7, Text_color, 1, 0);
            
            // 装甲板检测（在车辆ROI内）
             if (box.x >=0 && box.y >=0 && box.x + box.width <= frame.cols && box.y + box.height <= frame.rows && box.width > 0 && box.height > 0)
            {

            try
            {
              cv::Mat roi = frame(box);
              std::vector<Detection> output_armor = inf_armor.runInference(roi);
                for (const auto& detection_armor : output_armor)
                {
                    cv::Rect box_armor = detection_armor.box;
                    cv::Scalar armor_color(255, 0, 0); // 蓝色表示装甲板

                     if (box_armor.x >= 0 && box_armor.y >= 0 &&
                         box_armor.x + box_armor.width <= roi.cols &&
                         box_armor.y + box_armor.height <= roi.rows)
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
                    
                    // 绘制中心点
                    cv::circle(roi, cv::Point(box_armor.x + box_armor.width/2, 
                                            box_armor.y + box_armor.height/2), 
                            3, cv::Scalar(0, 0, 255), -1, 8, 0);
                    points.push_back(cv::Point2f(box_armor.x + box_armor.width/2, box_armor.y + box_armor.height/2));
                    classes_armor.push_back(detection_armor.className);
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
        return std::make_tuple(classes_armor, points);
    }

class Publisher : public rclcpp::Node
{
    public:
    Publisher():Node("publisher_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(500ms, [this]() {
        this-> timer_callback();
        });
    }

    void set_result_classes_armor(std::vector<std::string> &result_classes_armor)
    {
        result_classes_armor_ = result_classes_armor;
    }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::string> result_classes_armor_;
        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            for (const auto &result : result_classes_armor_)
            {
                message.data = result;
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            }

            result_classes_armor_.clear();
             
        }
};

class inference_node : public rclcpp::Node
{
    public:
    {
        inference_node():Node("inference_node"),runOnGpu_(true),
        {

            std::vector<std::string> classes_all{"car","armor","ignore","watcher","base"};
            std::vector<std::string> classes_armor{"B1", "B2", "B3", "B4", "B5", "B7","R1", "R2", "R3", "R4", "R5", "R7"};
            Inference inf_car("/home/zqz/ros2_ws/model2/car.onnx", cv::Size(640, 640), classes_all, runOnGPU);
            Inference inf_armor("/home/zqz/ros2_ws/model2/armor.onnx", cv::Size(640, 640), classes_armor, runOnGPU);
            publisher_ = this->create_publisher<std_msgs::msg::String
        }
    }


    private:
    {
        void timerCallback()
        {
            if (!cap_.isOpened()) return;

            cv::Mat frame;
            cap_ >> frame;
            if (frame.empty()) {
                RCLCPP_INFO(this->get_logger(), "Video finished");
                rclcpp::shutdown();
                return;
            }

            // --- 推理 1: 车辆 ---
            auto detections = inferencethrow(*inf_car_, frame);

            // --- 跟踪 + 装甲板 ---
            auto [result_classes_armor, result_points] =
                trackerthrow(detections, tracker_, *inf_armor_, frame);

            // --- 发布结果 ---
            for (const auto &cls : result_classes_armor) {
                std_msgs::msg::String msg;
                msg.data = cls;
                publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Publishing armor: %s", cls.c_str());
            }

            // --- 显示 ---
            cv::imshow("Detection", frame);
            if (cv::waitKey(1) == 27) {
                rclcpp::shutdown();
            }
        }



        bool runOnGPU_;
        std::vector<std::string> classes_car_;
        std::vector<std::string> classes_armor_;

        std::unique_ptr<Inference> inf_car_;
        std::unique_ptr<Inference> inf_armor_;

        BYTETracker tracker_;

        cv::VideoCapture cap_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    }

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto publisher_node = std::make_shared<Publisher>();
    std::vector<std::string> classes_all{"car","armor","ignore","watcher","base"};
    std::vector<std::string> classes_armor{"B1", "B2", "B3", "B4", "B5", "B7","R1", "R2", "R3", "R4", "R5", "R7"};
    bool runOnGPU = true;
    int fps_a = 0;
    cv::Mat roi;
    cv::Rect box;
    std::vector<double> fps_all;
    int frameCount = 0;
    double totalTime = 0;
    //VideoCapture cap("/home/zqz/ros2_ws/image/test20250512.mp4");
    VideoCapture cap("/home/zqz/ros2_ws/image/test.mp4");
    //VideoCapture cap("/home/zqz/ros2_ws/image/test_image.jpg");
    if (!cap.isOpened())
    {
        std::cerr << "Failed to open video" << std::endl;
        return -1;
    }
    int framewidth = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frameheight = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    cv::VideoWriter writer;
    writer.open("output.avi",
                cv::CAP_FFMPEG,
                cv::VideoWriter::fourcc('h', '2', '6', '4'),
                60, cv::Size(framewidth, frameheight));
    BYTETracker tracker(fps, 30);
    int num_frames = 0;
    int keyvalue = 0;
    int total_ms = 0;
    cv::namedWindow("Detection", cv::WINDOW_NORMAL);
    Inference inf_car("/home/zqz/ros2_ws/model2/car.onnx", cv::Size(640, 640), classes_all, runOnGPU);
    Inference inf_armor("/home/zqz/ros2_ws/model2/armor.onnx", cv::Size(640, 640), classes_armor, runOnGPU);

   while(true)
   {
        cv::Mat frame;
        cap >> frame;
        std::vector<std::string> result_classes_armor;
        std::vector<cv::Point2f> result_points;
        auto startTime = high_resolution_clock::now();
        if (frame.empty())
        {
            std::cerr << "Failed to read frame" << std::endl;
            break;
        } 
        std::vector<Detection> output = inferencethrow(inf_car, frame);
        std::tuple<std::vector<std::string>, std::vector<cv::Point2f>> result = trackerthrow(output, tracker, inf_armor, frame);
        result_classes_armor = std::get<0>(result);
        result_points = std::get<1>(result);
        publisher_node->set_result_classes_armor(result_classes_armor);
        rclcpp::spin(publisher_node);
        rclcpp::shutdown();

        auto endTime = high_resolution_clock::now();
        double durationMs = duration_cast<milliseconds>(endTime - startTime).count();
        double currentFPS = 1000.0 / durationMs;
        fps_all.push_back(currentFPS);

        putText(frame, "FPS:" + to_string(currentFPS).substr(0,4), Point(50,50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0),2);

        imshow("Detection", frame);
        writer.write(frame);

        frameCount++;
        if(waitKey(1) == 27) break;
   }
   std::cout << "over" << std::endl;
    double totalFPS = 0;
    for(double f : fps_all) totalFPS += f;
    double avgFPS = totalFPS / fps_all.size();
    cout << "Average FPS: " << avgFPS << endl;

    cap.release();
    writer.release();
    destroyAllWindows();

    return 0;

    
}






