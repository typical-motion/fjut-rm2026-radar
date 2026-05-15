// Ultralytics 🚀 AGPL-3.0 License

#include <iostream>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <chrono>
#include <cstring>

#include <libserial/SerialPort.h>

#include "laser_detect2.hpp"
#include "kalman_filter.hpp"
#include "GalaxyCamera.h"
#include "config.h"

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/detection.hpp"
#include "tutorial_interfaces/msg/target.hpp"

using namespace std::chrono;

// ================= TensorRT inference =================
static std::vector<Detection> inferencethrow_trt(Inference_trt& inf, cv::Mat& frame)
{
    return inf.runInference_TensorRT(frame);
}

// ================= Node =================
class laser_inference_node : public rclcpp::Node
{
public:
    laser_inference_node(const std::string& mode)
    : Node("laser_inference_node"),
      runOnGPU_(true),
      angle_filter(20, 30.0f),
      mode_(mode)
    {
        Config cfg;
        hik_config = cfg.hik_cfg;

        angle_filter.setKalmanParams(0.01f, 4.0f, 0.01f, 4.0f);

        std::vector<std::string> classes_light{"light"};
        std::vector<std::string> classes_plane{"plane_0", "plane_1", "plane_2", "plane_3", "plane_4"};

        std::string engine_path = "/home/zqz/ros2_ws/model2/laser-v11.engine";
        std::string plane_path = "/home/zqz/ros2_ws/model2/plane.engine";

        inf_light_trt = std::make_unique<Inference_trt>(
            engine_path, cv::Size(640, 640), classes_light, runOnGPU_);
        inf_plane_trt = std::make_unique<Inference_trt>(
            plane_path, cv::Size(640, 640), classes_plane, runOnGPU_);

        inf_plane_trt->setModelConfidenceThreshold(0.25f);
        inf_light_trt->setModelConfidenceThreshold(0.6f);
        inf_light_trt->setLetterBoxForSquare(true);

        publisher_detection =
            this->create_publisher<tutorial_interfaces::msg::Detection>(
                "light_detection_topic", 10);

        if (mode_ == "hik")
        {
            cam_ = std::make_unique<GalaxyCamera>();
            cam_->init();
        }
        else if (mode_ == "test")
        {
            cap_.open("/home/zqz/Downloads/ros2_daheng2_ws/Video_20260416080506563.avi");
        }

        angle_filter.reset();

        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&laser_inference_node::timerCallback, this));

        cv::namedWindow("detection", cv::WINDOW_NORMAL);
    }

private:
    void timerCallback()
    {
        cv::Mat frame;

        // ---- 获取帧 ----
        if (mode_ == "hik")
        {
            cv::Mat frame_rgb;
            if (!cam_->getFrame(frame_rgb))
                return;
            cv::cvtColor(frame_rgb, frame, cv::COLOR_RGB2BGR);
        }
        else if (mode_ == "test")
        {
            cap_ >> frame;
            if (frame.empty())
            {
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                return;
            }
        }

        if (frame.empty())
            return;

        // ---- 双层神经网络检测 ----
        msg.targets.clear();

        // 第一层：检测 ROI (plane)
        std::vector<Detection> detections_plane = inferencethrow_trt(*inf_plane_trt, frame);

        for (const auto& plane : detections_plane)
        {
            cv::Rect plane_box(
                (int)plane.box.x,
                (int)plane.box.y,
                (int)plane.box.width,
                (int)plane.box.height
            );

            // Clamp to frame boundaries
            cv::Rect clamped_box = plane_box & cv::Rect(0, 0, frame.cols, frame.rows);
            if (clamped_box.width <= 0 || clamped_box.height <= 0)
                continue;

            // 绘制 plane 框（蓝色）
            cv::rectangle(frame, plane_box, cv::Scalar(255, 0, 0), 2);
            char text[50];
            snprintf(text, sizeof(text), "plane %.2f", plane.confidence);
            cv::putText(frame, text,
                        cv::Point(plane_box.x, plane_box.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 0, 0), 2);

            // 裁剪 ROI
            cv::Mat roi = frame(clamped_box).clone();

            // 第二层：在 ROI 内检测 light
            std::vector<Detection> detections_light = inferencethrow_trt(*inf_light_trt, roi);

            for (const auto& light : detections_light)
            {
                // 坐标偏移回全图
                int lx = (int)light.box.x + clamped_box.x;
                int ly = (int)light.box.y + clamped_box.y;
                int lw = (int)light.box.width;
                int lh = (int)light.box.height;

                cv::Rect light_box(lx, ly, lw, lh);

                // 绘制 light 框（绿色）
                cv::rectangle(frame, light_box, cv::Scalar(0, 255, 0), 2);
                char t[50];
                snprintf(t, sizeof(t), "light %.2f", light.confidence);
                cv::putText(frame, t,
                            cv::Point(lx, ly - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 255, 0), 2);

                // 卡尔曼滤波
                float yaw, pitch;
                bool ok = angle_filter.processDetection(
                    (float)lx, (float)ly,
                    (float)lw, (float)lh,
                    camera_focal_length_x, camera_focal_length_y,
                    image_center_x, image_center_y,
                    yaw, pitch);

                if (ok)
                {
                    tutorial_interfaces::msg::Target t;
                    t.yaw = yaw;
                    t.pitch = pitch;
                    msg.targets.push_back(t);
                }
            }
        }

        // 每帧发布一次
        publisher_detection->publish(msg);

        cv::imshow("detection", frame);
        if (cv::waitKey(1) == 27)
            rclcpp::shutdown();
    }

private:
    std::string mode_;
    bool runOnGPU_;
    std::unique_ptr<Inference_trt> inf_light_trt;
    std::unique_ptr<Inference_trt> inf_plane_trt;

    tutorial_interfaces::msg::Detection msg;
    rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr publisher_detection;
    rclcpp::TimerBase::SharedPtr timer_;

    Config::HikConfig hik_config;
    TargetAngleFilter angle_filter;

    std::unique_ptr<GalaxyCamera> cam_;
    cv::VideoCapture cap_;

    const float camera_focal_length_x = 500.0f;
    const float camera_focal_length_y = 500.0f;
    const float image_center_x = 640.0f;
    const float image_center_y = 360.0f;
};

// ================= main =================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string mode;
    std::cout << "模式选择(test/hik): ";
    std::cin >> mode;

    if (mode != "hik" && mode != "test")
    {
        std::cerr << "Invalid mode. Use 'hik' or 'test'." << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<laser_inference_node>(mode);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
