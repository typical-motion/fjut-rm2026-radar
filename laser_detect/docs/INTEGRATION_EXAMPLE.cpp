/**
 * @file INTEGRATION_EXAMPLE.cpp
 * @brief 卡尔曼滤波器在main.cpp中的集成示例
 *
 * 这个文件展示了如何将TargetAngleFilter集成到laser_inference_node中。
 * 请根据实际情况复制相关代码到main.cpp。
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include "laser_detect.hpp"
#include "kalman_filter.hpp"      // 新增：引入卡尔曼滤波器
#include "HikCamera.h"
#include "config.h"
#include "rclcpp/rclcpp.hpp"

// ============================================================================
// 改进的laser_inference_node类定义
// ============================================================================

class laser_inference_node : public rclcpp::Node
{
public:
    laser_inference_node() : Node("laser_inference_node"),
                             runOnGPU_(true),
                             // 【新增】初始化卡尔曼滤波器
                             // min_box_size=20px, outlier_threshold=30°
                             angle_filter(20, 30.0f)
    {
        // ... 相机配置代码 ...
        Config cfg;
        hik_config.sn = cfg.hik_cfg.sn;
        hik_config.exposure = cfg.hik_cfg.exposure;
        hik_config.gain = cfg.hik_cfg.gain;
        hik_config.frame_rate = cfg.hik_cfg.frame_rate;
        hik_config.rotate_180 = cfg.hik_cfg.rotate_180;
        hik_config.log_level = cfg.hik_cfg.log_level;

        // ... TensorRT模型加载 ...
        std::vector<std::string> classes_all{"light"};
        std::string light_engine = "/home/zqz/ros2_ws/model2/light.engine";
        inf_light_trt = std::make_unique<Inference_trt>(light_engine,
                                                         cv::Size(640, 640),
                                                         classes_all,
                                                         runOnGPU_);

        inf_light_trt->setModelConfidenceThreshold(0.25f);
        inf_light_trt->setLetterBoxForSquare(true);

        // ... 其他初始化 ...
        publisher_detection = this->create_publisher<tutorial_interfaces::msg::Detection>(
            "light_detection_topic", 10);
        timer_ = this->create_wall_timer(
            20ms, std::bind(&laser_inference_node::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "laser_inference_node initialized with Kalman filter");
    }

private:
    /**
     * @brief HikCamera模式下的处理
     */
    void processHikCameraMode()
    {
        HikCamera hik_camera(hik_config);
        hik_camera.info();
        RCLCPP_INFO(this->get_logger(), "Starting HikCamera capture");

        bool running = true;
        cv::VideoWriter writer;
        int frame_count = 0;

        auto start_time = std::chrono::high_resolution_clock::now();
        auto fps_start_time = std::chrono::steady_clock::now();
        int fps_frame_count = 0;
        float fps_value = 0.0f;

        // 【新增】重置滤波器
        angle_filter.reset();

        while (running)
        {
            cv::Mat frame_rgb = hik_camera.getLatestFrame();
            stl_video.push_back(frame_rgb);
            cv::cvtColor(frame_rgb, frame, cv::COLOR_RGB2BGR);

            if (frame.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get frame");
                return;
            }

            // === 推理 ===
            std::vector<Detection> detections;
            try
            {
                detections = inf_light_trt->runInference_TensorRT(frame);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
                continue;
            }

            // === 处理检测结果 ===
            if (!detections.empty())
            {
                auto msg = tutorial_interfaces::msg::Detection();

                for (const auto& detection : detections)
                {
                    // 绘制检测框
                    cv::rectangle(frame, detection.box, cv::Scalar(0, 255, 0), 2);

                    std::string label = detection.className + ": " +
                                        std::to_string(detection.confidence);
                    int baseline = 0;
                    cv::Size text_size = cv::getTextSize(
                        label, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &baseline);
                    cv::Point text_pos(detection.box.x,
                                      std::max(0, detection.box.y - 10));
                    cv::putText(frame, label, text_pos,
                               cv::FONT_HERSHEY_SIMPLEX, 0.9,
                               cv::Scalar(0, 255, 0), 2);

                    // 【改进】使用卡尔曼滤波处理角度计算
                    float yaw = 0.0f, pitch = 0.0f;
                    bool valid = angle_filter.processDetection(
                        detection.box.x, detection.box.y,
                        detection.box.width, detection.box.height,
                        camera_focal_length_x, camera_focal_length_y,
                        image_center_x, image_center_y,
                        yaw, pitch
                    );

                    if (!valid)
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                           500,  // 每500ms最多打印一次
                                           "Detection box too small (w=%.0f, h=%.0f) or outlier detected",
                                           detection.box.width, detection.box.height);
                        continue;
                    }

                    // 填充消息
                    tutorial_interfaces::msg::Target target_msg;
                    target_msg.yaw = yaw;
                    target_msg.pitch = pitch;
                    target_msg.confidence = detection.confidence;
                    target_msg.class_name = detection.className;
                    msg.targets.push_back(target_msg);

                    RCLCPP_INFO(this->get_logger(),
                               "Target detected: yaw=%.2f°, pitch=%.2f° "
                               "(confidence=%.3f, box=%dx%d)",
                               yaw, pitch, detection.confidence,
                               (int)detection.box.width,
                               (int)detection.box.height);

                    // 在图像上显示滤波后的角度
                    std::string angle_label = cv::format("yaw:%.1f° pitch:%.1f°",
                                                         yaw, pitch);
                    cv::putText(frame, angle_label,
                               cv::Point(detection.box.x, detection.box.y - 30),
                               cv::FONT_HERSHEY_SIMPLEX, 0.7,
                               cv::Scalar(0, 255, 255), 2);
                }

                if (!msg.targets.empty())
                {
                    publisher_detection->publish(msg);
                }
            }

            frame_count++;
            fps_frame_count++;

            // FPS计算
            auto fps_now = std::chrono::steady_clock::now();
            float fps_elapsed = std::chrono::duration_cast<
                std::chrono::milliseconds>(fps_now - fps_start_time).count();

            if (fps_elapsed >= 200.0f)
            {
                fps_value = fps_frame_count * 1000.0f / fps_elapsed;
                fps_frame_count = 0;
                fps_start_time = fps_now;
            }

            // 显示FPS
            cv::rectangle(frame, cv::Point(5, 5), cv::Point(220, 45),
                         cv::Scalar(0, 0, 0), -1);
            cv::putText(frame, cv::format("FPS: %.2f", fps_value),
                       cv::Point(10, 35), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                       cv::Scalar(0, 255, 0), 2);

            cv::namedWindow("hik", cv::WINDOW_NORMAL);
            cv::resizeWindow("hik", 800, 600);
            cv::imshow("hik", frame);

            if (frame_count % 30 == 0)
            {
                auto current_time = std::chrono::high_resolution_clock::now();
                auto elapsed = current_time - start_time;
                double total_sec = std::chrono::duration_cast<
                    std::chrono::duration<double>>(elapsed).count();
                double fps_calc = frame_count / total_sec;
                RCLCPP_INFO(this->get_logger(),
                           "Frames: %d | FPS: %.2f", frame_count, fps_calc);
            }

            if (cv::waitKey(1) == 27)
            {
                running = false;
                RCLCPP_INFO(this->get_logger(), "Stopping capture");
            }
        }

        cv::destroyAllWindows();
    }

    /**
     * @brief 视频文件模式下的处理
     */
    void processVideoFileMode()
    {
        cap_.open("/home/zqz/ros2_ws/image/raw.mp4");
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Video file opened");

        bool running = true;
        auto start_time = std::chrono::steady_clock::now();
        auto fps_start_time = std::chrono::steady_clock::now();
        int fps_frame_count = 0;
        float fps_value = 0.0f;
        int frame_count = 0;

        // 【新增】重置滤波器
        angle_filter.reset();

        while (running)
        {
            cap_ >> frame;
            if (frame.empty())
            {
                running = false;
                break;
            }

            // === 推理 ===
            std::vector<Detection> detections;
            try
            {
                detections = inf_light_trt->runInference_TensorRT(frame);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
                continue;
            }

            // === 处理检测结果 ===
            if (!detections.empty())
            {
                auto msg = tutorial_interfaces::msg::Detection();

                for (const auto& detection : detections)
                {
                    cv::rectangle(frame, detection.box, cv::Scalar(0, 255, 0), 2);

                    std::string label = detection.className + ": " +
                                        std::to_string(detection.confidence);
                    cv::putText(frame, label,
                               cv::Point(detection.box.x,
                                        std::max(0, detection.box.y - 10)),
                               cv::FONT_HERSHEY_SIMPLEX, 0.9,
                               cv::Scalar(0, 255, 0), 2);

                    // 【改进】使用卡尔曼滤波
                    float yaw = 0.0f, pitch = 0.0f;
                    bool valid = angle_filter.processDetection(
                        detection.box.x, detection.box.y,
                        detection.box.width, detection.box.height,
                        camera_focal_length_x, camera_focal_length_y,
                        image_center_x, image_center_y,
                        yaw, pitch
                    );

                    if (!valid)
                    {
                        continue;
                    }

                    tutorial_interfaces::msg::Target target_msg;
                    target_msg.yaw = yaw;
                    target_msg.pitch = pitch;
                    target_msg.confidence = detection.confidence;
                    msg.targets.push_back(target_msg);

                    RCLCPP_INFO(this->get_logger(),
                               "Frame %d: yaw=%.2f°, pitch=%.2f°",
                               frame_count, yaw, pitch);

                    std::string angle_label =
                        cv::format("yaw:%.1f° pitch:%.1f°", yaw, pitch);
                    cv::putText(frame, angle_label,
                               cv::Point(detection.box.x, detection.box.y - 30),
                               cv::FONT_HERSHEY_SIMPLEX, 0.7,
                               cv::Scalar(0, 255, 255), 2);
                }

                if (!msg.targets.empty())
                {
                    publisher_detection->publish(msg);
                }
            }

            frame_count++;
            fps_frame_count++;

            auto fps_now = std::chrono::steady_clock::now();
            float fps_elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    fps_now - fps_start_time).count();

            if (fps_elapsed >= 200.0f)
            {
                fps_value = fps_frame_count * 1000.0f / fps_elapsed;
                fps_frame_count = 0;
                fps_start_time = fps_now;
            }

            cv::rectangle(frame, cv::Point(5, 5), cv::Point(220, 45),
                         cv::Scalar(0, 0, 0), -1);
            cv::putText(frame, cv::format("FPS: %.2f", fps_value),
                       cv::Point(10, 35), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                       cv::Scalar(0, 255, 0), 2);

            cv::namedWindow("Detection", cv::WINDOW_NORMAL);
            cv::resizeWindow("Detection", 800, 600);
            cv::imshow("Detection", frame);

            if (cv::waitKey(1) == 27)
            {
                running = false;
            }
        }

        cv::destroyAllWindows();

        auto end_time = std::chrono::steady_clock::now();
        double total_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                end_time - start_time).count();
        double avg_fps = frame_count / std::max(total_sec, 1e-6);

        RCLCPP_INFO(this->get_logger(),
                   "Video processing complete: %d frames, %.2f sec, "
                   "%.2f FPS",
                   frame_count, total_sec, avg_fps);
    }

    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Select mode (hik/test):");
        std::string mode;
        std::cin >> mode;

        if (mode == "hik")
        {
            processHikCameraMode();
        }
        else if (mode == "test")
        {
            processVideoFileMode();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s", mode.c_str());
        }
    }

    // === 成员变量 ===
    bool runOnGPU_;
    std::unique_ptr<Inference_trt> inf_light_trt;
    cv::VideoCapture cap_;
    rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr
        publisher_detection;
    rclcpp::TimerBase::SharedPtr timer_;
    Config::HikConfig hik_config;
    cv::Mat frame;
    std::string save_path = "/home/zqz/ros2_ws/output.avi";
    std::vector<cv::Mat> stl_video;
    int fourcc_code;

    // 相机参数
    const float camera_focal_length_x = 500.0f;
    const float camera_focal_length_y = 500.0f;
    const float image_center_x = 640.0f;
    const float image_center_y = 360.0f;
    const float camera_pitch_offset = 0.0f;
    const float camera_yaw_offset = 0.0f;

    // 【新增】卡尔曼滤波器
    TargetAngleFilter angle_filter;
};

// ============================================================================
// main函数
// ============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<laser_inference_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// ros2 run yolov8decbytetrack inference_node
