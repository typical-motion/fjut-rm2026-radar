#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "HikCamera.h"
#include <opencv2/opencv.hpp>

class HikCameraNode : public rclcpp::Node
{
public:
    HikCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("hik_camera_node", options)
    {
        // 获取参数
        this->declare_parameter<std::string>("sn", "");
        this->declare_parameter<double>("exposure", 10000.0);
        this->declare_parameter<double>("gain", 1.0);
        this->declare_parameter<double>("frame_rate", 30.0);
        this->declare_parameter<bool>("rotate_180", false);
        this->declare_parameter<int>("log_level", 1);

        // 组装配置
        Config::HikConfig config;
        config.sn = this->get_parameter("sn").as_string();
        config.exposure = this->get_parameter("exposure").as_double();
        config.gain = this->get_parameter("gain").as_double();
        config.frame_rate = this->get_parameter("frame_rate").as_double();
        config.rotate_180 = this->get_parameter("rotate_180").as_bool();
        config.log_level = this->get_parameter("log_level").as_int();

        camera_ = std::make_shared<HikCamera>(config);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("hik_camera/image_raw", 10);

        // 启动一个线程实时发布捕获到的图像
        publish_thread_ = std::thread(&HikCameraNode::publishLoop, this);
    }

    ~HikCameraNode() {
        running_ = false;
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }
    }

private:
    void publishLoop()
    {
        while (rclcpp::ok() && running_) {
            cv::Mat frame = camera_->getLatestFrame();
            if (!frame.empty())
            {
                auto msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(),
                    frame.channels() == 1 ? "mono8" : "bgr8",
                    frame
                ).toImageMsg();

                msg->header.stamp = this->now();
                image_pub_->publish(*msg);
            }
            // 可以适当休眠，防止CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::shared_ptr<HikCamera> camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::thread publish_thread_;
    bool running_ = true;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikCameraNode>());
    rclcpp::shutdown();
    return 0;
}