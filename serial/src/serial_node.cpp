#include <rclcpp/rclcpp.hpp>
#include "tutorial_interfaces/msg/detection.hpp"
#include "serial.hpp"

#include <iostream>
#include <string>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

class serialnode : public rclcpp::Node
{
public:
    serialnode()
        : Node("serial_node")
    {
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("color", "R");
        //提示信息
        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        std::string color_str = this->get_parameter("color").as_string();
        color_ = color_str.empty() ? 'R' : color_str[0];
        //获取参数
        if (!initialize_serial())
        {
            RCLCPP_ERROR(this->get_logger(), "串口初始化失败，节点将继续运行但无串口通信");
            return;
        }

        subscription_ = this->create_subscription<tutorial_interfaces::msg::Detection>(
            "parsed_topic", 10,
            std::bind(&serialnode::message_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "串口节点初始化完成 端口=%s 波特率=%d 颜色=%c",
                    port_.c_str(), baudrate_, color_);
    }

    ~serialnode()
    {
        running_ = false;
        if (serial_manager_)
        {
            serial_manager_->stop();
        }
        if (send_thread_.joinable()) send_thread_.join();
        if (receive_thread_.joinable()) receive_thread_.join();
        std::cout << "串口节点已关闭" << std::endl;
    }

private:
    bool initialize_serial()
    {
        try
        {
            serial_manager_ = std::make_unique<SerialManager>(port_, baudrate_, color_);
            if (!serial_manager_->serial_set())
            {
                return false;
            }

            receive_thread_ = std::thread(&SerialManager::receive_serial, serial_manager_.get());

            send_thread_ = std::thread([this]()
            {
                while (running_)
                {
                    {
                        std::lock_guard<std::mutex> lock(map_mutex_);
                        serial_manager_->send_serial(&robot_map_);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            });

            std::cout << "串口初始化成功" << std::endl;
            return true;
        }
        catch (const std::exception& e)
        {
            std::cerr << "串口初始化失败: " << e.what() << std::endl;
            return false;
        }
    }

    void message_callback(std::shared_ptr<const tutorial_interfaces::msg::Detection> all_msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        for (const auto& target : all_msg->targets)
        {
            robot_map_[target.class_name] = {target.x, target.y};
        }
    }

    std::unique_ptr<SerialManager> serial_manager_;
    std::unordered_map<std::string, std::pair<float, float>> robot_map_;
    std::mutex map_mutex_;
    rclcpp::Subscription<tutorial_interfaces::msg::Detection>::SharedPtr subscription_;

    std::thread send_thread_;
    std::thread receive_thread_;
    std::atomic<bool> running_{true};

    std::string port_;
    int baudrate_;
    char color_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serialnode>());
    rclcpp::shutdown();
    return 0;
}
