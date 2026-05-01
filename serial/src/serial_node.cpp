#include <rclcpp/rclcpp.hpp>
#include "tutorial_interfaces/msg/detection.hpp"
#include "tutorial_interfaces/msg/target.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
//#include "serial/serial.h"
#include "serial.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <memory>
#include <unordered_map>
#include <utility>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/highgui.hpp>






class serialnode : public rclcpp::Node
{
    public:

        bool initialize_serial(const std::string& port, int bandrate = 115200, char color = 'R')
        {
            try
            {
                SerialManager serial_manager(this->port, bandrate, color);
                //std::thread receive_thread(&SerialManager::receive_serial, &serial_manager);
                //receive_thread.detach();
                std::cout << "串口初始化成功" << std::endl;
                std::cout << "端口: " << port << ", 波特率: " << bandrate << ", 队伍: " << color << std::endl;
                std::thread send_thread(&SerialManager::send_serial, &serial_manager, &robot_map);
                send_thread.join();
                return true;
            }
            catch (const std::exception& e)
            {
                std::cerr << "串口初始化失败: " << e.what() << std::endl;
                return false;
            }
        }


        serialnode():Node("serial_node")
        {
            subscription_ = this->create_subscription<tutorial_interfaces::msg::Detection>("parsed_topic", 10, 
                std::bind(&serialnode::message_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),"开始接收数据...");
        }
    private:
        void message_callback(std::shared_ptr<const tutorial_interfaces::msg::Detection> all_msg)
        {
            for (const auto& target : all_msg->targets)
            {
                std::string robot_id = target.class_name;
                std::pair<float, float> position = {target.x, target.y};
                robot_map.insert({robot_id, position});
                initialize_serial(this->port);
            }
        }


        std::unordered_map<std::string, std::pair<float,float>> robot_map;
        rclcpp::Subscription<tutorial_interfaces::msg::Detection>::SharedPtr subscription_;
        std::vector<std::string> red = {"R1", "R2", "R3", "R4", "R5", "R7"};
        std::vector<std::string> blue = {"B1", "B2", "B3", "B4", "B5", "B7"};
        std::string port = "/dev/ttyUSB0";
        
        
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serialnode>());
    rclcpp::shutdown();
    return 0;
}