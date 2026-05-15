#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/detection.hpp"

#include <libserial/SerialPort.h>
#include <chrono>
#include <iostream>
#include <cstdint>
#include <cstring>

using std::placeholders::_1;
using namespace std::chrono_literals;

// static void floattobytesbe(float value, uint8_t* msg)
// {
//     uint32_t intvalue;
//     memcpy(&intvalue, &value, sizeof(float));

//     intvalue = htonl(intvalue);

//     out[0] = (intvalue >> 24) & 0xFF;
//     out[1] = (intvalue >> 16) & 0xFF;
//     out[2] = (intvalue >> 8) & 0xFF;
//     out[3] = intvalue  & 0xFF;
// }

static void floattobytesle(float value, uint8_t* msg)
{
    uint32_t intvalue;
    memcpy(&intvalue, &value, sizeof(float));

    msg[0] = intvalue  & 0xFF;
    msg[1] = (intvalue >> 8) & 0xFF;
    msg[2] = (intvalue >> 16) & 0xFF;
    msg[3] = (intvalue >> 24) & 0xFF;
}

class SerialNode : public rclcpp::Node
{
public:
    SerialNode()
    : Node("serial_node")
    {
        try
        {
            serial_.Open("/dev/ttyUSB0");
            serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

            RCLCPP_INFO(this->get_logger(), "Serial opened successfully");
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Serial open failed: %s",
                         e.what());
        }

        sub_ = this->create_subscription<tutorial_interfaces::msg::Detection>(
            "light_detection_topic",
            10,
            std::bind(&SerialNode::callback, this, _1));

        last_send_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "serial node started");
    }

private:

    void callback(const tutorial_interfaces::msg::Detection::SharedPtr msg)
    {
        if (msg->targets.empty())
            return;

        // ⏱ 限频：20Hz
        auto now = this->now();
        if ((now - last_send_time_).seconds() < 0.05)
            return;

        last_send_time_ = now;

        // 👉 取第一个目标
        const auto& target = msg->targets[0];

        yaw_ = target.yaw;
        pitch_ = target.pitch;

        uint8_t pitch_byte[4]{};
        uint8_t yaw_byte[4]{};
        floattobytesle(yaw_, yaw_byte);
        floattobytesle(pitch_, pitch_byte);



        // 📦 串口数据帧
         std::string data =
             "yaw=" + std::to_string(yaw_) +
             ",pitch=" + std::to_string(pitch_) +
             "\n";
        std::string data_t[9]{};
        uint8_t frame[10];
        frame[0] = 0xff;
        for (int i = 0; i < 4; i++)
        {
            frame[i+1] = pitch_byte[i];
            frame[i+5] = yaw_byte[i];
        }
        frame[9] = 0x0d;
        
        std::cout << "TX HEX: ";

        for (int i = 0; i < 10; i++)
        {
            printf("%02X ", frame[i]);
        }

        std::cout << std::endl;

        try
        {
            if (serial_.IsOpen())
            {
                LibSerial::DataBuffer buffer(frame, frame + 10);
                serial_.Write(buffer);
            }

            // ================= 终端输出 =================
            std::cout << "yaw=" << yaw_
                      << " pitch=" << pitch_
                      << std::endl;

            RCLCPP_INFO(this->get_logger(),
                        "yaw=%.3f pitch=%.3f",
                        yaw_, pitch_);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "serial write error: %s",
                         e.what());
        }
    }

private:

    rclcpp::Subscription<tutorial_interfaces::msg::Detection>::SharedPtr sub_;

    LibSerial::SerialPort serial_;

    rclcpp::Time last_send_time_;

    // ⭐ 必须加的成员变量（关键修复）
    float yaw_ = 0.0f;
    float pitch_ = 0.0f;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SerialNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}