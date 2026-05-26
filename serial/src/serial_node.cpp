#include <rclcpp/rclcpp.hpp>
#include "tutorial_interfaces/msg/detection.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
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
        // 串口参数声明 (可通过 YAML 文件或命令行参数覆盖)
        this->declare_parameter("port", "/dev/ttyUSB1");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("color", "B");
        this->declare_parameter("debug", false);

        // 获取参数
        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        std::string color_str = this->get_parameter("color").as_string();
        color_ = color_str.empty() ? 'R' : color_str[0];
        debug_mode_ = this->get_parameter("debug").as_bool();

        // 调试模式判断
        if (debug_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "调试模式已开启");
        }
        if (!initialize_serial())
        {
            RCLCPP_ERROR(this->get_logger(), "串口初始化失败，节点将继续运行但无串口通信");
            return;
        }

        Subscription_key = this->create_subscription<tutorial_interfaces::msg::Detection>("/radar/detection",10,std::bind(&serialnode::key_message_callback, this, std::placeholders::_1));
        subscription_ = this->create_subscription<tutorial_interfaces::msg::Detection>(
            "parsed_topic", 10,
            std::bind(&serialnode::message_callback, this, std::placeholders::_1));

        // 定时打印调试信息（每 2 秒）
        debug_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&serialnode::debug_print_callback, this));

        // 干扰波等级发布者
        wave_level_publisher_ = this->create_publisher<std_msgs::msg::Int32>("wave_level_topic", 10);
        wave_level_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&serialnode::wave_level_callback, this));

        RCLCPP_INFO(this->get_logger(), "串口节点初始化完成 端口=%s 波特率=%d 颜色=%c 调试=%s",
                    port_.c_str(), baudrate_, color_, debug_mode_ ? "开启" : "关闭");

        // 调试模式: 手动输入线程
        if (debug_mode_)
        {
            debug_input_thread_ = std::thread([this]()
            {
                while (running_)
                {
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    std::cout << "\n>>> 按 Enter 进入手动调试模式，或等待继续..." << std::endl;
                    std::string input;
                    std::getline(std::cin, input);
                    if (!running_) break;
                    if (serial_manager_)
                        serial_manager_->manual_debug_send();
                }
            });
        }
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
        if (debug_input_thread_.joinable()) debug_input_thread_.join();
        std::cout << "串口节点已关闭" << std::endl;
    }

private:
    bool initialize_serial()
    {
        try
        {
            serial_manager_ = std::make_unique<SerialManager>(port_, baudrate_, color_, debug_mode_);
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
                        if (has_data_)
                        {
                            serial_manager_->send_serial(&robot_map_);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
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

        has_data_ = true;
    }

    void key_message_callback(std::shared_ptr<const tutorial_interfaces::msg::Detection> msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        for (const auto& target : msg->targets)
        {
            robot_map_[target.class_name] = {target.x, target.y};
        }
        has_data_ = true;
    }

    void wave_level_callback()
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = encryption_level;
        wave_level_publisher_->publish(msg);
    }

    void debug_print_callback()
    {
        const char* game_type_names[] = {
            "未知",
            "超级对抗赛",
            "高校单项赛",
            "ICRA AI挑战赛",
            "联盟赛 3V3",
            "联盟赛步兵对抗"
        };
        const char* game_progress_names[] = {
            "未开始",
            "准备阶段",
            "自检阶段",
            "五秒倒计时",
            "比赛中",
            "结算中"
        };

        const char* type_str = (game_type >= 0 && game_type <= 5) ? game_type_names[game_type] : "?";
        const char* progress_str = (game_progress >= 0 && game_progress <= 5) ? game_progress_names[game_progress] : "?";

        std::cout << "\n===== [比赛状态] =====" << std::endl;
        std::cout << "  比赛类型: " << type_str << " (" << game_type << ")" << std::endl;
        std::cout << "  比赛阶段: " << progress_str << " (" << game_progress << ")"
                  << "  剩余时间: " << stage_remain_time << "s" << std::endl;
        std::cout << "  双倍易伤机会: " << double_vulnerability_chance
                  << "  对方触发中: " << (opponent_double_vulnerability ? "是" : "否") << std::endl;
        std::cout << "  己方加密等级: " << encryption_level
                  << "  可修改密钥: " << (key_modifiable ? "是" : "否") << std::endl;
        std::cout << "  密钥状态: " << (key_staus ? "已接收" : "未接收")
                  << "  wave_key: " << wave_key_received << std::endl;
        std::cout << "  当前密钥: [" << key_ << "]" << std::endl;
        std::cout << "======================\n" << std::endl;
    }

    std::unique_ptr<SerialManager> serial_manager_;
    std::unordered_map<std::string, std::pair<float, float>> robot_map_;
    std::mutex map_mutex_;
    rclcpp::Subscription<tutorial_interfaces::msg::Detection>::SharedPtr subscription_;
    rclcpp::Subscription<tutorial_interfaces::msg::Detection>::SharedPtr Subscription_key;
    std::string key_{};
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr wave_level_publisher_;
    rclcpp::TimerBase::SharedPtr wave_level_timer_;
    rclcpp::TimerBase::SharedPtr debug_timer_;
    std::thread send_thread_;
    std::thread receive_thread_;
    std::thread debug_input_thread_;
    std::atomic<bool> running_{true};
    std::atomic<bool> has_data_{false};

    std::string port_;
    int baudrate_;
    char color_;
    bool debug_mode_;
    bool key_changed_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serialnode>());
    rclcpp::shutdown();
    return 0;
}


//sudo chmod 666 /dev/ttyUSB0
//sudo chmod 666 /tmp/ttyV0
//sudo chmod 666 /dev/tnt0 