#include "MapVisualizer.h"
#include <thread>
#include <chrono>
#include <vector>
#include <queue>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "tutorial_interfaces/msg/detection.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"

struct detection_info
{
    std::string class_name;
    cv::Point2f points;
    
    std::string toString() const
    {
        return class_name + "(" + std::to_string(points.x) + "," + std::to_string(points.y) + ")";
    };
};
struct real_info
{
    std::string class_name;
    cv::Point3f points;
     std::string toString() const
    {
        return class_name + "(" + std::to_string(points.x) + "," + std::to_string(points.y) + ")";
    };
};


class ui_design_node : public rclcpp::Node 
{
public:
    ui_design_node() : Node("ui_design_node")
    {
        // 初始化参数 - 使用5m场地的地图
        map_path = "/home/zqz/ros2_ws/image/map_5m.jpg";  // 使用5m场地地图
        history_length = 20;  // 增加历史轨迹长度
        frame_count = 0;
        field_size_ = 5.0f;  // 5m场地

        RCLCPP_INFO(this->get_logger(), "=== 5m场地定位测试节点启动 ===");
        RCLCPP_INFO(this->get_logger(), "地图路径: %s", map_path.c_str());
        RCLCPP_INFO(this->get_logger(), "场地大小: %.1f x %.1f m", field_size_, field_size_);
        RCLCPP_INFO(this->get_logger(), "历史轨迹长度: %d", history_length);

        // 初始化MapVisualizer
        vis_ = std::make_shared<MapVisualizer>(map_path, history_length);
        
        RCLCPP_INFO(this->get_logger(), "开始接收解算数据...");
        
        // 订阅所有机器人位置话题
        all_robot_subscription_ = this->create_subscription<tutorial_interfaces::msg::Detection>(
            "parsed_topic", 10,  // 增大队列大小  parsed_topic
            std::bind(&ui_design_node::robot_pose_callback, this, std::placeholders::_1));
        
        // 发布地图点话题
        robot_publisher_ = this->create_publisher<tutorial_interfaces::msg::Detection>
            ("map_point_topic", 10);
        
        // 添加测试数据发布器（用于模拟测试）
        test_publisher_ = this->create_publisher<tutorial_interfaces::msg::Detection>
            ("test_data_topic", 10);
        
        // 启动定时器定期发布测试数据
        test_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000),  // 每2秒发布一次测试数据
            std::bind(&ui_design_node::publish_test_data, this));

        visualization_thread_ = std::thread(&ui_design_node::visualization_loop, this);
        worker_thread_ = std::thread(&ui_design_node::process_loop, this);
        
        // 启动性能监控线程
        monitor_thread_ = std::thread(&ui_design_node::monitor_performance, this);
    }
    
    ~ui_design_node()
    {
        running_ = false;
        if (worker_thread_.joinable()) {worker_thread_.join();}
        if (visualization_thread_.joinable()) {visualization_thread_.join();}
        if (monitor_thread_.joinable()) {monitor_thread_.join();}
    }
    
private:
    void robot_pose_callback(std::shared_ptr<const tutorial_interfaces::msg::Detection> parsed_msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        msg_queue_.push(std::const_pointer_cast<tutorial_interfaces::msg::Detection>(parsed_msg));
        
        // 统计接收消息数
        received_count_++;
    }

    void process_loop()
    {
        RCLCPP_INFO(this->get_logger(), "数据处理线程启动");
        
        while (running_ && rclcpp::ok())
        {
            tutorial_interfaces::msg::Detection::SharedPtr parsed_msg = nullptr;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (!msg_queue_.empty())
                {
                    parsed_msg = msg_queue_.front();
                    msg_queue_.pop();
                }
            }

            if (parsed_msg)
            {
                process_start_time_ = std::chrono::steady_clock::now();
                
                for (auto& target : parsed_msg->targets)
                {
                    std::string robot_id = target.class_name;
                    
                    // 5m场地坐标处理 - 确保坐标在0-5范围内
                    float x = std::max(0.0f, std::min(target.x, field_size_));
                    float y = std::max(0.0f, std::min(target.y, field_size_));
                    
                    // 根据队伍分配颜色
                    if (std::find(red.begin(), red.end(), robot_id) != red.end())
                    {
                        vis_->addEnemy(robot_id, x, y);  // 红方作为敌人（红色）
                        
                        RCLCPP_INFO(this->get_logger(), "[敌方红] %s: (%.2f, %.2f)", 
                                   robot_id.c_str(), x, y);
                    }
                    else if (std::find(blue.begin(), blue.end(), robot_id) != blue.end())
                    {
                        vis_->addFriendly(robot_id, x, y);  // 蓝方作为友方（蓝色）
                        
                        RCLCPP_INFO(this->get_logger(), "[友方蓝] %s: (%.2f, %.2f)", 
                                   robot_id.c_str(), x, y);
                    }
                    else
                    {
                        // 未知队伍默认作为中立单位（绿色）
                        vis_->addFriendly(robot_id, x, y);
                        RCLCPP_INFO(this->get_logger(), "[中立] %s: (%.2f, %.2f)", 
                                   robot_id.c_str(), x, y);
                    }

                    // 构建发布消息
                    tutorial_interfaces::msg::Target robot_target;
                    robot_target.class_name = robot_id;
                    robot_target.x = x;
                    robot_target.y = y;
                    all_msg.targets.push_back(robot_target);
                    
                    processed_count_++;
                }
                
                // 发布处理后的数据
                if (!all_msg.targets.empty())
                {
                    robot_publisher_->publish(all_msg);
                    
                    // 记录处理时间
                    auto process_end = std::chrono::steady_clock::now();
                    auto process_duration = std::chrono::duration_cast<std::chrono::microseconds>
                                           (process_end - process_start_time_).count();
                    
                    // 使用原子操作更新总处理时间
                    total_process_time_us_ += process_duration;
                    
                    all_msg.targets.clear();
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    
    void visualization_loop()
    {
        RCLCPP_INFO(this->get_logger(), "可视化线程启动");
        
        while(running_ && rclcpp::ok())
        {
            try
            {
                auto viz_start = std::chrono::steady_clock::now();
                
                vis_->update();
                
                auto viz_end = std::chrono::steady_clock::now();
                auto viz_duration = std::chrono::duration_cast<std::chrono::microseconds>
                                   (viz_end - viz_start).count();
                
                // 统计帧率
                frame_count_++;
                if (viz_duration > 0) {
                    current_fps_ = 1000000.0 / viz_duration;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(1000/60)); // 60 FPS
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "可视化错误: %s", e.what());
            }
        }
    }
    
    void monitor_performance()
    {
        RCLCPP_INFO(this->get_logger(), "性能监控线程启动");
        
        while (running_ && rclcpp::ok())
        {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            // 每5秒输出一次性能统计
            RCLCPP_INFO(this->get_logger(), "=== 性能统计 ===");
            RCLCPP_INFO(this->get_logger(), "接收消息数: %ld", received_count_.load());
            RCLCPP_INFO(this->get_logger(), "处理目标数: %ld", processed_count_.load());
            RCLCPP_INFO(this->get_logger(), "当前FPS: %.1f", current_fps_);
            
            long processed = processed_count_.load();
            if (processed > 0) {
                long long total_time_us = total_process_time_us_.load();
                double avg_time_us = static_cast<double>(total_time_us) / processed;
                RCLCPP_INFO(this->get_logger(), "平均处理时间: %.2f us", avg_time_us);
                RCLCPP_INFO(this->get_logger(), "总处理时间: %lld us", total_time_us);
            }
            
            // 可选：重置计数器
            // received_count_ = 0;
            // processed_count_ = 0;
            // total_process_time_us_ = 0;
        }
    }
    
    void publish_test_data()
    {
        // 发布测试数据，用于验证5m场地显示
        auto test_msg = std::make_shared<tutorial_interfaces::msg::Detection>();
        test_msg->header.stamp = this->get_clock()->now();
        
        // 生成一些测试点，覆盖5m场地的关键位置
        std::vector<std::tuple<std::string, float, float, std::string>> test_points = {
            {"R1", 0.5f, 0.5f, "red"},     // 红方基地
            {"B1", 4.5f, 4.5f, "blue"},    // 蓝方基地
            {"R2", 2.5f, 2.5f, "red"},     // 场地中心
            {"B2", 1.0f, 4.0f, "blue"},    // 左上区域
            {"R3", 4.0f, 1.0f, "red"},     // 右下区域
            {"R4", 1.5f, 3.5f, "red"},     // 随机点1
            {"B4", 3.5f, 1.5f, "blue"},    // 随机点2
            {"R5", 0.2f, 4.8f, "red"},     // 边界点1
            {"B5", 4.8f, 0.2f, "blue"},    // 边界点2
            {"R6", 2.0f, 3.0f, "red"},     // 中间区域
            {"B6", 3.0f, 2.0f, "blue"}     // 中间区域
        };
        
        for (const auto& [id, x, y, team] : test_points) {
            tutorial_interfaces::msg::Target target;
            target.class_name = id;
            target.x = x;
            target.y = y;
            test_msg->targets.push_back(target);
        }
        
        test_publisher_->publish(*test_msg);
        RCLCPP_INFO(this->get_logger(), "发布测试数据，包含%zu个目标，覆盖5m场地各区域", test_msg->targets.size());
    }

    // 成员变量声明
    int frame_count;
    std::string map_path;
    int history_length;
    float field_size_;  // 场地大小

    // ROS2通信相关
    rclcpp::Subscription<tutorial_interfaces::msg::Detection>::SharedPtr all_robot_subscription_;
    rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr robot_publisher_;
    rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr test_publisher_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    
    tutorial_interfaces::msg::Detection all_msg;
    std::vector<detection_info> detection_data_list_;
    std::shared_ptr<MapVisualizer> vis_;
    std::vector<real_info> current_targets_;

    // 线程相关
    std::atomic<bool> running_{true};
    std::thread visualization_thread_;
    std::thread worker_thread_;
    std::thread monitor_thread_;

    // 队列和锁
    std::mutex queue_mutex_;
    std::queue<tutorial_interfaces::msg::Detection::SharedPtr> msg_queue_;
    
    // 队伍标识
    std::vector<std::string> red = {"R1", "R2", "R3", "R4", "R5", "R6", "R7"};
    std::vector<std::string> blue = {"B1", "B2", "B3", "B4", "B5", "B6", "B7"};
    
    // 性能统计 - 使用整数类型存储微秒
    std::atomic<long> received_count_{0};
    std::atomic<long> processed_count_{0};
    std::atomic<long long> total_process_time_us_{0};  // 存储总处理时间（微秒）
    std::chrono::steady_clock::time_point process_start_time_;
    double current_fps_ = 0.0;
    std::atomic<int> frame_count_{0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ui_design_node>();
    
    RCLCPP_INFO(node->get_logger(), "5m场地定位测试节点已启动");
    RCLCPP_INFO(node->get_logger(), "等待接收数据...");
    RCLCPP_INFO(node->get_logger(), "提示：可使用以下命令发送测试数据：");
    RCLCPP_INFO(node->get_logger(), "  ros2 topic pub /test_data_topic tutorial_interfaces/msg/Detection \"{targets: [{class_name: 'R1', x: 1.0, y: 1.0}]}\"");
    
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "节点关闭");
    rclcpp::shutdown();
    return 0;
}

