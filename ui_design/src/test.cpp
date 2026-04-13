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
        // 初始化参数
        map_path = "/home/zqz/ros2_ws/image/map.jpg";
        history_length = 5;
        frame_count = 0;

        RCLCPP_INFO(this->get_logger(), "Map path: %s, history_length: %d", map_path.c_str(), history_length);

        // 初始化MapVisualizer
        vis_ = std::make_shared<MapVisualizer>(map_path, history_length);
        
        RCLCPP_INFO(this->get_logger(), "开始接收解算数据...");
        all_robot_subscription_ = this->create_subscription<tutorial_interfaces::msg::Detection>(
            "parsed_topic", 1,
            std::bind(&ui_design_node::robot_pose_callback, this, std::placeholders::_1));
        robot_publisher_ = this->create_publisher<tutorial_interfaces::msg::Detection>
        ("map_point_topic", 10);

        visualization_thread_ = std::thread(&ui_design_node::visualization_loop, this);
        worker_thread_ = std::thread(&ui_design_node::process_loop, this);
    }
    ~ui_design_node()
    {
        running_ = false;
        if (worker_thread_.joinable()) {worker_thread_.join();}
        if (visualization_thread_.joinable()) {visualization_thread_.join();}
    }
    
private:
    void robot_pose_callback(std::shared_ptr<const tutorial_interfaces::msg::Detection> parsed_msg)
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            msg_queue_.push(std::const_pointer_cast<tutorial_interfaces::msg::Detection>(parsed_msg));
            
        }
    void friend_pose_callback()
    {
        std::cout << "1" << std::endl;
    }

    void enemy_pose_callback()
    {
        std::cout << "1" << std::endl;
    }



    void process_loop()
    {
        while (running_ && rclcpp::ok())
        {
            tutorial_interfaces::msg::Detection::SharedPtr pared_msg = nullptr;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (!msg_queue_.empty())
                {
                    pared_msg = msg_queue_.front();
                    msg_queue_.pop();
                }
            }

            if (pared_msg)
            {
                for (auto& target : pared_msg->targets)
                {
                    std::string robot_id = target.class_name;
                    float x = target.x;
                    float y = -target.y;
                    if (std::find(red.begin(), red.end(), robot_id)!= red.end())
                    {
                        //vis_->addEnemy(robot_id, x, y);
                        vis_->addFriendly(robot_id, x, y);
                        tutorial_interfaces::msg::Target robot_target;
                        robot_target.class_name = robot_id;
                        robot_target.x = x;
                        robot_target.y = y;
                        all_msg.targets.push_back(robot_target);

                    }
                    else if (std::find(blue.begin(), blue.end(), robot_id)!= blue.end())
                    {
                        //vis_->addFriendly(robot_id, x, y);
                        vis_->addEnemy(robot_id, x, y);
                        tutorial_interfaces::msg::Target robot_target;
                        robot_target.class_name = robot_id;
                        robot_target.x = x;
                        robot_target.y = y;
                        all_msg.targets.push_back(robot_target);
                    }
                    //vis_->addEnemy(robot_id, x, y);
                    //vis_->addaddFriendly(robot_id, x, y);


                    RCLCPP_INFO(this->get_logger(), "Processed robot info: %s(%f,%f)", 
                                robot_id.c_str(), x, y);
                    robot_publisher_->publish(all_msg);
                    all_msg.targets.clear();
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
    }
       
    

    void visualization_loop()
    {
        while(running_ && rclcpp::ok())
        {
            try
            {
                vis_->update();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000/30));
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "visualizer error: %s", e.what());
            }
        }
    }

    // 成员变量声明
    int frame_count;
    std::string map_path;
    int history_length;

    rclcpp::Subscription<tutorial_interfaces::msg::Detection>::SharedPtr all_robot_subscription_;
    rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr robot_publisher_;
    tutorial_interfaces::msg::Detection all_msg;
    std::vector<detection_info> detection_data_list_;
    std::shared_ptr<MapVisualizer> vis_;
    std::vector<real_info> current_targets_;

    std::atomic<bool> running_{true};
    std::thread visualization_thread_;
    std::thread worker_thread_;

    std::mutex queue_mutex_;
    std::queue<tutorial_interfaces::msg::Detection::SharedPtr> msg_queue_;
    std::vector<std::string> red = {"R1", "R2", "R3", "R4", "R5", "R7"};
    std::vector<std::string> blue = {"B1", "B2", "B3", "B4", "B5", "B7"};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ui_design_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}