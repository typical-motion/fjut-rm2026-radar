#include "location.h"
#include "config.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "tutorial_interfaces/msg/detection.hpp"
#include "tutorial_interfaces/msg/target.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <queue>
#include <mutex>
#include <thread>

using namespace std::chrono;

struct detection_info
{
    std::string class_name;
    cv::Point2f points;
    //int class_number;
    //float confidence;
    std::string toString() const
    {
        return class_name + "(" + std::to_string(points.x) + "," + std::to_string(points.y) + ")";
    };
};

struct real_points
{
    std::string class_name;
    cv::Point3f points;
    real_points(std::string class_name, cv::Point3f points) : class_name(class_name), points(points) {}
    void display() const
    {
        std::cout << "class_name: " << class_name << ", real_points.x: " << points.x << ", real_points.y: " << points.y << ", real_points.z: " << points.z << std::endl;
    }
};




class location_node : public rclcpp::Node 
{
    public:
        int img_width = 3072, img_height = 2048;
        Location locator;
        location_node():Node("location_node")
        {
            RCLCPP_INFO(this->get_logger() , "开始接受被识别的装甲板的信息");
            subscription_ = this->create_subscription<tutorial_interfaces::msg::Detection>( 
                    "detection_topic",1,
                    std::bind(&location_node::timerCallback_subscription, this,std::placeholders::_1));
            publisher_ = this->create_publisher<tutorial_interfaces::msg::Detection>("parsed_topic", 10);
            //timer_ = this->create_wall_timer(40ms,std::bind(&location_node::publish_msg_callback, this));

            //sub_msg_thread_ = std::thread(&location_node::sub_msg_thread, this);
            //pub_msg_thread_ = std::thread(&location_node::pub_msg_thread, this);
            
        }
        //~location_node()
        //{
        //    running_ = true;
        //    if (sub_msg_thread_.joinable()){sub_msg_thread_.join();}
        //    if (pub_msg_thread_.joinable()){pub_msg_thread_.join();}
        //}
        
    private:
        std::string color_ = "red";//red or blue
        std::vector<cv::Point2d> car_points_;
        std::vector<cv::Point3d> real_points_;
        //std::vector<cv::Point3i> real_points_int;
        rclcpp::Subscription<tutorial_interfaces::msg::Detection>::SharedPtr subscription_;
        rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr publisher_;
        //rclcpp::TimerBase::SharedPtr timer_;
        std::vector<detection_info> detection_info_;
        std::vector<real_points> real_points_msg_;
        std::vector<std::string> red = {"R1", "R2", "R3", "R4", "R5", "R7"};
        std::vector<std::string> blue = {"B1", "B2", "B3", "B4", "B5", "B7"};
        tutorial_interfaces::msg::Detection parsed_msg;

        //std::atomic<bool> running_{true};
        //std::thread sub_msg_thread_;
        //std::thread pub_msg_thread_;

        //std::mutex queue_mutex_;
        //std::queue<tutorial_interfaces::msg::Detection::SharedPtr> msg_queue_;
        //void sub_msg_thread(std::shared_ptr<const tutorial_interfaces::msg::Detection> msg)
        //{
         //   std::lock_guard<std::mutex> lock(queue_mutex_);

        //}
        //void pub_msg_thread()
        //{

        //}
        void timerCallback_subscription(std::shared_ptr<const tutorial_interfaces::msg::Detection> msg)
        {   

            int img_width = 3072, img_height = 2048;
            cv::Mat frame(img_height, img_width, CV_8UC3, cv::Scalar::all(0));
            locator.drawRegions(frame, 2);
           for (const auto& detection : msg->targets)
           {
                detection_info info;
                //real_points real;
                bool should_process = false;
                info.class_name = detection.class_name;
            //    if (color_ == "red")
           //     {
            //        if (std::find(red.begin(), red.end(), info.class_name) != red.end())
             //       {
             //           should_process = true;
             //           RCLCPP_DEBUG(this->get_logger(), info.class_name.c_str());
             //       }
              //  }
                
              //  else if (color_ == "blue")
             //   {
              //      if (std::find(blue.begin(), blue.end(), info.class_name) != blue.end())
              //      {
                //        should_process = true;
               //         RCLCPP_DEBUG(this->get_logger(), info.class_name.c_str());
                //    }
                //}
                if (!should_process)
                {
                    info.points = cv::Point2f(detection.x, detection.y);
                    info.class_name = detection.class_name;
                    //RCLCPP_INFO(this->get_logger(), "detection_info: %s",info.toString().c_str());
                    //RCLCPP_INFO(this->get_logger(), "detection_info: %s",info.toString().c_str());
                    cv::Point3f points = process_date(info.points);
                    tutorial_interfaces::msg::Target target;
                    target.class_name = info.class_name;
                    target.x = points.x;
                    target.y = points.y;
                    parsed_msg.targets.push_back(target);
                    //std::cout << real_points_.size() << std::endl;
                    cv::circle(frame, info.points, 5, cv::Scalar(0, 0, 255), -1);
                    cv::putText(frame, "Test Point: " + std::to_string(info.points.x) + "," + std::to_string(info.points.y),
                    info.points + cv::Point2f(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
                    //std::cout << "解算结果(原): X=" << real_points_.back().x << ", Y=" << real_points_.back().y << ", Z=" << real_points_.back().z << std::endl;
                    //std::cout << "解算结果: X=" << 28 - real_points_.back().x << ", Y=" << -real_points_.back().y << std::endl;
                    //std::string result_text = "解算结果: X=" + std::to_string(real_points_.back().x) + ", Y=" + std::to_string(real_points_.back().y);
                    //cv::putText(frame, result_text, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
                    //RCLCPP_INFO(this->get_logger(), "real_points.x: %f, real_points.y: %f, real_points.z: %f",real_points_.back().x, real_points_.back().y, real_points_.back().z);
                    //parsed_msg.targets.push_back(tutorial_interfaces::msg::Detection::Target());
                }
                

           }
           publisher_->publish(parsed_msg);
           parsed_msg.targets.clear();
           cv::Mat frame1; 
           cv::resize(frame, frame1, cv::Size(1500, 1000));
           cv::imshow("Regions and Test Point", frame1);
           if(cv::waitKey(1) == 'q')
           {
                cv::destroyAllWindows();
           }
        }

        cv::Point3d process_date(cv::Point2f &points)
        {
            cv::Point2d cv_point_2d(points.x, points.y);
            cv::Point3d cv_point_3d = locator.parse(cv_point_2d);
            return cv_point_3d;
        }


      
        

    };

        

           

           


            //std::cout << "camera_matrix.type = " << locator.camera_matrix.type() << std::endl;
            //std::cout << "camera_matrix.size = " << locator.camera_matrix.size() << std::endl; 
            //std::cout << "dist_coeffs.type = " << locator.dist_coeffs.type() << std::endl;
            //std::cout << "dist_coeffs.size = " << locator.dist_coeffs.size() << std::endl;
                
            
            //detection_info detection;
            //detection.class_number = msg->class_number;
            //detection.confidence = msg->confidence;
            //detection.class_name = msg->targets.class_name;
            //detection.points = cv::Point2f(msg->x, msg->y);
            //detection_info_.push_back(detection);
            //RCLCPP_INFO(this->get_logger(), "detection_info: %s",detection.toString().c_str());
            //RCLCPP_INFO(this->get_logger(), "msg get: x=%.2f, y=%.2f, class=%s" , msg->x, msg->y, msg->class_name.c_str());
            //std::cout << "detection_info size: " << detection_info_.size() << std::endl;
            //RCLCPP_INFO(this->get_logger(), "msg get: x=%.2f, y=%.2f, class=%s" , detection.points.x, detection.points.y, detection.class_name.c_str());
            //std::cout << typeid(decltype(detection.point.x)).name() << std::endl;
            //real_points_.push_back(locator.parse(detection.points));
            //RCLCPP_INFO(this->get_logger(), "real_points.x: %f, real_points.y: %f, real_points.z: %f",real_points_.back().x, real_points_.back().y, real_points_.back().z);
                //cv::Point2f pt_cv(it.points.x, it.points.y);
            //std::cout << "detection.back()" << detection_info_.back().points << std::endl;
            //cv::Point2d test_pt(detection.points.x, detection.points.y);
            //cv::Point2d test_pt(179.00,315.00);
            //std::cout << "test_pt: " << test_pt << std::endl;
            //cv::circle(frame, test_pt, 5, cv::Scalar(0, 0, 255), -1);
            //cv::putText(frame, "Test Point: " + std::to_string(test_pt.x) + "," + std::to_string(test_pt.y),
            //test_pt + cv::Point2d(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
            //cv::Point3d pt_3d = locator.parse(test_pt);
            //cv::Point3f pt_3d_2 = locator.parse(detection_info_.back().points);
            //std::cout << pt_3d.x << " ,"
            //          << pt_3d.y << " ,"
            //          << pt_3d.z << std::endl;
            //real_points_.push_back(pt_3d);
            //real_points_int.assign(real_points_.begin(), real_points_.end());
            //cv::circle(frame, (real_points_int.back().x, - real_points_int.back().y), 5, cv::Scalar(0, 255, 0), -1)
            //RCLCPP_INFO(this->get_logger(), "real_points.x: %f, real_points.y: %f, real_points.z: %f",real_points_.back().x, real_points_.back().y, real_points_.back().z);
            //parsed_msg.class_name = detection.class_name;                                                                                                                                            
            //parsed_msg.center.x = pt_3d.x;
            //parsed_msg.center.y = - pt_3d.y;
            //parsed_msg.center.z = pt_3d.z;
            //publisher_->publish(parsed_msg);
            //std::cout << "解算结果(原): X=" << pt_3d.x << ", Y=" << pt_3d.y << ", Z=" << pt_3d.z << std::endl;
            //std::cout << "解算结果: X=" << 28 - pt_3d.x << ", Y=" << -pt_3d.y << std::endl;

            //std::string result_text = "解算结果: X=" + std::to_string(pt_3d.x) + ", Y=" + std::to_string(pt_3d.y);
            //cv::putText(frame, result_text, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

            //cv::Mat frame1;
            //cv::resize(frame, frame1, cv::Size(1500, 1000));
            //cv::imshow("Regions and Test Point", frame1);
            //cv::waitKey(0);
            
            
            //std::cout << "detection_info size: " << detection_info_.size() << std::endl;
            //std::cout << "real_points size: " << real_points_.size() << std::endl;
        


   




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<location_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



