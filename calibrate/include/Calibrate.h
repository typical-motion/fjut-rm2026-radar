#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <functional>
#include <map>
#include <fstream>
//#include <rclcpp/rclcpp.hpp>
#include "config.h"
#include <Eigen/Dense>
#include <unordered_map>
#include "HikCamera.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class Calibrate
{
    public:
        struct calibrate_map_point
        {
            std::vector<double> left_buff = {(9.535-0.160), -(7.500-0.750), 0};
            std::vector<double> right_buff = {(9.535-0.160), -(7.500+0.750), 0};
            std::vector<double> self_tower = {10.9945, -11.517, 1.331+0.400};
            std::vector<double> enemy_base = {25.504, -7.500, (1.043+0.200)};
            std::vector<double> enemy_tower = {16.832, -3.6435, 1.331+0.400};
        }c_map_point;

        int current_point = 0;
        std::vector<cv::Point2f> real_map_point;
        std::vector<std::string> map_point_name = {"left_buff", "right_buff", "self_tower", "enemy_base", "enemy_tower"};
        cv::Mat display_frame;
        std::vector<double> rvec_msg;
        std::vector<double> tvec_msg;

        static void click_callback(int event, int x, int y, int flags, void* param);

        void draw_existing_points(const cv::Mat& display_frame)
        {
            if(!display_frame.empty())
            {
                int index = 0;
                for(int i = 0; i < real_map_point.size(); i++)
                {
                    index++;
                    cv::circle(display_frame, cv::Point(real_map_point[i].x, real_map_point[i].y), 5, cv::Scalar(0, 0, 255), -1);
                    cv::putText(display_frame, map_point_name[index], cv::Point(real_map_point[i].x+10, real_map_point[i].y-10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                }
            }
        }

        cv::Mat get_object_points(const calibrate_map_point& c_map_point, const std::vector<std::string>& point_name)
            {
                std::vector<cv::Point3f> points;
                points.emplace_back(c_map_point.left_buff[0], c_map_point.left_buff[1], c_map_point.left_buff[2]);
                points.emplace_back(c_map_point.right_buff[0], c_map_point.right_buff[1], c_map_point.right_buff[2]);
                points.emplace_back(c_map_point.self_tower[0], c_map_point.self_tower[1], c_map_point.self_tower[2]);
                points.emplace_back(c_map_point.enemy_base[0], c_map_point.enemy_base[1], c_map_point.enemy_base[2]);
                points.emplace_back(c_map_point.enemy_tower[0], c_map_point.enemy_tower[1], c_map_point.enemy_tower[2]);

                cv::Mat object_points(points.size(), 3, CV_64FC1);
                for (int i = 0; i < points.size(); i++)
                {
                    object_points.at<double>(i, 0) = points[i].x;
                    object_points.at<double>(i, 1) = points[i].y;
                    object_points.at<double>(i, 2) = points[i].z;
                }
                return object_points;
            }

        cv::Mat get_image_points(const std::vector<cv::Point2f>& real_map_point)
            {
                cv::Mat image_points(real_map_point.size(), 2, CV_64FC1);
                for (int i = 0; i < real_map_point.size(); i++)
                {
                    image_points.at<double>(i, 0) = real_map_point[i].x;
                    image_points.at<double>(i, 1) = real_map_point[i].y;
                    
                }
                return image_points;
            }

        bool finalize_calibrate()
        {
            Config cfg;
            cv::destroyAllWindows();
            auto len_map_point = real_map_point.size();
            if(len_map_point != 5)
            {
                std::cout << "需要五个标定点,当前只有:" << len_map_point << std::endl;
                return false;
            }

            calibrate_map_point c_map_point;
            cv::Mat object_points = get_object_points(c_map_point, map_point_name);
            //cv::Mat object_points = get_object_points():
            cv::Mat image_points = get_image_points(real_map_point);

            auto camera_matrix = cfg.camera_matrix;
            auto dist_coeffs = cfg.dist_coeffs;

            cv::Mat cv_camera_matrix;
            cv::eigen2cv(camera_matrix, cv_camera_matrix);

            cv::Mat cv_dist_coeffs;
            cv::eigen2cv(dist_coeffs, cv_dist_coeffs);
            
            cv::Mat rvec, tvec;
            bool sucess = cv::solvePnP(object_points, image_points, cv_camera_matrix, cv_dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
            if (!sucess)
            {
                std::cout << "求解pnp失败" << std::endl;
                return false;
            }

            
            rvec_msg = {rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)};
            tvec_msg = {tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0)};
            std::map<std::string, std::vector<double>> real_point;
            for (int i = 0; i < map_point_name.size(); i++)
            {
                real_point.insert({map_point_name[i], {real_map_point[i].x, real_map_point[i].y}});
            }
            json calibrate_date;
            calibrate_date["rvec"] = rvec_msg;
            calibrate_date["tvec"] = tvec_msg;
            calibrate_date["real_point"] = real_point;
            //for (auto& pair : real_point)
            //{
              //  std::cout << pair.first << " : " << pair.second[0] << " " << pair.second[1] << " " << pair.second[2] << std::endl;
            //}
            std::ofstream out("/home/zqz/ros2_ws/json/calibrate.json");
            out << std::setw(4) << calibrate_date << std::endl;
            out.close();

            std::cout << "标定结果保存到calibrate.json" << std::endl;


            //std::cout << "求解pnp成功" << std::endl;
            //std::cout << "rvec:" << rvec << std::endl;
            //std::cout << "tvec:" << tvec << std::endl;
            //std::cout << result_map << std::endl;

            
           
            
            //发送旋转，平移矩阵
            //
            //
            //
            //

            return true;

        }

        void calibrate_with_video(const std::string& video_path)
        {
            cv::Mat frame;
            cv::VideoCapture cap(video_path);
            if (!cap.isOpened())
            {
                std::cout << "无法打开视频文件" << std::endl;
                return;
            }

            cv::namedWindow("calibrate", cv::WINDOW_NORMAL);
            cv::setMouseCallback("calibrate", &Calibrate::click_callback, this);

            try{
                while (cap.read(frame) && current_point < 5)
                {
                    if (!cap.read(frame))
                    {
                        break;
                    }

                    frame.copyTo(display_frame);
                    draw_existing_points(display_frame);
                    cv::imshow("calibrate", display_frame);

                    int key = cv::waitKey(30);
                    if (key == 27 || current_point >= 5)
                    {
                        break;
                    }
                }
            } catch (const std::exception& e)
            {
                std::cerr << e.what() << std::endl;
                std::cerr << "视频标定出错" << std::endl;
            }

            cap.release();
            cv::destroyWindow("calibrate");
            finalize_calibrate();
        }
        
        void calibrate_with_camera(const Config::HikConfig& config)

        {
            HikCamera camera(config);
            
            cv::namedWindow("calibrate", cv::WINDOW_NORMAL);
            cv::setMouseCallback("calibrate", &Calibrate::click_callback, this);
            try
            {
                while (current_point < 5)
                {
                    
                    cv::Mat frame = camera.getLatestFrame().clone();
                    if(!frame.empty())
                    {
                        frame.copyTo(display_frame);
                        draw_existing_points(display_frame);
                        cv::imshow("calibrate", display_frame);
                    }

                    if(current_point >= 5)
                    {
                        break;
                    }

                    auto key = cv::waitKey(1);

                    if (key == 27)
                    {
                        break;
                    }
                }
            } catch (const std::exception& e)
            {
                std::cerr << e.what() << std::endl;
                std::cerr << "相机标定出错" << std::endl;
            }

            camera.stop();
            cv::destroyWindow("calibrate");
            finalize_calibrate();
        }

        void calibrate_with_image(const std::string& image_path)
        {
            cv::Mat frame = cv::imread(image_path);
            if(frame.empty())
            {
                std::cout << "无法打开图片文件" << std::endl;
                return;
            }
            cv::namedWindow("calibrate", cv::WINDOW_NORMAL);
            cv::resizeWindow("calibrate", 1080, 960);
            cv::setMouseCallback("calibrate", &Calibrate::click_callback, this);
            try
            {
                display_frame = frame.clone();
                draw_existing_points(display_frame);
                cv::imshow("calibrate", display_frame);
                while(current_point < 5)
                {
                    int key = cv::waitKey(30);
                    if(key == 27)
                    {
                        break;
                    }
                }

                 finalize_calibrate();
            }
            catch (const std::exception& e)
            {
                std::cerr << "图像标定异常" << e.what() << std::endl;
            }
        }
};

void click_callback(int event, int x, int y, int flags, void* param)
        {
            Calibrate* calibrate = static_cast<Calibrate*>(param);
            if(event == cv::EVENT_LBUTTONDOWN && calibrate->current_point < 5)
            {
                cv::Point2f real_point(x,y);
                cv::circle(calibrate->display_frame, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
                cv::putText(calibrate->display_frame, calibrate->map_point_name[calibrate->current_point], 
                    cv::Point(x+10, y-10), cv::FONT_HERSHEY_SIMPLEX, 
                    0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                 
                calibrate->real_map_point.push_back(real_point);
                calibrate->current_point++;
                cv::imshow("calibrate", calibrate->display_frame);
            }
        }
#endif // CONFIG_H

        