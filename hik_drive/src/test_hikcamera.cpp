#include "HikCamera.h"
#include <iostream>
#include <thread>
#include "config.h"
#include <opencv2/opencv.hpp>
//#include "HikCamera.hpp"
// 假设有 config.h 文件定义如下内容
// namespace Config {
//     struct HikConfig {
//         std::string sn;
//         float exposure;
//         float gain;
//         float frame_rate;
//         bool rotate_180;
//         int log_level;
//     };
// }


int main() {
    // 构造测试用的HikConfig（需填写正确的相机SN）
    //Config::HikConfig config;
    //config.sn = "YOUR_CAMERA_SN";  // 替换为你的相机序列号
    //config.exposure = 2000.0f;
    //config.gain = 1.0f;
    //config.frame_rate = 15.0f;
    //config.rotate_180 = false;
    //config.log_level = 0;

    Config cfg;

    HikCamera camera(cfg.hik_cfg);

    // 等待相机初始化
    std::this_thread::sleep_for(std::chrono::seconds(2));

    //int ret = camera.initialize(cfg.hik_cfg);
    //if (ret != MV_OK)
    //{
      //  std::cerr << "初始化失败, 错误码: 0x" << std::hex << ret << std::endl;
        
    //    return -1;
   // }

  // 检查相机是否成功初始化
    if (!camera.isCameraConnected()) {
        std::cerr << "Camera connection failed!" << std::endl;
        return -1;
    }

    // 创建视频输出对象，用于保存视频文件
    int frame_width = 1920; // 替换为你的相机分辨率宽度
    int frame_height = 1080; // 替换为你的相机分辨率高度
    cv::VideoWriter video_writer("output_video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(frame_width, frame_height));
    
    if (!video_writer.isOpened()) {
        std::cerr << "Could not open the video writer!" << std::endl;
        return -1;
    }

    const int display_width = 640;
    const int display_height = 480;

    std::cout << "Starting video recording..." << std::endl;

    // 录像循环
    while (true) {
        cv::Mat frame = camera.getLatestFrame(); // 获取最新帧

        if (!frame.empty()) {
            // 设置窗口
            cv::Mat resized_frame;
            cv::resize(frame, resized_frame, cv::Size(display_width, display_height));
            //显示帧
            cv::imshow("Camera Feed", resized_frame);

            // 将帧写入视频文件
            video_writer.write(frame);

            // 按 'q' 键退出
            char key = cv::waitKey(1);
            if (key == 'q' || key == 27) {
                std::cout << "Recording stopped." << std::endl;
                break;
            }
        } else {
            std::cerr << "Failed to capture frame!" << std::endl;
        }
    }

    // 清理工作
    camera.stop();
    video_writer.release();
    cv::destroyAllWindows();

    return 0;
}