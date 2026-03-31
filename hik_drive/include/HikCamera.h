#pragma once

#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <optional>
#include <string>
#include <vector>
#include <memory>
#include "MvCameraControl.h"  
#include "config.h" 

class HikCamera {
public:
    struct CameraConfig {
        std::string sn;
        float exposure;
        float gain;
        float frame_rate;
        bool rotate_180;
        std::string log_level;

        explicit CameraConfig(const Config::HikConfig& config);
    };

    explicit HikCamera(const Config::HikConfig& config);
    ~HikCamera();

    bool isCameraConnected(int retries = 3);
    cv::Mat getLatestFrame();
    void stop();
    void reset();
    bool initialize(const Config::HikConfig& config);

private:
    void initLogger();
    bool setCameraParameters();
    bool initCamera();
    void closeDevice();
    void captureLoop();
    void monitorLoop();
    cv::Mat processFrame(MV_FRAME_OUT& stFrame);

    CameraConfig config_;
    //void* camera_handle_;
    void* camera_handle_=nullptr;
    MV_CC_DEVICE_INFO_LIST device_list_;
    std::mutex camera_mutex_;
    std::atomic<bool> camera_active_{false};
    std::atomic<bool> stop_threads_{false};
    std::atomic<bool> stop_capture_{false};
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    std::atomic<bool> running_{true};
    std::thread capture_thread_;
    std::thread monitor_thread_;
    MV_CC_DEVICE_INFO* target_device_info_ = nullptr;
    
};