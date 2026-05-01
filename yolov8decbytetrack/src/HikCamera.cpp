#include "HikCamera.h"
#include <iostream>
#include <chrono>
#include <thread>

// 构造CameraConfig
HikCamera::CameraConfig::CameraConfig(const Config::HikConfig& config)
    : sn(config.sn),
      exposure(config.exposure),
      gain(config.gain),
      frame_rate(config.frame_rate),
      rotate_180(config.rotate_180),
      log_level(config.log_level) {}


//构造函数
HikCamera::HikCamera(const Config::HikConfig& config)
    : config_(config), camera_handle_(nullptr) {
    initLogger();
    if (!initCamera()) {
        std::cerr << "Failed to initialize Hikvision camera!" << std::endl;
    }
    running_ = true;
    video_thread_running_ = false;  // 初始化
    capture_thread_ = std::thread(&HikCamera::captureLoop, this);
    monitor_thread_ = std::thread(&HikCamera::monitorLoop, this);
}

// 析构函数
HikCamera::~HikCamera() {
    stop();
    
    // 确保视频写入线程已停止
    if (recording_) {
        stopRecording();
    }
    
    if (capture_thread_.joinable())
        capture_thread_.join();
    if (monitor_thread_.joinable())
        monitor_thread_.join();
    
    closeDevice();
}

void HikCamera::initLogger() {
    // 可根据config_.log_level设置日志等级
    // 此处简单处理
}

// 初始化相机
bool HikCamera::initCamera() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list_);
    if (nRet != MV_OK || device_list_.nDeviceNum == 0) {
        std::cerr << "MV_CC_EnumDevices failed! No devices found or error:" << nRet << std::endl;
        return false;
    }


    nRet = MV_CC_CreateHandle(&camera_handle_, device_list_.pDeviceInfo[0]);
    if (nRet != MV_OK) 
    {
        std::cerr << "Open camera device failed! Error:"<< nRet << std::endl;
        return false;
    }




 
    // 查找指定SN
   // for (unsigned int i = 0; i < device_list_.nDeviceNum; ++i) {
   //     auto* device_info = device_list_.pDeviceInfo[i];
   //     if (device_info->nTLayerType == MV_GIGE_DEVICE) {
    //        MV_GIGE_DEVICE_INFO* pInfo = &device_info->SpecialInfo.stGigEInfo;
      //      std::string serial_number(reinterpret_cast<char*>(pInfo->chSerialNumber), sizeof(pInfo->chSerialNumber));
        //    if (config_.sn == serial_number) {
          //      nRet = MV_CC_CreateHandle(&camera_handle_, device_info);
            //    if (nRet != MV_OK) {
              //      std::cerr << "Create camera handle failed!" << std::endl;
                //    return false;
                //}
      //          break;
            //}
        //}
        // TODO: 可添加USB相机支持
    //}

   // if (!camera_handle_) {
     //   std::cerr << "Camera with SN " << config_.sn << " not found." << std::endl;
       // return false;
    //}

    nRet = MV_CC_OpenDevice(camera_handle_);
    if (nRet != MV_OK) {
        std::cerr << "Open camera device failed!" << std::endl;
        return false;
    }

    if (!setCameraParameters()) {
        std::cerr << "Set camera parameters failed!" << std::endl;
        return false;
    }

    nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed);
    if (nRet != MV_OK) 
    {
        std::cerr << "设置像素格式失败,错误码:0x" << std::hex << nRet << std::dec << std::endl;
    }


    MV_CC_SetImageNodeNum(camera_handle_, 3);
    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
    //MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);

    // 启动采集
    nRet = MV_CC_StartGrabbing(camera_handle_);
    if (nRet != MV_OK) {
        std::cerr << "Start grabbing failed!" << std::endl;
        return false;
    }

    camera_active_ = true;
    return true;
}

// 设置相机参数
bool HikCamera::setCameraParameters() {
    if (!camera_handle_) return false;

    int ret = MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0);
    if (ret != MV_OK) {
        std::cerr << "设置曝光模式失败，错误码: " << ret << std::endl;
        return false;
    }
    ret = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", config_.exposure);
    if (ret != MV_OK) {
        std::cerr << "设置曝光时间失败，错误码: " << ret << std::endl;
        return false;
    }
    ret = MV_CC_SetFloatValue(camera_handle_, "Gain", config_.gain);
    if (ret != MV_OK) {
        std::cerr << "增益设置失败，错误码: " << ret << std::endl;
        return false;
    }
    ret = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", config_.frame_rate);
    if (ret != MV_OK) {
        std::cerr << "设置帧率失败，错误码: " << ret << std::endl;
        return false;
    }
    ret = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    if (ret != MV_OK) {
        std::cerr << "设置像素格式失败，错误码: " << ret << std::endl;
        return false;
    }
    return true;
}
// 关闭设备
void HikCamera::closeDevice() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    camera_active_ = false;
}

// 判断相机是否连接
bool HikCamera::isCameraConnected(int retries) {
    for (int i = 0; i < retries; ++i) {
        if (camera_active_) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

// 采集线程
void HikCamera::captureLoop() {
    while (running_) {
        if (!camera_active_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        MV_FRAME_OUT stFrame = { 0 };
        int nRet = MV_CC_GetImageBuffer(camera_handle_, &stFrame, 1000);
        if (nRet == MV_OK) {
            cv::Mat frame = processFrame(stFrame);
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = frame.clone();
            }
            
            // 将帧添加到录制缓冲（不是直接写入）
            if (recording_) {
                writeFrameToVideo(frame);
            }
            
            MV_CC_FreeImageBuffer(camera_handle_, &stFrame);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

// 监控线程
void HikCamera::monitorLoop() {
    while (running_) {
        if (!isCameraConnected()) {
            std::cerr << "Camera disconnected! Try to reinitialize..." << std::endl;
            closeDevice();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            initCamera();
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// 获取最新帧
cv::Mat HikCamera::getLatestFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_frame_.clone();
}

// 处理帧
cv::Mat HikCamera::processFrame(MV_FRAME_OUT& stFrame) {
    cv::Mat result;
    bool need_rotate = config_.rotate_180;

    if (stFrame.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8) {
        //std::cout << "PixelType_Gvsp_Mono8" << std::endl;
        cv::Mat img(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth, 
                   CV_8UC1, stFrame.pBufAddr);
        result = need_rotate ? img.clone() : img;
        if (need_rotate) cv::rotate(result, result, cv::ROTATE_180);
    } 
    else if (stFrame.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
        //std::cout << "PixelType_Gvsp_RGB8_Packed" << std::endl;
        cv::Mat img(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth,
                   CV_8UC3, stFrame.pBufAddr);
        result = need_rotate ? img.clone() : img;
        if (need_rotate) cv::rotate(result, result, cv::ROTATE_180);
    }
    else if (stFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8) {
        //std::cout << "PixelType_Gvsp_BayerGR8" << std::endl;
        cv::Mat img(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth,
                   CV_8UC1, stFrame.pBufAddr);
        result = need_rotate ? img.clone() : img;
        if (need_rotate) cv::rotate(result, result, cv::ROTATE_180);
    }
    else if (stFrame.stFrameInfo.enPixelType == PixelType_Gvsp_YUV422_Packed) {
        //std::cout << "PixelType_Gvsp_YUV422_Packed" << std::endl;
        cv::Mat yuv(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth,
                   CV_8UC2, stFrame.pBufAddr);
        cv::cvtColor(yuv, result, cv::COLOR_YUV2BGR_YUYV);
        if (need_rotate) cv::rotate(result, result, cv::ROTATE_180);
    }
    else if (stFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG12) 
    {
        //std::cout << "PixelType_Gvsp_BayerRG12" << std::endl;
        // 创建16UC1矩阵接收12bit数据
        cv::Mat raw(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth,
                CV_16UC1, stFrame.pBufAddr);
        
        // 12bit转8bit（右移4位）
        cv::Mat img8bit;
        raw.convertTo(img8bit, CV_8UC1, 1.0/16.0);
        
        // Bayer RGGB转RGB
        cv::Mat rgb;
        cv::cvtColor(img8bit, rgb, cv::COLOR_BayerRG2RGB);
        
        if (config_.rotate_180) {
            cv::rotate(rgb, rgb, cv::ROTATE_180);
        }
        return rgb;
    }
    else {
        std::cerr << "Unsupported pixel type: 0x" 
                 << std::hex << stFrame.stFrameInfo.enPixelType 
                 << std::dec << std::endl;
        return cv::Mat();
    }

    return result;
}

// 停止相机
void HikCamera::stop() {
    running_ = false;
    stop_threads_ = true;
    stop_capture_ = true;
}

// 复位相机
void HikCamera::reset() {
    closeDevice();
    initCamera();
}

void HikCamera::info()
{
    std::cout << "Camera Info:" << std::endl;
    std::cout << "sn:" << config_.sn << std::endl;
    //std::cout << "  Model Name: " << config_.model_name_ << std::endl;
    //std::cout << "  Serial Number: " << config_.serial_number_ << std::endl;
    //std::cout << "  IP Address: " << config_.ip_address_ << std::endl;
    //std::cout << "  Port: " << config_.port_ << std::endl;
    std::cout << "曝光时间：" << config_.exposure << std::endl;
    std::cout << "增益：" << config_.gain << std::endl;
    
}

void HikCamera::videoWriteLoop() {
    while (video_thread_running_) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
            
            // 等待帧到达或超时
            frame_buffer_cv_.wait_for(lock, std::chrono::milliseconds(100), 
                                      [this] { return !frame_buffer_.empty() || !video_thread_running_; });
            
            if (frame_buffer_.empty()) {
                continue;
            }
            
            frame = frame_buffer_.front();
            frame_buffer_.pop();
        }
        
        if (!frame.empty() && video_writer_.isOpened()) {
            try {
                video_writer_.write(frame);
                frame_count_video_++;
                
                // 每100帧输出一次日志
                if (frame_count_video_ % 100 == 0) {
                    std::cout << "[VIDEO] Frames written: " << frame_count_video_ 
                              << " | Buffer size: " << frame_buffer_.size() << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "[VIDEO ERROR] Failed to write frame: " << e.what() << std::endl;
            }
        }
    }
    
    // 线程退出前，刷新剩余的帧
    std::cout << "[VIDEO] Flushing remaining frames..." << std::endl;
    while (!frame_buffer_.empty()) {
        cv::Mat remaining_frame = frame_buffer_.front();
        frame_buffer_.pop();
        
        if (!remaining_frame.empty() && video_writer_.isOpened()) {
            video_writer_.write(remaining_frame);
            frame_count_video_++;
        }
    }
    
    std::cout << "[VIDEO] Video write thread finished. Total frames: " << frame_count_video_ << std::endl;
}


bool HikCamera::startRecording(const std::string& output_path, int fourcc_code) {
    std::lock_guard<std::mutex> lock(video_mutex_);
    
    if (recording_) {
        std::cerr << "[VIDEO] Already recording!" << std::endl;
        return false;
    }
    
    if (!camera_active_) {
        std::cerr << "[VIDEO] Camera is not active!" << std::endl;
        return false;
    }
    
    // 获取当前帧以确定分辨率
    cv::Mat frame = getLatestFrame();
    if (frame.empty()) {
        std::cerr << "[VIDEO] Cannot get frame for video initialization!" << std::endl;
        return false;
    }
    
    int width = frame.cols;
    int height = frame.rows;
    int fps = static_cast<int>(config_.frame_rate);
    
    if (fps <= 0) fps = 30;
    
    // 默认使用 MJPEG 编码（兼容性好、写入可靠）
    if (fourcc_code == 0) {
        fourcc_code = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    }
    
    std::cout << "[VIDEO] Opening video writer with:" << std::endl;
    std::cout << "  Path: " << output_path << std::endl;
    std::cout << "  FourCC: 0x" << std::hex << fourcc_code << std::dec << std::endl;
    std::cout << "  Resolution: " << width << "x" << height << std::endl;
    std::cout << "  FPS: " << fps << std::endl;
    
    // 打开 VideoWriter
    video_writer_.open(output_path, fourcc_code, fps, cv::Size(width, height), true);
    
    if (!video_writer_.isOpened()) {
        std::cerr << "[VIDEO ERROR] Failed to open video writer!" << std::endl;
        return false;
    }
    
    // 验证 VideoWriter 是否真的打开了
    if (!video_writer_.isOpened()) {
        std::cerr << "[VIDEO ERROR] VideoWriter opened but isOpened() returns false!" << std::endl;
        return false;
    }
    
    video_output_path_ = output_path;
    recording_ = true;
    frame_count_video_ = 0;
    start_time_video_ = std::chrono::high_resolution_clock::now();
    
    // 清空帧缓冲
    while (!frame_buffer_.empty()) {
        frame_buffer_.pop();
    }
    
    // 启动视频写入线程
    video_thread_running_ = true;
    video_write_thread_ = std::thread(&HikCamera::videoWriteLoop, this);
    
    std::cout << "[VIDEO] Recording started successfully!" << std::endl;
    
    return true;
}

bool HikCamera::stopRecording() {
    {
        std::lock_guard<std::mutex> lock(video_mutex_);
        
        if (!recording_) {
            std::cerr << "[VIDEO] Not recording!" << std::endl;
            return false;
        }
        
        recording_ = false;
    }
    
    // 等待视频写入线程完成
    std::cout << "[VIDEO] Waiting for video write thread to finish..." << std::endl;
    video_thread_running_ = false;
    
    if (video_write_thread_.joinable()) {
        video_write_thread_.join();
    }
    
    // 关闭 VideoWriter
    if (video_writer_.isOpened()) {
        video_writer_.release();
        std::cout << "[VIDEO] VideoWriter released" << std::endl;
    }
    
    // 计算统计信息
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time_video_;
    double actual_duration = elapsed.count();
    
    std::cout << std::endl;
    std::cout << "========== Video Recording Summary ==========" << std::endl;
    std::cout << "File: " << video_output_path_ << std::endl;
    std::cout << "Total frames written: " << frame_count_video_ << std::endl;
    std::cout << "Actual duration: " << actual_duration << " seconds" << std::endl;
    if (actual_duration > 0) {
        std::cout << "Average FPS: " << (frame_count_video_ / actual_duration) << std::endl;
    }
    std::cout << "===========================================" << std::endl;
    
    return true;
}

void HikCamera::writeFrameToVideo(const cv::Mat& frame) {
    if (!recording_ || frame.empty()) {
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
        
        // 限制缓冲区大小，避免内存溢出
        if (frame_buffer_.size() > 300) {
            std::cerr << "[VIDEO WARNING] Frame buffer full! Dropping frame." << std::endl;
            return;
        }
        
        // 克隆帧（防止原数据被修改）
        frame_buffer_.push(frame.clone());
    }
    
    // 通知写入线程有新帧到达
    frame_buffer_cv_.notify_one();
}
// 判断是否正在录制
bool HikCamera::isRecording() const {
    return recording_;
}

// 获取录制文件路径
std::string HikCamera::getRecordingPath() const {
    std::lock_guard<std::mutex> lock(video_mutex_);
    return video_output_path_;
}