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

// 构造函数
HikCamera::HikCamera(const Config::HikConfig& config)
    : config_(config), camera_handle_(nullptr) {
    initLogger();
    if (!initCamera()) {
        std::cerr << "Failed to initialize Hikvision camera!" << std::endl;
    }
    running_ = true;
    capture_thread_ = std::thread(&HikCamera::captureLoop, this);
    monitor_thread_ = std::thread(&HikCamera::monitorLoop, this);
}

// 析构函数
HikCamera::~HikCamera() {
    stop();
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
        MV_CC_Finalize();
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
            MV_CC_FreeImageBuffer(camera_handle_, &stFrame);
        } else {
            // 采集失败可重试
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
        cv::Mat img(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth, 
                   CV_8UC1, stFrame.pBufAddr);
        result = need_rotate ? img.clone() : img;
        if (need_rotate) cv::rotate(result, result, cv::ROTATE_180);
    } 
    else if (stFrame.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
        cv::Mat img(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth,
                   CV_8UC3, stFrame.pBufAddr);
        result = need_rotate ? img.clone() : img;
        if (need_rotate) cv::rotate(result, result, cv::ROTATE_180);
    }
    else if (stFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8) {
        cv::Mat img(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth,
                   CV_8UC1, stFrame.pBufAddr);
        result = need_rotate ? img.clone() : img;
        if (need_rotate) cv::rotate(result, result, cv::ROTATE_180);
    }
    else if (stFrame.stFrameInfo.enPixelType == PixelType_Gvsp_YUV422_Packed) {
        cv::Mat yuv(stFrame.stFrameInfo.nHeight, stFrame.stFrameInfo.nWidth,
                   CV_8UC2, stFrame.pBufAddr);
        cv::cvtColor(yuv, result, cv::COLOR_YUV2BGR_YUYV);
        if (need_rotate) cv::rotate(result, result, cv::ROTATE_180);
    }
    else if (stFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG12) 
    {
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
    closeDevice();
}

// 复位相机
void HikCamera::reset() {
    closeDevice();
    initCamera();
}