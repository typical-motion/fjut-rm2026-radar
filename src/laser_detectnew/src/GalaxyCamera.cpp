#include "GalaxyCamera.h"
#include <iostream>

using namespace std;

GalaxyCamera::GalaxyCamera()
{
    hDevice = nullptr;
}

GalaxyCamera::~GalaxyCamera()
{
    release();
}

// =========================
// 初始化（等价3.cpp前半段）
// =========================
bool GalaxyCamera::init()
{
    GX_STATUS status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        cout << "GXInitLib failed!" << endl;
        return false;
    }

    uint32_t device_num = 0;
    status = GXUpdateDeviceList(&device_num, 1000);

    cout << "设备数量: " << device_num << endl;

    if (device_num == 0)
    {
        cout << "没有找到相机" << endl;
        GXCloseLib();
        return false;
    }

    status = GXOpenDeviceByIndex(1, &hDevice);

    if (status != GX_STATUS_SUCCESS)
    {
        cout << "打开设备失败" << endl;
        GXCloseLib();
        return false;
    }

    GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);

    GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO,
              GX_BALANCE_WHITE_AUTO_CONTINUOUS);

    GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO,
              GX_EXPOSURE_AUTO_CONTINUOUS);

    GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO,
              GX_GAIN_AUTO_CONTINUOUS);

    GXStreamOn(hDevice);

    cout << "开始取流..." << endl;
    return true;
}

// =========================
// 取帧（等价3.cpp while内部）
// =========================
bool GalaxyCamera::getFrame(cv::Mat &img)
{
    GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);

    GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure);
    GXSetFloat(hDevice, GX_FLOAT_GAIN, gain);

    GX_FRAME_BUFFER *frame = nullptr;

    GX_STATUS status = GXDQBuf(hDevice, &frame, 1000);

    if (status != GX_STATUS_SUCCESS || frame == nullptr)
        return false;

    if (frame->pImgBuf == nullptr)
    {
        GXQBuf(hDevice, frame);
        return false;
    }

    // ⚡ Bayer -> RGB（解决红蓝反）
    cv::Mat raw(frame->nHeight, frame->nWidth, CV_8UC1, frame->pImgBuf);
    cv::cvtColor(raw, img, cv::COLOR_BayerRG2RGB);

    // 显示参数（可选）
    string text = "Exp:" + to_string((int)exposure) +
                  " Gain:" + to_string((int)gain);

    cv::putText(img, text,
                cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                cv::Scalar(0, 255, 0),
                2);

    GXQBuf(hDevice, frame);

    return true;
}

// =========================
// 控制曝光
// =========================
void GalaxyCamera::setExposure(double exp)
{
    exposure = exp;
}

// =========================
// 控制增益
// =========================
void GalaxyCamera::setGain(double g)
{
    gain = g;
}

// =========================
// 释放
// =========================
void GalaxyCamera::release()
{
    if (hDevice)
    {
        GXStreamOff(hDevice);
        GXCloseDevice(hDevice);
        hDevice = nullptr;
    }

    GXCloseLib();
}