#ifndef GALAXY_CAMERA_H
#define GALAXY_CAMERA_H

#include <opencv2/opencv.hpp>
#include "GxIAPI.h"

class GalaxyCamera
{
public:
    GalaxyCamera();
    ~GalaxyCamera();

    bool init();
    bool getFrame(cv::Mat &img);
    void release();

    // ⭐ 新增：曝光/增益控制（来自3.cpp）
    void setExposure(double exp);
    void setGain(double g);

private:
    GX_DEV_HANDLE hDevice;

    // ⭐ 新增：对应3.cpp变量
    double exposure = 30000;
    double gain = 10;
};

#endif