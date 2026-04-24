#ifndef DNN_DETECT_HPP
#define DNN_DETECT_HPP

#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// Reuse the Detection struct defined in laser_detect.hpp
#include "laser_detect.hpp"

class Inference_dnn
{
public:
    // onnxPath: YOLOv11 .onnx model path
    // modelInputShape: model input size (default 640x640)
    // classes_: class name list (e.g. {"light"})
    // runWithCuda: use CUDA backend for OpenCV DNN
    Inference_dnn(const std::string &onnxPath,
                  const cv::Size &modelInputShape = {640, 640},
                  const std::vector<std::string> &classes_ = {},
                  bool runWithCuda = false);

    std::vector<Detection> runInference(const cv::Mat &input);

    void setModelConfidenceThreshold(float val) { modelConfidenceThreshold = val; }
    void setModelNMSThreshold(float val)         { modelNMSThreshold = val; }
    void setLetterBoxForSquare(bool val)          { letterBoxForSquare = val; }

private:
    cv::dnn::Net net;
    cv::Size2f   modelShape;
    std::vector<std::string> classes;

    float modelConfidenceThreshold{0.25f};
    float modelNMSThreshold{0.45f};
    bool  letterBoxForSquare{true};

    // Letterbox: keep aspect ratio, pad to square
    cv::Mat formatToSquare(const cv::Mat &source, int &pad_x, int &pad_y, float &scale);
};

#endif // DNN_DETECT_HPP
