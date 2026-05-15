#ifndef INFERENCE_H
#define INFERENCE_H

#include <fstream>
#include <vector>
#include <string>
#include <random>
#include <memory>

#include <chrono>
#include <iostream>
#include <iomanip>

// OpenCV / DNN / Inference
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
//#include <opencv2/dnn.hpp>

//TensorRT
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>




struct Detection
{
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    cv::Scalar color{};
    cv::Rect box{};
};

class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        // 忽略INFO级别的日志
        if (severity <= Severity::kWARNING) {
            std::cout << msg << std::endl;
        }
    }
};

class Inference_trt
{
public:
    Inference_trt(const std::string &enginePath, const cv::Size &modelInputShape = {640, 640},
                  const std::vector<std::string> &classes_ = {}, const bool &runWithCuda = true);
    void setclasses(std::vector<std::string> &newclasses)
    {
        this->classes = newclasses;
    }

    std::vector<Detection> runInference(const cv::Mat &input);
    std::vector<Detection> runInference_TensorRT(const cv::Mat &input);
    void loadTensorRTEngine(const std::string &enginePath);
    void setModelConfidenceThreshold(float val) { modelConfidenceThreshold = val; }
    void setLetterBoxForSquare(bool val) { letterBoxForSquare = val; }
    //void setTensoRTOptions(bool fp16 = true, bool useINT8 = false, size_t workspaceSize = 1 << 30);

private:
    std::shared_ptr<nvinfer1::IRuntime> runtime;
    std::shared_ptr<nvinfer1::ICudaEngine> trtEngine;
    std::shared_ptr<nvinfer1::IExecutionContext> trtContext;


    void loadClassesFromFile();
    size_t getSizeByDims(const nvinfer1::Dims& dims);
    size_t getElementSize(nvinfer1::DataType t);
    cv::Mat formatToSquare(const cv::Mat &source, int *pad_x, int *pad_y, float *scale);

    std::string modelPath{};
    std::string classesPath{};
    std::string enginePath{};
    bool cudaEnabled{};
    bool useTensorRT{};
    Logger gLogger;
    void** buffers;

    std::vector<std::string> classes{};
    cv::Size2f modelShape{};

    float modelConfidenceThreshold {0.25};
    float modelScoreThreshold      {0.51};
    float modelNMSThreshold        {0.50};

    bool letterBoxForSquare = true;

    cudaStream_t cudaStream{nullptr};

    std::vector<void*> deviceBuffers;

    int inputIndex{-1};
    int outputIndex{-1};
    size_t inputSize{0};
    size_t outputSize{0};
    int numClasses;
    std::string inputTensorName;
    std::string outputTensorName;

    bool useFP16{true};
    bool useINT8{false};
    size_t workspaceSize{1 << 30};
};

#endif // INFERENCE_H