// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

#ifndef INFERENCE_H
#define INFERENCE_H

// Cpp native
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
#include <opencv2/dnn.hpp>

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


class Inference
{
public:
    Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape = {640, 640}, 
              const std::vector<std::string> &classes_ = {}, const bool &runWithCuda = true); 
              //~Inference();
    //std::vector<Detection> runInference(const cv::Mat &input);
    //Inference_trt(const std::string &enginePath, const cv::Size &modelInputShape = {640, 640},
    //             const std::vector<std::string> &classes_ = {}, const bool &runWithCuda = true);
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
    void loadOnnxNetwork();
    size_t getSizeByDims(const nvinfer1::Dims& dims);
    size_t getElementSize(nvinfer1::DataType t);
    //void preprocessresults(float *data, nvinfer1::Dims outputDims);
    //void loadTensorRTEngine();
    cv::Mat formatToSquare(const cv::Mat &source, int *pad_x, int *pad_y, float *scale);

    //bool converonnxToeigen(const std::string& onnxModelPath, const std::string& eigenModelPath);
    //std::vector<float> pre_process_image_TRT(const cv::Mat& img, const int* pad_x, const int* pad_y, const float* scale);
    //std::vector<Detection> post_process_output_TRT(const float* output, int rows, 
    //                                               int dimensions, const int* pad_x, const int* pad_y, 
    //                                              const float* scale, bool yolov8);
    //std::vector<Detection> run_Inference_TRT(const cv::Mat& input);

    std::string modelPath{};
    std::string classesPath{};
    std::string enginePath{};
    bool cudaEnabled{};
    //bool TRTEnabled{false};
    bool useTensorRT{};
    //Logger gLogger;
    void** buffers;
    //std::shared_ptr<nvinfer1::IRuntime> runtimeTRT;
    //std::shared_ptr<nvinfer1::ICudaEngine> trtEngine;
    //std::shared_ptr<nvinfer1::IExecutionContext> trtContext;

    //std::vector<std::string> classes{"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
    //std::vector<std::string> classes{"car","armor","ignore","watcher","base"};
    //std::vector<std::string> classes{"B1", "B2", "B3", "B4", "B5", "B7","R1", "R2", "R3", "R4", "R5", "R7"};
    std::vector<std::string> classes{};
    cv::Size2f modelShape{};

    float modelConfidenceThreshold {0.25};
    float modelScoreThreshold      {0.60};
    float modelNMSThreshold        {0.50};

    bool letterBoxForSquare = true;

    //nvinfer1::ICudaEngine* trtEngine{nullptr};
    //nvinfer1::IExecutionContext* trtContext{nullptr};
    cudaStream_t cudaStream{nullptr};
    //std::vector<void*> deviceBuffers[2]{nullptr, nullptr};
    std::vector<void*> deviceBuffers;
    int inputIndex{-1};
    int outputIndex{-1};
    size_t inputSize{0};
    size_t outputSize{0};
    int numClasses;
    std::string inputTensorName;
    std::string outputTensorName;
    //TensorRTLogger trtLogger;

    bool useFP16{true};
    bool useINT8{false};
    size_t workspaceSize{1 << 30};

    cv::dnn::Net net;
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
    void loadOnnxNetwork();
    size_t getSizeByDims(const nvinfer1::Dims& dims);
    size_t getElementSize(nvinfer1::DataType t);
    //void preprocessresults(float *data, nvinfer1::Dims outputDims);
    //void loadTensorRTEngine();
    cv::Mat formatToSquare(const cv::Mat &source, int *pad_x, int *pad_y, float *scale);

    //bool converonnxToeigen(const std::string& onnxModelPath, const std::string& eigenModelPath);
    //std::vector<float> pre_process_image_TRT(const cv::Mat& img, const int* pad_x, const int* pad_y, const float* scale);
    //std::vector<Detection> post_process_output_TRT(const float* output, int rows, 
    //                                               int dimensions, const int* pad_x, const int* pad_y, 
    //                                              const float* scale, bool yolov8);
    //std::vector<Detection> run_Inference_TRT(const cv::Mat& input);

    std::string modelPath{};
    std::string classesPath{};
    std::string enginePath{};
    bool cudaEnabled{};
    //bool TRTEnabled{false};
    bool useTensorRT{};
    Logger gLogger;
    void** buffers;
    //std::shared_ptr<nvinfer1::IRuntime> runtimeTRT;
    //std::shared_ptr<nvinfer1::ICudaEngine> trtEngine;
    //std::shared_ptr<nvinfer1::IExecutionContext> trtContext;

    //std::vector<std::string> classes{"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
    //std::vector<std::string> classes{"car","armor","ignore","watcher","base"};
    //std::vector<std::string> classes{"B1", "B2", "B3", "B4", "B5", "B7","R1", "R2", "R3", "R4", "R5", "R7"};
    std::vector<std::string> classes{};
    cv::Size2f modelShape{};

    float modelConfidenceThreshold {0.25};
    float modelScoreThreshold      {0.51};
    float modelNMSThreshold        {0.50};

    bool letterBoxForSquare = true;

    //nvinfer1::ICudaEngine* trtEngine{nullptr};
    //nvinfer1::IExecutionContext* trtContext{nullptr};
    cudaStream_t cudaStream{nullptr};
    //std::vector<void*> deviceBuffers[2]{nullptr, nullptr};
    std::vector<void*> deviceBuffers;
    //void* deviceBuffers[100];
    int inputIndex{-1};
    int outputIndex{-1};
    size_t inputSize{0};
    size_t outputSize{0};
    int numClasses;
    std::string inputTensorName;
    std::string outputTensorName;
    //TensorRTLogger trtLogger;

    bool useFP16{true};
    bool useINT8{false};
    size_t workspaceSize{1 << 30};

    cv::dnn::Net net;
};

#endif // INFERENCE_H
