// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

#include "inference.h"
#include <cuda_fp16.h>
#include <opencv4/opencv2/core/types.hpp>

Inference::Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape, const std::vector<std::string> &classes_, const bool &runWithCuda)
{
    modelPath = onnxModelPath;
    modelShape = modelInputShape;
    classes = classes_;
    cudaEnabled = runWithCuda;

    loadOnnxNetwork();
    // loadClassesFromFile(); The classes are hard-coded for this example
}

Inference_trt::Inference_trt(const std::string &enginePath,const cv::Size &modelInputShape, const std::vector<std::string> &classes_, const bool &runWithCuda)
{
    modelPath = enginePath;
    modelShape = modelInputShape;
    classes = classes_;
    cudaEnabled = runWithCuda;

    loadTensorRTEngine(enginePath);
    //cudaStreamCreate(&stream);
}


// Inference_trt::Inference_trt(const std::string &onnxModelPath, const std::string &engineSavePath,
//                              const cv::Size &modelInputShape, const std::vector<std::string> &classes_,
//                              const bool &runWithCuda, bool fp16, size_t workspace)
// {
//     modelShape = modelInputShape;
//     classes = classes_;
//     cudaEnabled = runWithCuda;
//     useFP16 = fp16;
//     workspaceSize = workspace;
//     onnxPath_ = onnxModelPath;

//     if (!buildEngineFromONNX(onnxModelPath, engineSavePath))
//     {
//         throw std::runtime_error("Failed to build TensorRT engine from ONNX model");
//     }
//     loadTensorRTEngine(engineSavePath);
// }

// void Inference_trt::setTensoRTOptions(bool fp16, size_t workspaceSize)
// {
//     useFP16 = fp16;
//     this->workspaceSize = workspaceSize;
// }

// bool Inference_trt::buildEngineFromONNX(const std::string &onnxPath, const std::string &engineSavePath)
// {
//     std::cout << "Building TensorRT engine from ONNX: " << onnxPath << std::endl;

//     auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(gLogger));
//     if (!builder)
//     {
//         std::cerr << "Failed to create TensorRT builder" << std::endl;
//         return false;
//     }

//     const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
//     auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
//     if (!network)
//     {
//         std::cerr << "Failed to create network definition" << std::endl;
//         return false;
//     }

//     auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, gLogger));
//     if (!parser)
//     {
//         std::cerr << "Failed to create ONNX parser" << std::endl;
//         return false;
//     }

//     if (!parser->parseFromFile(onnxPath.c_str(), static_cast<int32_t>(nvinfer1::ILogger::Severity::kWARNING)))
//     {
//         std::cerr << "Failed to parse ONNX model" << std::endl;
//         return false;
//     }

//     auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
//     if (!config)
//     {
//         std::cerr << "Failed to create builder config" << std::endl;
//         return false;
//     }

//     config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspaceSize);

//     if (useFP16)
//     {
//         if (builder->platformHasFastFp16())
//         {
//             config->setFlag(nvinfer1::BuilderFlag::kFP16);
//             std::cout << "FP16 mode enabled" << std::endl;
//         }
//         else
//         {
//             std::cout << "FP16 not supported on this platform, falling back to FP32" << std::endl;
//         }
//     }

//     if (useINT8)
//     {
//         config->setFlag(nvinfer1::BuilderFlag::kINT8);
//         std::cout << "INT8 mode enabled" << std::endl;
//     }

//     std::cout << "Building serialized network... (this may take a while)" <<std::endl;
//     auto serializedEngine = std::unique_ptr<nvinfer1::IHostMemory>(
//         builder->buildSerializedNetwork(*network, *config));

//     if (!serializedEngine)
//     {
//         std::cerr << "Failed to build serialized engine" << std::endl;
//         return false;
//     }

//     std::ofstream engineFile(engineSavePath, std::ios::binary);
//     if (!engineFile.good())
//     {
//         std::cerr << "Failed to open engine save path: " << engineSavePath << std::endl;
//         return false;
//     }

//     engineFile.write(static_cast<const char*>(serializedEngine->data()), serializedEngine->size());
//     engineFile.close();

//     std::cout << "TensorRT engine saved to: " << engineSavePath << std::endl;
//     std::cout << "Engine size: " << serializedEngine->size() / (1024 * 1024) << " MB" << std::endl;

//     return true;
// }



inline float sigmoid(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

void nms_(std::vector<Detection>& detections, float iouThreshold)
{
    const size_t n = detections.size();
    if (n <= 1) return;

    // 按置信度降序排序
    std::sort(detections.begin(), detections.end(),
              [](const Detection& a, const Detection& b) { return a.confidence > b.confidence; });

    // 预计算所有框的坐标和面积（一次性提取，避免内层循环重复计算 cv::Rect 成员访问）
    std::vector<int> x1(n), y1(n), x2(n), y2(n), area(n);
    for (size_t i = 0; i < n; ++i) {
        const cv::Rect& b = detections[i].box;
        x1[i] = b.x;
        y1[i] = b.y;
        x2[i] = b.x + b.width;
        y2[i] = b.y + b.height;
        area[i] = b.width * b.height;
    }

    std::vector<uint8_t> keep(n, 1);

    for (size_t i = 0; i < n; ++i) {
        if (!keep[i]) continue;

        const int xi1 = x1[i], yi1 = y1[i], xi2 = x2[i], yi2 = y2[i];
        const int area_i = area[i];

        for (size_t j = i + 1; j < n; ++j) {
            if (!keep[j]) continue;

            int inter_x1 = xi1 > x1[j] ? xi1 : x1[j];
            int inter_y1 = yi1 > y1[j] ? yi1 : y1[j];
            int inter_x2 = xi2 < x2[j] ? xi2 : x2[j];
            int inter_y2 = yi2 < y2[j] ? yi2 : y2[j];

            int inter_w = inter_x2 - inter_x1;
            if (inter_w <= 0) continue;
            int inter_h = inter_y2 - inter_y1;
            if (inter_h <= 0) continue;

            int inter_area = inter_w * inter_h;
            int union_area = area_i + area[j] - inter_area;

            // 用乘法代替除法: inter/union > thr  ⟺  inter > thr * union
            if (static_cast<float>(inter_area) > iouThreshold * static_cast<float>(union_area)) {
                keep[j] = 0;
            }
        }
    }

    // 原地压缩保留的检测结果（单次遍历）
    size_t w = 0;
    for (size_t i = 0; i < n; ++i) {
        if (keep[i]) {
            if (w != i) detections[w] = std::move(detections[i]);
            ++w;
        }
    }
    detections.resize(w);
}

void fast_nms(std::vector<Detection>& detections, float iouThreshold)
{
    nms_(detections, iouThreshold);
}

void adaptive_nms(std::vector<Detection>& detections, float baseIouThreshold, const cv::Size& modelShape) {
    float density = static_cast<float>(detections.size()) / (modelShape.width * modelShape.height);
    float adaptiveThreshold = baseIouThreshold * (1.0f + density * 10.0f);
    nms_(detections, adaptiveThreshold);
}


static void print_dims(const nvinfer1::Dims& d)
{
    std::cout << "Dims.nbDims=" << d.nbDims << "[";
    for (int i = 0; i < d.nbDims; i++)
    {
        std::cout << d.d[i];
        if (i<d.nbDims-1) std::cout << ",";
    }
    std::cout << "]" << std::endl;
}

std::vector<Detection> Inference::runInference(const cv::Mat &input)
{
    cv::Mat modelInput = input;
    int pad_x, pad_y;
    float scale;
    if (letterBoxForSquare && modelShape.width == modelShape.height)
        modelInput = formatToSquare(modelInput, &pad_x, &pad_y, &scale);

    cv::Mat blob;
    cv::dnn::blobFromImage(modelInput, blob, 1.0/255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    int rows = outputs[0].size[1];
    int dimensions = outputs[0].size[2];

    bool yolov8 = false;
    // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
    // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
    if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8)
    {
        yolov8 = true;
        rows = outputs[0].size[2];
        dimensions = outputs[0].size[1];

        outputs[0] = outputs[0].reshape(1, dimensions);
        cv::transpose(outputs[0], outputs[0]);
    }
    float *data = (float *)outputs[0].data;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i)
    {
        if (yolov8)
        {
            float *classes_scores = data+4;
            cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;

            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

            if (maxClassScore > modelScoreThreshold)
            {
                confidences.push_back(maxClassScore);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                int left = int((x - 0.5 * w - pad_x) / scale);
                int top = int((y - 0.5 * h - pad_y) / scale);

                int width = int(w / scale);
                int height = int(h / scale);

                boxes.push_back(cv::Rect(left, top, width, height));
            }
            //std::cout << "is v8" << std::endl;
        }
        else // yolov5
        {
            float confidence = data[4];

            if (confidence >= modelConfidenceThreshold)
            {
                float *classes_scores = data+5;

                cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
                cv::Point class_id;
                double max_class_score;

                minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

                if (max_class_score > modelScoreThreshold)
                {
                    confidences.push_back(confidence);
                    class_ids.push_back(class_id.x);

                    float x = data[0];
                    float y = data[1];
                    float w = data[2];
                    float h = data[3];

                    int left = int((x - 0.5 * w - pad_x) / scale);
                    int top = int((y - 0.5 * h - pad_y) / scale);

                    int width = int(w / scale);
                    int height = int(h / scale);

                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
            //std::cout << "is v5" << std::endl;
        }

        data += dimensions;
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    std::vector<Detection> detections{};
    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(100, 255);
        result.color = cv::Scalar(dis(gen),
                                  dis(gen),
                                  dis(gen));

        result.className = classes[result.class_id];
        result.box = boxes[idx];

        detections.push_back(result);
    }

    return detections;
}

size_t Inference_trt::getSizeByDims(const nvinfer1::Dims& dims) {
    size_t size = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        size *= dims.d[i];
    }
    return size;
}//检查张量维度

size_t Inference_trt::getElementSize(nvinfer1::DataType t) {
    switch (t) {
        case nvinfer1::DataType::kFLOAT: return 4;
        case nvinfer1::DataType::kHALF: return 2;
        case nvinfer1::DataType::kINT8: return 1;
        case nvinfer1::DataType::kINT32: return 4;
        case nvinfer1::DataType::kBOOL: return 1;
        default: throw std::runtime_error("Invalid DataType");
    }
}//根据类型返回数据大小

void Inference_trt::loadTensorRTEngine(const std::string &enginePath)//初始化推理引擎
{
    runtime = std::shared_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(gLogger));
    std::ifstream engineFile(enginePath, std::ios::binary);
    if (!engineFile.good())
    {
        throw std::runtime_error("TensorRT engine file not found");
    }

    engineFile.seekg(0, std::ios::end);
    size_t modelsize = engineFile.tellg();
    engineFile.seekg(0, std::ios::beg);
    std::vector<char> modelData(modelsize);
    engineFile.read(modelData.data(), modelsize);
    engineFile.close();

     // 创建runtime
    runtime = std::shared_ptr<nvinfer1::IRuntime>(
        nvinfer1::createInferRuntime(gLogger)
    );
    
    if (!runtime) {
        throw std::runtime_error("Failed to create TensorRT runtime");
    }
    
    // 反序列化引擎 - TensorRT 10.x版本只需要2个参数
    trtEngine = std::shared_ptr<nvinfer1::ICudaEngine>(
        runtime->deserializeCudaEngine(modelData.data(), modelsize)
    );
    
    if (!trtEngine) {
        throw std::runtime_error("Failed to deserialize CUDA engine");
    }
    
    // 创建执行上下文
    trtContext = std::shared_ptr<nvinfer1::IExecutionContext>(
        trtEngine->createExecutionContext()
    );
    
    if (!trtContext) {
        throw std::runtime_error("Failed to create execution context");
    }
    
     // 获取IO tensor数量
    int numIOTensors = trtEngine->getNbIOTensors();
    deviceBuffers.resize(numIOTensors);
    
    // 查找输入输出tensor
    for (int i = 0; i < numIOTensors; ++i) {
        const char* tensorName = trtEngine->getIOTensorName(i);
        nvinfer1::TensorIOMode ioMode = trtEngine->getTensorIOMode(tensorName);
        
        if (ioMode == nvinfer1::TensorIOMode::kINPUT) {
            inputIndex = i;
            inputTensorName = tensorName;
            //std::cout << "Input tensor: " << inputTensorName << std::endl;
        } else if (ioMode == nvinfer1::TensorIOMode::kOUTPUT) {
            outputIndex = i;
            outputTensorName = tensorName;
            //std::cout << "Output tensor: " << outputTensorName << std::endl;
        }
    }
    
    // 获取输入输出维度
    nvinfer1::Dims inputDims = trtEngine->getTensorShape(inputTensorName.c_str());
    nvinfer1::Dims outputDims = trtEngine->getTensorShape(outputTensorName.c_str());
    //std::cout << "input dims:" ;print_dims(inputDims);
    //std::cout <<"output dims:" ;print_dims(outputDims);
    
    if (inputDims.nbDims >= 3)
    {
        int H = 0;
        int W = 0;
        if(inputDims.nbDims == 4) //NCHW
        {
            H = inputDims.d[2];
            W = inputDims.d[3];
        }
        else if(inputDims.nbDims ==3) //CHW
        {
            H = inputDims.d[1];
            W = inputDims.d[2];
        }
        if(H >0 && W >0)
        {
            modelShape = cv::Size(W,H);
        }
    }
    // 获取数据类型
    inputDataType = trtEngine->getTensorDataType(inputTensorName.c_str());
    outputDataType = trtEngine->getTensorDataType(outputTensorName.c_str());

    // 计算缓冲区大小
    inputSize = getSizeByDims(inputDims) * getElementSize(inputDataType);
    outputSize = getSizeByDims(outputDims) * getElementSize(outputDataType);

    // 预分配host端缓冲区
    if (inputDataType == nvinfer1::DataType::kHALF) {
        hostInputHalf.resize(inputSize / sizeof(__half));
    }
    if (outputDataType == nvinfer1::DataType::kHALF) {
        hostOutputHalf.resize(outputSize / sizeof(__half));
    }
    hostOutputFloat.resize(outputSize / getElementSize(outputDataType));
    
    // 分配设备内存
    cudaMalloc(&deviceBuffers[inputIndex], inputSize);
    cudaMalloc(&deviceBuffers[outputIndex], outputSize);    

    buffers = deviceBuffers.data();
    
    // 设置执行上下文的tensor地址
    trtContext->setInputTensorAddress(inputTensorName.c_str(), deviceBuffers[inputIndex]);
    trtContext->setOutputTensorAddress(outputTensorName.c_str(), deviceBuffers[outputIndex]);
    
    std::cout << "TensorRT engine loaded successfully" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Input tensor: " << inputTensorName << ", size: " << inputSize << " bytes" << std::endl;
    std::cout << "Output tensor: " << outputTensorName << ", size: " << outputSize << " bytes" << std::endl;
    cudaStreamCreate(&cudaStream);

}

/**
 * @brief 使用TensorRT引擎执行目标检测推理
 * @param input 输入图像（OpenCV Mat格式）
 * @return std::vector<Detection> 检测结果向量，包含检测到的目标框、置信度和类别信息
 */
std::vector<Detection> Inference_trt::runInference_TensorRT(const cv::Mat &input)
{

    // 检查TensorRT引擎是否已加载
    if (!trtContext) 
    {
        throw std::runtime_error("TensorRT engine not loaded");
    }

    // 记录开始时间
    auto t_start = std::chrono::high_resolution_clock::now();

    // --- 图像预处理 ---
    cv::Mat modelInput = input;
    int pad_x{0};
    int pad_y{0};
    float scale{1.0f};
    // 如果需要letterbox处理且模型输入是正方形
    if (letterBoxForSquare && modelShape.width == modelShape.height)
        modelInput = formatToSquare(modelInput, &pad_x, &pad_y, &scale);
    // --- 预处理 ---
    auto t_pre_start = std::chrono::high_resolution_clock::now();
    cv::Mat inputBlob;
    cv::dnn::blobFromImage(modelInput, inputBlob, 1.0f/255.0f, modelShape, cv::Scalar(), true, false);

    auto t_pre_end = std::chrono::high_resolution_clock::now();

    // --- H2D ---
    auto t_h2d_start = std::chrono::high_resolution_clock::now();

    if (inputDataType == nvinfer1::DataType::kHALF) {
        // 将 4D blob 展平为 1D，用 OpenCV convertTo 做硬件加速的 FP32→FP16
        cv::Mat blobFlat(1, static_cast<int>(inputBlob.total()), CV_32F, inputBlob.data);
        cv::Mat outHalf(1, static_cast<int>(hostInputHalf.size()), CV_16FC1, hostInputHalf.data());
        blobFlat.convertTo(outHalf, CV_16F);
        cudaError_t err = cudaMemcpyAsync(deviceBuffers[inputIndex], hostInputHalf.data(), inputSize, cudaMemcpyHostToDevice, cudaStream);
        if(err != cudaSuccess) {
            std::cerr << "cudamem failed" << cudaGetErrorString(err) << std::endl;
            throw std::runtime_error("cudamem failed");
        }
    } else {
        cudaError_t err = cudaMemcpyAsync(deviceBuffers[inputIndex], inputBlob.data, inputSize, cudaMemcpyHostToDevice, cudaStream);
        if(err != cudaSuccess) {
            std::cerr << "cudamem failed" << cudaGetErrorString(err) << std::endl;
            throw std::runtime_error("cudamem failed");
        }
    }

    auto t_h2d_end = std::chrono::high_resolution_clock::now();

    // --- 推理 ---
    auto t_inf_start = std::chrono::high_resolution_clock::now();

    if (!trtContext->enqueueV3(cudaStream)) {
        throw std::runtime_error("Failed to execute inference");
    }

    auto t_inf_end = std::chrono::high_resolution_clock::now();

    // --- D2H ---
    auto t_d2h_start = std::chrono::high_resolution_clock::now();
    std::vector<float> outputData(outputSize / sizeof(float));
    cudaError_t err_mem2 = cudaMemcpyAsync(outputData.data(), deviceBuffers[outputIndex], outputSize, cudaMemcpyDeviceToHost, cudaStream);
    if(err_mem2 != cudaSuccess)
    {
        std::cerr << "cudamem failed" << cudaGetErrorString(err_mem2) << std::endl;
        throw std::runtime_error("cudamem failed");
    }
    cudaError_t err_sync = cudaStreamSynchronize(cudaStream);
    if (err_sync != cudaSuccess)
    {
        std::cerr << "cudaStreamSynchronize after D2H failed: " << cudaGetErrorString(err_sync) << std::endl;
        throw std::runtime_error("cudaStreamSynchronize failed");
    }
    auto t_d2h_end = std::chrono::high_resolution_clock::now();
    //nvinfer1::Dims outputDims = trtEngine->getTensorShape(outputTensorName.c_str());
    //float* output = static_cast<float*>(buffers[outputIndex]);

    //---- postprocess + nms ----
    auto t_post_start = std::chrono::high_resolution_clock::now();
    float* output = outputData.data();
    std::vector<Detection> detections;
    
    nvinfer1::Dims outputDims = trtEngine->getTensorShape(outputTensorName.c_str());
    //print_dims(outputDims);
    
    int num_preds = 0;
    int elem_per_pred = 0;
    bool is_field_major =false;

    if (outputDims.nbDims == 3)
    {
        int d0 = (outputDims.nbDims > 0) ? outputDims.d[0] : 0;
        int d1 = (outputDims.nbDims > 1) ? outputDims.d[1] : 0;
        int d2 = (outputDims.nbDims > 2) ? outputDims.d[2] : 0;

        //std::cout << "DEBUG outputDims: nbDims=" << outputDims.nbDims
        //        << " d0=" << d0 << " d1=" << d1 << " d2=" << d2 << std::endl;

        // 更稳健的退化处理：优先检测某维为5（即只有 cx,cy,w,h,obj_logit）
        if (d1 == 5 && d2 > 5) {
            elem_per_pred = 5;
            num_preds = d2;
            is_field_major = true; // layout: [1,5,8400]
            //std::cout << "DEBUG layout: field-major [1,5,N], elem_per_pred=5, num_preds=" << num_preds << std::endl;
        }
        else if (d2 == 5 && d1 > 5) {
            elem_per_pred = 5;
            num_preds = d1;
            is_field_major = false; // layout: [1,N,5]
            //std::cout << "DEBUG layout: pred-major [1,N,5], elem_per_pred=5, num_preds=" << num_preds << std::endl;
        }//car特用
        else {
            // 通用判定：较小维度为特征数(elem_per_pred)，较大维度为预测数(num_preds)
            // [1, C, N] field-major 且 C < N  → elem=C, num=N, field_major=true
            // [1, N, C] pred-major  且 N > C  → elem=C, num=N, field_major=false
            if (d1 >= 5 && d2 >= 5) {
                elem_per_pred = std::min(d1, d2);
                num_preds      = std::max(d1, d2);
                is_field_major = (d1 < d2);  // d1<d2 即 [1, C, N] field-major
            } else {
                // 退回原先的保守猜测（尽量不颠倒 5 与 N）
                elem_per_pred = std::max(d1, d2);
                num_preds = std::min(d1, d2);
                is_field_major = (d1 > d2);
            }
            //std::cout << "DEBUG fallback layout: elem_per_pred=" << elem_per_pred << " num_preds=" << num_preds
            //        << " is_field_major=" << is_field_major << std::endl;
        }
    }
    else if (outputDims.nbDims == 2)
    {
        int d0 = outputDims.d[0];
        int d1 = outputDims.d[1];
        // 较小维度为特征数，较大维度为预测数
        elem_per_pred = std::min(d0, d1);
        num_preds     = std::max(d0, d1);
        is_field_major = (d0 < d1); // [C, N] field-major if d0<d1
        //std::cout << "DEBUG layout: 2D, elem_per_pred=" << elem_per_pred << ", num_preds=" << num_preds << std::endl;
    }
    else 
    {
        size_t total_elems = outputSize / sizeof(float);
        if (total_elems % 5 == 0)
        {
            elem_per_pred = 5;
            num_preds = static_cast<int>(total_elems / 5);
            is_field_major = true; // layout: [N,5]
            //std::cout << "DEBUG fallback flat layout: elem_per_pred=5, num_preds=" << num_preds << std::endl;
        }
        else
        {
            throw std::runtime_error("Unsupported output dimensions");
        }
    }
    //int num_preds = outputDims.d[2];
    //int elem_per_pred = 5;
    
    //const int num_preds = 8400;
    //const int elem_per_pred = 5;//cx,cy,w,h,conf
    //bool is_field_major = (outputDims.nbDims == 3 && outputDims.d[0] == 1 && outputDims.d[1] == 5);
    auto get_output = [&](int pred_idx,int field_idx)->float
    {
        if(is_field_major)
        {
            return outputData[field_idx * num_preds + pred_idx];
        }
        else
        {
            return outputData[pred_idx * elem_per_pred + field_idx];
        }
    };

    //std::vector<Detection> detections;
    detections.reserve(std::min(num_preds, 4096));
    const int INPUT_SIZE = modelShape.width; // assume square input
    for (int i = 0; i < num_preds; i++)
    {
        int best_class = 0;
        float best_score = 0.0f;
        if (yolov5Format)
        {
            // YOLOv5: [cx,cy,w,h, obj, cls_0..cls_N]  4 bbox + 1 obj + N classes = elem_per_pred
            float obj_conf = sigmoid(get_output(i, 4));
            if (obj_conf < modelConfidenceThreshold) continue;

            int num_classes = elem_per_pred - 5;
            float best_cls = 1.0f;
            if (num_classes > 0) {
                best_cls = 0.0f;
                for (int c = 0; c < num_classes; ++c) {
                    float cls_prob = sigmoid(get_output(i, 5 + c));
                    if (c == 0 || cls_prob > best_cls) {
                        best_cls = cls_prob;
                        best_class = c;
                    }
                }
            }
            best_score = obj_conf * best_cls;
            if (best_score < modelScoreThreshold) continue;
        }
        else
        {
            // YOLOv11: [cx,cy,w,h, cls_0..cls_N]  4 bbox + N classes = elem_per_pred（无独立 objectness）
            int num_classes = elem_per_pred - 4;
            for (int c = 0; c < num_classes; ++c) {
                float score = sigmoid(get_output(i, 4 + c));
                //std::cout << score << std::endl;
                if (c == 0 || score > best_score) {
                    best_score = score;
                    best_class = c;
                }
            }
            if (best_score < modelScoreThreshold) continue;
        }

        float cx = get_output(i, 0);//类比二维组合先x后y顺序在上面
        float cy = get_output(i, 1);
        float w  = get_output(i, 2);
        float h  = get_output(i, 3);

        bool likely_normalized = (std::abs(cx) <= 1.01f && std::abs(cy) <= 1.01f && std::abs(w) <= 1.01f && std::abs(h) <= 1.01f);
        if (likely_normalized && INPUT_SIZE > 0) {
            cx *= INPUT_SIZE;
            cy *= INPUT_SIZE;
            w  *= INPUT_SIZE;
            h  *= INPUT_SIZE;
        }

        float x1 = cx - w * 0.5f;
        float y1 = cy - h * 0.5f;
        float x2 = cx + w * 0.5f;
        float y2 = cy + h * 0.5f;

        // de-letterbox
        x1 = (x1 - pad_x) / scale;
        y1 = (y1 - pad_y) / scale;
        x2 = (x2 - pad_x) / scale;
        y2 = (y2 - pad_y) / scale;

        x1 = std::clamp(x1, 0.f, static_cast<float>(input.cols - 1));
        y1 = std::clamp(y1, 0.f, static_cast<float>(input.rows - 1));
        x2 = std::clamp(x2, 0.f, static_cast<float>(input.cols - 1));
        y2 = std::clamp(y2, 0.f, static_cast<float>(input.rows - 1));

        Detection det;
        det.box = cv::Rect(static_cast<int>(std::round(x1)),
                           static_cast<int>(std::round(y1)),
                           static_cast<int>(std::round(x2 - x1)),
                           static_cast<int>(std::round(y2 - y1)));
        //std::cout << det.box << std::endl;
        det.confidence = best_score;

        if (!classes.empty())
        {
            if (best_class >=0 && best_class < static_cast<int>(classes.size()))
            {
                det.className = classes[best_class];
                //std::cout << det.className << std::endl;
            }
            else
            {
                det.className = classes[0];
                //std::cout << det.className << std::endl;
            }
        }
        else
        {
            det.className = (best_class >=0) ? ("class_" + std::to_string(best_class)) : "unknown";
            //std::cout << det.className << std::endl;
        }

        detections.push_back(det);
    }
    nms_(detections,modelNMSThreshold);
    auto t_post_end = std::chrono::high_resolution_clock::now();



    // --- 计时输出 ---
    //void time_logger(const std::chrono::steady_clock::time_point& t_pre_end,const std::chrono::steady_clock::time_point& t_pre_start, const std::chrono::steady_clock::time_point& t_h2d_end, const std::chrono::steady_clock::time_point& t_h2d_start, const std::chrono::steady_clock::time_point& t_inf_end, const std::chrono::steady_clock::time_point& t_inf_start, const std::chrono::steady_clock::time_point& t_d2h_end, const std::chrono::steady_clock::time_point& t_d2h_start, const std::chrono::steady_clock::time_point& t_post_end, const std::chrono::steady_clock::time_point& t_post_start, const std::chrono::steady_clock::time_point& t_start);

    //  double pre_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_pre_end - t_pre_start).count();
    //  double h2d_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_h2d_end - t_h2d_start).count();
    //  double inf_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_inf_end - t_inf_start).count();
    //  double d2h_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_d2h_end - t_d2h_start).count();
    //  double post_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_post_end - t_post_start).count();
    //  double total_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_post_end - t_start).count();

    //  std::cout << std::fixed << std::setprecision(2)
    //            << "Timing (ms): pre=" << pre_ms
    //            << " h2d=" << h2d_ms // cpu->gpu
    //            << " infer=" << inf_ms // 推理
    //            << " d2h=" << d2h_ms // gpu->cput
    //            << " post=" << post_ms // 后处理（先前测试中耗时最长的部分）
    //            << " total=" << total_ms // 总时长
    //            << std::endl;


    //size_t total_elems = outputSize / sizeof(float);
    //int num_classes_known = static_cast<int>(classes.size());

    //性能评估调试
    return detections;
}



void Inference::loadClassesFromFile()
{
    std::ifstream inputFile(classesPath);
    if (inputFile.is_open())
    {
        std::string classLine;
        while (std::getline(inputFile, classLine))
            classes.push_back(classLine);
        inputFile.close();
    }
}

void Inference::loadOnnxNetwork()
{
    net = cv::dnn::readNetFromONNX(modelPath);
    if (cudaEnabled && cv::cuda::getCudaEnabledDeviceCount() > 0)
    {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        if (cudaEnabled) {
            std::cout << "CUDA DNN not available, falling back to CPU" << std::endl;
        }
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
}

cv::Mat Inference::formatToSquare(const cv::Mat &source, int *pad_x, int *pad_y, float *scale)
{
    int col = source.cols;
    int row = source.rows;
    int m_inputWidth = modelShape.width;
    int m_inputHeight = modelShape.height;

    *scale = std::min(m_inputWidth / (float)col, m_inputHeight / (float)row);
    int resized_w = col * *scale;
    int resized_h = row * *scale;
    *pad_x = (m_inputWidth - resized_w) / 2;
    *pad_y = (m_inputHeight - resized_h) / 2;

    cv::Mat resized;
    cv::resize(source, resized, cv::Size(resized_w, resized_h));
    cv::Mat result = cv::Mat::zeros(m_inputHeight, m_inputWidth, source.type());
    resized.copyTo(result(cv::Rect(*pad_x, *pad_y, resized_w, resized_h)));
    resized.release();
    return result;
}

cv::Mat Inference_trt::formatToSquare(const cv::Mat &source, int *pad_x, int *pad_y, float *scale)
{
    int col = source.cols;
    int row = source.rows;
    int m_inputWidth = modelShape.width;
    int m_inputHeight = modelShape.height;

    *scale = std::min(m_inputWidth / (float)col, m_inputHeight / (float)row);
    int resized_w = col * *scale;
    int resized_h = row * *scale;
    *pad_x = (m_inputWidth - resized_w) / 2;
    *pad_y = (m_inputHeight - resized_h) / 2;

    cv::Mat resized;
    cv::resize(source, resized, cv::Size(resized_w, resized_h));
    cv::Mat result = cv::Mat::zeros(m_inputHeight, m_inputWidth, source.type());
    resized.copyTo(result(cv::Rect(*pad_x, *pad_y, resized_w, resized_h)));
    resized.release();
    return result;
}
