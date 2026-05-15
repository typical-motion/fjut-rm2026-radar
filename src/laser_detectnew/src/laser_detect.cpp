#include "laser_detect.hpp"

Inference_trt::Inference_trt(const std::string &enginePath,const cv::Size &modelInputShape, const std::vector<std::string> &classes_, const bool &runWithCuda)
{
    modelPath = enginePath;
    modelShape = modelInputShape;
    classes = classes_;
    cudaEnabled = runWithCuda;

    loadTensorRTEngine(enginePath);
    //cudaStreamCreate(&stream);
}

inline float sigmoid(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

void nms_(std::vector<Detection>& detections,float iouThreshold)
{
    auto t_post_start = std::chrono::high_resolution_clock::now();

    std::sort(detections.begin(), detections.end(),[](const Detection& a, const Detection& b) {return a.confidence > b.confidence;});//置信度排序
    //std::cout << "detections size before nms: " << detections.size() << std::endl;
    
    std::vector<bool> keep(detections.size(), true);

    for (size_t i = 0; i < detections.size(); i++)
    {
        if (!keep[i]) continue;
        const cv::Rect& box_i = detections[i].box;
        for (size_t j = i + 1; j < detections.size(); j++)
        {
            if (!keep[j]) continue;
            const cv::Rect& box_j = detections[j].box;
            float interArea = (box_i & box_j).area();
            float unionArea = box_i.area() + box_j.area() - interArea;
            float iou = interArea / unionArea;
            if (iou > iouThreshold) keep[j] = false;
        }
    }
    auto t_post_end = std::chrono::high_resolution_clock::now();
    std::vector<Detection> nmsDetections;
    for (size_t i = 0; i< detections.size(); i++)
    {
        if (keep[i]) nmsDetections.push_back(detections[i]);
    }
    detections = std::move(nmsDetections);
    double post_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_post_end - t_post_start).count();
    //std::cout << std::fixed << std::setprecision(2)
    //            << " post:" << post_ms
    //            << std::endl;

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

    std::cout << "Number of IO tensors: " << numIOTensors << std::endl;
    std::cout << "Number of optimization profiles: " << trtEngine->getNbOptimizationProfiles() << std::endl;

    inputIndex = -1;
    outputIndex = -1;

    // 查找输入输出tensor，优先匹配名为"output"的主输出
    for (int i = 0; i < numIOTensors; ++i) {
        const char* tensorName = trtEngine->getIOTensorName(i);
        nvinfer1::TensorIOMode ioMode = trtEngine->getTensorIOMode(tensorName);
        nvinfer1::Dims shape = trtEngine->getTensorShape(tensorName);
        nvinfer1::DataType dtype = trtEngine->getTensorDataType(tensorName);
        size_t tensorSize = getSizeByDims(shape) * getElementSize(dtype);

        std::cout << "  Tensor[" << i << "]: " << tensorName
                  << " mode=" << (ioMode == nvinfer1::TensorIOMode::kINPUT ? "INPUT" : "OUTPUT");
        std::cout << " shape=[";
        for (int d = 0; d < shape.nbDims; ++d) {
            if (d > 0) std::cout << ",";
            std::cout << shape.d[d];
        }
        std::cout << "] size=" << tensorSize << " bytes" << std::endl;

        if (ioMode == nvinfer1::TensorIOMode::kINPUT) {
            if (inputIndex < 0) {
                inputIndex = i;
                inputTensorName = tensorName;
            }
        } else if (ioMode == nvinfer1::TensorIOMode::kOUTPUT) {
            // 优先使用主输出"output"，否则使用第一个输出
            if (outputIndex < 0 || std::string(tensorName) == "output") {
                outputIndex = i;
                outputTensorName = tensorName;
            }
        }

        // 为所有IO tensor分配内存并设置地址
        cudaMalloc(&deviceBuffers[i], tensorSize);
        bool ok = trtContext->setTensorAddress(tensorName, deviceBuffers[i]);
        std::cout << "    -> setTensorAddress: " << (ok ? "OK" : "FAIL") << std::endl;
    }

    if (inputIndex < 0 || outputIndex < 0) {
        throw std::runtime_error("Failed to find input or output tensor");
    }

    // 获取输入输出维度
    nvinfer1::Dims inputDims = trtEngine->getTensorShape(inputTensorName.c_str());
    nvinfer1::Dims outputDims = trtEngine->getTensorShape(outputTensorName.c_str());

    if (inputDims.nbDims >= 3)
    {
        int H = 0;
        int W = 0;
        if(inputDims.nbDims == 4)
        {
            H = inputDims.d[2];
            W = inputDims.d[3];
        }
        else if(inputDims.nbDims ==3)
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
    nvinfer1::DataType inputDataType = trtEngine->getTensorDataType(inputTensorName.c_str());
    nvinfer1::DataType outputDataType = trtEngine->getTensorDataType(outputTensorName.c_str());

    // 计算主输入输出缓冲区大小
    inputSize = getSizeByDims(inputDims) * getElementSize(inputDataType);
    outputSize = getSizeByDims(outputDims) * getElementSize(outputDataType);

    // 先设置输入shape
    if (!trtContext->setInputShape(inputTensorName.c_str(), inputDims)) {
        std::cerr << "WARNING: setInputShape failed for " << inputTensorName << std::endl;
    }

    buffers = deviceBuffers.data();
    cudaStreamCreate(&cudaStream);
    trtContext->setOptimizationProfileAsync(0, cudaStream);

    std::cout << "Primary input:  " << inputTensorName << " (index " << inputIndex << ")" << std::endl;
    std::cout << "Primary output: " << outputTensorName << " (index " << outputIndex << ")" << std::endl;

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
    size_t blob_bytes = inputBlob.total() * inputBlob.elemSize();

    if (blob_bytes > inputSize)
    {
        std::cerr << "Warning: input blob size (" << blob_bytes << ") > expected inputSize (" << inputSize << "). Using min size to copy.\n";
    }

    size_t copy_bytes = std::min(blob_bytes, inputSize);
    auto t_pre_end = std::chrono::high_resolution_clock::now();

    std::vector<float> inputData(inputSize / sizeof(float), 0.0f);
    memcpy(inputData.data(), inputBlob.data, copy_bytes);

    // --- H2D (measure) ---
    auto t_h2d_start = std::chrono::high_resolution_clock::now();

    cudaError_t err_mem1 = cudaMemcpyAsync(deviceBuffers[inputIndex], inputData.data(), inputSize, cudaMemcpyHostToDevice, cudaStream);
    if(err_mem1 != cudaSuccess)
    {
        std::cerr << "cudamem failed" << cudaGetErrorString(err_mem1) << std::endl;
        throw std::runtime_error("cudamem failed");
    }

    auto t_h2d_end = std::chrono::high_resolution_clock::now();

    // --- 推理 ---
    auto t_inf_start = std::chrono::high_resolution_clock::now();

    if (!trtContext->enqueueV3(cudaStream)) {
        throw std::runtime_error("Failed to execute inference");
    }
    
    cudaError_t err_stream = cudaStreamSynchronize(cudaStream);
    if (err_stream != cudaSuccess)
    {
        std::cerr << "cudastream failed" << cudaGetErrorString(err_stream) << std::endl;
        throw std::runtime_error("cudastream failed");
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

        if (d1 == 17 && d2 >17)
        {
            elem_per_pred = 17;
            num_preds = d2;
            is_field_major = true; // layout: [1,17,8400]
        }

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
        }
        else {
            // 原有的通用判定：如果一个维度明显大于另一个，按字段/预测顺序判断
            if (d1 >= 5 && d2 >= 5) {
                if (d1 > d2) {
                    elem_per_pred = d1;
                    num_preds = d2;
                    is_field_major = true;
                } else {
                    elem_per_pred = d2;
                    num_preds = d1;
                    is_field_major = false;
                }
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
        num_preds = outputDims.d[0];
        elem_per_pred = outputDims.d[1];
        is_field_major = false; // layout: [N, elem_per_pred]
        //std::cout << "DEBUG layout: pred-major [N, elem_per_pred], elem_per_pred=" << elem_per_pred << ", num_preds=" << num_preds << std::endl;
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
    //const int INPUT_SIZE = 640;
    const int strides[3] = {8, 16, 32};
    const int grids[3]   = {80, 40, 20};
    for (int i = 0; i <num_preds; i++)
    {
        float raw_obj = get_output(i, 4);
        float conf_obj = sigmoid(raw_obj);
        if (conf_obj < modelScoreThreshold) continue;

        float cx = get_output(i, 0);
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

        //float conf = output[4 * num_preds + i];
        //if (conf < modelScoreThreshold) continue;
        //float best_score = sigmoid(conf);
        //float cx = output[0 * num_preds + i];
        //float cy = output[1 * num_preds + i];
        //float w  = output[2 * num_preds + i];
        //float h  = output[3 * num_preds + i];

        float x1 = cx - w * 0.5f;
        float y1 = cy - h * 0.5f;
        float x2 = cx + w * 0.5f;
        float y2 = cy + h * 0.5f;

        // handle category logits if present (elem_per_pred > 5)
        int best_class = 0;
        float best_class_score = 0.0f;
        if (elem_per_pred > 5)
        {
            int num_classes = elem_per_pred - 5;
            // find argmax among class logits
            for (int c = 0; c < num_classes; ++c)
            {
                float cls_raw = get_output(i, 5 + c);
                float cls_prob = sigmoid(cls_raw);
                if (c == 0 || cls_prob > best_class_score)
                {
                    best_class_score = cls_prob;
                    best_class = c;
                }
            }
            // combine objectness and class prob (optional) - here we use product
            float combined_conf = conf_obj * best_class_score;
            if (combined_conf < modelScoreThreshold) continue;
        }

        // de-letterbox
        x1 = (x1 - pad_x) / scale;
        y1 = (y1 - pad_y) / scale;
        x2 = (x2 - pad_x) / scale;
        y2 = (y2 - pad_y) / scale;

        x1 = std::clamp(x1, 0.f, static_cast<float>(input.cols - 1));
        y1 = std::clamp(y1, 0.f, static_cast<float>(input.rows - 1));
        x2 = std::clamp(x2, 0.f, static_cast<float>(input.cols - 1));
        y2 = std::clamp(y2, 0.f, static_cast<float>(input.rows - 1));
        //int best_class = 0;
        Detection det;
        det.box = cv::Rect(static_cast<int>(std::round(x1)),
                           static_cast<int>(std::round(y1)),
                           static_cast<int>(std::round(x2 - x1)),
                           static_cast<int>(std::round(y2 - y1)));
        det.confidence = conf_obj;
        
        if (!classes.empty())
        {
            if (best_class >=0 && best_class <static_cast<int>(classes.size()))
            {
                det.className = classes[best_class];
            }
            else 
            {
                det.className = classes[0];
            }
        }
        else 
        {
            det.className = (best_class >=0) ? ("class_" + std::to_string(best_class)) : "unknown";
        }

        detections.push_back(det);
    }
    
    detections.erase(
    std::remove_if(detections.begin(), detections.end(),
        [](const Detection& d)
        {
            return d.box.width <= 0 ||
                   d.box.height <= 0;
        }),
    detections.end());
    
        nms_(detections,0.001f);
    //fast_nms(detections, 0.0001f);
    auto t_post_end = std::chrono::high_resolution_clock::now();

    // --- 计时输出 ---
    // double pre_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_pre_end - t_pre_start).count();
    // double h2d_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_h2d_end - t_h2d_start).count();
    // double inf_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_inf_end - t_inf_start).count();
    // double d2h_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_d2h_end - t_d2h_start).count();
    // double post_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_post_end - t_post_start).count();
    // double total_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_post_end - t_start).count();

    // std::cout << std::fixed << std::setprecision(2)
    //           << "Timing (ms): pre=" << pre_ms
    //           << " h2d=" << h2d_ms // cpu->gpu
    //           << " infer=" << inf_ms // 推理
    //           << " d2h=" << d2h_ms // gpu->cput
    //           << " post=" << post_ms // 后处理（先前测试中耗时最长的部分）
    //           << " total=" << total_ms // 总时长
    //           << std::endl;


    //size_t total_elems = outputSize / sizeof(float);
    //int num_classes_known = static_cast<int>(classes.size());

    //性能评估调试
    return detections;
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