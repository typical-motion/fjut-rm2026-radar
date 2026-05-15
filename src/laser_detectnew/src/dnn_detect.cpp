#include "dnn_detect.hpp"
#include <algorithm>

Inference_dnn::Inference_dnn(const std::string &onnxPath,
                              const cv::Size &modelInputShape,
                              const std::vector<std::string> &classes_,
                              bool runWithCuda)
    : modelShape(modelInputShape), classes(classes_)
{
    net = cv::dnn::readNetFromONNX(onnxPath);
    if (net.empty())
        throw std::runtime_error("Failed to load ONNX model: " + onnxPath);

    if (runWithCuda) {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    } else {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
}

std::vector<Detection> Inference_dnn::runInference(const cv::Mat &input)
{
    int   pad_x = 0, pad_y = 0;
    float scale = 1.0f;

    cv::Mat modelInput = letterBoxForSquare
        ? formatToSquare(input, pad_x, pad_y, scale)
        : input.clone();

    // BGR→RGB, normalize to [0,1]
    cv::Mat blob;
    cv::dnn::blobFromImage(modelInput, blob, 1.0 / 255.0,
                           cv::Size(static_cast<int>(modelShape.width),
                                    static_cast<int>(modelShape.height)),
                           cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    if (outputs.empty()) return {};

    // YOLOv11 standard ONNX export: output0 shape [1, 4+nc, 8400]
    cv::Mat out = outputs[0];

    // Flatten batch dim → 2D matrix
    int rows, cols;
    if (out.dims == 3) {
        rows = out.size[1];
        cols = out.size[2];
        out  = out.reshape(1, rows);
    } else {
        rows = out.rows;
        cols = out.cols;
    }

    // Determine layout:
    //   field-major [4+nc, 8400]: rows < cols → each row is one field for all preds
    //   pred-major  [8400, 4+nc]: rows > cols → each row is one prediction
    bool field_major = (rows < cols);
    int  num_preds   = field_major ? cols : rows;
    int  num_fields  = field_major ? rows : cols;
    int  num_classes = std::max(1, num_fields - 4);

    auto get_val = [&](int pred_i, int field_j) -> float {
        return field_major ? out.at<float>(field_j, pred_i)
                           : out.at<float>(pred_i,  field_j);
    };

    std::vector<cv::Rect> boxes;
    std::vector<float>    confidences;
    std::vector<int>      class_ids;

    for (int i = 0; i < num_preds; i++) {
        // cx, cy, w, h are in model-input pixel space (0..input_size)
        float cx = get_val(i, 0);
        float cy = get_val(i, 1);
        float w  = get_val(i, 2);
        float h  = get_val(i, 3);

        // Find best class score (YOLOv11 ONNX: no separate objectness, scores are direct)
        float max_score  = 0.0f;
        int   best_class = 0;
        for (int c = 0; c < num_classes; c++) {
            float score = get_val(i, 4 + c);
            if (score > max_score) {
                max_score  = score;
                best_class = c;
            }
        }
        if (max_score < modelConfidenceThreshold) continue;

        // De-letterbox: model-input space → original image coords
        float x1 = (cx - w * 0.5f - pad_x) / scale;
        float y1 = (cy - h * 0.5f - pad_y) / scale;
        float bw = w / scale;
        float bh = h / scale;

        x1 = std::clamp(x1, 0.f, static_cast<float>(input.cols - 1));
        y1 = std::clamp(y1, 0.f, static_cast<float>(input.rows - 1));
        bw = std::min(bw, static_cast<float>(input.cols) - x1);
        bh = std::min(bh, static_cast<float>(input.rows) - y1);

        boxes.push_back(cv::Rect(static_cast<int>(x1), static_cast<int>(y1),
                                 static_cast<int>(bw),  static_cast<int>(bh)));
        confidences.push_back(max_score);
        class_ids.push_back(best_class);
    }

    // NMS via OpenCV DNN
    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(boxes, confidences,
                      modelConfidenceThreshold, modelNMSThreshold, nms_indices);

    std::vector<Detection> detections;
    detections.reserve(nms_indices.size());
    for (int idx : nms_indices) {
        Detection det;
        det.box        = boxes[idx];
        det.confidence = confidences[idx];
        det.class_id   = class_ids[idx];
        if (!classes.empty() && class_ids[idx] < static_cast<int>(classes.size()))
            det.className = classes[class_ids[idx]];
        else
            det.className = "class_" + std::to_string(class_ids[idx]);
        detections.push_back(det);
    }
    return detections;
}

cv::Mat Inference_dnn::formatToSquare(const cv::Mat &source,
                                       int &pad_x, int &pad_y, float &scale)
{
    int col = source.cols;
    int row = source.rows;
    int iw  = static_cast<int>(modelShape.width);
    int ih  = static_cast<int>(modelShape.height);

    scale = std::min(iw / static_cast<float>(col), ih / static_cast<float>(row));
    int rw = static_cast<int>(col * scale);
    int rh = static_cast<int>(row * scale);
    pad_x  = (iw - rw) / 2;
    pad_y  = (ih - rh) / 2;

    cv::Mat resized;
    cv::resize(source, resized, cv::Size(rw, rh));
    cv::Mat result = cv::Mat::zeros(ih, iw, source.type());
    resized.copyTo(result(cv::Rect(pad_x, pad_y, rw, rh)));
    return result;
}
