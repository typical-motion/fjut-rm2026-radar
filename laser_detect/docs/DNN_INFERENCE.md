# OpenCV DNN 推理使用文档

## 概述

`Inference_dnn` 是基于 OpenCV DNN 模块的 YOLOv11 ONNX 推理类，作为 TensorRT (`Inference_trt`) 的替代后端。适用于没有 TensorRT 环境、或需要快速部署验证的场景。

---

## 文件结构

```
include/dnn_detect.hpp   # 类声明
src/dnn_detect.cpp       # 实现
```

---

## 快速开始

### 1. 包含头文件

```cpp
#include "dnn_detect.hpp"
```

### 2. 初始化

```cpp
std::vector<std::string> classes{"light"};

Inference_dnn inf(
    "/path/to/model/light.onnx",  // ONNX 模型路径
    cv::Size(640, 640),           // 模型输入尺寸
    classes,                      // 类别名称
    false                         // false=CPU, true=CUDA
);

// 可选参数
inf.setModelConfidenceThreshold(0.25f); // 置信度阈值（默认 0.25）
inf.setModelNMSThreshold(0.45f);        // NMS IoU 阈值（默认 0.45）
inf.setLetterBoxForSquare(true);        // 保持宽高比填充（默认开启）
```

### 3. 推理

```cpp
cv::Mat frame = ...; // 输入图像（BGR）

std::vector<Detection> detections = inf.runInference(frame);

for (const auto& det : detections) {
    cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
    std::cout << det.className << " conf=" << det.confidence << "\n";
}
```

---

## 构造函数参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `onnxPath` | `std::string` | YOLOv11 `.onnx` 模型文件路径 |
| `modelInputShape` | `cv::Size` | 模型输入分辨率，默认 `{640, 640}` |
| `classes_` | `vector<string>` | 类别名称列表，顺序与训练时一致 |
| `runWithCuda` | `bool` | `true` 使用 CUDA 后端，`false` 使用 CPU |

---

## Detection 结构体（与 TensorRT 版共用）

```cpp
struct Detection {
    int         class_id;    // 类别索引
    std::string className;   // 类别名称
    float       confidence;  // 置信度 [0, 1]
    cv::Scalar  color;       // 显示颜色（可选）
    cv::Rect    box;         // 检测框（像素坐标，原图尺寸）
};
```

---

## 与 TensorRT 版对比

| 特性 | `Inference_dnn` | `Inference_trt` |
|------|-----------------|-----------------|
| 模型格式 | `.onnx` | `.engine` |
| 推理速度 | 较慢（CPU）/ 中等（CUDA） | 快（TensorRT 优化） |
| 环境依赖 | OpenCV（已有依赖） | CUDA + TensorRT |
| 适用场景 | 开发调试、无 TensorRT 环境 | 生产部署 |
| 接口 | `runInference()` | `runInference_TensorRT()` |

---

## 在 main.cpp 中切换推理后端

```cpp
// TensorRT 版（当前使用）
auto inf_trt = std::make_unique<Inference_trt>(
    "/path/to/light.engine", cv::Size(640, 640), classes, true);
detections = inf_trt->runInference_TensorRT(frame);

// OpenCV DNN 版（替换上面两行即可）
auto inf_dnn = std::make_unique<Inference_dnn>(
    "/path/to/light.onnx", cv::Size(640, 640), classes, false);
detections = inf_dnn->runInference(frame);
```

---

## 模型导出（Ultralytics）

```bash
# 从 PyTorch 导出 YOLOv11 ONNX 模型
yolo export model=light.pt format=onnx imgsz=640 opset=12

# 验证导出结果
python -c "import onnx; m=onnx.load('light.onnx'); print(onnx.checker.check_model(m))"
```

> **注意**：导出时使用 `opset=12` 以确保 OpenCV DNN 兼容性。

---

## ONNX 输出格式说明

`Inference_dnn` 自动识别以下两种标准输出布局：

| 布局 | 形状示例（1类） | 说明 |
|------|----------------|------|
| field-major | `[1, 5, 8400]` | 每行为一个字段（cx/cy/w/h/score） |
| pred-major  | `[1, 8400, 5]` | 每行为一个预测框 |

坐标含义：`cx, cy, w, h` 为模型输入空间（0\~640）的像素值，类推理后自动还原到原图坐标。

---

## 常见问题

**Q: 推理结果为空？**
- 检查置信度阈值是否过高，尝试调低：`setModelConfidenceThreshold(0.1f)`
- 打印模型输出形状确认格式：`net.forward()` 后检查 `outputs[0].size`

**Q: CUDA 后端报错？**
- 确认 OpenCV 编译时启用了 CUDA DNN 支持：`cv::getBuildInformation()` 中查看 `CUDA: YES`
- 或改用 CPU 模式：构造函数最后一个参数传 `false`

**Q: ONNX 模型加载失败？**
- 检查 opset 版本，OpenCV 4.x 支持 opset ≤ 13
- 重新导出时指定 `opset=12`
