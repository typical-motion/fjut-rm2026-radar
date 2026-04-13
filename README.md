# FJUT rm2026视觉雷达

本项目基于 ROS2，集成了相机标定、目标检测与跟踪、地图绘制、串口通信等功能模块，适用于机器人感知与导航应用。

## 功能包说明

| 包名 | 功能描述 | 状态 |
|------|----------|------|
| `calibrate` | 相机标定（使用 CMake 单独编译） | 稳定 |
| `location` | 定位功能（ROS2 功能包） | 稳定 |
| `ui_design` | 地图绘制与界面交互 | 稳定 |
| `yolov8decbytetrack` | **YOLOv11 + TensorRT 加速推理 + ByteTrack 目标跟踪** | 稳定 |
| `tutorial-interfaces` | 自定义 ROS2 消息接口 | 稳定 |
| `serial` | 串口通信功能包 | **未测试** |

> 注：原包名 `yolov8decbytetrack` 实际使用 YOLOv11 模型 + TensorRT 推理加速，并集成 ByteTrack 多目标跟踪算法。

## 环境要求

- Ubuntu 22.04 
- ROS2 Humble
- OpenCV
- TensorRT ≥ 10.13.3
- CUDA ≥ 12.9
- CMake ≥ 3.8
- Python 3.8+（部分辅助脚本）
- yaml
- json

## 编译与构建

### 1. 编译 ROS2 功能包

cd ~/ros2_ws
colcon build --packages-select location ui_design yolov8decbytetrack yolov11-trt tutorial-interfaces serial
注意：serial 包虽然编译，但功能未测试。

### 2. 单独编译 calibrate 包（CMake 项目）

cd ~/ros2_ws/src/calibrate
mkdir -p build && cd build
cmake ..
make

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# 将本项目所有功能包放入 src 目录下
