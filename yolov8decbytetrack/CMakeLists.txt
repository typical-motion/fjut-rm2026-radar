cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)
project(yolov8decbytetrack)

set(CMAKE_BUILD_TYPE Debug)

# ===================== C++ 标准 =====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wno-deprecated-declarations)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ===================== CUDA =====================
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.9")
find_package(CUDAToolkit REQUIRED)
find_package(CUDA 12.9 REQUIRED)

# ===================== TensorRT (系统路径) =====================
set(TensorRT_INCLUDE_DIR "/usr/include/x86_64-linux-gnu")
set(TensorRT_LIB_DIR "/usr/lib/x86_64-linux-gnu")

find_library(NVINFER_LIB NAMES nvinfer PATHS ${TensorRT_LIB_DIR} REQUIRED)
find_library(NVONNXPARSER_LIB NAMES nvonnxparser PATHS ${TensorRT_LIB_DIR} REQUIRED)
find_library(NVINFER_PLUGIN_LIB NAMES nvinfer_plugin PATHS ${TensorRT_LIB_DIR})

if(NOT EXISTS "${TensorRT_INCLUDE_DIR}/NvInfer.h")
    message(FATAL_ERROR "TensorRT headers not found in ${TensorRT_INCLUDE_DIR}")
endif()

# ===================== 海康威视 SDK (路径修正) =====================
set(HIK_SDK_ROOT "/opt/MVS")
set(HIK_INCLUDE_DIR "${HIK_SDK_ROOT}/include")
set(HIK_LIB_DIR "${HIK_SDK_ROOT}/lib/64")

if(NOT EXISTS "${HIK_INCLUDE_DIR}")
    message(FATAL_ERROR "Hikvision SDK include directory not found: ${HIK_INCLUDE_DIR}")
endif()

# ===================== ROS 2 & 第三方库 =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tutorial_interfaces REQUIRED)  
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV 4.13 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ===================== 全局包含路径 =====================
# 显式添加全局包含，确保编译器在任何地方都能搜到海康头文件
include_directories(
    include
    ${HIK_INCLUDE_DIR}
    ${TensorRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

# 显式添加库搜索路径
link_directories(${HIK_LIB_DIR})

# ===================== 源文件 =====================
set(SOURCES
    src/inference_node.cpp
    src/inference.cpp
    src/BYTETracker.cpp
    src/KalmanFilter.cpp
    src/lapjv.cpp
    src/Object.cpp
    src/Rect.cpp
    src/STrack.cpp
    src/HikCamera.cpp
)

# ===================== 可执行文件 =====================
add_executable(inference_node ${SOURCES})

# ===================== 目标包含目录 (更加明确) =====================
target_include_directories(inference_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${HIK_INCLUDE_DIR}
    
)

# ===================== 链接库 =====================
target_link_libraries(inference_node
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${NVINFER_LIB}
    ${NVONNXPARSER_LIB}
    ${NVINFER_PLUGIN_LIB}
    ${CUDA_LIBRARIES}
    ${rclcpp_LIBRARIES}
    cudart
    stdc++fs
    MvCameraControl  # 简写，CMake会根据link_directories找到它
    yaml-cpp#::yaml-cpp
)

# ===================== ROS 2 依赖 =====================
ament_target_dependencies(inference_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tutorial_interfaces
)

# ===================== 安装 =====================
install(TARGETS inference_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# ===================== 调试信息 =====================
message(STATUS "Hikvision SDK include: ${HIK_INCLUDE_DIR}")
message(STATUS "Hikvision SDK lib: ${HIK_LIB_DIR}")