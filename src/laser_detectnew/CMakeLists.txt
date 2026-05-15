cmake_minimum_required(VERSION 3.5)

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

project(laser_detect)

set(CMAKE_BUILD_TYPE Debug)

# ===================== C++ =====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wno-deprecated-declarations)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ===================== CUDA =====================
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.8")

find_package(CUDAToolkit REQUIRED)

# ===================== TensorRT =====================
set(TensorRT_INCLUDE_DIR "/usr/include/x86_64-linux-gnu")
set(TensorRT_LIB_DIR "/usr/lib/x86_64-linux-gnu")

find_library(NVINFER_LIB nvinfer PATHS ${TensorRT_LIB_DIR} REQUIRED)
find_library(NVONNXPARSER_LIB nvonnxparser PATHS ${TensorRT_LIB_DIR} REQUIRED)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin PATHS ${TensorRT_LIB_DIR})

if(NOT NVINFER_LIB)
    message(FATAL_ERROR "TensorRT nvinfer not found")
endif()

# ===================== Daheng SDK =====================
set(DAHENG_SDK_ROOT "/home/zqz/Downloads/Galaxy_Linux-x86_Gige-U3_32bits-64bits_2.4.2507.9231/Galaxy_camera")
set(DAHENG_INCLUDE_DIR "${DAHENG_SDK_ROOT}/inc")
set(DAHENG_LIB_DIR "${DAHENG_SDK_ROOT}/lib/x86_64")

if(NOT EXISTS "${DAHENG_INCLUDE_DIR}")
    message(FATAL_ERROR "Daheng SDK include not found")
endif()

# ===================== ROS2 =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)

# ===================== OpenCV =====================
find_package(OpenCV REQUIRED)

# ===================== Eigen =====================
find_package(Eigen3 REQUIRED)

# ===================== 🔥 Serial (CH340 / LibSerial) =====================
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# ===================== include =====================
include_directories(
    include
    ${DAHENG_INCLUDE_DIR}
    ${TensorRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIBSERIAL_INCLUDE_DIRS}
)

# ===================== lib =====================
link_directories(${DAHENG_LIB_DIR})

# ===================== sources =====================
set(SOURCES
    src/main.cpp
    src/laser_detect.cpp
    src/dnn_detect.cpp
    src/GalaxyCamera.cpp
    src/kalman_filter.cpp
)

# ===================== executable =====================
add_executable(laser_inference_node ${SOURCES})

# ⭐ 新增 serial_node（不影响原工程）
add_executable(serial_node src/serial_node.cpp)

target_include_directories(laser_inference_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${DAHENG_INCLUDE_DIR}
)

target_include_directories(serial_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ===================== link libs =====================
target_link_libraries(laser_inference_node
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${NVINFER_LIB}
    ${NVONNXPARSER_LIB}
    ${NVINFER_PLUGIN_LIB}
    ${CUDA_LIBRARIES}
    cudart
    gxiapi
    ${LIBSERIAL_LIBRARIES}
)

# ⭐ serial_node 链接串口库
target_link_libraries(serial_node
    ${LIBSERIAL_LIBRARIES}
)

# ===================== ROS2 =====================
ament_target_dependencies(laser_inference_node
    rclcpp
    geometry_msgs
    sensor_msgs
    tutorial_interfaces
)

ament_target_dependencies(serial_node
    rclcpp
    tutorial_interfaces
)

# ===================== install =====================
install(TARGETS laser_inference_node serial_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# ===================== debug =====================
message(STATUS "Daheng SDK include: ${DAHENG_INCLUDE_DIR}")
message(STATUS "Daheng SDK lib: ${DAHENG_LIB_DIR}")
message(STATUS "OpenCV: ${OpenCV_VERSION}")
message(STATUS "LibSerial found: ${LIBSERIAL_LIBRARIES}")