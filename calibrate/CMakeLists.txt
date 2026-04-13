cmake_minimum_required(VERSION 3.16)
project(calibrate LANGUAGES CXX)

# 设置编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置调试符号
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# 查找必备依赖
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc calib3d)  # 添加calib3d模块
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

# 海康威视SDK配置
set(HIK_SDK_ROOT "/opt/MVS")
set(HIK_INCLUDE_DIR "${HIK_SDK_ROOT}/include")
set(HIK_LIB_DIR "${HIK_SDK_ROOT}/lib/64")

# 验证海康SDK
if(NOT EXISTS "${HIK_INCLUDE_DIR}/MvCameraControl.h")
    message(FATAL_ERROR "海康SDK头文件未找到，请确认路径: ${HIK_INCLUDE_DIR}")
endif()

# 项目特定路径
set(HIK_DRIVE_DIR "/home/zqz/ros2_ws/hik_drive")
set(CONFIG_DIR "/home/zqz/ros2_ws/config")

# 添加可执行文件
add_executable(${PROJECT_NAME}
    src/Calibration.cpp
    ${HIK_DRIVE_DIR}/src/HikCamera.cpp
)

# 使用现代CMake方式设置包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${HIK_INCLUDE_DIR}
    ${HIK_DRIVE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CONFIG_DIR}/include
)

# 设置链接库
target_link_directories(${PROJECT_NAME} PRIVATE
    ${HIK_LIB_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    MvCameraControl
    ${OpenCV_LIBS}
    Eigen3::Eigen
    Threads::Threads
)

# 设置RPATH（仅Unix系统）
if(UNIX)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH "${HIK_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
endif()

# 安装配置
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# 配置信息输出
message(STATUS "==============================")
message(STATUS "项目配置信息:")
message(STATUS "海康SDK路径: ${HIK_SDK_ROOT}")
message(STATUS "OpenCV版本: ${OpenCV_VERSION} (包含: ${OpenCV_LIBS})")
message(STATUS "Eigen3版本: ${EIGEN3_VERSION_STRING}")
message(STATUS "==============================")