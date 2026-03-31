cmake_minimum_required(VERSION 3.8)
project(hik_camera_driver)

# ---------------------- 编译选项 ----------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# ---------------------- 查找依赖 ----------------------
find_package(OpenCV REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4 NO_DEFAULT_PATH)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# ---------------------- 海康 SDK ----------------------
set(HIK_SDK_ROOT "/opt/MVS" CACHE PATH "Hikvision SDK root directory")
set(HIK_INCLUDE_DIR "${HIK_SDK_ROOT}/include")
set(HIK_LIB_DIR "${HIK_SDK_ROOT}/lib/64")
set(CONFIG_DIR "/home/zqz/ros2_ws/config")

# 检查 SDK 文件
if(NOT EXISTS "${HIK_INCLUDE_DIR}/MvCameraControl.h")
    message(FATAL_ERROR "❌ Hikvision SDK headers not found at: ${HIK_INCLUDE_DIR}")
endif()

# ---------------------- 代码路径 ----------------------
set(HIK_DRIVE_SOURCE_DIR "/home/zqz/ros2_ws/hik_drive" CACHE PATH "Path to hik_drive source")

if(NOT EXISTS "${HIK_DRIVE_SOURCE_DIR}/src/HikCamera.cpp")
    message(FATAL_ERROR "❌ HikCamera.cpp not found at: ${HIK_DRIVE_SOURCE_DIR}/src/HikCamera.cpp")
endif()

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${HIK_INCLUDE_DIR}
    ${CONFIG_DIR}/include
    ${HIK_DRIVE_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    
)

link_directories(${HIK_LIB_DIR})

# ---------------------- 源文件 ----------------------
set(SRC_FILES
    src/test.cpp
    #${HIK_DRIVE_SOURCE_DIR}/src/HikCamera.cpp
)

# ---------------------- 可执行文件 ----------------------
add_executable(${PROJECT_NAME} ${SRC_FILES})

# ❗ 使用“普通语法”，不带 PRIVATE / PUBLIC
target_link_libraries(${PROJECT_NAME}
    MvCameraControl
    ${OpenCV_LIBS}
    Eigen3::Eigen
    Threads::Threads
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    cv_bridge
)

# ---------------------- 安装配置 ----------------------
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(UNIX)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH "${HIK_LIB_DIR}"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
endif()

ament_package()
