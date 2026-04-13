cmake_minimum_required(VERSION 3.5)
project(location_project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(tutorial_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(location_node
    src/location.cpp
    src/location_main.cpp
    #src/test.cpp
)

# 添加包含目录
target_include_directories(location_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /home/zqz/ros2_ws/config/include
)

# 手动添加所有依赖（避免使用 ament_target_dependencies）
target_include_directories(location_node
    PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${tutorial_interfaces_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
)

# 链接库 - 统一使用关键字语法
target_link_libraries(location_node
    PRIVATE
    Eigen3::Eigen
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
    ${rclcpp_LIBRARIES}
    ${tutorial_interfaces_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
)

# 安装目标
install(TARGETS location_node
   RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()