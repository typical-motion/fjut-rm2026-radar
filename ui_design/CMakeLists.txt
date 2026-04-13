cmake_minimum_required(VERSION 3.5)
project(ui_design_pkg)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 包含目录
include_directories(
  include
  src
  ${OpenCV_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(map_visualizer_node 
  #src/MapVisualizerNode.cpp
  src/MapVisualizer.cpp
  src/test.cpp
)

# 链接库
target_link_libraries(map_visualizer_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tutorial_interfaces_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 包含路径
target_include_directories(map_visualizer_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tutorial_interfaces_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 安装可执行文件
install(TARGETS map_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()