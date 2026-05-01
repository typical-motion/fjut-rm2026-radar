cmake_minimum_required(VERSION 3.5)
project(serial_node)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED) # 保留 OpenCV 
# find_package(serial_driver REQUIRED)
find_package(Threads REQUIRED)
#find_package(serial REQUIRED)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 将包内的源文件一起编译（不依赖外部 serialConfig.cmake）
add_executable(serial_node src/serial_node.cpp src/serial3.cpp)

target_include_directories(serial_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  include
)

ament_target_dependencies(serial_node
  rclcpp
  tutorial_interfaces
  std_msgs
  geometry_msgs
  # serial_driver
)

target_link_libraries(serial_node
  ${OpenCV_LIBS}
  serial
  #serial_driver::serial_driver
)

install(TARGETS serial_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()