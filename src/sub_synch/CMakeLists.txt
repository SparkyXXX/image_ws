cmake_minimum_required(VERSION 3.5)
project(sub_synch)

# 寻找依赖的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

# 添加包含文件路径
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(time_synch src/time_synch.cpp)

# 链接依赖的库
ament_target_dependencies(time_synch
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  OpenCV
)

# 添加 OpenCV 库的链接
target_link_libraries(time_synch
  ${OpenCV_LIBRARIES}
  ${CURL_LIBRARIES}
)

# 安装可执行文件
install(TARGETS
  time_synch
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 声明 ROS 2 软件包
ament_package()
