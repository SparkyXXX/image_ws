cmake_minimum_required(VERSION 3.8)
project(two_camera)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找依赖库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# 添加包含目录
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${GST_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
    /usr/include/glib-2.0
    /usr/include/gstreamer-1.0
    /usr/lib/aarch64-linux-gnu/glib-2.0/include
    /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include
)

# 添加可执行文件
add_executable(appsink1 src/appsink1.cpp)
add_executable(appsink2 src/appsink2.cpp)
add_executable(appsink3 src/appsink3.cpp)
add_executable(appsink4 src/appsink4.cpp)

# 链接库
target_link_libraries(appsink1
    ${OpenCV_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    -lgobject-2.0
    -lglib-2.0
)
target_link_libraries(appsink2
    ${OpenCV_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    -lgobject-2.0
    -lglib-2.0
)
target_link_libraries(appsink3
    ${OpenCV_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    -lgobject-2.0
    -lglib-2.0
)
target_link_libraries(appsink4
    ${OpenCV_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    -lgobject-2.0
    -lglib-2.0
)

# 安装目标
install(TARGETS appsink1
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS appsink2
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS appsink3
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS appsink4
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
