cmake_minimum_required(VERSION 3.8)
project(merge_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 添加可执行文件
add_executable(merge_cloud_node Combine.cpp)

# 链接依赖库
ament_target_dependencies(merge_cloud_node 
    rclcpp 
    pcl_conversions 
    pcl_ros 
    sensor_msgs 
    message_filters 
    Eigen3
    livox_ros_driver2)

# 安装可执行文件
install(TARGETS
  merge_cloud_node
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装其他资源
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()