cmake_minimum_required(VERSION 3.8)
project(pb_option1_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准为 17（用于 filesystem）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)   # 新增

# 包含目录
include_directories(include)

# 定义可执行文件
add_executable(object_detector_node
  src/object_detector_node.cpp
  src/object_detector.cpp
)
ament_target_dependencies(object_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  ament_index_cpp   # 新增
)

add_executable(follow_behavior_node
  src/follow_behavior_node.cpp
)
ament_target_dependencies(follow_behavior_node
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(command_interpreter_node
  src/command_interpreter_node.cpp
)
ament_target_dependencies(command_interpreter_node
  rclcpp
  std_msgs
  geometry_msgs
)

# 安装可执行文件
install(TARGETS
  object_detector_node
  follow_behavior_node
  command_interpreter_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装模型文件（确保 models 目录下的文件被安装到 share/pb_option1_vision/models/）
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()