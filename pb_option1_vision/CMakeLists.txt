cmake_minimum_required(VERSION 3.8)
project(pb_option1_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # OpenCV
find_package(yaml-cpp REQUIRED)  # YAML
find_package(ament_index_cpp REQUIRED)

# 包含目录
include_directories(include ${OpenCV_INCLUDE_DIRS})

# 添加可执行文件
add_executable(object_detector src/object_detector_node.cpp)
ament_target_dependencies(object_detector rclcpp sensor_msgs cv_bridge vision_msgs OpenCV yaml-cpp)

add_executable(follow_behavior src/follow_behavior_node.cpp)
ament_target_dependencies(follow_behavior rclcpp geometry_msgs vision_msgs yaml-cpp)

add_executable(command_interpreter src/command_interpreter_node.cpp)
ament_target_dependencies(command_interpreter rclcpp geometry_msgs vision_msgs yaml-cpp)


target_link_libraries(object_detector yaml-cpp ament_index_cpp::ament_index_cpp ${OpenCV_LIBRARIES})
target_link_libraries(follow_behavior yaml-cpp ament_index_cpp::ament_index_cpp)
target_link_libraries(command_interpreter yaml-cpp ament_index_cpp::ament_index_cpp)

# 安装
install(TARGETS
  object_detector
  follow_behavior
  command_interpreter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()