cmake_minimum_required(VERSION 3.8)
project(pb_option1_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(object_detector src/object_detector_node.cpp)
ament_target_dependencies(object_detector rclcpp sensor_msgs cv_bridge vision_msgs ament_index_cpp visualization_msgs)
target_link_libraries(object_detector yaml-cpp ${OpenCV_LIBRARIES})

add_executable(command_interpreter src/command_interpreter_node.cpp)
ament_target_dependencies(command_interpreter rclcpp geometry_msgs vision_msgs ament_index_cpp)
target_link_libraries(command_interpreter yaml-cpp ${OpenCV_LIBRARIES})

add_executable(follow_behavior src/follow_behavior_node.cpp)
ament_target_dependencies(follow_behavior rclcpp geometry_msgs vision_msgs ament_index_cpp)
target_link_libraries(follow_behavior yaml-cpp ${OpenCV_LIBRARIES})

install(TARGETS
  object_detector
  command_interpreter
  follow_behavior
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()