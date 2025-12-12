cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create executable
add_executable(navigation_core
  src/navigation_core.cpp
  src/main.cpp
)

target_include_directories(navigation_core PUBLIC
  include
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(navigation_core
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

# Install
install(TARGETS navigation_core
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/spawn_robots.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
