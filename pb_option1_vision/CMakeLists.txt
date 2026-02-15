cmake_minimum_required(VERSION 3.8)
project(pb_option1_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 安装Python模块
ament_python_install_package(${PROJECT_NAME})

# 安装Python可执行文件
install(PROGRAMS
  src/object_detector.py
  src/follow_behavior_node.py
  src/command_interpreter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装config文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 安装models目录
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
