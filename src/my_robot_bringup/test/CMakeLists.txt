# 测试配置
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  
  # 代码检查
  ament_lint_auto_find_test_dependencies()
  
  # 导航测试
  ament_add_gtest(test_navigation
    test/test_navigation.cpp
  )
  if(TARGET test_navigation)
    target_include_directories(test_navigation PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_navigation
      rclcpp
      geometry_msgs
      nav_msgs
      gtest
    )
  endif()
  
  # SLAM测试
  ament_add_gtest(test_slam
    test/test_slam_processor.cpp
  )
  if(TARGET test_slam)
    target_include_directories(test_slam PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_slam
      rclcpp
      sensor_msgs
      nav_msgs
      gtest
    )
  endif()
endif()