cmake_minimum_required(VERSION 3.8)
project(cube_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bringup REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Ensure ament_lint_auto is found
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    rviz
    launch
    maps
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/apriltag_detection.py  
  DESTINATION lib/${PROJECT_NAME}   
)

ament_package()

