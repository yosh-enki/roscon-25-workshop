cmake_minimum_required(VERSION 3.8)
project(full_self_driving)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Executables
add_executable(fsd_launch_probe
  src/runtime/launch_probe.cpp
)
target_include_directories(fsd_launch_probe PRIVATE
  include
)
ament_target_dependencies(fsd_launch_probe
  rclcpp
  rosgraph_msgs
  sensor_msgs
  px4_msgs
)

# Install rules
install(TARGETS
  fsd_launch_probe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  simulation
  config
  resource
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(launch_manifest_test
    test/launch/launch_manifest_test.cpp
  )
  target_include_directories(launch_manifest_test PRIVATE
    include
  )
  ament_target_dependencies(launch_manifest_test
    ament_index_cpp
  )

  ament_add_pytest_test(forbidden_dependency_scan
    test/security/forbidden_dependency_scan.py
  )

  ament_add_pytest_test(launch_boundary_test
    test/launch/launch_boundary_test.py
  )
endif()

ament_package()
