cmake_minimum_required(VERSION 3.8)
project(full_self_driving)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# ROS Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MessageHeader.msg"
  "msg/TargetIdentity.msg"
  "msg/AllIdObservation.msg"
  "msg/AllIdObservationBatch.msg"
  "msg/ComponentHealth.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Perception Core Library
add_library(fsd_perception_core
  src/perception/aruco_detector.cpp
)
target_include_directories(fsd_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(fsd_perception_core
  ${OpenCV_LIBS}
  OpenSSL::Crypto
)
ament_target_dependencies(fsd_perception_core
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fsd_perception_core "${cpp_typesupport_target}")

# Executables
add_executable(fsd_launch_probe
  src/runtime/launch_probe.cpp
)
target_include_directories(fsd_launch_probe PRIVATE
  include
  src
)
ament_target_dependencies(fsd_launch_probe
  rclcpp
  rosgraph_msgs
  sensor_msgs
  px4_msgs
)

add_executable(fsd_perception
  src/perception/perception_node.cpp
)
target_include_directories(fsd_perception PRIVATE
  include
  src
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(fsd_perception
  fsd_perception_core
  ${OpenCV_LIBS}
  OpenSSL::Crypto
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_perception
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

add_executable(fsd_replay_fixture_publisher
  src/runtime/replay_fixture_publisher.cpp
)
target_include_directories(fsd_replay_fixture_publisher PRIVATE
  include
  src
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(fsd_replay_fixture_publisher
  ${OpenCV_LIBS}
  yaml-cpp
)
ament_target_dependencies(fsd_replay_fixture_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  ament_index_cpp
)

# Install rules
install(TARGETS
  fsd_perception_core
  fsd_launch_probe
  fsd_perception
  fsd_replay_fixture_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  simulation
  config
  resource
  test
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
    src
  )
  ament_target_dependencies(launch_manifest_test
    ament_index_cpp
  )

  ament_add_gtest(aruco_replay_test
    test/perception/aruco_replay_test.cpp
  )
  target_include_directories(aruco_replay_test PRIVATE
    include
    src
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(aruco_replay_test
    fsd_perception_core
    ${OpenCV_LIBS}
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(aruco_replay_test
    rclcpp
    sensor_msgs
    geometry_msgs
    ament_index_cpp
  )

  ament_add_pytest_test(forbidden_dependency_scan
    test/security/forbidden_dependency_scan.py
  )

  ament_add_pytest_test(launch_boundary_test
    test/launch/launch_boundary_test.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(fsd_perception_core)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
)

ament_package()
