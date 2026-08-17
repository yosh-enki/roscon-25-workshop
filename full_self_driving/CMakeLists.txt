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
  "msg/LiveTargetLock.msg"
  "msg/PadRecord.msg"
  "msg/PadRegistrySnapshot.msg"
  "msg/PadRegistryStatus.msg"
  "msg/FullSelfDrivingState.msg"
  "msg/VehicleTelemetry.msg"
  "msg/LandingTargetSetpoint.msg"
  "msg/FlightSafetyStatus.msg"
  "msg/ErrorReport.msg"
  "msg/PlanArtifactReference.msg"
  "msg/SearchCheckpoint.msg"
  "msg/WorkingPlanStatus.msg"
  "msg/ReadinessReport.msg"
  "msg/PayloadStatus.msg"
  "msg/MissionContext.msg"
  "srv/FullSelfDrivingCommand.srv"
  "srv/EmergencyStop.srv"
  "srv/ClearPadRegistry.srv"
  "srv/BackupPadRegistry.srv"
  "srv/UploadPlanArtifact.srv"
  "srv/SelectPlanArtifact.srv"
  "srv/CreateOrSelectWorkingPlan.srv"
  "srv/ResetWorkingPlan.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Domain Core Library
add_library(fsd_domain_core
  src/domain/target_identity.cpp
  src/domain/live_target_lock.cpp
  src/domain/engineering_config.cpp
  src/domain/mission_context.cpp
  src/domain/plan_parser.cpp
  src/domain/plan_printer.cpp
  src/domain/working_plan.cpp
  src/runtime/plan_manager.cpp
  src/runtime/working_plan_store.cpp
)
target_include_directories(fsd_domain_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fsd_domain_core
  rclcpp
  geometry_msgs
  builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fsd_domain_core
  OpenSSL::Crypto
  yaml-cpp
  "${cpp_typesupport_target}"
)

# Perception Core Library
add_library(fsd_perception_core
  src/perception/aruco_detector.cpp
  src/perception/target_coordinator.cpp
)
target_include_directories(fsd_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(fsd_perception_core
  fsd_domain_core
  ${OpenCV_LIBS}
  OpenSSL::Crypto
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_perception_core
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  builtin_interfaces
)

# Registry Core Library
add_library(fsd_registry_core
  src/registry/pad_registry.cpp
)
target_include_directories(fsd_registry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fsd_registry_core
  fsd_domain_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_registry_core
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  builtin_interfaces
)

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
  fsd_domain_core
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

add_executable(fsd_pad_registry
  src/registry/pad_registry_node.cpp
)
target_include_directories(fsd_pad_registry PRIVATE
  include
  src
)
target_link_libraries(fsd_pad_registry
  fsd_registry_core
  fsd_domain_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_pad_registry
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
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

add_executable(fsd_target_selection_provider
  test/fixtures/target_selection_provider.cpp
)
target_include_directories(fsd_target_selection_provider PRIVATE
  include
  src
  test
)
target_link_libraries(fsd_target_selection_provider
  fsd_domain_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_target_selection_provider
  rclcpp
)

# Install rules
install(TARGETS
  fsd_domain_core
  fsd_perception_core
  fsd_registry_core
  fsd_launch_probe
  fsd_perception
  fsd_pad_registry
  fsd_replay_fixture_publisher
  fsd_target_selection_provider
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
    fsd_domain_core
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

  ament_add_gtest(fsd_property_6_registry_isolation
    test/property/property_6_registry_isolation.cpp
  )
  target_include_directories(fsd_property_6_registry_isolation PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_6_registry_isolation
    fsd_registry_core
    fsd_domain_core
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_6_registry_isolation
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(fsd_property_7_all_id_live_lock
    test/property/property_7_all_id_live_lock.cpp
  )
  target_include_directories(fsd_property_7_all_id_live_lock PRIVATE
    include
    src
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(fsd_property_7_all_id_live_lock
    fsd_perception_core
    fsd_registry_core
    fsd_domain_core
    ${OpenCV_LIBS}
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_7_all_id_live_lock
    rclcpp
    geometry_msgs
    sensor_msgs
  )

  ament_add_gtest(fsd_property_1_authoritative_config
    test/property/property_1_authoritative_config.cpp
  )
  target_include_directories(fsd_property_1_authoritative_config PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_1_authoritative_config
    fsd_domain_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_1_authoritative_config
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(fsd_property_2_armed_immutability
    test/property/property_2_armed_immutability.cpp
  )
  target_include_directories(fsd_property_2_armed_immutability PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_2_armed_immutability
    fsd_domain_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_2_armed_immutability
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(fsd_property_3_scope_isolation
    test/property/property_3_scope_isolation.cpp
  )
  target_include_directories(fsd_property_3_scope_isolation PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_3_scope_isolation
    fsd_domain_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_3_scope_isolation
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(fsd_property_9_readiness
    test/property/property_9_readiness.cpp
  )
  target_include_directories(fsd_property_9_readiness PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_9_readiness
    fsd_domain_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_9_readiness
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(fsd_property_4_plan_immutability
    test/property/property_4_plan_immutability.cpp
  )
  target_include_directories(fsd_property_4_plan_immutability PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_4_plan_immutability
    fsd_domain_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_4_plan_immutability
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(fsd_property_5_working_plan
    test/property/property_5_working_plan.cpp
  )
  target_include_directories(fsd_property_5_working_plan PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_5_working_plan
    fsd_domain_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_5_working_plan
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(plan_round_trip_test
    test/plan/plan_round_trip_test.cpp
  )
  target_include_directories(plan_round_trip_test PRIVATE
    include
    src
  )
  target_link_libraries(plan_round_trip_test
    fsd_domain_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(plan_round_trip_test
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(working_plan_parity_test
    test/plan/working_plan_parity_test.cpp
  )
  target_include_directories(working_plan_parity_test PRIVATE
    include
    src
  )
  target_link_libraries(working_plan_parity_test
    fsd_domain_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(working_plan_parity_test
    rclcpp
    geometry_msgs
    ament_index_cpp
  )

  ament_add_pytest_test(fsd_property_21_ros_interface_boundary
    test/property/property_21_ros_interface_boundary.py
  )

  ament_add_pytest_test(forbidden_dependency_scan
    test/security/forbidden_dependency_scan.py
  )

  ament_add_pytest_test(launch_boundary_test
    test/launch/launch_boundary_test.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(fsd_domain_core fsd_perception_core fsd_registry_core)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
)

ament_package()
