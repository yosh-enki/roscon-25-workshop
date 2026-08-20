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
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

# Auto-patch upstream px4_ros2_cpp header typo if installed version has it
set(_potential_inc_dirs ${px4_ros2_cpp_INCLUDE_DIRS} "/home/ubuntu/px4_ros_ws/install/px4_ros2_cpp/include")
foreach(_inc_dir ${_potential_inc_dirs})
  set(_home_pos_h "${_inc_dir}/px4_ros2/vehicle_state/home_position.hpp")
  if(EXISTS "${_home_pos_h}")
    file(READ "${_home_pos_h}" _hp_content)
    if(_hp_content MATCHES "\\+ \\+px4_ros2::getMessageNameVersion")
      string(REPLACE "+ +px4_ros2::getMessageNameVersion" "+ px4_ros2::getMessageNameVersion" _hp_fixed "${_hp_content}")
      file(WRITE "${_home_pos_h}" "${_hp_fixed}")
      message(STATUS "[FSD] Auto-patched upstream typo in: ${_home_pos_h}")
    endif()
  endif()
endforeach()

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
  "msg/RecoveryStatus.msg"
  "srv/FullSelfDrivingCommand.srv"
  "srv/EmergencyStop.srv"
  "srv/ClearPadRegistry.srv"
  "srv/BackupPadRegistry.srv"
  "srv/UploadPlanArtifact.srv"
  "srv/SelectPlanArtifact.srv"
  "srv/CreateOrSelectWorkingPlan.srv"
  "srv/ResetWorkingPlan.srv"
  "srv/SelectMapScenario.srv"
  "srv/SelectTargetIdentity.srv"
  "srv/PreparePayload.srv"
  "srv/ValidateMissionContext.srv"
  "srv/CommitMissionContext.srv"
  "srv/ResolveRecovery.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Domain Core Library
add_library(fsd_domain_core
  src/domain/target_identity.cpp
  src/domain/live_target_lock.cpp
  src/domain/engineering_config.cpp
  src/domain/mission_context.cpp
  src/domain/route.cpp
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

# Persistence Core Library
add_library(fsd_persistence_core
  src/persistence/persistence_manager.cpp
)
target_include_directories(fsd_persistence_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fsd_persistence_core
  fsd_domain_core
  OpenSSL::Crypto
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_persistence_core
  rclcpp
  geometry_msgs
  builtin_interfaces
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

# Payload Core Library
add_library(fsd_payload_core
  src/payload/simulation_payload_adapter.cpp
  src/payload/hardware_payload_adapter.cpp
  src/payload/px4_gripper_payload_adapter.cpp
  src/payload/payload_controller.cpp
)
target_include_directories(fsd_payload_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fsd_payload_core
  fsd_domain_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_payload_core
  rclcpp
  px4_msgs
  builtin_interfaces
)

# Gateway Core Library
add_library(fsd_gateway_core
  src/gateway/fsd_gateway.cpp
)
target_include_directories(fsd_gateway_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fsd_gateway_core
  fsd_domain_core
  fsd_persistence_core
  fsd_registry_core
  fsd_payload_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_gateway_core
  rclcpp
  geometry_msgs
  builtin_interfaces
)

# Runtime Core Library
add_library(fsd_runtime_core
  src/runtime/lifecycle_supervisor.cpp
)
target_include_directories(fsd_runtime_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fsd_runtime_core
  fsd_domain_core
  fsd_persistence_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_runtime_core
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  builtin_interfaces
)

# Adapters Core Library
add_library(fsd_adapters_core
  src/adapters/px4_api_capabilities.cpp
  src/adapters/px4_state_cache.cpp
  src/adapters/hardware_fmu_adapter.cpp
  src/adapters/hardware_camera_adapter.cpp
)
target_include_directories(fsd_adapters_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fsd_adapters_core
  yaml-cpp
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_adapters_core
  rclcpp
  px4_ros2_cpp
  px4_msgs
  geometry_msgs
  builtin_interfaces
)

# Launch Core Library (Hardware Manifest Validator)
add_library(fsd_launch_core
  src/launch/hardware_manifest_validator.cpp
)
target_include_directories(fsd_launch_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fsd_launch_core
  yaml-cpp
  OpenSSL::Crypto
)


# Flight Core Library
add_library(fsd_flight_core
  src/flight/full_self_driving_mode.cpp
  src/flight/full_self_driving_mode_executor.cpp
  src/flight/strategies/takeoff_strategy.cpp
  src/flight/strategies/transit_in_strategy.cpp
  src/flight/strategies/search_strategy.cpp
  src/flight/strategies/direct_strategy.cpp
  src/flight/strategies/precision_land_strategy.cpp
  src/flight/strategies/payload_operation_strategy.cpp
  src/flight/strategies/transit_out_strategy.cpp
  src/flight/strategies/return_strategy.cpp
  src/domain/mission_coordinator.cpp
)
target_include_directories(fsd_flight_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fsd_flight_core
  fsd_domain_core
  fsd_persistence_core
  fsd_registry_core
  fsd_adapters_core
  fsd_payload_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_flight_core
  rclcpp
  px4_ros2_cpp
  px4_msgs
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
  tf2
  tf2_ros
  tf2_geometry_msgs
  px4_msgs
)

add_executable(fsd_evidence
  src/runtime/evidence_node.cpp
)
target_include_directories(fsd_evidence PRIVATE
  include
  src
)
target_link_libraries(fsd_evidence
  fsd_domain_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_evidence
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
)

add_executable(fsd_gateway
  src/gateway/fsd_gateway_node.cpp
)
target_include_directories(fsd_gateway PRIVATE
  include
  src
)
target_link_libraries(fsd_gateway
  fsd_gateway_core
  fsd_domain_core
  fsd_persistence_core
  fsd_registry_core
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_gateway
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

add_executable(fsd_flight_runtime
  src/runtime/flight_runtime_node.cpp
)
target_include_directories(fsd_flight_runtime PRIVATE
  include
  src
)
target_link_libraries(fsd_flight_runtime
  fsd_flight_core
  fsd_adapters_core
  fsd_runtime_core
  fsd_persistence_core
  fsd_registry_core
  fsd_domain_core
  fsd_payload_core
  yaml-cpp
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_flight_runtime
  rclcpp
  px4_ros2_cpp
  px4_msgs
  geometry_msgs
  builtin_interfaces
  ament_index_cpp
)

add_executable(fsd_hardware_manifest_validator
  src/launch/fsd_hardware_manifest_validator_main.cpp
)
target_include_directories(fsd_hardware_manifest_validator PRIVATE
  include
  src
)
target_link_libraries(fsd_hardware_manifest_validator
  fsd_launch_core
)

# Install rules
install(TARGETS
  fsd_domain_core
  fsd_persistence_core
  fsd_perception_core
  fsd_registry_core
  fsd_payload_core
  fsd_gateway_core
  fsd_runtime_core
  fsd_adapters_core
  fsd_launch_core
  fsd_flight_core
  fsd_launch_probe
  fsd_perception
  fsd_pad_registry
  fsd_evidence
  fsd_gateway
  fsd_flight_runtime
  fsd_replay_fixture_publisher
  fsd_target_selection_provider
  fsd_hardware_manifest_validator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  simulation
  config
  resource
  scripts
  test
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_sros2_keystore.py
  scripts/manage_sros2_keystore.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(hardware_manifest_validator_test
    test/launch/hardware_manifest_validator_test.cpp
  )
  target_include_directories(hardware_manifest_validator_test PRIVATE
    include
    src
  )
  target_link_libraries(hardware_manifest_validator_test
    fsd_launch_core
    ament_index_cpp::ament_index_cpp
  )
  ament_target_dependencies(hardware_manifest_validator_test
    ament_index_cpp
  )

  ament_add_gtest(launch_manifest_test
    test/launch/launch_manifest_test.cpp
  )
  target_include_directories(launch_manifest_test PRIVATE
    include
    src
  )
  target_link_libraries(launch_manifest_test
    fsd_launch_core
    ament_index_cpp::ament_index_cpp
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

  ament_add_gtest(fsd_property_10_gateway_boundary
    test/property/property_10_gateway_boundary.cpp
  )
  target_include_directories(fsd_property_10_gateway_boundary PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_10_gateway_boundary
    fsd_gateway_core
    fsd_domain_core
    fsd_persistence_core
    fsd_registry_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_10_gateway_boundary
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(fsd_property_16_durable_boundary
    test/property/property_16_durable_boundary.cpp
  )
  target_include_directories(fsd_property_16_durable_boundary PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_16_durable_boundary
    fsd_persistence_core
    fsd_domain_core
    OpenSSL::Crypto
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_16_durable_boundary
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(fsd_property_17_recovery_safety
    test/property/property_17_recovery_safety.cpp
  )
  target_include_directories(fsd_property_17_recovery_safety PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_17_recovery_safety
    fsd_persistence_core
    fsd_domain_core
    OpenSSL::Crypto
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_17_recovery_safety
    rclcpp
    geometry_msgs
  )

  ament_add_gtest(fsd_property_18_snapshot_commit
    test/property/property_18_snapshot_commit.cpp
  )
  target_include_directories(fsd_property_18_snapshot_commit PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_18_snapshot_commit
    fsd_runtime_core
    fsd_persistence_core
    fsd_domain_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_18_snapshot_commit
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
  )

  ament_add_gtest(px4_api_probe_test
    test/px4_api_probe/px4_api_probe_test.cpp
  )
  target_include_directories(px4_api_probe_test PRIVATE
    include
    src
  )
  target_link_libraries(px4_api_probe_test
    fsd_adapters_core
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(px4_api_probe_test
    rclcpp
    px4_ros2_cpp
    px4_msgs
    ament_index_cpp
  )

  ament_add_gtest(registered_mode_authority_smoke
    test/integration/registered_mode_authority_smoke.cpp
  )
  target_include_directories(registered_mode_authority_smoke PRIVATE
    include
    src
  )
  target_link_libraries(registered_mode_authority_smoke
    fsd_flight_core
    fsd_adapters_core
    fsd_domain_core
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(registered_mode_authority_smoke
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
  )

  ament_add_gtest(fsd_property_12_coordinator_transitions
    test/property/property_12_coordinator_transitions.cpp
  )
  target_include_directories(fsd_property_12_coordinator_transitions PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_12_coordinator_transitions
    fsd_flight_core
    fsd_adapters_core
    fsd_domain_core
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_12_coordinator_transitions
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
  )

  ament_add_gtest(fsd_property_20_authority
    test/property/property_20_authority.cpp
  )
  target_include_directories(fsd_property_20_authority PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_20_authority
    fsd_flight_core
    fsd_adapters_core
    fsd_domain_core
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_20_authority
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
  )

  ament_add_gtest(fsd_property_22_lifecycle_registration
    test/property/property_22_lifecycle_registration.cpp
  )
  target_include_directories(fsd_property_22_lifecycle_registration PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_22_lifecycle_registration
    fsd_runtime_core
    fsd_persistence_core
    fsd_domain_core
    fsd_adapters_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_22_lifecycle_registration
    rclcpp
    rclcpp_lifecycle
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
  )

  ament_add_gtest(transit_in_parity_test
    test/flight/transit_in_parity_test.cpp
  )
  target_include_directories(transit_in_parity_test PRIVATE
    include
    src
  )
  target_link_libraries(transit_in_parity_test
    fsd_flight_core
    fsd_adapters_core
    fsd_domain_core
    fsd_persistence_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(transit_in_parity_test
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
    ament_index_cpp
  )

  ament_add_gtest(search_parity_test
    test/flight/search_parity_test.cpp
  )
  target_include_directories(search_parity_test PRIVATE
    include
    src
  )
  target_link_libraries(search_parity_test
    fsd_flight_core
    fsd_adapters_core
    fsd_domain_core
    fsd_persistence_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(search_parity_test
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
    ament_index_cpp
  )

  ament_add_gtest(fsd_property_8_direct_lock_separation
    test/property/property_8_direct_lock_separation.cpp
  )
  target_include_directories(fsd_property_8_direct_lock_separation PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_8_direct_lock_separation
    fsd_flight_core
    fsd_registry_core
    fsd_adapters_core
    fsd_domain_core
    fsd_persistence_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_8_direct_lock_separation
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
    ament_index_cpp
  )

  ament_add_gtest(acquisition_branch_test
    test/flight/acquisition_branch_test.cpp
  )
  target_include_directories(acquisition_branch_test PRIVATE
    include
    src
  )
  target_link_libraries(acquisition_branch_test
    fsd_flight_core
    fsd_registry_core
    fsd_adapters_core
    fsd_domain_core
    fsd_persistence_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(acquisition_branch_test
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
    ament_index_cpp
  )

  ament_add_gtest(precision_land_parity_test
    test/flight/precision_land_parity_test.cpp
  )
  target_include_directories(precision_land_parity_test PRIVATE
    include
    src
  )
  target_link_libraries(precision_land_parity_test
    fsd_flight_core
    fsd_registry_core
    fsd_adapters_core
    fsd_domain_core
    fsd_persistence_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(precision_land_parity_test
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
    ament_index_cpp
  )

  ament_add_gtest(payload_controller_test
    test/payload/payload_controller_test.cpp
  )
  target_include_directories(payload_controller_test PRIVATE
    include
    src
  )
  target_link_libraries(payload_controller_test
    fsd_payload_core
    fsd_domain_core
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(payload_controller_test
    rclcpp
    builtin_interfaces
  )

  ament_add_gtest(px4_gripper_payload_adapter_test
    test/payload/px4_gripper_payload_adapter_test.cpp
  )
  target_include_directories(px4_gripper_payload_adapter_test PRIVATE
    include
    src
  )
  target_link_libraries(px4_gripper_payload_adapter_test
    fsd_payload_core
    fsd_domain_core
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(px4_gripper_payload_adapter_test
    rclcpp
    px4_msgs
    builtin_interfaces
  )

  ament_add_gtest(payload_operation_test
    test/flight/payload_operation_test.cpp
  )
  target_include_directories(payload_operation_test PRIVATE
    include
    src
  )
  target_link_libraries(payload_operation_test
    fsd_flight_core
    fsd_payload_core
    fsd_persistence_core
    fsd_domain_core
    OpenSSL::Crypto
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(payload_operation_test
    rclcpp
    builtin_interfaces
  )

  ament_add_gtest(transit_out_parity_test
    test/flight/transit_out_parity_test.cpp
  )
  target_include_directories(transit_out_parity_test PRIVATE
    include
    src
  )
  target_link_libraries(transit_out_parity_test
    fsd_flight_core
    fsd_domain_core
    fsd_adapters_core
    fsd_persistence_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(transit_out_parity_test
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
    ament_index_cpp
  )

  ament_add_gtest(return_strategy_test
    test/flight/return_strategy_test.cpp
  )
  target_include_directories(return_strategy_test PRIVATE
    include
    src
  )
  target_link_libraries(return_strategy_test
    fsd_flight_core
    fsd_domain_core
    fsd_adapters_core
    fsd_persistence_core
    OpenSSL::Crypto
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(return_strategy_test
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
  )

  ament_add_gtest(fsd_property_14_payload_safety
    test/property/property_14_payload_safety.cpp
  )
  target_include_directories(fsd_property_14_payload_safety PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_14_payload_safety
    fsd_payload_core
    fsd_gateway_core
    fsd_flight_core
    fsd_domain_core
    fsd_adapters_core
    fsd_persistence_core
    OpenSSL::Crypto
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_14_payload_safety
    rclcpp
    builtin_interfaces
    px4_msgs
    geometry_msgs
  )

  ament_add_gtest(fsd_property_15_return_strategy_explicitness
    test/property/property_15_return_strategy_explicitness.cpp
  )
  target_include_directories(fsd_property_15_return_strategy_explicitness PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_15_return_strategy_explicitness
    fsd_flight_core
    fsd_domain_core
    fsd_adapters_core
    fsd_persistence_core
    OpenSSL::Crypto
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_15_return_strategy_explicitness
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
  )

  ament_add_gtest(fsd_property_11_mission_sequence
    test/property/property_11_mission_sequence.cpp
  )
  target_include_directories(fsd_property_11_mission_sequence PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_11_mission_sequence
    fsd_payload_core
    fsd_flight_core
    fsd_domain_core
    fsd_adapters_core
    fsd_persistence_core
    OpenSSL::Crypto
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_11_mission_sequence
    rclcpp
    builtin_interfaces
    px4_msgs
    geometry_msgs
  )

  ament_add_gtest(fsd_property_26_security_rejection
    test/property/property_26_security_rejection.cpp
  )
  target_include_directories(fsd_property_26_security_rejection PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_26_security_rejection
    fsd_payload_core
    fsd_gateway_core
    fsd_flight_core
    fsd_domain_core
    fsd_adapters_core
    fsd_persistence_core
    OpenSSL::Crypto
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_26_security_rejection
    rclcpp
    builtin_interfaces
    px4_msgs
    geometry_msgs
  )

  ament_add_gtest(fsd_property_25_observability_noninterference
    test/property/property_25_observability_noninterference.cpp
  )
  target_include_directories(fsd_property_25_observability_noninterference PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_25_observability_noninterference
    fsd_flight_core
    fsd_payload_core
    fsd_adapters_core
    fsd_persistence_core
    fsd_domain_core
    fsd_registry_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_25_observability_noninterference
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
    builtin_interfaces
  )

  ament_add_gtest(fsd_property_24_adapter_invariance
    test/property/property_24_adapter_invariance.cpp
  )
  target_include_directories(fsd_property_24_adapter_invariance PRIVATE
    include
    src
  )
  target_link_libraries(fsd_property_24_adapter_invariance
    fsd_flight_core
    fsd_payload_core
    fsd_adapters_core
    fsd_persistence_core
    fsd_domain_core
    fsd_registry_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_property_24_adapter_invariance
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
    builtin_interfaces
  )

  ament_add_gtest(fsd_gateway_security_test
    test/security/gateway_security_test.cpp
  )
  target_include_directories(fsd_gateway_security_test PRIVATE
    include
    src
  )
  target_link_libraries(fsd_gateway_security_test
    fsd_gateway_core
    fsd_flight_core
    fsd_payload_core
    fsd_domain_core
    fsd_persistence_core
    fsd_registry_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_gateway_security_test
    rclcpp
    geometry_msgs
    builtin_interfaces
  )

  ament_add_gtest(fsd_resource_failure_test
    test/integration/resource_failure_test.cpp
  )
  target_include_directories(fsd_resource_failure_test PRIVATE
    include
    src
  )
  target_link_libraries(fsd_resource_failure_test
    fsd_flight_core
    fsd_payload_core
    fsd_adapters_core
    fsd_domain_core
    fsd_persistence_core
    fsd_registry_core
    OpenSSL::Crypto
    yaml-cpp
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(fsd_resource_failure_test
    rclcpp
    px4_ros2_cpp
    px4_msgs
    geometry_msgs
    builtin_interfaces
  )

  ament_add_pytest_test(fsd_property_21_ros_interface_boundary
    test/property/property_21_ros_interface_boundary.py
  )

  ament_add_pytest_test(forbidden_dependency_scan
    test/security/forbidden_dependency_scan.py
  )

  ament_add_pytest_test(production_boundary_scan
    test/security/production_boundary_scan.py
  )

  ament_add_pytest_test(security_policy_enforcement_test
    test/security/security_policy_enforcement_test.py
  )

  ament_add_pytest_test(launch_boundary_test
    test/launch/launch_boundary_test.py
  )

  ament_add_pytest_test(full_sortie_acceptance
    test/acceptance/full_sortie_acceptance.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(
  fsd_domain_core
  fsd_persistence_core
  fsd_perception_core
  fsd_registry_core
  fsd_gateway_core
  fsd_payload_core
  fsd_runtime_core
  fsd_adapters_core
  fsd_flight_core
)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_lifecycle
  px4_ros2_cpp
  px4_msgs
  sensor_msgs
  geometry_msgs
)

ament_package()
