cmake_minimum_required(VERSION 3.8)
project(aruco_database)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoDetection.msg"
  "msg/ArucoDetectionArray.msg"
  "msg/ArucoMarker.msg"
  "msg/ArucoMarkerArray.msg"
  "msg/ArucoDatabaseStatus.msg"
  "srv/GetArucoPosition.srv"
  "srv/ListArucoMarkers.srv"
  "srv/ClearArucoDatabase.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(aruco_database_node
  src/ArucoDatabase.cpp
)

target_include_directories(aruco_database_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(aruco_database_node PUBLIC cxx_std_17)

target_link_libraries(aruco_database_node yaml-cpp)

ament_target_dependencies(
  aruco_database_node
  ament_index_cpp
  geometry_msgs
  px4_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(aruco_database_node "${cpp_typesupport_target}")

install(TARGETS aruco_database_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch database
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The workshop packages use the existing PX4/ROS 2 tab-based style and do
  # not carry per-file copyright headers.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
