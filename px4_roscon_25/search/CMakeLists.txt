cmake_minimum_required(VERSION 3.5)
project(search)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(search
    src/SearchMode.cpp
    src/SearchPlanner.cpp
    src/PlanParser.cpp
)

ament_target_dependencies(search
  ament_index_cpp
  rclcpp
  Eigen3
  px4_ros2_cpp
)

target_include_directories(search PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(search PUBLIC c_std_99 cxx_std_20)

add_executable(search_bridge
    src/SearchBridge.cpp
    src/SearchPlanner.cpp
    src/PlanParser.cpp
)

ament_target_dependencies(search_bridge
  ament_index_cpp
  rclcpp
  px4_msgs
)

target_include_directories(search_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(search_bridge
  PkgConfig::MOSQUITTO
)

target_compile_features(search_bridge PUBLIC cxx_std_20)

install(TARGETS search search_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plans
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # The workshop packages do not carry per-file copyright headers and use
    # the existing PX4/ROS2 tab-based style, so skip these optional style checks.
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
