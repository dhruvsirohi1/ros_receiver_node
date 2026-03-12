cmake_minimum_required(VERSION 3.8)
project(aeye_ros2_driver)

# Default to C++17 — required by rclcpp
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings — treat warnings as errors in CI
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
#  Find dependencies
# ============================================================
#
#  ament_cmake:    ROS 2 build system macros
#  rclcpp:         C++ client library (nodes, publishers, params)
#  sensor_msgs:    PointCloud2 message definition
#
#  Note: no PCL dependency. We construct PointCloud2 manually
#  to avoid pulling in a heavy library for something simple.
#
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ============================================================
#  Build the driver library
# ============================================================
#
#  We build receiver and node as a shared library so that:
#    1. main.cpp links against it for the standalone executable
#    2. It can later be loaded as a component (composable node)
#
add_library(${PROJECT_NAME}_lib SHARED
        src/aeye_receiver.cpp
        src/aeye_driver_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
        rclcpp
        sensor_msgs
)

# ============================================================
#  Build the standalone executable
# ============================================================
add_executable(aeye_driver_node src/main.cpp)

target_link_libraries(aeye_driver_node
        ${PROJECT_NAME}_lib
)

ament_target_dependencies(aeye_driver_node
        rclcpp
)

# ============================================================
#  Test sender tool (no ROS 2 dependency — pure C++)
# ============================================================
add_executable(aeye_test_sender tools/aeye_test_sender.cpp)

target_include_directories(aeye_test_sender PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ============================================================
#  Install
# ============================================================

# Headers
install(DIRECTORY include/
        DESTINATION include
)

# Library
install(TARGETS ${PROJECT_NAME}_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Executable
install(TARGETS aeye_driver_node aeye_test_sender
        DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

# Config files
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)

# ============================================================
#  Testing (placeholder — we'll add tests later)
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
