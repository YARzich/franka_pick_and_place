# Responsibility: Build the canonical Panda scenario, retained diagnostics, and tests.
cmake_minimum_required(VERSION 3.16)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
project(franka_motion)

option(
  FRANKA_MOTION_ENABLE_SIMULATOR_LAUNCH_TESTS
  "Register Gazebo and Isaac Sim launch tests (requires a runtime-capable host)"
  OFF
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(manipulation REQUIRED)
find_package(manipulation_moveit REQUIRED)
find_package(manipulation_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

function(franka_motion_configure_cpp_target target_name)
  target_compile_features(${target_name} PUBLIC cxx_std_17)
  target_include_directories(
    ${target_name}
    PUBLIC
      "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>"
  )
endfunction()

include(cmake/FrankaMotionRobotProfileBootstrapTarget.cmake)
include(cmake/FrankaMotionMoveItParameterSnapshotTarget.cmake)
include(cmake/FrankaMotionCompatibilityTargets.cmake)
include(cmake/FrankaMotionV2ProviderTargets.cmake)
include(cmake/FrankaMotionV2CompositionTargets.cmake)
include(cmake/FrankaMotionControlTargets.cmake)
include(cmake/FrankaMotionRuntimeInstall.cmake)
include(cmake/FrankaMotionCompatibilityInstall.cmake)

if(BUILD_TESTING)
  include(cmake/FrankaMotionTesting.cmake)
endif()

ament_export_include_directories(include)
ament_export_targets(export_franka_motion_scenario HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  control_msgs
  Eigen3
  manipulation
  manipulation_moveit
  geometry_msgs
  rcl_interfaces
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  moveit_task_constructor_core
  sensor_msgs
  shape_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
