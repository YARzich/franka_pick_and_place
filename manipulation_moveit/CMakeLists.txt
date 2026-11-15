# Responsibility: Compose V2 MoveIt adapters, Servo task, safety components, and tests.
cmake_minimum_required(VERSION 3.16)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
project(manipulation_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(cmake/Dependencies.cmake)
include(cmake/Targets.cmake)
include(cmake/CompatibilityInstall.cmake)

install(
  TARGETS
    arm_planning
    pick_lift_place_strategy
    pick_lift_place_ros_adapters
    v2_scene_adapter
    moveit_servo_ros_adapter
    moveit_servo_task
    moveit_servo_task_host
    octomap_tf_gate_support
    fail_closed_octomap_updater
  EXPORT export_manipulation_moveit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  TARGETS moveit_servo_task_server moveit_octomap_tf_gate
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS exact_scene_move_group_capability cancelable_execute_trajectory_capability
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  FILES
    API.md
    CHANGELOG.rst
    LICENSE
    README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  include(cmake/Tests.cmake)
endif()

ament_export_targets(export_manipulation_moveit HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  control_msgs
  diagnostic_msgs
  diagnostic_updater
  Eigen3
  geometric_shapes
  geometry_msgs
  manipulation
  manipulation_msgs
  message_filters
  moveit_core
  moveit_msgs
  moveit_ros_occupancy_map_monitor
  moveit_ros_perception
  moveit_ros_move_group
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  shape_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  unique_identifier_msgs
)
ament_export_include_directories(include/${PROJECT_NAME})
ament_package()
