# Responsibility: Build the private Franka V2 composition and its standalone process.

include_guard(GLOBAL)

add_library(
  franka_motion_v2_composition
  STATIC
  src/workflow_v2/composition.cpp
  src/workflow_v2/isaac_physical_scene_port.cpp
)
franka_motion_configure_cpp_target(franka_motion_v2_composition)
target_include_directories(
  franka_motion_v2_composition
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
  franka_motion_v2_composition
  franka_motion_v2_providers
  manipulation::workflow
  manipulation_moveit::arm_planning
  manipulation_moveit::pick_lift_place_ros_adapters
  manipulation_moveit::pick_lift_place_strategy
  manipulation_moveit::v2_scene_adapter
)
ament_target_dependencies(
  franka_motion_v2_composition
  manipulation_msgs
  moveit_core
  moveit_ros_planning
  rclcpp
  std_srvs
)

add_executable(
  franka_transfer_sequence_v2_server
  src/workflow_v2/franka_transfer_sequence_v2_server_main.cpp
  src/workflow/franka_transfer_sequence_startup.cpp
)
franka_motion_configure_cpp_target(franka_transfer_sequence_v2_server)
target_include_directories(
  franka_transfer_sequence_v2_server
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
  franka_transfer_sequence_v2_server
  franka_motion_moveit_parameter_snapshot
  franka_motion_robot_profile_bootstrap
  franka_motion_v2_composition
)
ament_target_dependencies(
  franka_transfer_sequence_v2_server
  rclcpp
)

install(
  TARGETS franka_transfer_sequence_v2_server
  DESTINATION lib/${PROJECT_NAME}
)

# The composition library remains private; only the process executable is installed.
