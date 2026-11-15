# Responsibility: Build the private immutable MoveIt parameter snapshot helper used by V2.

include_guard(GLOBAL)

add_library(
  franka_motion_moveit_parameter_snapshot
  STATIC
  src/moveit_parameter_snapshot.cpp
  src/moveit_parameter_sync.cpp
  src/parameter_snapshot_apply.cpp
  src/parameter_snapshot_client.cpp
  src/parameter_snapshot_helper_node.cpp
)
franka_motion_configure_cpp_target(franka_motion_moveit_parameter_snapshot)
ament_target_dependencies(
  franka_motion_moveit_parameter_snapshot
  rcl_interfaces
  rclcpp
)

# This startup helper is an implementation detail of the V2 composition root.
# It deliberately has no install or export rule.
