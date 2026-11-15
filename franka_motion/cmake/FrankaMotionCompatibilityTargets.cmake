# Responsibility: Define retained low-level diagnostics and scene compatibility executables.

include_guard(GLOBAL)

add_executable(
  franka_moveit_sequence
  src/franka_moveit_sequence.cpp
  src/moveit_sequence_scenario.cpp
)
franka_motion_configure_cpp_target(franka_moveit_sequence)
ament_target_dependencies(
  franka_moveit_sequence
  geometry_msgs
  rclcpp
  sensor_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)
target_link_libraries(
  franka_moveit_sequence
  franka_motion_moveit_parameter_snapshot
  franka_motion_robot_profile_bootstrap
  manipulation::validation
)

add_executable(static_table_scene src/static_table_scene.cpp)
franka_motion_configure_cpp_target(static_table_scene)
ament_target_dependencies(
  static_table_scene
  ament_index_cpp
  rclcpp
)

add_executable(
  static_table_scene_coordinated
  src/scene/static_table_scene_main.cpp
  src/scene/static_table_scene_node.cpp
)
franka_motion_configure_cpp_target(static_table_scene_coordinated)
target_link_libraries(
  static_table_scene_coordinated
  manipulation::scene_client
  manipulation::scene_registry
)
ament_target_dependencies(
  static_table_scene_coordinated
  manipulation_msgs
  rclcpp
)

install(
  TARGETS franka_moveit_sequence
          static_table_scene
  DESTINATION lib/${PROJECT_NAME}
)

# Keep the implementation outside `lib/franka_motion`, which is recursively
# scanned by `ros2 pkg executables`, while retaining a stable private path.
install(
  TARGETS static_table_scene_coordinated
  DESTINATION libexec/${PROJECT_NAME}
)
