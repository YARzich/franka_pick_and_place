# Responsibility: Build V2 adapters, Servo runtime, and fail-closed Octomap components.
add_library(arm_planning
  ${PROJECT_SOURCE_DIR}/src/arm_planning_port.cpp
  ${PROJECT_SOURCE_DIR}/src/moveit_arm_planning_adapter.cpp
  ${PROJECT_SOURCE_DIR}/src/trajectory_state_validation.cpp
)
target_compile_features(arm_planning PUBLIC cxx_std_17)
set_target_properties(arm_planning PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  arm_planning
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
  arm_planning
  manipulation::validation
  manipulation::workflow
)
ament_target_dependencies(
  arm_planning
  geometry_msgs
  manipulation_msgs
  moveit_core
  moveit_msgs
  rclcpp
  rclcpp_action
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(arm_planning PRIVATE -Werror=switch)
endif()
set_target_properties(arm_planning PROPERTIES EXPORT_NAME arm_planning)

add_library(exact_scene_move_group_capability SHARED
  ${PROJECT_SOURCE_DIR}/src/exact_planning_scene.cpp
  ${PROJECT_SOURCE_DIR}/src/goal_planning_cancellation.cpp
  ${PROJECT_SOURCE_DIR}/src/exact_planning_scene_move_group_capability.cpp
)
target_compile_features(exact_scene_move_group_capability PUBLIC cxx_std_17)
target_include_directories(
  exact_scene_move_group_capability PRIVATE "${PROJECT_SOURCE_DIR}/src")
ament_target_dependencies(
  exact_scene_move_group_capability
  moveit_core
  moveit_msgs
  moveit_ros_move_group
  pluginlib
  rclcpp
  rclcpp_action
)
set_target_properties(
  exact_scene_move_group_capability
  PROPERTIES OUTPUT_NAME manipulation_moveit_exact_scene_move_group_capability)

pluginlib_export_plugin_description_file(
  moveit_ros_move_group
  exact_planning_scene_move_group_capability_plugin_description.xml
)

add_library(cancelable_execute_trajectory_capability SHARED
  ${PROJECT_SOURCE_DIR}/src/cancelable_trajectory_execution.cpp
  ${PROJECT_SOURCE_DIR}/src/cancelable_execute_trajectory_capability.cpp
)
target_compile_features(cancelable_execute_trajectory_capability PUBLIC cxx_std_17)
target_include_directories(
  cancelable_execute_trajectory_capability PRIVATE "${PROJECT_SOURCE_DIR}/src")
ament_target_dependencies(
  cancelable_execute_trajectory_capability
  moveit_core
  moveit_msgs
  moveit_ros_move_group
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_action
)
set_target_properties(
  cancelable_execute_trajectory_capability
  PROPERTIES OUTPUT_NAME manipulation_moveit_cancelable_execute_trajectory_capability)
pluginlib_export_plugin_description_file(
  moveit_ros_move_group
  cancelable_execute_trajectory_capability_plugin_description.xml
)

add_library(pick_lift_place_strategy
  ${PROJECT_SOURCE_DIR}/src/pick_lift_place_ports.cpp
  ${PROJECT_SOURCE_DIR}/src/pick_lift_place_strategy.cpp
)
target_compile_features(pick_lift_place_strategy PUBLIC cxx_std_17)
set_target_properties(pick_lift_place_strategy PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  pick_lift_place_strategy
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
  pick_lift_place_strategy
  arm_planning
  Eigen3::Eigen
  manipulation::task
  manipulation::validation
  manipulation::workflow
)
ament_target_dependencies(
  pick_lift_place_strategy
  geometry_msgs
  moveit_msgs
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(pick_lift_place_strategy PRIVATE -Werror=switch)
endif()
set_target_properties(
  pick_lift_place_strategy PROPERTIES EXPORT_NAME pick_lift_place_strategy)

add_library(pick_lift_place_ros_adapters
  ${PROJECT_SOURCE_DIR}/src/ros_pick_lift_place_health_port.cpp
  ${PROJECT_SOURCE_DIR}/src/ros_v2_physical_ports.cpp
)
target_compile_features(pick_lift_place_ros_adapters PUBLIC cxx_std_17)
set_target_properties(
  pick_lift_place_ros_adapters PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  pick_lift_place_ros_adapters
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
  pick_lift_place_ros_adapters
  pick_lift_place_strategy
  manipulation::task
  manipulation::workflow
)
ament_target_dependencies(
  pick_lift_place_ros_adapters
  control_msgs
  manipulation_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(pick_lift_place_ros_adapters PRIVATE -Werror=switch)
endif()
set_target_properties(
  pick_lift_place_ros_adapters PROPERTIES EXPORT_NAME pick_lift_place_ros_adapters)

add_library(v2_scene_adapter
  ${PROJECT_SOURCE_DIR}/src/geometric_shapes_mesh_resource_loader.cpp
  ${PROJECT_SOURCE_DIR}/src/ros_v2_scene_adapter.cpp
  ${PROJECT_SOURCE_DIR}/src/ros_v2_scene_adapter_calls.cpp
  ${PROJECT_SOURCE_DIR}/src/ros_v2_scene_adapter_mutations.cpp
  ${PROJECT_SOURCE_DIR}/src/v2_scene_adapter_support.cpp
)
target_compile_features(v2_scene_adapter PUBLIC cxx_std_17)
set_target_properties(v2_scene_adapter PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  v2_scene_adapter
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
  v2_scene_adapter
  arm_planning
  pick_lift_place_strategy
  manipulation::scene_support
  manipulation::transfer_domain
  manipulation::validation
)
ament_target_dependencies(
  v2_scene_adapter
  builtin_interfaces
  geometric_shapes
  geometry_msgs
  manipulation_msgs
  moveit_core
  moveit_msgs
  rclcpp
  shape_msgs
  tf2_eigen
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(v2_scene_adapter PRIVATE -Werror=switch)
endif()
set_target_properties(v2_scene_adapter PROPERTIES EXPORT_NAME v2_scene_adapter)

add_library(moveit_servo_ros_adapter
  ${PROJECT_SOURCE_DIR}/src/servo/moveit_servo_ros_adapter.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/moveit_servo_ros_adapter_feedback.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/moveit_servo_ros_adapter_services.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/moveit_servo_ros_adapter_tf.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/moveit_servo_ros_adapter_types.cpp
)
target_compile_features(moveit_servo_ros_adapter PUBLIC cxx_std_17)
set_target_properties(moveit_servo_ros_adapter PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  moveit_servo_ros_adapter
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  PRIVATE "${PROJECT_SOURCE_DIR}/src/servo"
)
target_link_libraries(
  moveit_servo_ros_adapter
  Eigen3::Eigen
  manipulation::servo
  manipulation::validation
)
ament_target_dependencies(
  moveit_servo_ros_adapter
  builtin_interfaces
  geometry_msgs
  moveit_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)
set_target_properties(
  moveit_servo_ros_adapter PROPERTIES EXPORT_NAME moveit_servo_ros_adapter)

add_library(moveit_servo_task
  ${PROJECT_SOURCE_DIR}/src/servo/servo_to_pose_action_callbacks.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/servo_to_pose_action_feedback.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/servo_to_pose_action_recording.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/servo_to_pose_action_server.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/servo_to_pose_action_worker.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/servo_to_pose_port.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/servo_to_pose_task.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/servo_to_pose_task_control.cpp
  ${PROJECT_SOURCE_DIR}/src/servo/servo_to_pose_task_policy.cpp
)
target_compile_features(moveit_servo_task PUBLIC cxx_std_17)
set_target_properties(moveit_servo_task PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  moveit_servo_task
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  PRIVATE "${PROJECT_SOURCE_DIR}/src/servo"
)
target_link_libraries(
  moveit_servo_task
  moveit_servo_ros_adapter
  manipulation::recording
  manipulation::servo
  manipulation::skill
  manipulation::task
  manipulation::validation
  manipulation::workflow
)
ament_target_dependencies(
  moveit_servo_task
  builtin_interfaces
  geometry_msgs
  manipulation_msgs
  rclcpp
  rclcpp_action
  unique_identifier_msgs
)
set_target_properties(moveit_servo_task PROPERTIES EXPORT_NAME moveit_servo_task)

add_library(moveit_servo_task_host
  ${PROJECT_SOURCE_DIR}/src/config/moveit_servo_process_config.cpp
  ${PROJECT_SOURCE_DIR}/src/config/moveit_servo_process_identity.cpp
  ${PROJECT_SOURCE_DIR}/src/moveit_servo_task_host.cpp
)
target_compile_features(moveit_servo_task_host PUBLIC cxx_std_17)
set_target_properties(moveit_servo_task_host PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  moveit_servo_task_host
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
  moveit_servo_task_host
  moveit_servo_task
  manipulation::skill
  manipulation::workflow
)
ament_target_dependencies(moveit_servo_task_host rclcpp)
set_target_properties(
  moveit_servo_task_host PROPERTIES EXPORT_NAME moveit_servo_task_host)

add_executable(
  moveit_servo_task_server
  ${PROJECT_SOURCE_DIR}/src/moveit_servo_task_main.cpp
)
target_compile_features(moveit_servo_task_server PUBLIC cxx_std_17)
target_link_libraries(moveit_servo_task_server moveit_servo_task_host)
ament_target_dependencies(moveit_servo_task_server rclcpp)

add_library(
  octomap_tf_gate_support
  ${PROJECT_SOURCE_DIR}/src/stamped_cloud_tf_gate.cpp
)
target_compile_features(octomap_tf_gate_support PUBLIC cxx_std_17)
set_target_properties(octomap_tf_gate_support PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  octomap_tf_gate_support
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(octomap_tf_gate_support manipulation::validation)
ament_target_dependencies(
  octomap_tf_gate_support
  diagnostic_msgs
  diagnostic_updater
  message_filters
  rcl_interfaces
  rclcpp
  sensor_msgs
  tf2_ros
)
set_target_properties(
  octomap_tf_gate_support PROPERTIES EXPORT_NAME octomap_tf_gate_support)

add_executable(
  moveit_octomap_tf_gate
  ${PROJECT_SOURCE_DIR}/src/stamped_cloud_tf_gate_main.cpp
)
target_compile_features(moveit_octomap_tf_gate PUBLIC cxx_std_17)
target_link_libraries(moveit_octomap_tf_gate octomap_tf_gate_support)
ament_target_dependencies(moveit_octomap_tf_gate rclcpp)

add_library(
  fail_closed_octomap_updater SHARED
  ${PROJECT_SOURCE_DIR}/src/fail_closed_pointcloud_octomap_updater.cpp
)
target_compile_features(fail_closed_octomap_updater PUBLIC cxx_std_17)
target_include_directories(
  fail_closed_octomap_updater PRIVATE "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(
  fail_closed_octomap_updater
  moveit_ros_perception::moveit_pointcloud_octomap_updater_core
  pluginlib::pluginlib
)
ament_target_dependencies(
  fail_closed_octomap_updater
  moveit_ros_occupancy_map_monitor
  rclcpp
)
set_target_properties(
  fail_closed_octomap_updater
  PROPERTIES
    EXPORT_NAME fail_closed_octomap_updater
    OUTPUT_NAME manipulation_moveit_fail_closed_octomap_updater
)

pluginlib_export_plugin_description_file(
  moveit_ros_occupancy_map_monitor
  fail_closed_octomap_updater_plugin_description.xml
)
