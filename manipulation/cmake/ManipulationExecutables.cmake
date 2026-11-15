# Responsibility: Build thin ROS process adapters around exported manipulation libraries.

add_executable(planning_scene_coordinator
  src/scene/planning_scene_coordinator_main.cpp
  src/scene/planning_scene_coordinator_backend.cpp
  src/scene/planning_scene_coordinator_diagnostics.cpp
  src/scene/planning_scene_coordinator_frame_adapter.cpp
  src/scene/planning_scene_coordinator_node.cpp
  src/scene/planning_scene_coordinator_model_frame.cpp
  src/scene/planning_scene_coordinator_reconciliation.cpp
  src/scene/planning_scene_coordinator_services.cpp
  src/scene/planning_scene_coordinator_state_services.cpp
)
target_compile_features(planning_scene_coordinator PUBLIC cxx_std_17)
target_link_libraries(planning_scene_coordinator manipulation_scene_support)
ament_target_dependencies(
  planning_scene_coordinator
  diagnostic_msgs
  diagnostic_updater
  manipulation_msgs
  moveit_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

add_executable(system_health_monitor
  src/monitoring/duration_conversion.cpp
  src/monitoring/health_message_adapter.cpp
  src/monitoring/perception_diagnostics_adapter.cpp
  src/monitoring/service_request_tracker.cpp
  src/monitoring/system_health_monitor.cpp
  src/monitoring/system_health_monitor_diagnostics.cpp
  src/monitoring/system_health_monitor_health.cpp
  src/monitoring/system_health_monitor_main.cpp
  src/monitoring/system_health_monitor_services.cpp
)
target_compile_features(system_health_monitor PUBLIC cxx_std_17)
target_include_directories(system_health_monitor PRIVATE src/monitoring)
target_link_libraries(system_health_monitor manipulation_monitoring manipulation_validation)
ament_target_dependencies(
  system_health_monitor
  builtin_interfaces
  controller_manager_msgs
  control_msgs
  diagnostic_msgs
  diagnostic_updater
  manipulation_msgs
  rclcpp
  rclcpp_action
  rosgraph_msgs
  sensor_msgs
  tf2
  tf2_ros
)

install(
  TARGETS
    planning_scene_coordinator
    system_health_monitor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
