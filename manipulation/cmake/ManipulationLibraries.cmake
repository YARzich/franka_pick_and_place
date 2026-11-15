# Responsibility: Build exported libraries while preserving explicit layer boundaries.

function(manipulation_configure_library target export_name)
  target_compile_features(${target} PUBLIC cxx_std_17)
  # Exported archives must remain linkable from shared ROS components/plugins.
  set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(
    ${target}
    PUBLIC
      "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>"
  )
  set_target_properties(${target} PROPERTIES EXPORT_NAME ${export_name})
endfunction()

# Header-only validation rules are an explicit dependency boundary. Consumers
# should not link a scene, workflow, or task implementation merely to include
# deterministic validators.
add_library(manipulation_validation INTERFACE)
target_compile_features(manipulation_validation INTERFACE cxx_std_17)
target_include_directories(
  manipulation_validation
  INTERFACE
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
set_target_properties(manipulation_validation PROPERTIES EXPORT_NAME validation)

# Header-only ROS transport utilities form their own dependency boundary so a
# backend does not need to link workflow or scene policy merely to issue a
# bounded service request.
add_library(manipulation_transport INTERFACE)
target_compile_features(manipulation_transport INTERFACE cxx_std_17)
target_include_directories(
  manipulation_transport
  INTERFACE
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(manipulation_transport INTERFACE rclcpp::rclcpp)
set_target_properties(manipulation_transport PROPERTIES EXPORT_NAME transport)

add_library(manipulation_task
  src/task/execution_control.cpp
  src/task/task_status.cpp
)
manipulation_configure_library(manipulation_task task)

add_library(manipulation_servo
  src/servo/servo_to_pose_state_machine.cpp
)
manipulation_configure_library(manipulation_servo servo)

add_library(manipulation_skill
  src/skill/capability_set.cpp
  src/skill/preset_identity.cpp
  src/skill/resource_claim.cpp
  src/skill/resource_registry.cpp
  src/skill/skill_descriptor.cpp
)
manipulation_configure_library(manipulation_skill skill)
target_link_libraries(manipulation_skill OpenSSL::Crypto)

add_library(manipulation_world src/world/world_state_snapshot.cpp)
manipulation_configure_library(manipulation_world world)
target_link_libraries(manipulation_world manipulation_validation)
ament_target_dependencies(manipulation_world builtin_interfaces geometry_msgs)

add_library(manipulation_recording src/recording/episode_event_sink.cpp)
manipulation_configure_library(manipulation_recording recording)
target_link_libraries(
  manipulation_recording
  manipulation_skill
  manipulation_task
  manipulation_world
)

add_library(manipulation_scene_registry
  src/scene/resource_key.cpp
  src/scene/scene_lease_registry.cpp
  src/scene/scene_lease_registry_expiry.cpp
  src/scene/scene_lease_registry_leases.cpp
  src/scene/scene_lease_registry_operations.cpp
)
manipulation_configure_library(manipulation_scene_registry scene_registry)

add_library(manipulation_scene_client src/scene/scene_client_support.cpp)
manipulation_configure_library(manipulation_scene_client scene_client)
ament_target_dependencies(manipulation_scene_client builtin_interfaces)

add_library(manipulation_scene_support
  src/scene/collision_object_validation.cpp
  src/scene/coordinator_contract_utils.cpp
  src/scene/coordinator_digest.cpp
  src/scene/planning_scene_diff_validator.cpp
  src/scene/robot_description_frame.cpp
  src/scene/scene_acm_state.cpp
  src/scene/scene_frame_transform.cpp
  src/scene/scene_object_canonicalization.cpp
  src/scene/scene_state_canonicalization.cpp
  src/scene/scene_state_model.cpp
  src/scene/transfer_object_scene.cpp
)
manipulation_configure_library(manipulation_scene_support scene_support)
target_link_libraries(
  manipulation_scene_support
  manipulation_scene_registry
  OpenSSL::Crypto
)
ament_target_dependencies(
  manipulation_scene_support
  builtin_interfaces
  geometry_msgs
  manipulation_msgs
  moveit_msgs
  object_recognition_msgs
  rclcpp
  shape_msgs
  tf2
  tf2_geometry_msgs
  urdf
)

add_library(manipulation_transfer_domain
  src/workflow/object_catalog.cpp
  src/workflow/object_geometry_conversion.cpp
  src/workflow/strategy_registry.cpp
  src/workflow/transfer_execution_context.cpp
  src/workflow/transfer_scene_workflow_port.cpp
  src/workflow/transfer_workflow_types.cpp
)
manipulation_configure_library(manipulation_transfer_domain transfer_domain)
target_link_libraries(
  manipulation_transfer_domain
  manipulation_task
  manipulation_validation
)
ament_target_dependencies(
  manipulation_transfer_domain
  builtin_interfaces
  geometry_msgs
  moveit_msgs
  shape_msgs
)

add_library(manipulation_workflow
  src/workflow/resolved_transfer_workflow.cpp
  src/workflow/transfer_target_resolution.cpp
  src/workflow/resolved_transfer_workflow_action_adapter.cpp
  src/workflow/scene_lease_release_policy.cpp
  src/workflow/single_goal_task_worker.cpp
  src/workflow/skill_runtime.cpp
  src/workflow/task_message_adapter.cpp
  src/workflow/transfer_sequence_action_core.cpp
  src/workflow/transfer_sequence_action_core_callbacks.cpp
  src/workflow/transfer_sequence_action_core_completion.cpp
  src/workflow/transfer_sequence_action_core_feedback.cpp
  src/workflow/transfer_sequence_action_core_pump.cpp
  src/workflow/transfer_sequence_action_core_reservation.cpp
  src/workflow/transfer_sequence_action_core_snapshot.cpp
  src/workflow/detail/transfer_sequence_goal_transport.cpp
  src/workflow/transfer_sequence_execution_policy.cpp
  src/workflow/transfer_sequence_goal_reservation.cpp
  src/workflow/transfer_object_sequence_v2_action_server.cpp
  src/workflow/transfer_object_sequence_v2_goal_validation.cpp
  src/workflow/transfer_sequence_session.cpp
  src/workflow/transfer_sequence_task_context.cpp
  src/workflow/wait_lifecycle_logger.cpp
)
manipulation_configure_library(manipulation_workflow workflow)
target_link_libraries(
  manipulation_workflow
  manipulation_recording
  manipulation_skill
  manipulation_task
  manipulation_transfer_domain
  manipulation_world
)
ament_target_dependencies(
  manipulation_workflow
  builtin_interfaces
  geometry_msgs
  manipulation_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  shape_msgs
  unique_identifier_msgs
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(manipulation_workflow PRIVATE -Werror=switch)
endif()

add_library(manipulation_monitoring
  src/monitoring/freshness_monitor.cpp
  src/monitoring/health_aggregator.cpp
  src/monitoring/perception_health_policy.cpp
  src/monitoring/planning_scene_health_policy.cpp
)
manipulation_configure_library(manipulation_monitoring monitoring)
