# Responsibility: Register PlanningScene policy, registry, and coordinator tests.
# Does not own: Workflow execution or action-client tests.

include_guard(GLOBAL)

ament_add_gtest(test_resource_key test/scene/test_resource_key.cpp)
target_link_libraries(test_resource_key manipulation_scene_registry)

ament_add_gtest(test_box_collision test/scene/test_box_collision.cpp)
target_link_libraries(test_box_collision manipulation_scene_support)
ament_target_dependencies(test_box_collision geometry_msgs moveit_msgs shape_msgs)

ament_add_gtest(
  test_collision_object_validation
  test/scene/test_collision_object_validation.cpp
)
target_link_libraries(test_collision_object_validation manipulation_scene_support)

ament_add_gtest(test_scene_client_support test/scene/test_scene_client_support.cpp)
target_link_libraries(test_scene_client_support manipulation_scene_client)

ament_add_gtest(
  test_transfer_object_scene
  test/scene/test_transfer_object_scene.cpp
)
target_link_libraries(test_transfer_object_scene manipulation_scene_support)

ament_add_gtest(test_coordinator_utils test/scene/test_coordinator_utils.cpp)
target_link_libraries(test_coordinator_utils manipulation_scene_support)
ament_target_dependencies(test_coordinator_utils geometry_msgs shape_msgs)

ament_add_gtest(
  test_scene_lease_registry_leases
  test/scene/test_scene_lease_registry_leases.cpp
)
target_link_libraries(test_scene_lease_registry_leases manipulation_scene_registry)

ament_add_gtest(
  test_scene_lease_registry_operations
  test/scene/test_scene_lease_registry_operations.cpp
)
target_link_libraries(test_scene_lease_registry_operations manipulation_scene_registry)

add_launch_test(test/scene/test_validate_lease_launch.py TIMEOUT 30)
add_launch_test(test/scene/test_ambiguous_apply_launch.py TIMEOUT 30)
add_launch_test(test/scene/test_quarantine_recovery_launch.py TIMEOUT 45)
add_launch_test(test/scene/test_frame_contract_launch.py TIMEOUT 45)
