# Responsibility: Register skill metadata, resource, world-state, and recording unit tests.
# Does not own: ROS workflow, scene, client, or monitoring test registration.

include_guard(GLOBAL)

ament_add_gtest(test_preset_identity test/skill/test_preset_identity.cpp)
target_link_libraries(test_preset_identity manipulation_skill)

ament_add_gtest(test_capability_set test/skill/test_capability_set.cpp)
target_link_libraries(test_capability_set manipulation_skill)

ament_add_gtest(test_resource_registry test/skill/test_resource_registry.cpp)
target_link_libraries(test_resource_registry manipulation_skill)

ament_add_gtest(test_skill_descriptor test/skill/test_skill_descriptor.cpp)
target_link_libraries(test_skill_descriptor manipulation_skill)

ament_add_gtest(test_world_state_snapshot test/world/test_world_state_snapshot.cpp)
target_link_libraries(test_world_state_snapshot manipulation_world)

ament_add_gtest(test_episode_event_sink test/recording/test_episode_event_sink.cpp)
target_link_libraries(test_episode_event_sink manipulation_recording)
