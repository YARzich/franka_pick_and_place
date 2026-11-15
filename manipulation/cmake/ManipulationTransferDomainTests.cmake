# Responsibility: Register fast tests for backend-neutral transfer-domain contracts.

include_guard(GLOBAL)

ament_add_gtest(
  test_object_catalog
  test/workflow/test_object_catalog.cpp
)
target_include_directories(test_object_catalog PRIVATE test/workflow)
target_link_libraries(test_object_catalog manipulation_transfer_domain)

ament_add_gtest(
  test_transfer_workflow_types
  test/workflow/test_transfer_workflow_types.cpp
)
target_include_directories(test_transfer_workflow_types PRIVATE test/workflow)
target_link_libraries(test_transfer_workflow_types manipulation_transfer_domain)

ament_add_gtest(
  test_strategy_registry
  test/workflow/test_strategy_registry.cpp
)
target_include_directories(test_strategy_registry PRIVATE test/workflow)
target_link_libraries(test_strategy_registry manipulation_transfer_domain)

ament_add_gtest(
  test_resolved_transfer_workflow
  test/workflow/test_resolved_transfer_workflow.cpp
)
target_include_directories(test_resolved_transfer_workflow PRIVATE test/workflow)
target_link_libraries(test_resolved_transfer_workflow manipulation_workflow)

ament_add_gtest(
  test_transfer_target_resolution
  test/workflow/test_transfer_target_resolution.cpp
)
target_include_directories(test_transfer_target_resolution PRIVATE src/workflow test/workflow)
target_link_libraries(test_transfer_target_resolution manipulation_workflow)
