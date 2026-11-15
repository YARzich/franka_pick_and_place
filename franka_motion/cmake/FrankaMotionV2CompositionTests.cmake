# Responsibility: Register CPU-only contract tests for the private V2 composition root.

franka_motion_add_gtest(
  test_v2_composition_contract
  test/workflow_v2/test_composition_contract.cpp
)
target_include_directories(
  test_v2_composition_contract
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_compile_definitions(
  test_v2_composition_contract
  PRIVATE FRANKA_MOTION_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
target_link_libraries(
  test_v2_composition_contract
  franka_motion_v2_composition
)

franka_motion_add_gtest(
  test_isaac_physical_scene_port
  test/workflow_v2/test_isaac_physical_scene_port.cpp
)
target_include_directories(
  test_isaac_physical_scene_port
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(
  test_isaac_physical_scene_port
  franka_motion_v2_composition
)
