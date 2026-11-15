# Responsibility: Register CPU-only tests for private V2 catalog and provider adapters.

franka_motion_add_gtest(
  test_v2_provider_config
  test/workflow_v2/test_provider_config.cpp
)
target_include_directories(
  test_v2_provider_config
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_compile_definitions(
  test_v2_provider_config
  PRIVATE FRANKA_MOTION_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
target_link_libraries(test_v2_provider_config franka_motion_v2_providers)

franka_motion_add_gtest(
  test_v2_process_config
  test/workflow_v2/test_process_config.cpp
)
target_include_directories(
  test_v2_process_config
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_compile_definitions(
  test_v2_process_config
  PRIVATE FRANKA_MOTION_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
target_link_libraries(test_v2_process_config franka_motion_v2_providers)

franka_motion_add_gtest(
  test_v2_observation_providers
  test/workflow_v2/test_observation_providers.cpp
)
target_include_directories(
  test_v2_observation_providers
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(test_v2_observation_providers franka_motion_v2_providers)

franka_motion_add_gtest(
  test_v2_provider_runtime
  test/workflow_v2/test_provider_runtime.cpp
)
target_include_directories(
  test_v2_provider_runtime
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_compile_definitions(
  test_v2_provider_runtime
  PRIVATE FRANKA_MOTION_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
target_link_libraries(test_v2_provider_runtime franka_motion_v2_providers)

franka_motion_add_gtest(
  test_v2_mtc_grasp_provider
  test/workflow_v2/test_mtc_grasp_provider.cpp
)
target_include_directories(
  test_v2_mtc_grasp_provider
  PRIVATE "${PROJECT_SOURCE_DIR}/src"
)
target_link_libraries(test_v2_mtc_grasp_provider franka_motion_v2_providers)
