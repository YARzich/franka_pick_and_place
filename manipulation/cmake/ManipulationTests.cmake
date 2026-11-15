# Responsibility: Compose manipulation test registration by architectural domain.
# Does not own: Individual test targets or launch-test definitions.

include_guard(GLOBAL)

find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

include(${CMAKE_CURRENT_LIST_DIR}/ManipulationTransportTaskTests.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ManipulationServoTests.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ManipulationSkillWorldTests.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ManipulationSceneTests.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ManipulationTransferDomainTests.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ManipulationWorkflowTests.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ManipulationMonitoringTests.cmake)

ament_add_pytest_test(
  test_architecture_boundaries
  test/test_architecture_boundaries.py
  TIMEOUT 30
)

ament_add_pytest_test(
  test_incremental_install_surface
  test/test_incremental_install_surface.py
  ENV "MANIPULATION_TEST_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
  TIMEOUT 30
)

list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cppcheck)
ament_lint_auto_find_test_dependencies()

# The automatic hook cannot pass the GoogleTest macro configuration.
ament_cppcheck(
  LANGUAGE c++
  LIBRARIES googletest
  INCLUDE_DIRS
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/src/monitoring"
    "${PROJECT_SOURCE_DIR}/test/servo"
    "${PROJECT_SOURCE_DIR}/test/workflow"
    "${PROJECT_SOURCE_DIR}/src/workflow"
)
# Jazzy ament_cppcheck otherwise skips every cppcheck 2.x release.
set_property(TEST cppcheck APPEND PROPERTY ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
