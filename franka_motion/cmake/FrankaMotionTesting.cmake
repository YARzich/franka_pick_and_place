# Responsibility: Configure linting and compose canonical Panda scenario tests.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)

set(ament_cmake_flake8_CONFIG_FILE "${PROJECT_SOURCE_DIR}/setup.cfg")
list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cppcheck)
ament_lint_auto_find_test_dependencies()

# The automatic hook cannot pass the GoogleTest macro configuration.
ament_cppcheck(
  LANGUAGE c++
  LIBRARIES googletest
  INCLUDE_DIRS
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/src"
    "${PROJECT_SOURCE_DIR}/src/control"
    "${PROJECT_SOURCE_DIR}/src/mtc"
)
# Jazzy ament_cppcheck otherwise skips every cppcheck 2.x release.
set_property(TEST cppcheck APPEND PROPERTY ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")

include(cmake/FrankaMotionGTests.cmake)
include(cmake/FrankaMotionV2ProviderTests.cmake)
include(cmake/FrankaMotionV2CompositionTests.cmake)
include(cmake/FrankaMotionControlTests.cmake)
include(cmake/FrankaMotionSimulationTests.cmake)
include(cmake/FrankaMotionPyTests.cmake)
include(cmake/FrankaMotionLaunchTests.cmake)
