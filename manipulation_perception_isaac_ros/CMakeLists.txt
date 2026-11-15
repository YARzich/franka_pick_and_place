# Responsibility: Install and validate the optional Isaac ROS dependency bundle for perception.
cmake_minimum_required(VERSION 3.16)
project(manipulation_perception_isaac_ros)

find_package(ament_cmake REQUIRED)

install(
  FILES CHANGELOG.rst LICENSE README.md THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_dependency_contract
    test/test_dependency_contract.py
    TIMEOUT 30
  )
endif()

ament_package()
